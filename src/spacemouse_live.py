#!/usr/bin/env python3
"""Live 6-DoF readout from the SpaceMouse. Read-only — commands nothing.

This is the first milestone of the teleop chain: prove the input device is readable
and correctly decoded before anything is ever wired to a motor.

    uv run src/spacemouse_live.py

Ctrl-C to stop.

Protocol note. A 3Dconnexion device reports as:
  report 0x01 — translation, 3x int16 little-endian  (x, y, z)
  report 0x02 — rotation,    3x int16 little-endian  (rx, ry, rz)
  report 0x03 — buttons, a bitfield
Some firmware packs translation *and* rotation into a single 13-byte 0x01 report.
Both shapes are handled below; which one this unit uses is printed on first sight.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

from spacemouse import countdown_hands_off, describe, find_device, is_multi_axis, open_device

FULL_SCALE = 350.0  # raw counts at full deflection; nominal, calibrate if it matters
DEADZONE = 0.02


def scale(raw: int) -> float:
    v = max(-1.0, min(1.0, raw / FULL_SCALE))
    return 0.0 if abs(v) < DEADZONE else v


def bar(v: float, width: int = 11) -> str:
    """A centred bar: left of centre for negative, right for positive."""
    half = width // 2
    n = int(round(abs(v) * half))
    if v >= 0:
        return " " * half + "|" + "█" * n + " " * (half - n)
    return " " * (half - n) + "█" * n + "|" + " " * half


def main() -> int:
    ap = argparse.ArgumentParser(description="Live 6-DoF SpaceMouse readout (read-only).")
    ap.add_argument(
        "--seconds",
        type=float,
        default=None,
        help="stop after N seconds instead of running until Ctrl-C (needed when run non-interactively)",
    )
    ap.add_argument(
        "--raw",
        type=int,
        default=0,
        metavar="N",
        help="dump the first N raw HID reports as hex, to verify the decode against real bytes",
    )
    ap.add_argument(
        "--until-complete",
        action="store_true",
        help="stop as soon as every axis has been deflected past --threshold, "
        "so the operator does not have to match a countdown they cannot see",
    )
    ap.add_argument("--threshold", type=float, default=0.25, help="deflection counting as 'this axis moved'")
    ap.add_argument("--countdown", type=int, default=3, help="seconds to let the puck centre before seizing")
    ap.add_argument(
        "--no-countdown",
        action="store_true",
        help="skip the hands-off countdown (only safe if the puck is already at rest)",
    )
    args = ap.parse_args()

    info = find_device()
    if info is None:
        print("No SpaceMouse found. Plugged in?")
        return 1

    print(f"Found {describe(info)}")
    if not is_multi_axis(info):
        print("  ⚠️  this is NOT the multi-axis interface — if nothing moves, that is why.")

    if not args.no_countdown:
        countdown_hands_off(args.countdown)

    try:
        h = open_device(info)
    except OSError as exc:
        print(f"Could not open ({exc}). macOS: Privacy & Security → Input Monitoring.")
        return 2

    h.set_nonblocking(True)
    live = sys.stdout.isatty()
    if args.seconds:
        print(f"Move the SpaceMouse. Stopping automatically after {args.seconds:g} s.\n")
    else:
        print("Move the SpaceMouse. Ctrl-C to stop.\n")

    t = [0.0, 0.0, 0.0]      # x, y, z
    r = [0.0, 0.0, 0.0]      # roll, pitch, yaw
    buttons = 0
    shapes_seen: set[str] = set()

    # Peak absolute deflection per axis, so a non-interactive run still reports
    # something meaningful: the bar display is useless when stdout is captured.
    peak = [0.0] * 6
    reports = 0
    buttons_seen: set[int] = set()
    axis_names = ("x", "y", "z", "roll", "pitch", "yaw")

    deadline = time.time() + args.seconds if args.seconds else None
    next_tick = time.time() + 2.0
    raw_shown = 0
    read_errors = 0

    try:
        while deadline is None or time.time() < deadline:
            try:
                data = h.read(64)
            except OSError as exc:
                # hidapi surfaces a transient -1 read as OSError. Observed once on macOS
                # mid-session with the device still perfectly healthy, so a single failure
                # must not end the run — but a persistent one is a real fault, not noise.
                read_errors += 1
                if read_errors > 50:
                    print(f"\n✗ giving up after {read_errors} consecutive read errors: {exc}")
                    break
                time.sleep(0.02)
                continue
            if data:
                read_errors = 0
                if raw_shown < args.raw:
                    raw_shown += 1
                    print(f"  raw[{raw_shown:02d}] id={data[0]:#04x} len={len(data)} {bytes(data).hex(' ')}")
                rid, payload = data[0], bytes(data[1:])
                if rid == 0x01 and len(payload) >= 12:
                    vals = struct.unpack("<6h", payload[:12])
                    t = [scale(v) for v in vals[:3]]
                    r = [scale(v) for v in vals[3:]]
                    shapes_seen.add("combined 0x01 (translation+rotation)")
                    reports += 1
                elif rid == 0x01 and len(payload) >= 6:
                    t = [scale(v) for v in struct.unpack("<3h", payload[:6])]
                    shapes_seen.add("split 0x01/0x02")
                    reports += 1
                elif rid == 0x02 and len(payload) >= 6:
                    r = [scale(v) for v in struct.unpack("<3h", payload[:6])]
                    shapes_seen.add("split 0x01/0x02")
                    reports += 1
                elif rid == 0x03 and payload:
                    buttons = int.from_bytes(payload[:2], "little")
                    if buttons:
                        buttons_seen.add(buttons)
                    reports += 1

                for i, v in enumerate((*t, *r)):
                    peak[i] = max(peak[i], abs(v))

                if args.until_complete and all(p >= args.threshold for p in peak):
                    print("\n  ✓ all six axes seen — stopping early, nothing more is needed.")
                    break

            if live:
                sys.stdout.write(
                    f"\r x{bar(t[0])} y{bar(t[1])} z{bar(t[2])} │"
                    f" roll{bar(r[0])} pitch{bar(r[1])} yaw{bar(r[2])} │ btn {buttons:04b} "
                )
                sys.stdout.flush()
            elif time.time() >= next_tick:
                next_tick = time.time() + 3.0
                still_needed = [n for n, p in zip(axis_names, peak) if p < args.threshold]
                print(
                    f"  {reports:5d} reports   still needed: "
                    + (", ".join(still_needed) if still_needed else "none")
                )
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        h.close()

    print(f"\n\nstopped — macOS has the SpaceMouse back. {reports} motion/button report(s).")
    if shapes_seen:
        print("report shape(s) observed: " + ", ".join(sorted(shapes_seen)))
        print("peak deflection per axis (1.0 = full scale):")
        for name, v in zip(axis_names, peak):
            flag = "" if v > 0.05 else "   ← never moved"
            print(f"  {name:<6} {v:.2f}{flag}")
        print(f"buttons seen: {sorted(buttons_seen) if buttons_seen else 'none pressed'}")
    else:
        print("no motion reports were received — the device was never moved.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
