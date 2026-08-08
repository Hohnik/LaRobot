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

import struct
import sys
import time

import hid

SPACEMOUSE_VID = 0x256F  # 3Dconnexion's own
LEGACY_VID = 0x046D      # Logitech — 3Dconnexion shipped under it for years
VIDS = {SPACEMOUSE_VID, LEGACY_VID}
FULL_SCALE = 350.0  # raw counts at full deflection; nominal, calibrate if it matters
DEADZONE = 0.02


def find_device() -> dict | None:
    """Pick the SpaceMouse's motion interface.

    The order matters, and the reason is specific to this dock. VID 0x046D is Logitech,
    which older 3Dconnexion units shipped under — but it is *also* the C920 webcam on
    this same hub, and a webcam presents HID interfaces too. So a Logitech device is
    accepted only if it independently identifies as multi-axis; blind fallback is
    allowed for 3Dconnexion's own VID and nothing else. Picking the webcam would open
    cleanly, print a plausible product string, and then never report motion.
    """
    cands = [d for d in hid.enumerate() if d["vendor_id"] in VIDS]
    multi = [d for d in cands if d.get("usage_page") == 0x01 and d.get("usage") == 0x08]
    if multi:
        return multi[0]
    own = [d for d in cands if d["vendor_id"] == SPACEMOUSE_VID]
    return own[0] if own else None


def open_device(info: dict):
    if hasattr(hid, "device"):
        h = hid.device()
        h.open_path(info["path"])
        return h
    return hid.Device(path=info["path"])


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
    info = find_device()
    if info is None:
        print("No SpaceMouse found. Plugged in?")
        return 1

    print(
        f"Opening {info.get('product_string')} "
        f"[{info['vendor_id']:#06x}:{info['product_id']:#06x}] …"
    )
    if not (info.get("usage_page") == 0x01 and info.get("usage") == 0x08):
        print("  ⚠️  this is NOT the multi-axis interface — if nothing moves, that is why.")
    try:
        h = open_device(info)
    except OSError as exc:
        print(f"Could not open ({exc}). macOS: Privacy & Security → Input Monitoring.")
        return 2

    h.set_nonblocking(True)
    print("Move the SpaceMouse. Ctrl-C to stop.\n")

    t = [0.0, 0.0, 0.0]      # x, y, z
    r = [0.0, 0.0, 0.0]      # roll, pitch, yaw
    buttons = 0
    shapes_seen: set[str] = set()

    try:
        while True:
            data = h.read(64)
            if data:
                rid, payload = data[0], bytes(data[1:])
                if rid == 0x01 and len(payload) >= 12:
                    vals = struct.unpack("<6h", payload[:12])
                    t = [scale(v) for v in vals[:3]]
                    r = [scale(v) for v in vals[3:]]
                    shapes_seen.add("combined 0x01 (translation+rotation)")
                elif rid == 0x01 and len(payload) >= 6:
                    t = [scale(v) for v in struct.unpack("<3h", payload[:6])]
                    shapes_seen.add("split 0x01/0x02")
                elif rid == 0x02 and len(payload) >= 6:
                    r = [scale(v) for v in struct.unpack("<3h", payload[:6])]
                    shapes_seen.add("split 0x01/0x02")
                elif rid == 0x03 and payload:
                    buttons = int.from_bytes(payload[:2], "little")

            sys.stdout.write(
                f"\r x{bar(t[0])} y{bar(t[1])} z{bar(t[2])} │"
                f" roll{bar(r[0])} pitch{bar(r[1])} yaw{bar(r[2])} │ btn {buttons:04b} "
            )
            sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n\nstopped.")
        if shapes_seen:
            print("report shape(s) observed: " + ", ".join(sorted(shapes_seen)))
        else:
            print("no motion reports were received — the device was never moved.")
    finally:
        h.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
