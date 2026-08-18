#!/usr/bin/env python3
"""How fast can macOS actually talk to this arm? Measure it, don't guess.

    uv run apps/bench_can.py               # dry run
    uv run apps/bench_can.py --yes         # measure (register reads only)

⭐ WHY THIS NUMBER MATTERS

There is no Linux machine yet. Everything so far says the Mac is fine for
bring-up — but teleop needs a *sustained* loop: I2RT run their low-level
controller at 100 Hz over CAN, and each cycle costs one request/response per
motor. On Linux that goes through SocketCAN in the kernel. Here it goes through
libusb, in userspace, one USB bulk transfer at a time. That is the suspected
bottleneck and nobody has measured it.

This script measures the real round trip: put a frame on the wire, wait for the
motor's reply, time it. Then it derives what control rate a 7-motor loop could
sustain.

⛔ SAFETY — this cannot move anything.

It uses the register-read path (`0x7FF`, sub-command `0x33`), which asks the
motor firmware for a stored constant. No motor is enabled, no setpoint of any
kind is transmitted. It is the same traffic `identify_arm.py` sends, just in a
tight loop.

⚠️ HOW TO READ THE RESULT — this is a *conservative proxy*, not the real loop.
A register read and an MIT control frame are both one request plus one response,
so the transport cost is the same, but the firmware may service them at
different speeds. Treat the number as a lower bound on achievable rate: if
register reads already sustain 100 Hz × 7 motors, the real loop very probably
can too. If they do not, that is a genuine warning rather than a verdict.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam.can import ARM_SERIALS, DEFAULT_ARM, YAM_BITRATE, open_raw_can_interface  # noqa: E402

REG_GEAR_RATIO = 20  # from i2rt register_addr_map; any read-only register works
N_MOTORS = 7
FRAMES_PER_MOTOR_PER_CYCLE = 1  # one request + its response = one round trip
TARGET_HZ = 100.0  # I2RT's low-level controller rate (ENPIRE App. B.3)


def pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(p / 100.0 * (len(ordered) - 1)))))
    return ordered[idx]


def main() -> int:
    ap = argparse.ArgumentParser(description="Measure CAN round-trip latency from macOS.")
    ap.add_argument("--yes", action="store_true", help="actually run (default: dry run)")
    ap.add_argument("--samples", type=int, default=500)
    ap.add_argument("--motor", type=int, default=1, help="which motor to interrogate")
    ap.add_argument(
        "--cycle",
        action="store_true",
        help="measure whole 7-motor CYCLES instead of single round trips — this is what a "
        "control loop actually does, so it is measured rather than extrapolated",
    )
    ap.add_argument("--motors", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6, 7])
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS), help="WHICH ARM. Selected by serial, never by index.")
    ap.add_argument("--timeout", type=float, default=0.05, help="per-response timeout, seconds")
    args = ap.parse_args()

    print(f"samples   : {args.samples} round trips against motor {args.motor}")
    print("traffic   : 0x7FF sub-command 0x33 (READ). No enable, no setpoint.\n")
    if not args.yes:
        print("DRY RUN — nothing transmitted. Re-run with --yes.")
        return 0

    import can  # noqa: PLC0415

    iface = open_raw_can_interface(bitrate=args.bitrate, arm=args.arm)
    bus = iface.bus

    def make_request(motor_id: int) -> can.Message:
        return can.Message(
            arbitration_id=0x7FF,
            data=[motor_id, 0x00, 0x33, REG_GEAR_RATIO, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False,
        )

    requests = [make_request(m) for m in args.motors] if args.cycle else [make_request(args.motor)]

    latencies: list[float] = []
    timeouts = 0
    try:
        # Warm up: the first few transfers pay one-off USB and allocation costs
        # that would otherwise skew the tail statistics.
        for _ in range(20):
            bus.send(requests[0])
            bus.recv(timeout=args.timeout)

        t_start = time.perf_counter()
        for _ in range(args.samples):
            t0 = time.perf_counter()
            missed = False
            for request in requests:
                bus.send(request)
                if bus.recv(timeout=args.timeout) is None:
                    missed = True
            t1 = time.perf_counter()
            if missed:
                timeouts += 1
            else:
                latencies.append((t1 - t0) * 1000.0)
        wall = time.perf_counter() - t_start
    finally:
        iface.close()
        print("bus closed. No motor was enabled; nothing was commanded.\n")

    if not latencies:
        print("⛔ No replies at all. Is the arm powered? Nothing can be concluded about speed.")
        return 1

    unit = f"{len(requests)}-motor cycle" if args.cycle else "round trip"
    mean = statistics.fmean(latencies)
    print(f"complete  : {len(latencies)}/{args.samples}   with a missed reply: {timeouts}")
    print(f"wall      : {wall:.2f} s  →  {len(latencies) / wall:.0f} {unit}s/s sustained\n")
    print(f"{unit} latency (ms)")
    print(f"  mean {mean:7.3f}   min {min(latencies):7.3f}   max {max(latencies):7.3f}")
    print(
        f"  p50  {pct(latencies, 50):7.3f}   p95 {pct(latencies, 95):7.3f}   "
        f"p99 {pct(latencies, 99):7.3f}   p99.9 {pct(latencies, 99.9):7.3f}"
    )

    if args.cycle:
        # Measured directly — no extrapolation.
        per_cycle_mean = mean / 1000.0
        per_cycle_p95 = pct(latencies, 95) / 1000.0
        per_cycle_worst = max(latencies) / 1000.0
        print(f"\n{len(requests)}-motor control loop, MEASURED:")
    else:
        per_cycle_mean = N_MOTORS * FRAMES_PER_MOTOR_PER_CYCLE * mean / 1000.0
        per_cycle_p95 = N_MOTORS * FRAMES_PER_MOTOR_PER_CYCLE * pct(latencies, 95) / 1000.0
        per_cycle_worst = N_MOTORS * FRAMES_PER_MOTOR_PER_CYCLE * max(latencies) / 1000.0
        print(f"\n{N_MOTORS}-motor control loop, derived from single round trips:")

    print(f"  using mean  : {per_cycle_mean * 1000:7.2f} ms/cycle  →  {1 / per_cycle_mean:6.1f} Hz")
    print(f"  using p95   : {per_cycle_p95 * 1000:7.2f} ms/cycle  →  {1 / per_cycle_p95:6.1f} Hz   ← plan on this")
    print(f"  worst case  : {per_cycle_worst * 1000:7.2f} ms/cycle  →  {1 / per_cycle_worst:6.1f} Hz")

    budget_ms = 1000.0 / TARGET_HZ
    over = [x for x in latencies if (x if args.cycle else x * N_MOTORS) > budget_ms]
    print(
        f"\ncycles that would MISS a {TARGET_HZ:.0f} Hz deadline ({budget_ms:.1f} ms): "
        f"{len(over)}/{len(latencies)} ({100 * len(over) / len(latencies):.2f}%)"
    )

    achievable = 1 / per_cycle_p95
    print()
    if achievable >= TARGET_HZ:
        print(f"✅ Clears the {TARGET_HZ:.0f} Hz target at p95. macOS can plausibly run the real loop.")
    elif achievable >= 30:
        print(
            f"⚠️  {achievable:.0f} Hz at p95 — under the {TARGET_HZ:.0f} Hz target but at or above the 30 Hz\n"
            "   the policy layer runs at. Teleop and data collection are plausible; the 100 Hz\n"
            "   low-level loop is not. Linux still wanted for the real rig."
        )
    else:
        print(
            f"⛔ {achievable:.0f} Hz at p95 — too slow for teleop. macOS stays a bring-up and\n"
            "   simulation surface; the arm loop needs Linux/SocketCAN."
        )
    print("\nRemember this is a conservative proxy — see the note at the top of this file.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
