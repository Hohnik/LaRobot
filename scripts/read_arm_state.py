#!/usr/bin/env python3
"""Read all seven joints through the whole-arm CHAIN. No gravity comp, no motion.

    uv run scripts/read_arm_state.py                # dry run
    uv run scripts/read_arm_state.py --yes          # enables all 7 motors, reads, disables
    uv run scripts/read_arm_state.py --yes --seconds 10   # stream state

⛔ THIS ENABLES ALL SEVEN MOTORS. It sends no setpoint and starts no control
loop, so the arm should not move — but this is the first time motors 1-5 are
energised together, so treat it as a physical step.

⭐ WHAT IT IS ACTUALLY TESTING

Not the motors — those are known good. It tests that `DMChainCanInterface`, the
whole-arm layer, now works over gs_usb (`src/yam_can.py:patch_dm_driver_for_gs_usb`).
That layer is the only route to all seven motors at once, to gravity
compensation, and to the gripper force limiter, and until today it was
SocketCAN-only and therefore unreachable from macOS.

⚠️ WHY THIS AND NOT `get_yam_robot()` — the distinction matters

`get_yam_robot()` would ALSO work now, but it does considerably more:
`DMChainCanInterface.__init__` calls `_motor_on()`, then `get_yam_robot` calls
`start_thread()`, and `MotorChainRobot` is built with `use_gravity_comp=True` and
`zero_gravity_mode=True` by default. **That starts a 250 Hz control loop actively
holding the arm against gravity** — which is the gravity-compensation milestone,
a genuinely different physical event, and it deserves its own gated step with a
clear workspace rather than arriving as a side effect of a state read.

So this script constructs the chain with `start_thread=False` and only reads.
No control loop runs, so the motors receive no commands and their own 400 ms
timeout damps them. Nothing holds torque.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_BITRATE,
    YAM_JOINTS,
    YAM_MOTOR_TYPES,
    add_i2rt_to_path,
    chain_channel,
    patch_dm_driver_for_gs_usb,
)

MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7]


def main() -> int:
    ap = argparse.ArgumentParser(description="Read the whole arm through the chain interface.")
    ap.add_argument("--yes", action="store_true", help="actually enable and read (default: dry run)")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--seconds", type=float, default=0.0, help="stream for N seconds instead of one read")
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    args = ap.parse_args()

    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()
    channel = chain_channel(args.arm)

    print("=== plan ===")
    print(f"  ARM      : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  channel  : {channel!r}  → gs_usb, resolved by serial not by index")
    print(f"  motors   : {MOTOR_IDS}  ({', '.join(YAM_MOTOR_TYPES[m] for m in MOTOR_IDS)})")
    print("  action   : enable all 7, read state, disable. NO control loop, NO gravity comp.")
    print("  expect   : the arm does not move\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted. Re-run with --yes.")
        return 0

    from i2rt.motor_drivers.dm_driver import DMChainCanInterface, MotorType  # noqa: PLC0415

    motor_list = [(m, getattr(MotorType, YAM_MOTOR_TYPES[m])) for m in MOTOR_IDS]
    chain = DMChainCanInterface(
        motor_list,
        np.zeros(len(motor_list)),          # motor_offset — get_robot.py uses zeros too
        np.ones(len(motor_list)),           # directions   — yam_v1.yml is all +1
        channel,
        motor_chain_name=f"yam_{args.arm}",
        start_thread=False,                 # ⛔ no control loop, no gravity comp
    )
    print("✓ chain built over gs_usb — the whole-arm layer works on macOS\n")

    try:
        deadline = time.time() + args.seconds
        while True:
            states = chain.read_states()
            print(f"{'joint':<16}{'pos (rad)':>12}{'vel':>10}{'torque':>10}")
            for (mid, _), st in zip(motor_list, states):
                name = YAM_JOINTS[mid][0]
                print(f"  {mid} {name:<13}{st.pos:>12.4f}{st.vel:>10.4f}{st.eff:>10.4f}")
            if args.seconds <= 0 or time.time() >= deadline:
                break
            print()
            time.sleep(0.5)
    finally:
        try:
            chain.close()
        except Exception:  # noqa: BLE001
            try:
                chain.motor_interface.close()
            except Exception:  # noqa: BLE001, S110
                pass
        print("\nchain closed. No control loop ever ran; the 400 ms timeout damps the motors.")

    print("\n✅ All seven joints read through the chain interface.")
    print("   Next gated step: gravity compensation — see docs/ROADMAP.md step 3.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
