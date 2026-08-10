#!/usr/bin/env python3
"""Find the jaws' real limits ONCE, gently, and save them. Then never again.

    uv run scripts/calibrate_gripper.py              # dry run
    uv run scripts/calibrate_gripper.py --yes        # calibrates, saves the result

⛔ THIS MOVES THE GRIPPER JAWS to both mechanical stops. Nothing else moves.

WHY THIS SCRIPT EXISTS
----------------------
`linear_4310.yml` ships with `gripper_limits: null` and
`needs_calibration: true`, so `get_yam_robot()` runs I2RT's `detect_gripper_limits`
**every time it is constructed**. That routine applies a constant torque and
waits for the position to stop changing — it drives the jaws into each hard stop
and holds them there until they are confirmed stationary. Julien, watching it:
*"they move really quickly and quite hard… they seem to crash into the ends and
then seem to try to push further."*

He is right, and softening the routine would be the wrong fix, because it would
still happen on every single startup. **Calibrate once, cache, never again.**
After this runs, every script passes `gripper_limits_override` and the arm comes
up silently — `get_robot.py:223-225` disables the auto-detection whenever an
override is supplied.

WHAT "GENTLER" MEANS HERE
------------------------
`test_torque` drops from I2RT's 0.5 Nm to **0.3 Nm** (~60%). It still has to
reach both ends of a 6.57 rad stroke, so it cannot be arbitrarily soft — but it
arrives noticeably less hard. There is no speed parameter to turn down: the
routine is torque-controlled, and torque IS the speed here. Lowering it is the
only lever, and it lowers both.

⚠️ It has to touch the stops. That is what "find the limits" means — the stop is
the thing being located. What it must not do is touch them repeatedly, forever,
which is what was actually happening.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_MOTOR_TYPES,
    add_i2rt_to_path,
    chain_channel,
    patch_dm_driver_for_gs_usb,
)
from yam_robot import GENTLE_TEST_TORQUE, load_gripper_limits, save_gripper_limits  # noqa: E402

MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7]
GRIPPER_INDEX = 6  # last entry in the motor list


def main() -> int:
    ap = argparse.ArgumentParser(description="Calibrate the gripper jaw limits once, gently.")
    ap.add_argument("--yes", action="store_true", help="actually move the jaws (default: dry run)")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--torque", type=float, default=GENTLE_TEST_TORQUE,
                    help=f"test torque, Nm (I2RT default is 0.5; ours is {GENTLE_TEST_TORQUE})")
    ap.add_argument("--max-duration", type=float, default=2.0, help="seconds per direction")
    args = ap.parse_args()

    existing = load_gripper_limits(args.arm)
    print("=== plan ===")
    print(f"  ARM        : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  torque     : {args.torque} Nm  (I2RT default 0.5 — {args.torque / 0.5:.0%} of it)")
    print(f"  duration   : up to {args.max_duration}s per direction, two directions")
    print("  moves      : the JAWS only, to both hard stops. No arm joint moves.")
    print(f"  existing   : {existing if existing else 'none — this arm has never been calibrated'}")
    print("  result     : saved to config/gripper_limits.json; every later start is silent\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing moved. Re-run with --yes.")
        return 0

    logging.getLogger().setLevel(logging.INFO)
    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()

    from i2rt.motor_drivers.dm_driver import (  # noqa: PLC0415
        ControlMode,
        DMChainCanInterface,
        DMSingleMotorCanInterface,
        MotorType,
    )
    from i2rt.robots.utils import detect_gripper_limits  # noqa: PLC0415

    channel = chain_channel(args.arm)

    # Start from a known state — see scripts/read_arm_state.py for why.
    pre = DMSingleMotorCanInterface(control_mode=ControlMode.MIT, channel=channel, name="preclean")
    for m in MOTOR_IDS:
        try:
            pre.motor_off(m)
        except Exception:  # noqa: BLE001, S110
            pass
    pre.close()
    time.sleep(0.1)

    motor_list = [(m, getattr(MotorType, YAM_MOTOR_TYPES[m])) for m in MOTOR_IDS]
    chain = DMChainCanInterface(
        motor_list,
        np.zeros(len(motor_list)),
        np.ones(len(motor_list)),
        channel,
        motor_chain_name=f"yam_{args.arm}_cal",
        start_thread=True,   # detect_gripper_limits needs the control loop running
    )

    limits = None
    try:
        before = chain.read_states()[GRIPPER_INDEX].pos
        print(f"jaws start at {before:+.4f} rad — calibrating …\n")
        limits = detect_gripper_limits(
            motor_chain=chain,
            gripper_index=GRIPPER_INDEX,
            test_torque=args.torque,
            max_duration=args.max_duration,
            position_threshold=0.01,
            check_interval=0.1,
        )
    finally:
        try:
            chain.running = False
            time.sleep(0.15)
        except Exception:  # noqa: BLE001, S110
            pass
        off = []
        for mid, _ in motor_list:
            try:
                chain.motor_interface.motor_off(mid)
                off.append(mid)
            except Exception:  # noqa: BLE001, S110
                pass
        try:
            chain.close()
        except Exception:  # noqa: BLE001, S110
            pass
        print(f"\nmotors confirmed disabled: {off}")

    if limits is None:
        print("\n⛔ calibration did not complete; nothing saved.")
        return 1

    lo, hi = float(min(limits)), float(max(limits))
    path = save_gripper_limits(args.arm, (limits[0], limits[1]))
    span = hi - lo
    print("\n=== result ===")
    print(f"  detected limits : {limits[0]:+.4f} … {limits[1]:+.4f} rad")
    print(f"  usable stroke   : {span:.3f} rad")
    print(f"  saved to        : {path.relative_to(REPO)}")
    print(f"\n  cross-check: linear_4310.yml declares motor_stroke 6.57 rad → "
          f"measured is {span / 6.57:.0%} of it")
    if span < 3.0:
        print("  ⚠️  markedly short of the declared stroke — was something blocking the jaws?")
    print("\n✅ Done. Every later robot start will use these and the jaws will NOT move.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
