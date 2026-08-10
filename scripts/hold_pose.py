#!/usr/bin/env python3
"""Gravity compensation, the careful way: tell the arm to STAY EXACTLY WHERE IT IS.

    uv run scripts/hold_pose.py                       # dry run: explains the plan
    uv run scripts/hold_pose.py --yes                 # LIVE — the arm holds itself
    uv run scripts/hold_pose.py --yes --no-gripper-cal   # skip the jaw calibration

⛔ FIRST TIME ALL SIX ARM JOINTS HOLD REAL TORQUE.

WHAT GRAVITY COMPENSATION ACTUALLY IS
-------------------------------------
The arm weighs ~4.3 kg and nothing holds it up but its own motors. Gravity
compensation means the controller continuously computes how much torque each
joint needs purely to cancel the weight of everything beyond it, and applies
exactly that. The arm then behaves as if it were weightless: commands do what
they say instead of fighting gravity.

Without it, telling a joint "go to angle X" and watching it settle *below* X
looks exactly like a controller bug or bad IK. It is neither — it is 4 kg. This
is why gravity compensation comes BEFORE cartesian teleop in the roadmap rather
than after: it is what makes joint position commands mean what they say.

WHY "HOLD POSE" AND NOT THE DEFAULT ZERO-GRAVITY MODE
-----------------------------------------------------
I2RT's default is `zero_gravity_mode=True`, which cancels gravity and applies
only light damping — the arm becomes **back-drivable**, floating and pushable by
hand. That is the impressive demo, and it is the wrong first run: "floppy but
should not fall" is hard to distinguish from "about to fall".

`zero_gravity_mode=False` instead makes the constructor do
(`motor_chain_robot.py:251-253`)::

    self.command_joint_pos(self._joint_state.pos)

— it reads where the arm *is* and commands it to stay there, with the config's
PD gains on top of gravity compensation. **Success is defined as nothing moving
at all**, which is the easiest possible thing to verify and the safest possible
thing to get wrong. Hand-guiding comes after this passes.

⚠️ THE JAWS WILL MOVE AT STARTUP, and nothing else should.
`linear_4310.yml` has `gripper_limits: null, needs_calibration: true`, so
`get_yam_robot()` runs I2RT's `detect_gripper_limits` automatically: it pushes
the jaws at 0.5 Nm for up to 2 s in each direction to find both hard stops. That
is gentler than the 1.2 Nm the jaws already saw by accident today, it is
gripper-only motion, and it finally replaces our guessed jaw limits with
measured ones. `--no-gripper-cal` skips it if you would rather it did not move.

WHY THE STARTING POSE IS THE SAFE ONE
-------------------------------------
Measured: shoulder_pitch 0.0006 and elbow_pitch -0.0017, and the URDF puts both
of their mechanical limits at 0. **The arm is parked against its own stops**, so
it is mechanically supported rather than balanced. Energising from here cannot
release a joint into a fall.
"""

from __future__ import annotations

import argparse
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
    YAM_JOINTS,
    add_i2rt_to_path,
    chain_channel,
    patch_dm_driver_for_gs_usb,
)

MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7]
DRIFT_WARN = 0.05   # rad — worth reporting
DRIFT_ABORT = 0.20  # rad — gravity comp is not doing its job; stop


def main() -> int:
    ap = argparse.ArgumentParser(description="Hold the arm's current pose using gravity compensation.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm (default: dry run)")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--no-gripper-cal", action="store_true",
                    help="skip the automatic jaw-limit detection, so the gripper does not move")
    args = ap.parse_args()

    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()
    channel = chain_channel(args.arm)

    print("=== plan ===")
    print(f"  ARM        : {args.arm}  (serial {ARM_SERIALS[args.arm]})  channel {channel!r}")
    print("  mode       : zero_gravity_mode=False → hold the CURRENT pose, PD + gravity comp")
    print(f"  duration   : {args.seconds:.0f} s, then disable")
    print(f"  jaw cal    : {'SKIPPED' if args.no_gripper_cal else 'ON — the JAWS WILL MOVE at 0.5 Nm to find both stops'}")
    print(f"  abort if   : any joint drifts more than {DRIFT_ABORT} rad from where it started")
    print("  success is : NOTHING MOVES. The arm holds its own weight and stays put.\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.")
        return 0

    from i2rt.robots.get_robot import get_yam_robot  # noqa: PLC0415
    from i2rt.robots.utils import ArmType, GripperType  # noqa: PLC0415

    kwargs: dict = dict(
        channel=channel,
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=False,   # ⭐ hold the pose; do NOT go back-drivable
        sim=False,
    )
    if args.no_gripper_cal:
        # Supplying limits is what turns the auto-calibration off — get_robot.py:223-225
        # sets gripper_needs_cal = False whenever an override is given.
        kwargs["gripper_limits_override"] = [-6.4, -0.2]
        print("jaw calibration skipped; using conservative guessed limits [-6.4, -0.2]\n")

    robot = None
    start: np.ndarray | None = None
    worst = np.zeros(len(MOTOR_IDS))
    try:
        print("building robot — this enables all 7 motors and starts the control loop …")
        robot = get_yam_robot(**kwargs)
        start = np.asarray(robot.get_joint_pos(), dtype=float)
        print(f"\n✓ holding. start pose = {np.round(start, 4)}\n")

        t0 = time.perf_counter()
        next_report = 2.0
        while True:
            t = time.perf_counter() - t0
            if t >= args.seconds:
                break
            pos = np.asarray(robot.get_joint_pos(), dtype=float)
            drift = np.abs(pos - start)
            worst = np.maximum(worst, drift)

            if drift.max() > DRIFT_ABORT:
                j = int(np.argmax(drift))
                raise RuntimeError(
                    f"joint {MOTOR_IDS[j]} ({YAM_JOINTS[MOTOR_IDS[j]][0]}) drifted "
                    f"{drift[j]:.3f} rad — gravity compensation is not holding it"
                )

            if t >= next_report:
                next_report += 2.0
                print(f"  t={t:5.1f}s  max drift {drift.max():.4f} rad "
                      f"({drift.max() * 57.2958:.2f}°)  pose {np.round(pos, 3)}")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        print(f"\n⛔ {type(exc).__name__}: {exc}")
    finally:
        if robot is not None:
            try:
                robot.close()
            except Exception as exc:  # noqa: BLE001
                print(f"  ⚠️  robot.close() raised {type(exc).__name__}")
            # ⛔ close() prints "all torques set to zero" but only calls
            # motor_chain.close(), which does NOT disable the motors. Leaving them
            # enabled is exactly what broke consecutive runs of read_arm_state.py.
            try:
                chain = robot.motor_chain
                for mid in MOTOR_IDS:
                    try:
                        chain.motor_interface.motor_off(mid)
                    except Exception:  # noqa: BLE001, S110
                        pass
                print("all 7 motors explicitly disabled.")
            except Exception:  # noqa: BLE001, S110
                pass

    print("\n=== what actually happened ===")
    if start is None:
        print("  the robot never came up; nothing was held.")
        return 1
    for mid, d in zip(MOTOR_IDS, worst):
        flag = "  ← moved" if d > DRIFT_WARN else ""
        print(f"  {mid} {YAM_JOINTS[mid][0]:<15} max drift {d:.4f} rad ({d * 57.2958:5.2f}°){flag}")

    if worst.max() <= DRIFT_WARN:
        print(f"\n✅ The arm held itself. Max drift {worst.max() * 57.2958:.2f}° across all joints.")
        print("   Gravity compensation works. Next: zero-gravity mode for hand-guiding.")
    else:
        print(f"\n⚠️  Max drift {worst.max() * 57.2958:.2f}° — more than expected for a hold.")
        print("   Check gravity_comp_factor in yam_v1.yml before enabling hand-guiding.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
