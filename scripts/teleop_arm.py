#!/usr/bin/env python3
"""⭐ THE GOAL: drive the real arm with the SpaceMouse. Cartesian, via IK.

    uv run scripts/teleop_arm.py                    # dry run: explains the plan
    uv run scripts/teleop_arm.py --yes              # LIVE
    uv run scripts/teleop_arm.py --yes --rotation   # ...also allow wrist rotation

⛔ THIS MOVES THE WHOLE ARM THROUGH SPACE. Desk clear, hand near the power.

TWO PHASES, ON PURPOSE — and the reason is a real hazard
--------------------------------------------------------
**Phase 1, hand-posing (zero gravity).** The arm currently parks with
`shoulder_pitch` and `elbow_pitch` at 0.000, which the URDF says are their
mechanical *lower limits*. IK starting from a configuration pressed against two
joint stops is starting in a corner: most directions it might want to move are
forbidden, and the solver will fight rather than glide. So phase 1 makes the arm
back-drivable and lets you physically pull it into an open, mid-range pose.

**Phase 2, teleop.** Commands the arm to hold wherever you left it — which is
what switches it out of zero gravity — and then hands control to the SpaceMouse.

⚠️ **Why they are one script and not two.** Zero gravity cancels the arm's
weight. The instant a process disables the motors, the weight is back. Ending
hand-guiding with the arm raised would simply drop it. Keeping both phases in one
process means the arm is never un-commanded between being posed and being driven.
The same reason drives the exit: it holds the pose and counts down, so you can
park or support the arm before anything is released.

SAFETY ENVELOPE — added on top of everything the simulation already proved
-------------------------------------------------------------------------
· **Workspace box.** The IK target is confined to a box around wherever phase 2
  starts. Even a runaway twist cannot walk the arm across the room.
· **Per-cycle joint clamp.** However far IK wants to jump, no joint is allowed to
  move more than a fixed step per cycle. This is the backstop against a
  singularity, where IK legitimately returns enormous joint velocities.
· **Deadman.** Deflection is a velocity, so releasing the puck freezes the
  target. Inherent to the design, not a check that can be forgotten.
· **Translation only by default.** `--rotation` opts in. A wrong rotation sign
  swings the wrist; a wrong translation sign moves it gently the wrong way.
· **Abort** on joint-limit proximity or an IK solve that diverges.

The loop itself is `src/yam/teleop.py:CartesianTeleop` — the exact code validated in
simulation, unchanged. Only the robot behind it is different.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
from yam.inputs.spacemouse import TwistReader, countdown_hands_off, find_device, open_device  # noqa: E402
from yam.teleop import CartesianTeleop  # noqa: E402
from yam.can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam.robot import build_robot, load_gripper_limits, shutdown_robot  # noqa: E402

CONTROL_HZ = 100.0
N_ARM = 6

# Gentler than the simulation defaults. These numbers move a real 4.3 kg arm.
LINEAR_SCALE = 0.04    # m/s at full puck deflection
ANGULAR_SCALE = 0.25   # rad/s at full puck deflection

WORKSPACE_BOX = 0.20       # m, each way from the phase-2 start position
MAX_JOINT_STEP = 0.010     # rad per cycle ≈ 1.0 rad/s at 100 Hz
JOINT_LIMIT_MARGIN = 0.08  # rad — abort if a joint gets this close to a stop


def main() -> int:
    ap = argparse.ArgumentParser(description="SpaceMouse → IK → the real YAM arm.")
    ap.add_argument("--yes", action="store_true", help="actually move the arm (default: dry run)")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--pose-seconds", type=float, default=20.0,
                    help="phase 1: seconds of zero-gravity hand-posing before teleop starts")
    ap.add_argument("--seconds", type=float, default=90.0, help="phase 2: teleop duration")
    ap.add_argument("--rotation", action="store_true", help="also map puck rotation to wrist rotation")
    ap.add_argument("--linear-scale", type=float, default=LINEAR_SCALE)
    ap.add_argument("--box", type=float, default=WORKSPACE_BOX)
    ap.add_argument("--skip-posing", action="store_true", help="go straight to teleop from the current pose")
    args = ap.parse_args()

    saved = load_gripper_limits(args.arm)
    print("=== plan ===")
    print(f"  ARM        : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  jaw limits : {saved if saved else 'NONE SAVED — run scripts/calibrate_gripper.py --yes first'}")
    if not args.skip_posing:
        print(f"  phase 1    : {args.pose_seconds:.0f}s ZERO GRAVITY — pull the arm by hand into an open pose")
        print("               (it currently parks against two joint stops, a bad place for IK to start)")
    print(f"  phase 2    : {args.seconds:.0f}s teleop at {CONTROL_HZ:.0f} Hz")
    print(f"  mapping    : puck translation → EE translation, {args.linear_scale} m/s at full deflection")
    print(f"               puck rotation   → {'wrist rotation, ' + str(ANGULAR_SCALE) + ' rad/s' if args.rotation else 'IGNORED (use --rotation to enable)'}")
    print(f"  workspace  : ±{args.box} m box around wherever phase 2 begins")
    print(f"  clamp      : max {MAX_JOINT_STEP} rad per joint per cycle ({MAX_JOINT_STEP * CONTROL_HZ:.1f} rad/s)")
    print("  deadman    : release the puck and the arm stops")
    print("  on exit    : holds the pose and counts down — PARK OR SUPPORT THE ARM before it releases\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing moved. Re-run with --yes.")
        return 0

    info = find_device()
    if info is None:
        print("No SpaceMouse found.")
        return 1
    countdown_hands_off(3)
    handle = open_device(info)
    handle.set_nonblocking(True)
    reader = TwistReader(handle)

    robot = None
    teleop = None
    aborted: str | None = None
    try:
        # Phase 1 starts back-drivable so the arm can be posed by hand. If posing
        # is skipped we still build in zero gravity and immediately command a
        # hold — that keeps a single code path rather than two constructors.
        print("building robot — enables all 7 motors, starts the control loop …")
        robot, note = build_robot(args.arm, zero_gravity=not args.skip_posing)
        print(f"  {note}\n")

        if not args.skip_posing:
            print("⭐ PHASE 1 — the arm is weightless. Pull it into an open, mid-range pose now.")
            print("   Avoid leaving it fully folded or fully stretched; IK works best away from the stops.")
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < args.pose_seconds:
                left = args.pose_seconds - (time.perf_counter() - t0)
                pos = np.asarray(robot.get_joint_pos(), dtype=float)
                print(f"\r   {left:4.1f}s left   pose {np.round(pos[:N_ARM], 2)}   ", end="", flush=True)
                time.sleep(0.2)
            print("\n")

        # Commanding a position is what leaves zero-gravity mode: from here the
        # arm holds itself with PD gains instead of floating.
        q_now = np.asarray(robot.get_joint_pos(), dtype=float)
        robot.command_joint_pos(q_now)
        time.sleep(0.3)
        print(f"⭐ PHASE 2 — holding {np.round(q_now[:N_ARM], 3)} and handing control to the SpaceMouse.\n")

        teleop = CartesianTeleop()
        teleop.reset(q_now[:N_ARM])
        home_ee = teleop.ee_position().copy()
        gripper_value = float(q_now[N_ARM]) if len(q_now) > N_ARM else 0.0

        limits_lo = np.array([YAM_JOINTS[i][1] for i in range(1, N_ARM + 1)])
        limits_hi = np.array([YAM_JOINTS[i][2] for i in range(1, N_ARM + 1)])

        dt = 1.0 / CONTROL_HZ
        prev_q = q_now[:N_ARM].copy()
        t0 = time.perf_counter()
        next_report = 2.0
        clamped_cycles = 0

        while True:
            t = time.perf_counter() - t0
            if t >= args.seconds:
                break
            loop_start = time.perf_counter()

            axes = reader.read()
            twist = np.array([
                axes[0] * args.linear_scale, axes[1] * args.linear_scale, axes[2] * args.linear_scale,
                axes[3] * ANGULAR_SCALE if args.rotation else 0.0,
                axes[4] * ANGULAR_SCALE if args.rotation else 0.0,
                axes[5] * ANGULAR_SCALE if args.rotation else 0.0,
            ])

            q_target = teleop.step(twist, dt)

            # Workspace box — pull the IK target back if it has wandered outside.
            ee = teleop.ee_position()
            excursion = np.abs(ee - home_ee)
            if np.any(excursion > args.box):
                clipped = np.clip(ee, home_ee - args.box, home_ee + args.box)
                import mink  # noqa: PLC0415

                teleop.target = mink.SE3.from_rotation_and_translation(
                    rotation=teleop.target.rotation(), translation=clipped
                )

            # Per-cycle clamp — the backstop against a singularity, where IK
            # legitimately asks for enormous joint velocities.
            step = q_target - prev_q
            if np.any(np.abs(step) > MAX_JOINT_STEP):
                clamped_cycles += 1
                q_target = prev_q + np.clip(step, -MAX_JOINT_STEP, MAX_JOINT_STEP)

            if np.any(q_target < limits_lo + JOINT_LIMIT_MARGIN) or np.any(
                q_target > limits_hi - JOINT_LIMIT_MARGIN
            ):
                j = int(np.argmax(np.maximum(limits_lo + JOINT_LIMIT_MARGIN - q_target,
                                             q_target - (limits_hi - JOINT_LIMIT_MARGIN))))
                aborted = (
                    f"joint {j + 1} ({YAM_JOINTS[j + 1][0]}) reached its limit margin at "
                    f"{q_target[j]:+.3f} rad — stopping before the stop"
                )
                break

            full = np.zeros(robot.num_dofs())
            full[:N_ARM] = q_target
            if robot.num_dofs() > N_ARM:
                full[N_ARM] = gripper_value
            robot.command_joint_pos(full)
            prev_q = q_target.copy()

            if t >= next_report:
                next_report += 2.0
                print(f"  t={t:5.1f}s  EE {np.round(ee, 3)}  moved {np.linalg.norm(ee - home_ee):.3f} m"
                      f"  q {np.round(q_target, 2)}")

            time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        aborted = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            handle.close()
        except Exception:  # noqa: BLE001, S110
            pass
        if robot is not None:
            try:
                # Hold where we are, and give a human time to park or support the
                # arm. Disabling a raised arm drops it — that is the whole reason
                # this countdown exists.
                q = np.asarray(robot.get_joint_pos(), dtype=float)
                robot.command_joint_pos(q)
                for remaining in range(5, 0, -1):
                    print(f"\r⚠️  HOLDING — park or support the arm. Releasing in {remaining} … ",
                          end="", flush=True)
                    time.sleep(1.0)
                print()
            except Exception:  # noqa: BLE001, S110
                pass
            disabled = shutdown_robot(robot)
            print(f"motors confirmed disabled: {disabled}")

    if aborted:
        print(f"\n⛔ {aborted}")
    if teleop is not None:
        print(f"\nIK cycles that hit the per-cycle clamp: {clamped_cycles}")
        if clamped_cycles > 50:
            print("  ⚠️  frequent clamping — the arm is probably near a singularity, or the scale is too high.")
    print("\n✅ Teleop session finished." if not aborted else "\nSession ended early.")
    return 1 if aborted else 0


if __name__ == "__main__":
    sys.exit(main())
