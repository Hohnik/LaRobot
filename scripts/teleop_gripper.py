#!/usr/bin/env python3
"""Drive a REAL arm's gripper with the SpaceMouse. Two motors only, no IK.

    uv run scripts/teleop_gripper.py                      # dry run: prints the mapping
    uv run scripts/teleop_gripper.py --yes                # LIVE, B
    uv run scripts/teleop_gripper.py --yes --arm G

⛔ THIS MOVES A REAL ROBOT, but only its gripper.

    SpaceMouse YAW  (twist the puck)      →  motor 6, gripper_twist
    SpaceMouse Z    (push down / lift)    →  motor 7, gripper_jaws

⭐ WHY THIS AND NOT THE CARTESIAN IK LOOP

Julien, 2026-08-10: *"it's not really safe right now. The only thing that should
be moved is the gripper opening and closing and the gripper twisting… as soon as
the SpaceMouse is connected I can move everything from the desk and we can
control the whole thing."*

So the workspace, not the software, is the binding constraint. `teleop_sim.py`
already runs the full cartesian chain and it works — but cartesian teleop moves
the **whole arm** through space, which is exactly what the desk does not allow
yet. This script gets the real thing the workspace *does* allow: a SpaceMouse in
his hand, moving a physical arm, today.

It is not a throwaway. It proves the half of the teleop stack that IK cannot:
reading the device and driving real motors in one 100 Hz loop, with a deadman,
limits and a clean shutdown. When the desk is clear, IK drops in above it.

⚠️ Neither motor can move the arm through space. `gripper_twist` changes
orientation only; `gripper_jaws` moves the fingers. Nothing here can extend the
arm's reach or approach the other arm.

SAFETY
------
· **Velocity, not position.** Deflection is a *speed*; the target integrates it.
  Release the puck and the target freezes where it is — that is the deadman, and
  it is a property of the design rather than a check that could be forgotten.
· **motor 6** is clamped to the URDF joint limits (±2.0944 rad) with headroom.
· **motor 7** has NO trustworthy limits — `linear_4310.yml` says
  `gripper_limits: null`, `needs_calibration: true` — so it is clamped to a
  conservative window around wherever it started, never to an absolute target.
  ⚠️ It can still close on itself; `--max-torque` is what stops that.
· Per-cycle abort on error code, torque and tracking divergence.
· On exit by any path: both motors ramped back to their start positions,
  commanded to zero gain, and disabled. The 400 ms firmware timeout is the
  backstop if this process dies outright.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
from spacemouse import (  # noqa: E402
    TwistReader,
    countdown_hands_off,
    describe,
    find_device,
    open_device,
)
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_BITRATE,
    YAM_JOINTS,
    YAM_MOTOR_TYPES,
    add_i2rt_to_path,
    open_motor_interface,
)

TWIST_MOTOR = 6
JAW_MOTOR = 7
CONTROL_HZ = 100.0

# Speed at full puck deflection. Gentle on purpose — this is a first contact
# between a human hand and a real motor, not a speed run.
TWIST_SPEED = 0.9   # rad/s
JAW_SPEED = 0.6     # rad/s

DEFAULT_KP = 8.0
DEFAULT_KD = 1.0
# ⭐ Per-motor, because one global limit is the wrong shape. Measured 2026-08-10
# over three real teleop runs: gripper_twist never exceeded 0.168 Nm however hard
# the puck was twisted, while gripper_jaws sat at 1.20-1.23 Nm the moment it was
# pushed toward the stop it already rests against.
#
# A single number has to be loose enough for the jaws, which then leaves the
# twist joint with a limit 7x above anything it can legitimately need. Splitting
# them is TIGHTER overall, not looser: the twist joint now trips at ~3.5x its
# observed peak, and the jaws get the headroom their mechanism actually demands.
MAX_TORQUE_BY_MOTOR = {
    6: 0.6,   # gripper_twist — observed peak 0.168 Nm across 4222 samples
    7: 2.5,   # gripper_jaws  — needs ~1.2 Nm against its stop; DM4310 peak is 10 Nm
}
DEFAULT_MAX_TORQUE = 1.2  # fallback for any motor not in the table
MAX_TRACKING_ERROR = 0.35
LIMIT_MARGIN = 0.05
# ⭐ The jaws' real travel, derived 2026-08-10 from two independent facts:
#   · linear_4310.yml: motor_stroke 6.57 rad  <->  gripper_stroke 0.096 m,
#     which matches the URDF's two prismatic tips at 0.0469 m each.
#   · measured: this morning the jaws sat at ~0.000 rad and could NOT be driven
#     positive (instant torque abort) while moving 0.6 rad negative freely.
#     So motor position ~0 IS one of the two hard stops, and travel runs negative.
# Julien then repositioned them to -2.41 rad, roughly a third along, with room
# both ways — which is what made the aborts stop.
JAW_STROKE = 6.57
JAW_LIMITS = (-JAW_STROKE, 0.0)   # (far stop, near stop) in motor radians
JAW_WINDOW = 3.0  # rad either side of start; a per-run cap ON TOP of the limits

AXIS_Z = 2    # SpaceMouse translation z — push down / lift up
AXIS_YAW = 5  # SpaceMouse rotation about the vertical — twisting the puck


def main() -> int:
    ap = argparse.ArgumentParser(description="SpaceMouse → gripper twist + jaws on a real arm.")
    ap.add_argument("--yes", action="store_true", help="actually move the robot (default: dry run)")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--twist-speed", type=float, default=TWIST_SPEED)
    ap.add_argument("--jaw-speed", type=float, default=JAW_SPEED)
    ap.add_argument("--jaw-window", type=float, default=JAW_WINDOW)
    ap.add_argument("--no-jaws", action="store_true", help="twist only; leave motor 7 alone entirely")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--max-torque", type=float, default=None,
                    help="override the per-motor torque limits with one value for all")
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    args = ap.parse_args()

    twist_name, twist_lo, twist_hi = YAM_JOINTS[TWIST_MOTOR]
    motors = [TWIST_MOTOR] if args.no_jaws else [TWIST_MOTOR, JAW_MOTOR]
    torque_limit = {
        m: (args.max_torque if args.max_torque is not None else MAX_TORQUE_BY_MOTOR.get(m, DEFAULT_MAX_TORQUE))
        for m in motors
    }

    print("=== mapping ===")
    print(f"  ARM              : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  puck YAW  (twist)→ motor {TWIST_MOTOR} {twist_name:<14} "
          f"{args.twist_speed} rad/s at full deflection, limits [{twist_lo:+.3f}, {twist_hi:+.3f}]")
    if args.no_jaws:
        print(f"  motor {JAW_MOTOR} gripper_jaws  : DISABLED by --no-jaws")
    else:
        print(f"  puck Z (push/lift)→ motor {JAW_MOTOR} gripper_jaws   "
              f"{args.jaw_speed} rad/s at full deflection, "
              f"stroke [{JAW_LIMITS[0]:+.2f}, {JAW_LIMITS[1]:+.2f}] capped ±{args.jaw_window} rad/run")
    print(f"  control          : {CONTROL_HZ:.0f} Hz for {args.seconds:.0f} s, kp={args.kp} kd={args.kd}")
    print(f"  torque limits    : " + "  ".join(f"m{m} {torque_limit[m]} Nm" for m in motors))
    print(f"  abort if         : error != normal, torque over the above, "
          f"|tracking| > {MAX_TRACKING_ERROR} rad")
    print("  deadman          : release the puck and the target freezes\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing moved. Re-run with --yes.")
        return 0

    info = find_device()
    if info is None:
        print("No SpaceMouse found. Plugged in?")
        return 1
    print(f"SpaceMouse: {describe(info)}")
    countdown_hands_off(3)
    handle = open_device(info)
    handle.set_nonblocking(True)
    reader = TwistReader(handle)

    add_i2rt_to_path()
    from i2rt.motor_drivers.dm_driver import MotorType  # noqa: PLC0415

    types = {m: getattr(MotorType, YAM_MOTOR_TYPES[m]) for m in motors}
    iface = open_motor_interface(bitrate=args.bitrate, arm=args.arm)

    period = 1.0 / CONTROL_HZ
    start: dict[int, float] = {}
    target: dict[int, float] = {}
    peak_torque: dict[int, float] = {m: 0.0 for m in motors}
    travelled: dict[int, list[float]] = {m: [] for m in motors}
    abort_reason: str | None = None

    def shutdown() -> None:
        """Ramp every enabled motor home IN PARALLEL, then disable.

        Interleaved deliberately: handling motors one after another starves the
        others past their 400 ms watchdog. That exact bug tripped `loss
        communication` on the second arm in move_both_grippers.py on 2026-08-10.
        """
        if not start:
            iface.close()
            return
        here = {m: (travelled[m][-1] if travelled[m] else start[m]) for m in start}
        steps = int(1.2 * CONTROL_HZ)
        for i in range(steps + 1):
            frac = i / steps
            for m in start:
                try:
                    tgt = here[m] + (start[m] - here[m]) * frac
                    iface.set_control(m, types[m], tgt, 0.0, args.kp, args.kd, 0.0)
                except Exception:  # noqa: BLE001, S110
                    pass
            time.sleep(period)
        for m in start:
            try:
                iface.set_control(m, types[m], start[m], 0.0, 0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001, S110
                pass
            try:
                iface.motor_off(m)
            except Exception:  # noqa: BLE001, S110
                pass
        iface.close()
        print("\nmotors returned to start and disabled; bus closed.")

    try:
        for m in motors:
            fb = iface.motor_on(m, types[m])
            start[m] = fb.position
            target[m] = fb.position
            print(f"motor {m}: enabled, start = {fb.position:+.4f} rad, {fb.error_message}")

        # Jaw bounds are the INTERSECTION of the mechanism's real stroke and a
        # per-run window around the start. The stroke stops it reaching a hard
        # stop; the window stops a mis-derived stroke authorising a large move.
        jaw_start = start.get(JAW_MOTOR, 0.0)
        bounds = {
            TWIST_MOTOR: (twist_lo + LIMIT_MARGIN, twist_hi - LIMIT_MARGIN),
            JAW_MOTOR: (
                max(JAW_LIMITS[0] + LIMIT_MARGIN, jaw_start - args.jaw_window),
                min(JAW_LIMITS[1] - LIMIT_MARGIN, jaw_start + args.jaw_window),
            ),
        }
        if JAW_MOTOR in start:
            lo, hi = bounds[JAW_MOTOR]
            print(f"  jaw travel available: {lo:+.3f} … {hi:+.3f} rad "
                  f"(start {jaw_start:+.3f}, stroke {JAW_LIMITS[0]:+.2f}…{JAW_LIMITS[1]:+.2f})")
        print("\n⭐ Move the SpaceMouse. TWIST it to rotate the gripper"
              + ("" if args.no_jaws else ", PUSH DOWN / LIFT to work the jaws") + ".")
        print("   Release it and everything freezes. Ctrl-C to stop.\n")

        t0 = time.perf_counter()
        next_report = 1.5
        while True:
            t = time.perf_counter() - t0
            if t >= args.seconds:
                break
            loop_start = time.perf_counter()

            axes = reader.read()
            commands = {TWIST_MOTOR: axes[AXIS_YAW] * args.twist_speed}
            if not args.no_jaws:
                commands[JAW_MOTOR] = axes[AXIS_Z] * args.jaw_speed

            for m in motors:
                lo, hi = bounds[m]
                target[m] = max(lo, min(hi, target[m] + commands[m] * period))
                fb = iface.set_control(m, types[m], target[m], 0.0, args.kp, args.kd, 0.0)
                travelled[m].append(fb.position)
                peak_torque[m] = max(peak_torque[m], abs(fb.torque))

                if int(str(fb.error_code), 16) != 0x1:
                    abort_reason = f"motor {m} error: {fb.error_message}"
                    raise RuntimeError(abort_reason)
                if abs(fb.torque) > torque_limit[m]:
                    lag = fb.position - target[m]
                    hint = (
                        "the joint is being pushed against something it cannot move — most likely a "
                        "mechanical stop, since it barely moved while torque rose"
                        if abs(lag) > 0.05 and abs(fb.velocity) < 0.05
                        else "the joint was moving, so this is mechanism friction rather than an obstruction"
                    )
                    abort_reason = (
                        f"motor {m} ({YAM_JOINTS[m][0]}) torque {fb.torque:+.3f} Nm exceeded "
                        f"{torque_limit[m]} Nm at position {fb.position:+.4f} "
                        f"(target {target[m]:+.4f}, lag {lag:+.4f} rad, vel {fb.velocity:+.3f} rad/s)\n"
                        f"     → {hint}"
                    )
                    raise RuntimeError(abort_reason)
                if abs(fb.position - target[m]) > MAX_TRACKING_ERROR:
                    abort_reason = (
                        f"motor {m} tracking error {fb.position - target[m]:+.3f} rad "
                        f"exceeded {MAX_TRACKING_ERROR}"
                    )
                    raise RuntimeError(abort_reason)

            if t >= next_report:
                next_report += 1.5
                where = "  ".join(f"m{m} {travelled[m][-1]:+.3f}" for m in motors)
                drive = "  ".join(f"{k}={v:+.2f}" for k, v in commands.items())
                print(f"  t={t:5.1f}s  {where}   cmd {drive}")

            time.sleep(max(0.0, period - (time.perf_counter() - loop_start)))

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        abort_reason = abort_reason or f"{type(exc).__name__}: {exc}"
    finally:
        shutdown()
        handle.close()

    if abort_reason:
        print(f"\n⛔ ABORTED: {abort_reason}")

    print("\n=== what actually happened ===")
    moved_any = False
    for m in motors:
        pos = travelled[m]
        if len(pos) < 2:
            print(f"  motor {m}: no samples")
            continue
        span = max(pos) - min(pos)
        moved_any = moved_any or span > 0.05
        print(
            f"  motor {m} ({YAM_JOINTS[m][0]:<13}): {len(pos)} samples  "
            f"range {span:.3f} rad ({span * 57.2958:.1f}°)  peak |torque| {peak_torque[m]:.3f} Nm"
        )
    if moved_any:
        print("\n✅ A real robot was driven by the SpaceMouse.")
    else:
        print("\n⚠️  Nothing moved. Was the puck twisted? Deadzone is 0.06 of full deflection.")
    return 1 if abort_reason else 0


if __name__ == "__main__":
    sys.exit(main())
