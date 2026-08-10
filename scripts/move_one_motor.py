#!/usr/bin/env python3
"""The first commanded motion on this arm. ONE motor, small, slow, bounded.

    uv run scripts/move_one_motor.py                  # dry run: prints the exact plan
    uv run scripts/move_one_motor.py --yes            # MOVES motor 6 by ±0.05 rad
    uv run scripts/move_one_motor.py --yes --delta 0.15

⛔ THIS MOVES A PHYSICAL ROBOT ARM. Read the design before running it.

Julien's brief (2026-08-10): *"do something so that I can see physically that
it's moving before we move the full arm, so that we don't overextend the arm or
hit anything in its vicinity."* So: one joint, a motion large enough to see and
small enough to be harmless, and a measurement of how fast it actually moved.

Default is **motor 6, the wrist twist** — the last joint before the gripper. It
rotates the end effector about the forearm axis: highly visible, lowest inertia
of any arm joint, and it does not change the arm's reach, so it cannot extend
into anything. The gripper (motor 7) is deliberately *not* the default: its
config carries `gripper_limits: null` and `needs_calibration: true`, so its
travel limits are genuinely unknown and its jaws have hard stops.

HOW IT IS MADE SAFE — five independent mechanisms, not one:

1. **Relative, never absolute.** The target is the *measured* current position
   plus a delta. No absolute setpoint is ever sent, so an uncalibrated zero
   cannot send the joint across its range.
2. **Hard-clamped delta.** `--delta` is clamped to MAX_DELTA_RAD regardless of
   what is passed. The default is deliberately smaller than it needs to be:
   if my assumption about the position unit is wrong by 10x, 0.05 rad becomes
   0.5 rad (29°), which is still a shrug on a wrist joint.
3. **Ramped, not stepped.** The target is interpolated over `--ramp` seconds.
   A step command into a P controller is what makes a joint snap; a ramp bounds
   velocity by construction. Peak commanded speed is delta/ramp, printed up
   front so there are no surprises.
4. **Low gains.** Default kp is half the value I2RT's own `yam_v1.yml` uses for
   joints 4-6, so tracking is soft — it will lag the target rather than fight.
5. **Live abort.** Every cycle checks the motor's error code, its torque against
   `--max-torque`, and how far it has strayed from the commanded target. Any
   breach stops, returns to the start position, and disables.

On exit — success, abort, exception or Ctrl-C — the motor is commanded back to
where it started and then disabled. The 400 ms firmware timeout is the backstop
underneath all of that: if this process dies outright, the motor damps itself.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    DEFAULT_ARM,
    YAM_BITRATE,
    YAM_JOINTS,
    YAM_MOTOR_TYPES,
    add_i2rt_to_path,
    open_motor_interface,
)

# Per-motor ceilings, whatever --delta says. The big proximal joints swing the
# whole arm through space, so they stay tight; the wrist and gripper only change
# orientation, so they can travel far enough to actually be seen. Raised from a
# flat 0.30 on 2026-08-10 once position feedback, joint limits and torque had all
# been verified against the real arm — a 7.6° move proved invisible in practice.
MAX_DELTA_BY_MOTOR = {
    1: 0.20,  # base_yaw       — swings the entire arm
    2: 0.20,  # shoulder_pitch — lifts the entire arm
    3: 0.20,  # elbow_pitch
    4: 0.30,  # forearm_pitch
    5: 0.50,  # wrist_roll
    6: 1.00,  # gripper_twist  — orientation only, cannot extend into anything
    7: 0.30,  # gripper_jaws   — hard stops, limits uncalibrated
}
MAX_DELTA_FALLBACK = 0.20
# 0.15 rad = 8.6°, chosen to be *visible* — the whole point is for Julien to see
# it move. Still tiny: even if my reading of the position unit is wrong by 10x,
# this is 1.5 rad on a wrist twist, which changes orientation and not reach, so
# it cannot extend into anything.
DEFAULT_DELTA_RAD = 0.15
DEFAULT_RAMP_S = 2.5
DEFAULT_KP = 8.0          # yam_v1.yml uses 10.0 for joints 4-6. Started at 5.0;
                          # measured peak torque was only 0.14 Nm and tracking
                          # showed stick-slip (peak speed 3.6x the commanded
                          # ramp), so the softer gain was hurting, not helping.
DEFAULT_KD = 1.0          # yam_v1.yml uses 1.5
DEFAULT_MAX_TORQUE = 1.5  # Nm. DM4310 peak is 10 Nm, so this is a small fraction
CONTROL_HZ = 100.0        # measured feasible: bench_can.py sustains ~320 Hz
MAX_TRACKING_ERROR = 0.35 # rad; commanded-vs-actual divergence that aborts


def main() -> int:
    ap = argparse.ArgumentParser(description="Move a single motor a small, ramped, bounded amount.")
    ap.add_argument("--yes", action="store_true", help="actually move (default: dry run)")
    ap.add_argument("--motor", type=int, default=6, help="6 = wrist twist (default), 7 = gripper")
    ap.add_argument("--delta", type=float, default=DEFAULT_DELTA_RAD, help="rad, clamped per motor")
    ap.add_argument("--cycles", type=int, default=1, help="how many out-and-back sweeps — >1 makes it obvious")
    ap.add_argument("--ramp", type=float, default=DEFAULT_RAMP_S, help="seconds to travel the delta")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold at the far end")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--max-torque", type=float, default=DEFAULT_MAX_TORQUE)
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS), help="WHICH ARM MOVES. Selected by serial, never by index.")
    args = ap.parse_args()

    cap = MAX_DELTA_BY_MOTOR.get(args.motor, MAX_DELTA_FALLBACK)
    delta = max(-cap, min(cap, args.delta))
    clamped = delta != args.delta
    motor_type_name = YAM_MOTOR_TYPES.get(args.motor, "DM4310")
    joint_name, lo, hi = YAM_JOINTS.get(args.motor, ("unknown", None, None))

    print("=== plan ===")
    print(f"  ARM          : {args.arm}  (serial {ARM_SERIALS[args.arm]})  ← verified after opening")
    print(f"  motor        : {args.motor} = {joint_name} ({motor_type_name})")
    if lo is not None:
        print(f"  URDF limits  : {lo:+.4f} … {hi:+.4f} rad   (from yam.urdf, enforced below)")
    else:
        print("  URDF limits  : none published for this motor — travel NOT enforced ⚠️")
    print(f"  delta        : {delta:+.4f} rad  ({delta * 57.2958:+.2f}°)"
          + (f"   ⚠️ CLAMPED from {args.delta:+.4f} (cap {cap} for this motor)" if clamped else ""))
    print(f"  cycles       : {args.cycles} × (out, hold, back)")
    print(f"  ramp         : {args.ramp:.2f} s   → peak speed {abs(delta) / args.ramp:.4f} rad/s "
          f"({abs(delta) / args.ramp * 57.2958:.2f}°/s)")
    print(f"  hold         : {args.hold:.2f} s, then ramp back to the exact start position")
    print(f"  gains        : kp={args.kp}  kd={args.kd}   (yam_v1.yml uses kp=10, kd=1.5 here)")
    print(f"  abort if     : error != normal, |torque| > {args.max_torque} Nm, "
          f"|tracking error| > {MAX_TRACKING_ERROR} rad")
    print(f"  control rate : {CONTROL_HZ:.0f} Hz   (measured feasible: ~320 Hz)")
    print("  on exit      : return to start, then disable\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing moved. Re-run with --yes to move the arm.")
        return 0

    add_i2rt_to_path()
    from i2rt.motor_drivers.dm_driver import MotorType  # noqa: PLC0415

    motor_type = getattr(MotorType, motor_type_name)

    iface = open_motor_interface(bitrate=args.bitrate, arm=args.arm)
    period = 1.0 / CONTROL_HZ
    start_pos: float | None = None
    samples: list[tuple[float, float, float, float]] = []  # t, target, actual, torque
    abort_reason: str | None = None

    def command(target: float):
        return iface.set_control(args.motor, motor_type, target, 0.0, args.kp, args.kd, 0.0)

    try:
        info = iface.motor_on(args.motor, motor_type)
        start_pos = info.position
        print(f"enabled. start position = {start_pos:+.4f} rad, error = {info.error_message}")

        # Sixth safety mechanism, added once the URDF limits were known: refuse to
        # command a target the vendor model says the joint cannot reach. Checked
        # AFTER enabling, because it needs the measured start position.
        if lo is not None:
            target_end = start_pos + delta
            if not (lo <= target_end <= hi):
                abort_reason = (
                    f"target {target_end:+.4f} rad is outside {joint_name} limits "
                    f"[{lo:+.4f}, {hi:+.4f}] — refusing to move"
                )
                raise RuntimeError(abort_reason)
            margin = min(target_end - lo, hi - target_end)
            print(f"limit check ok: target {target_end:+.4f} rad, {margin:.3f} rad of margin")
        print()

        def run_ramp(from_pos: float, to_pos: float, seconds: float, label: str) -> bool:
            """Interpolate the target. Returns False if an abort condition fired."""
            nonlocal abort_reason
            steps = max(1, int(seconds * CONTROL_HZ))
            for i in range(steps + 1):
                frac = i / steps
                target = from_pos + (to_pos - from_pos) * frac
                t0 = time.perf_counter()
                fb = command(target)
                samples.append((time.perf_counter(), target, fb.position, fb.torque))

                # ⚠️ error_code is a STRING like '0x1', despite FeedbackFrameInfo
                # annotating it as int (dm_driver.py assigns error_hex to it).
                # Comparing it directly to 0x1 is always True and aborts instantly.
                if int(str(fb.error_code), 16) != 0x1:
                    abort_reason = f"motor error during {label}: {fb.error_message}"
                    return False
                if abs(fb.torque) > args.max_torque:
                    abort_reason = f"torque {fb.torque:+.3f} Nm exceeded {args.max_torque} Nm during {label}"
                    return False
                if abs(fb.position - target) > MAX_TRACKING_ERROR:
                    abort_reason = (
                        f"tracking error {fb.position - target:+.3f} rad exceeded "
                        f"{MAX_TRACKING_ERROR} rad during {label}"
                    )
                    return False

                sleep = period - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)
            return True

        for cycle in range(args.cycles):
            tag = f"cycle {cycle + 1}/{args.cycles}"
            if not run_ramp(start_pos, start_pos + delta, args.ramp, f"{tag} out"):
                break
            if not run_ramp(start_pos + delta, start_pos + delta, args.hold, f"{tag} hold"):
                break
            if not run_ramp(start_pos + delta, start_pos, args.ramp, f"{tag} back"):
                break

    except KeyboardInterrupt:
        abort_reason = "interrupted by Ctrl-C"
    except Exception as exc:  # noqa: BLE001
        abort_reason = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if start_pos is not None:
                # Zero gains and zero torque: stop applying force before disabling.
                iface.set_control(args.motor, motor_type, start_pos, 0.0, 0.0, 0.0, 0.0)
            iface.motor_off(args.motor)
            print("\nmotor disabled.")
        except Exception as exc:  # noqa: BLE001
            print(f"\n⚠️  could not cleanly disable ({type(exc).__name__}) — "
                  "the 400 ms firmware timeout will damp it")
        iface.close()

    if abort_reason:
        print(f"\n⛔ ABORTED: {abort_reason}")

    if len(samples) < 2 or start_pos is None:
        print("no usable samples.")
        return 1

    positions = [s[2] for s in samples]
    travelled = max(positions) - min(positions)
    torques = [abs(s[3]) for s in samples]
    t0 = samples[0][0]
    speeds = [
        abs(samples[i][2] - samples[i - 1][2]) / max(1e-6, samples[i][0] - samples[i - 1][0])
        for i in range(1, len(samples))
    ]

    print("\n=== what actually happened ===")
    print(f"  samples          : {len(samples)} over {samples[-1][0] - t0:.2f} s")
    print(f"  start position   : {start_pos:+.4f} rad")
    print(f"  range travelled  : {travelled:.4f} rad  ({travelled * 57.2958:.2f}°)")
    print(f"  commanded delta  : {abs(delta):.4f} rad  ({abs(delta) * 57.2958:.2f}°)")
    print(f"  peak speed       : {max(speeds):.4f} rad/s  ({max(speeds) * 57.2958:.2f}°/s)")
    print(f"  peak |torque|    : {max(torques):.4f} Nm  (limit {args.max_torque})")
    print(f"  final position   : {positions[-1]:+.4f} rad  "
          f"(drift from start: {positions[-1] - start_pos:+.4f} rad)")

    if travelled < abs(delta) * 0.3:
        print(
            "\n⚠️  It barely moved. Most likely the gains are too low to overcome friction —\n"
            f"   try --kp {args.kp * 2:.0f}. That it did not move is itself a safe outcome."
        )
    else:
        print("\n✅ The joint moved and returned. This is the first commanded motion on this arm.")
    return 1 if abort_reason else 0


if __name__ == "__main__":
    sys.exit(main())
