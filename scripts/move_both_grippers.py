#!/usr/bin/env python3
"""Twist BOTH grippers at once — different directions, different speeds.

    uv run scripts/move_both_grippers.py             # dry run: prints the plan
    uv run scripts/move_both_grippers.py --yes       # MOVES both arms

⛔ THIS MOVES TWO PHYSICAL ROBOT ARMS.

Only motor 6 (`gripper_twist`) on each arm. That joint rotates the gripper about
its own axis: it changes orientation, never reach, so neither arm can extend
toward the other or into anything else however far it turns. Julien's judgement,
2026-08-10: *"if you're just turning the gripper... nothing can happen."*

WHY THIS IS THE INTERESTING TEST, not just a nicer demo
-------------------------------------------------------
It is the first time this repo drives **two independent CAN buses in one control
loop**, which is the actual shape of the bimanual teleop the plan is aiming at
(`docs/Setup-Plan.md` §4.4: one SpaceMouse per arm). Everything it exercises is
load-bearing later:

  · two adapters open simultaneously, each verified by serial after opening
  · one 100 Hz loop servicing both, rather than two racing threads
  · per-arm independent trajectories — proving the arms are genuinely decoupled

Different amplitude, period and direction per arm is deliberate. If the two
grippers moved identically we could not tell "two arms correctly driven" from
"one arm driven twice, mirrored by coincidence" — the same trap that made the
earlier two-arm scan meaningless. **Visibly different motion IS the evidence.**

MOTION SHAPE
------------
A sine, not a ramp: it starts at zero velocity, ends at zero velocity, and never
exceeds `amplitude · 2π / period`. There is no discontinuity a P controller could
turn into a snap, and because sin(0)=0 the target begins exactly at the measured
start position, so enabling cannot cause a jump.

SAFETY — same five mechanisms as `move_one_motor.py`, per arm and per cycle:
relative to the measured start · amplitude clamped into the URDF joint limits
with headroom · smooth bounded velocity · gains below the vendor's · live abort
on error code, torque and tracking divergence. On exit, by any path, both arms
are ramped back to their start positions and disabled. The motors' own 400 ms
timeout is the backstop if this process dies outright.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from yam_can import (  # noqa: E402
    ARM_SERIALS,
    YAM_BITRATE,
    YAM_JOINTS,
    YAM_MOTOR_TYPES,
    add_i2rt_to_path,
    open_motor_interface,
)

MOTOR = 6  # gripper_twist on both arms
CONTROL_HZ = 100.0
DEFAULT_KP = 8.0
DEFAULT_KD = 1.0
DEFAULT_MAX_TORQUE = 1.5
MAX_TRACKING_ERROR = 0.35
LIMIT_MARGIN = 0.05
MAX_AMPLITUDE = 1.90  # matches move_one_motor.py's cap for gripper_twist


@dataclass
class Arm:
    name: str
    amplitude: float
    period: float
    direction: int
    iface: object = None
    start: float = 0.0
    motor_type: object = None
    samples: list = field(default_factory=list)

    def target_at(self, t: float) -> float:
        return self.start + self.direction * self.amplitude * math.sin(2 * math.pi * t / self.period)

    @property
    def peak_speed(self) -> float:
        return self.amplitude * 2 * math.pi / self.period


def main() -> int:
    ap = argparse.ArgumentParser(description="Twist both grippers simultaneously, differently.")
    ap.add_argument("--yes", action="store_true", help="actually move (default: dry run)")
    ap.add_argument("--seconds", type=float, default=20.0, help="how long to oscillate")
    ap.add_argument("--amp1", type=float, default=1.4, help="arm1 amplitude, rad")
    ap.add_argument("--amp2", type=float, default=0.5, help="arm2 amplitude, rad")
    ap.add_argument("--period1", type=float, default=10.0, help="arm1 seconds per full cycle")
    ap.add_argument("--period2", type=float, default=2.0, help="arm2 seconds per full cycle")
    ap.add_argument("--kp", type=float, default=DEFAULT_KP)
    ap.add_argument("--kd", type=float, default=DEFAULT_KD)
    ap.add_argument("--max-torque", type=float, default=DEFAULT_MAX_TORQUE)
    ap.add_argument("--bitrate", type=int, default=YAM_BITRATE)
    args = ap.parse_args()

    joint_name, lo, hi = YAM_JOINTS[MOTOR]
    arms = [
        Arm("arm1", min(abs(args.amp1), MAX_AMPLITUDE), args.period1, +1),
        Arm("arm2", min(abs(args.amp2), MAX_AMPLITUDE), args.period2, -1),
    ]

    print("=== plan ===")
    print(f"  motor        : {MOTOR} = {joint_name} on BOTH arms")
    print(f"  URDF limits  : {lo:+.4f} … {hi:+.4f} rad")
    print(f"  duration     : {args.seconds:.1f} s")
    print(f"  gains        : kp={args.kp}  kd={args.kd}   control {CONTROL_HZ:.0f} Hz, one loop for both")
    for arm in arms:
        print(
            f"  {arm.name:<5}        : ±{arm.amplitude:.2f} rad (±{arm.amplitude * 57.2958:.1f}°)  "
            f"period {arm.period:.1f} s  direction {arm.direction:+d}  "
            f"peak {arm.peak_speed:.2f} rad/s ({arm.peak_speed * 57.2958:.0f}°/s)"
            f"   serial {ARM_SERIALS[arm.name]}"
        )
    print("  on exit      : both ramped back to start, both disabled\n")

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing moved. Re-run with --yes.")
        return 0

    add_i2rt_to_path()
    from i2rt.motor_drivers.dm_driver import MotorType  # noqa: PLC0415

    motor_type = getattr(MotorType, YAM_MOTOR_TYPES[MOTOR])
    period = 1.0 / CONTROL_HZ
    abort_reason: str | None = None

    def shutdown() -> None:
        """Ramp every arm home IN PARALLEL, then disable. Never raises.

        ⛔ The arms must be interleaved, not handled one after the other. An
        earlier version ramped arm1 home over 1.5 s and only then started arm2 --
        so arm2 went 1.5 s without a command and tripped its own 400 ms firmware
        watchdog, surfacing as `loss communication` and a failed ramp home.
        Observed on the real hardware, 2026-08-10. The safety timeout worked
        exactly as designed; the shutdown code was starving it.

        The rule this generalises to: **once a motor is enabled, every code path
        must keep feeding it faster than 400 ms, including the paths that run
        while something else is being tidied up.**
        """
        live = [a for a in arms if a.iface is not None]
        if not live:
            return
        starts = {a.name: (a.samples[-1][2] if a.samples else a.start) for a in live}
        steps = int(1.5 * CONTROL_HZ)
        for i in range(steps + 1):
            frac = i / steps
            for arm in live:
                try:
                    tgt = starts[arm.name] + (arm.start - starts[arm.name]) * frac
                    arm.iface.set_control(MOTOR, motor_type, tgt, 0.0, args.kp, args.kd, 0.0)
                except Exception:  # noqa: BLE001, S110
                    pass
            time.sleep(period)
        for arm in live:
            try:
                arm.iface.set_control(MOTOR, motor_type, arm.start, 0.0, 0.0, 0.0, 0.0)
            except Exception:  # noqa: BLE001, S110
                pass
            try:
                arm.iface.motor_off(MOTOR)
            except Exception:  # noqa: BLE001, S110
                pass
            try:
                arm.iface.close()
            except Exception:  # noqa: BLE001, S110
                pass
        print("both arms returned to start and disabled; both buses closed.")

    try:
        # Open sequentially. Each open verifies the adapter serial after the fact
        # (src/yam_can.py), which matters doubly here: GsUsb.start() resets its USB
        # device, so enumeration order can shift between the two opens.
        for arm in arms:
            arm.iface = open_motor_interface(bitrate=args.bitrate, arm=arm.name, name=f"yam_{arm.name}")
            arm.motor_type = motor_type
            info = arm.iface.motor_on(MOTOR, motor_type)
            arm.start = info.position
            print(f"{arm.name}: enabled, start = {arm.start:+.4f} rad, error = {info.error_message}")

            reach_lo, reach_hi = arm.start - arm.amplitude, arm.start + arm.amplitude
            room = min(arm.start - (lo + LIMIT_MARGIN), (hi - LIMIT_MARGIN) - arm.start)
            if room < 0:
                raise RuntimeError(f"{arm.name} {joint_name} is already at its limit ({arm.start:+.4f} rad)")
            if reach_lo < lo + LIMIT_MARGIN or reach_hi > hi - LIMIT_MARGIN:
                arm.amplitude = max(0.0, room)
                print(f"  ⚠️  amplitude shortened to ±{arm.amplitude:.3f} rad to stay inside the joint limits")
            if arm.amplitude < 0.05:
                raise RuntimeError(f"{arm.name} has no usable room to move from {arm.start:+.4f} rad")
        print()

        t_start = time.perf_counter()
        next_report = 2.0
        while True:
            t = time.perf_counter() - t_start
            if t >= args.seconds:
                break
            for arm in arms:
                target = arm.target_at(t)
                fb = arm.iface.set_control(MOTOR, motor_type, target, 0.0, args.kp, args.kd, 0.0)
                arm.samples.append((t, target, fb.position, fb.torque))

                if int(str(fb.error_code), 16) != 0x1:
                    abort_reason = f"{arm.name} motor error: {fb.error_message}"
                    raise RuntimeError(abort_reason)
                if abs(fb.torque) > args.max_torque:
                    abort_reason = f"{arm.name} torque {fb.torque:+.3f} Nm exceeded {args.max_torque}"
                    raise RuntimeError(abort_reason)
                if abs(fb.position - target) > MAX_TRACKING_ERROR:
                    abort_reason = (
                        f"{arm.name} tracking error {fb.position - target:+.3f} rad "
                        f"exceeded {MAX_TRACKING_ERROR}"
                    )
                    raise RuntimeError(abort_reason)

            if t >= next_report:
                # 1.7 s, not 2.0: a report interval that is a multiple of a motion
                # period samples the same phase every time and shows a moving
                # joint as frozen. arm2's default period is exactly 2.0 s.
                next_report += 1.7
                where = "   ".join(f"{a.name} {a.samples[-1][2]:+.3f}" for a in arms)
                print(f"  t={t:5.1f}s   {where}")

            time.sleep(max(0.0, period - (time.perf_counter() - t_start - t)))

    except KeyboardInterrupt:
        abort_reason = "interrupted by Ctrl-C"
    except Exception as exc:  # noqa: BLE001
        abort_reason = abort_reason or f"{type(exc).__name__}: {exc}"
    finally:
        shutdown()

    if abort_reason:
        print(f"\n⛔ ABORTED: {abort_reason}")

    print("\n=== what actually happened ===")
    ok = True
    for arm in arms:
        if len(arm.samples) < 2:
            print(f"  {arm.name}: no usable samples")
            ok = False
            continue
        pos = [s[2] for s in arm.samples]
        travelled = max(pos) - min(pos)
        speeds = [
            abs(arm.samples[i][2] - arm.samples[i - 1][2]) / max(1e-6, arm.samples[i][0] - arm.samples[i - 1][0])
            for i in range(1, len(arm.samples))
        ]
        print(
            f"  {arm.name}: {len(arm.samples)} samples  "
            f"travelled {travelled:.3f} rad ({travelled * 57.2958:.1f}°)  "
            f"commanded ±{arm.amplitude:.2f} → span {2 * arm.amplitude * 57.2958:.1f}°  "
            f"peak speed {max(speeds) * 57.2958:.0f}°/s  "
            f"peak |torque| {max(abs(s[3]) for s in arm.samples):.3f} Nm"
        )
        if travelled < arm.amplitude * 0.5:
            ok = False

    rate = sum(len(a.samples) for a in arms) / 2 / max(1e-6, arms[0].samples[-1][0]) if arms[0].samples else 0
    print(f"\n  achieved control rate: {rate:.0f} Hz per arm (target {CONTROL_HZ:.0f})")

    if ok and not abort_reason:
        print("\n✅ Both arms driven simultaneously from one loop, on two independent CAN buses.")
    return 1 if abort_reason else 0


if __name__ == "__main__":
    sys.exit(main())
