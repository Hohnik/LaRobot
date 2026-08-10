#!/usr/bin/env python3
"""Run the teleop loop against the SIMULATED arm. Touches no hardware, ever.

    uv run scripts/teleop_sim.py --demo                 # scripted circle, no input device
    uv run scripts/teleop_sim.py                        # driven by the real SpaceMouse
    uv run scripts/teleop_sim.py --view                 # ...with the MuJoCo viewer

⛔ THIS CANNOT MOVE THE ROBOT. It builds `get_yam_robot(sim=True)`, which is a
MuJoCo model, and never opens a CAN bus. Safe to run at any time, with the arms
powered or unplugged.

WHY THIS EXISTS, rather than going straight to the arm
------------------------------------------------------
`get_yam_robot()` returns the same `Robot` interface whether `sim` is True or
False. So this is not a mock-up of the teleop loop — it *is* the teleop loop,
with the hardware swapped out. Every axis convention, frame, sign and
singularity gets found here, where being wrong costs a reprinted number instead
of a joint slamming into its stop.

`--demo` goes further and removes the SpaceMouse too, driving a scripted circle.
Debugging IK and a HID decoder simultaneously means never knowing which one is
at fault.
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

from teleop import CartesianTeleop, scripted_twist  # noqa: E402

CONTROL_HZ = 100.0
# Full SpaceMouse deflection maps to these speeds. Deliberately gentle: these are
# the numbers that will later move a real arm, so they are chosen for the
# hardware case and merely inherited by the simulation.
LINEAR_SCALE = 0.08   # m/s at full deflection
ANGULAR_SCALE = 0.40  # rad/s at full deflection


def read_spacemouse_twist(handle, state: dict) -> np.ndarray:
    """Drain pending HID reports and return the latest twist. Non-blocking."""
    import struct

    while True:
        data = handle.read(64)
        if not data:
            break
        rid, payload = data[0], bytes(data[1:])
        if rid == 0x01 and len(payload) >= 12:
            vals = struct.unpack("<6h", payload[:12])
            state["t"] = [v / 350.0 for v in vals[:3]]
            state["r"] = [v / 350.0 for v in vals[3:]]
        elif rid == 0x01 and len(payload) >= 6:
            state["t"] = [v / 350.0 for v in struct.unpack("<3h", payload[:6])]
        elif rid == 0x02 and len(payload) >= 6:
            state["r"] = [v / 350.0 for v in struct.unpack("<3h", payload[:6])]
        elif rid == 0x03 and payload:
            state["buttons"] = int.from_bytes(payload[:2], "little")

    def dead(v: float) -> float:
        return 0.0 if abs(v) < 0.06 else max(-1.0, min(1.0, v))

    t = [dead(v) for v in state["t"]]
    r = [dead(v) for v in state["r"]]
    return np.array(
        [
            t[0] * LINEAR_SCALE, t[1] * LINEAR_SCALE, t[2] * LINEAR_SCALE,
            r[0] * ANGULAR_SCALE, r[1] * ANGULAR_SCALE, r[2] * ANGULAR_SCALE,
        ]
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Teleop against the simulated YAM. No hardware.")
    ap.add_argument("--demo", action="store_true", help="scripted circle instead of the SpaceMouse")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--view", action="store_true", help="open the MuJoCo viewer")
    ap.add_argument("--position-cost", type=float, default=1.0)
    ap.add_argument("--orientation-cost", type=float, default=0.5)
    args = ap.parse_args()

    from i2rt.robots.get_robot import get_yam_robot  # noqa: PLC0415
    from i2rt.robots.utils import ArmType, GripperType  # noqa: PLC0415

    robot = get_yam_robot(arm_type=ArmType.YAM, gripper_type=GripperType.LINEAR_4310, sim=True)
    print(f"simulated robot: {type(robot).__name__}, {robot.num_dofs()} dofs")

    teleop = CartesianTeleop(
        position_cost=args.position_cost,
        orientation_cost=args.orientation_cost,
    )
    q0 = np.asarray(robot.get_joint_pos(), dtype=float)
    teleop.reset(q0)
    start_ee = teleop.ee_position().copy()
    print(f"start joints : {np.round(q0, 4)}")
    print(f"start EE pos : {np.round(start_ee, 4)} m  (site '{teleop.ee_site}')\n")

    handle = None
    state = {"t": [0.0] * 3, "r": [0.0] * 3, "buttons": 0}
    if not args.demo:
        sys.path.insert(0, str(REPO / "src"))
        from spacemouse import countdown_hands_off, find_device, open_device  # noqa: PLC0415

        info = find_device()
        if info is None:
            print("No SpaceMouse found. Use --demo to run without it.")
            return 1
        countdown_hands_off(3)
        handle = open_device(info)
        handle.set_nonblocking(True)
        print("Move the SpaceMouse — the SIMULATED arm will follow.\n")

    viewer = None
    if args.view:
        import mujoco.viewer  # noqa: PLC0415

        viewer = mujoco.viewer.launch_passive(teleop.model, teleop.configuration.data)

    dt = 1.0 / CONTROL_HZ
    t0 = time.perf_counter()
    next_report = 1.0
    max_step = 0.0
    prev_q = q0[:6].copy()

    try:
        while True:
            t = time.perf_counter() - t0
            if t >= args.seconds:
                break
            loop_start = time.perf_counter()

            twist = scripted_twist(t) if args.demo else read_spacemouse_twist(handle, state)
            q_arm = teleop.step(twist, dt)

            # Per-cycle joint step. This is the number that matters for hardware:
            # a large jump here would be a large jump at the real arm.
            max_step = max(max_step, float(np.max(np.abs(q_arm - prev_q))))
            prev_q = q_arm.copy()

            full = np.zeros(robot.num_dofs())
            full[:6] = q_arm
            robot.command_joint_pos(full)

            if viewer is not None:
                viewer.sync()

            if t >= next_report:
                next_report += 1.0
                ee = teleop.ee_position()
                print(
                    f"  t={t:5.1f}s  EE={np.round(ee, 3)}  "
                    f"moved={np.linalg.norm(ee - start_ee):.3f} m  "
                    f"q={np.round(q_arm, 3)}"
                )

            time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))
    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        if handle is not None:
            handle.close()
        if viewer is not None:
            viewer.close()
        robot.close()

    ee = teleop.ee_position()
    moved = float(np.linalg.norm(ee - start_ee))
    print("\n=== result ===")
    print(f"  EE start      : {np.round(start_ee, 4)}")
    print(f"  EE end        : {np.round(ee, 4)}")
    print(f"  EE displaced  : {moved:.4f} m")
    print(f"  final joints  : {np.round(prev_q, 4)}")
    print(f"  max joint step per cycle: {max_step:.5f} rad ({max_step * 57.2958:.3f}°)")

    ok = moved > 0.01 if args.demo else True
    if args.demo and not ok:
        print("\n⛔ The end effector barely moved. IK is not tracking the target.")
        return 1
    if max_step > 0.05:
        print(f"\n⚠️  Largest single-cycle joint step is {max_step * 57.2958:.2f}° — too jumpy for hardware.")
        print("   Raise --orientation-cost damping or lower the twist scales before going to the arm.")
    else:
        print("\n✅ Smooth: no single cycle moved a joint more than 0.05 rad.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
