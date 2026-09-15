"""Format the pre-start plan from resolved settings, maps and saved poses.

No devices are read or opened here. The application prints these lines before
saving requested defaults or acquiring camera, input and robot handles.
"""
from __future__ import annotations

from argparse import Namespace
from typing import Mapping, Sequence
import numpy as np

from yam.can import ARM_SERIALS
from yam.inputs.axis_map import AxisMapStore, motions_for
from yam.robot import SAFE_MAX_SPEED, SAFE_MAX_LAG, VEL_FF_CEILING
from yam.teleop import CartesianTeleop


def session_plan_lines(args: Namespace, arm_names: Sequence[str],
                       map_store: AxisMapStore, saved_slots: Mapping[str, dict], *,
                       angular_scale: float, base_slot: str, planned_speed_default: float,
                       temp_warn: float, temp_stop: float) -> list[str]:
    lines: list[str] = []
    rotation, start_frame = not args.no_rotation, args.frame
    lines.append("=== plan ===")
    for name in arm_names:
        lines.append(f"  ARM         : {name}  (serial {ARM_SERIALS[name]})")
    lines.append(f"  gripper     : {'NOT controlled — motor 7 left free' if args.no_gripper else 'controlled (o/c), frame-checked at startup'}")
    if args.no_gripper:
        lines.append("  ⚠️  gravity   : --no-gripper also swaps the DYNAMICS model, so ee_mass=0.695 kg is")
        lines.append("                passed to keep the arm holding itself. Without it the elbow is 39%")
        lines.append("                short and the arm falls in GUIDE. See FINDINGS §11.")
    lines.append(f"  start mode  : {args.start_mode}")
    lines.append(f"  speed       : {args.linear_scale} m/s linear, "
          f"{angular_scale if rotation else 0} rad/s angular  (rotation {'ON' if rotation else 'OFF'}, toggle with r)")
    for name in arm_names:
        plan_map = map_store.for_arm(name, start_frame)
        lines.append(f"  axis map {name}  : {plan_map.one_line(start_frame)}   (m to change it live)")
        lines.append(f"  map scope   : {map_store.scope_note(name)}")
        if plan_map.unbound():
            names = ", ".join(motions_for(start_frame)[i]["short"] for i in plan_map.unbound())
            lines.append(f"  ⚠️  UNBOUND  : {names} — arm {name} will NOT perform these until they "
                  "are bound (m)")
    lines.append(f"  control fr. : {CartesianTeleop.FRAME_NOTES[start_frame]}  (v cycles it live)")
    for name in arm_names:
        base = saved_slots[name].get(base_slot)
        lines.append(f"  park pose {name} : "
              f"{np.round(base, 3).tolist() if base else 'none saved — press s to set arm'}")
    lines.append(f"  workspace   : {args.reach} m from the base, tip stays above {args.floor} m")
    raised = []
    if args.max_speed != SAFE_MAX_SPEED:
        raised.append("--max-speed")
    if args.teleop_speed != planned_speed_default:
        raised.append("--teleop-speed")
    note = f"  ⚠️ RAISED: {', '.join(raised)}" if raised else ""
    lines.append(f"  joint speed : teleop {min(args.teleop_speed, args.max_speed):.2f} · "
          f"planned {min(args.teleop_speed, args.max_speed):.2f} · "
          f"mirror {args.max_speed:.2f} rad/s{note}")
    if args.mirror_catchup > 0.0:
        lines.append(f"  mirror fix  : ON at {args.mirror_catchup:g}/s — corrects the follower's "
              f"standing offset while the leader is slow, clamped to 0.06 rad")
    if args.vel_ff > 0.0:
        shown_ff = min(args.vel_ff, VEL_FF_CEILING)
        capped = "" if args.vel_ff <= VEL_FF_CEILING else \
            f"  ⚠️ capped from {args.vel_ff:g}: above 1 is a measured dead end " \
            f"(FINDINGS §68.6)"
        lines.append(f"  feedforward : ON at {shown_ff:g} — motors also receive that "
              f"fraction of the command's own speed, so torque starts before error builds "
              f"(item 44; jaw excluded){capped}")
    lag_note = "" if args.max_lag == SAFE_MAX_LAG else "  ⚠️ RAISED"
    lines.append(f"                (SafeRobot caps everything at {args.max_speed:.2f} rad/s AND holds "
          f"the command within {args.max_lag:.2f} rad of the measured pose{lag_note})")
    lines.append(f"  what limits  : ⭐ FOUR limits in series, and the SMALLEST one binds:")
    lines.append(f"                 linear {args.linear_scale:.2f} m/s  how fast a full puck push "
          f"asks the TIP to move  (- / + or setting 8)")
    lines.append(f"                 teleop {args.teleop_speed:.2f} rad/s  how far the IK answer may "
          f"move any ONE JOINT per cycle")
    lines.append(f"                 max-speed {args.max_speed:.2f} rad/s  the same cap again, below "
          f"all control logic, so nothing can reach around it")
    lines.append(f"                 max-lag {args.max_lag:.2f} rad  ⭐ how far the COMMAND may run "
          f"ahead of where the arm actually IS.")
    lines.append(f"                   ⚠️ This is not a speed. The command is pulled back to "
          f"measured+{args.max_lag:.2f} every cycle, so")
    lines.append(f"                   reaching a far target becomes a ratchet: the arm moves, the "
          f"command advances, repeat. A")
    lines.append(f"                   BLOCKED joint therefore never gets there, and that is the "
          f"point — it bounds the push.")
    lines.append(f"  temperature : warn {temp_warn}°C, stop {temp_stop}°C")
    return lines
