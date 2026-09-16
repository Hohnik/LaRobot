"""Explicit keyboard edits to one arm's controls; no motion or mode changes.

ArmSession retains the map and most recent input. Puck movement only supplies the
observed axis; invoking this handler with an editing key changes the mapping.
"""
from __future__ import annotations

from yam.inputs.axis_map import PUCK_AXES, motions_for


def edit_controls(arm, key: str, *, emit=print) -> bool:
    """Consume f, 1-6, u or 0; let the operator dispatch all other keys."""
    active = arm.last_active_axis
    driven = arm.axis_map.motion_driven_by(active) if active is not None else None
    if key == "f":
        if active is None:
            emit("\n  push the puck first — f reverses the control you just used.\n")
        elif driven is None:
            emit(f"\n  puck {PUCK_AXES[active]} drives nothing, so there is no "
                  f"direction to reverse. Press 1-6 to give it a motion.\n")
        else:
            arm.axis_map.flip(driven)
            emit(f"\n  ↔ REVERSED → {arm.axis_map.row(driven, arm.frame).strip()}"
                  f"   (push {PUCK_AXES[active]} again to feel it)\n")
    elif key in "123456":
        if active is None:
            emit("\n  push the puck first — 1-6 reassigns the control you just used.\n")
        else:
            target = int(key) - 1
            if driven == target:
                emit(f"\n  puck {PUCK_AXES[active]} already drives "
                     f"{motions_for(arm.frame)[target]['short']} — unchanged.\n")
            elif driven is not None:
                # Exchange both bindings; the previous motion key reverses this pair.
                arm.axis_map.swap(driven, target)
                emit(f"\n  ⇄ SWAPPED {motions_for(arm.frame)[driven]['short']} ↔ "
                      f"{motions_for(arm.frame)[target]['short']}")
                emit(f"      {arm.axis_map.row(target, arm.frame).strip()}")
                emit(f"      {arm.axis_map.row(driven, arm.frame).strip()}")
                emit(f"      (press {driven + 1} to swap back)\n")
            else:
                # The active control drove nothing, so there is nothing
                # to exchange with. The direction he was last pushing
                # becomes this motion's positive sense.
                displaced = arm.axis_map.bind(target, active, arm.last_active_value)
                emit(f"\n  ✓ puck {PUCK_AXES[active]} now drives "
                      f"{motions_for(arm.frame)[target]['short']} → "
                      f"{arm.axis_map.row(target, arm.frame).strip()}")
                if displaced is not None:
                    emit(f"  ⚠️  {motions_for(arm.frame)[displaced]['short']} was using that "
                          f"control and is now UNBOUND — it will not move.")
                emit("")
    elif key == "u":
        if driven is None:
            emit("\n  that control already drives nothing.\n")
        else:
            arm.axis_map.unbind(driven)
            emit(f"\n  unbound {motions_for(arm.frame)[driven]['short']} — it will not move\n")
    elif key == "0":
        arm.axis_map = arm.axis_map_at_start.copy()
        emit("\n  reverted to the controls this session started with:")
        emit(arm.axis_map.describe(arm.frame) + "\n")
    else:
        return False
    return True
