"""Read pucks once per cycle and interpret gripper-button edges and holds.

Acquired device lifetimes stay with SessionResources. A shared reader is drained
once and routed by selection; a read failure centers input and requests controlled
stopping. Mapping edits require explicit keys or the button-learning workflow.
"""
from __future__ import annotations

from yam.lifecycle import StopCause, StopRequest


class SessionInput:
    def __init__(self, *, shared_reader, selection, n_arm: int,
                 button_rate: float, clamp, emit=print):
        self.shared_reader = shared_reader
        self.selection = selection
        self.n_arm = n_arm
        self.button_rate = button_rate
        self.clamp = clamp
        self.emit = emit

    def poll(self, arms, dt: float, *, prompt_open: bool,
             stop: StopRequest | None = None) -> StopRequest | None:
        shared_axes: list[float] | None = None
        shared_buttons = 0
        if self.shared_reader is not None:
            try:
                shared_axes = self.shared_reader.read()
                shared_buttons = getattr(self.shared_reader, "buttons", 0)
            except Exception as exc:  # noqa: BLE001
                shared_axes = [0.0] * 6
                if not stop:
                    stop = StopRequest(
                        StopCause.FAULT, "the shared SpaceMouse stopped answering "
                        f"({type(exc).__name__}) — unplugged?")
                    self.emit(f"\n⛔ {stop}")
                    self.emit("   Treating it as centred and parking safely.\n")
        for one in arms:
            if shared_axes is not None:
                aimed_now = one.name in self.selection.names()
                one.raw_axes = list(shared_axes) if aimed_now else [0.0] * 6
                buttons = shared_buttons if aimed_now else 0
            else:
                try:
                    one.raw_axes = one.reader.read()
                except Exception as exc:  # noqa: BLE001
                    one.raw_axes = [0.0] * 6
                    if not stop:
                        stop = StopRequest(
                            StopCause.FAULT, f"arm {one.name}'s SpaceMouse stopped "
                            f"answering ({type(exc).__name__}) — unplugged?")
                        self.emit(f"\n⛔ {stop}")
                        self.emit("   Treating that puck as centred and parking "
                              "safely.\n")
                    continue
                buttons = getattr(one.reader, "buttons", 0)
            pressed = buttons & ~one.buttons_prev              # rising edge only
            one.buttons_prev = buttons

            if one.learn_button is not None and prompt_open:
                one.learn_button = None
                self.emit(f"\n  ⚠️ the gripper-button learning on arm {one.name} is "
                      f"CANCELLED — another prompt opened. Press b to restart it.\n")
            if one.learn_button is not None and pressed:
                warn = one.axis_map.learn_button(one.learn_button, pressed)
                if warn:
                    self.emit(f"\n  ⚠️  {warn}\n")
                elif one.learn_button == "open":
                    one.learn_button = "close"
                    self.emit(f"  ✓ OPEN  ← button 0x{pressed:02x}")
                    self.emit("   Now press the button you want for CLOSE …\n")
                else:
                    one.learn_button = None
                    self.emit(f"  ✓ CLOSE ← button 0x{pressed:02x}")
                    self.emit(one.axis_map.buttons_row())
                    self.emit("   (f swaps them if they are the wrong way round)\n")
            elif pressed:
                one.last_input_kind = "button"
                if one.axis_map.button_action(pressed) is None:
                    self.emit(f"\n  button 0x{pressed:02x} is not assigned — press b to set the "
                          f"gripper buttons (works in any mode)\n")
                elif one.mode not in ("teleop", "map"):
                    self.emit(f"\n  gripper buttons move the jaws in TELEOP (t) and CONTROLS (m); "
                          f"you are in {one.mode.upper()}\n")

            if one.learn_button is None and one.robot.num_dofs() > self.n_arm and one.mode in ("teleop", "map"):
                action = one.axis_map.button_action(buttons)
                if action == "open":
                    one.gripper_value = self.clamp(one.gripper_value + self.button_rate * dt)
                elif action == "close":
                    one.gripper_value = self.clamp(one.gripper_value - self.button_rate * dt)

        return stop
