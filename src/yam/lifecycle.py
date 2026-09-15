"""Typed stop causes and the controlled park/quit interaction.

The caller owns acquired devices and always disables them in its outer finally.
This flow only parks or changes modes; exceptions propagate to that cleanup.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time
from typing import Any, Callable, Sequence


class StopCause(Enum):
    QUIT = "quit"
    INTERRUPT = "interrupt"
    FAULT = "fault"


@dataclass(frozen=True)
class StopRequest:
    cause: StopCause
    message: str

    def __str__(self) -> str:
        return self.message

    @property
    def exit_code(self) -> int:
        return {StopCause.QUIT: 0, StopCause.INTERRUPT: 130, StopCause.FAULT: 1}[self.cause]


def controlled_stop(stop: StopRequest, arms: Sequence[Any], keys: Any,
                    park: Callable[[list[Any]], str], *,
                    emit: Callable[[str], None] = print,
                    sleep: Callable[[float], None] = time.sleep) -> None:
    """Keep planned quit interactive; attempt guarded parking on other stops.

    Only live arms can park. A failed automatic or menu park returns to HOLD and
    the menu; it never authorizes disabling by itself. A second interrupt escapes.
    """
    unplanned = stop.cause is not StopCause.QUIT
    auto_parked = False
    live = [one for one in arms if one.alive()]
    if live and unplanned and any(
            one.base_pose is not None for one in live):
        for one in live:
            one.enter_hold()
        if stop.cause is StopCause.INTERRUPT:
            emit("\n⭐ Ctrl-C — parking to the pose this session started in, then")
        else:
            emit(f"\n⭐ SAFE STOP after {stop.message!r} — the chain is still alive, so")
            emit("   the arms are being parked to the pose this session started in, then")
        emit("   disabling. Press any key to stop the motion; Ctrl-C again forces out.")
        outcome = park(live)
        if outcome == "arrived":
            auto_parked = True
            emit("\n   Disabling the motors now.\n")
        else:
            emit(f"\n⚠️  the automatic park ended as {outcome!r}, so nothing is "
                  "being released. Choose below.")

    if any(one.alive() for one in arms) and not auto_parked:
        for one in arms:
            if one.alive():
                one.enter_hold()
        emit("\nEvery arm is HOLDING its pose. Nothing is released until you choose.")
        emit("   q = PARK then DISABLE — the whole shutdown in one key")
        emit("   p = PARK — drive back to the park pose, then it holds there")
        emit("   g = go weightless so you can park it by hand")
        emit("   d = disable now (⚠️ a raised arm will sag)")
        emit("   ⭐ to park WITHOUT quitting, use p in the session itself, then t")
        while True:
            k = keys.get()
            if k == "q":
                outcome = park([one for one in arms if one.alive()])
                if outcome == "arrived":
                    emit("\n   Parked. Disabling the motors now.\n")
                    break
                for one in arms:
                    if one.alive():
                        one.enter_hold()
                emit(f"\n⚠️  the park ended as {outcome!r}, so nothing is being "
                      "released.")
                emit("   q = try again    p = park    g = weightless    d = disable")
            elif k == "p":
                park([one for one in arms if one.alive()])
                for one in arms:
                    if one.alive():
                        one.enter_hold()
                emit("   q = park+disable    p = park again    g = weightless    d = disable")
            elif k == "g":
                for one in arms:
                    if one.alive():
                        guide_warn = one.enter_guide()
                        if guide_warn:
                            emit(f"\n  ⚠️  {guide_warn}\n")
                guided = "+".join(one.name for one in arms
                                  if one.alive() and one.mode == "guide")
                if guided:
                    emit(f"\n⭐ weightless: {guided}"
                          " — park them by hand, then press d to disable.")
            elif k == "d":
                break
            sleep(0.05)
            if not any(one.alive() for one in arms):
                emit("\n⚠️  every chain died while waiting — disabling now.")
                break
    elif not any(one.alive() for one in arms):
        emit("⚠️  every chain is already dead, so no arm is being commanded.")
        emit("   They will be sagging under gravity. Support them now if raised.")
