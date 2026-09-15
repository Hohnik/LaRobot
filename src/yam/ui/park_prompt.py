"""Decode pose/take sequence entry and its second confirmation.

This prompt requests actions; it never resolves saved files or commands arms.
Speed/easing keys are handled by the operator before a key reaches this owner.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ParkAction(Enum):
    UPDATE = "update"
    TAKE_DIGIT = "take_digit"
    CONFIRM = "confirm"
    RUN = "run"
    CANCEL = "cancel"


@dataclass(frozen=True)
class ParkChoice:
    action: ParkAction
    entries: tuple[str, ...] = ()
    confirmed: bool = False


class ParkPrompt:
    def __init__(self) -> None:
        self.open()

    def open(self) -> None:
        self._entries: list[str] = []
        self._take_next = False
        self.confirming = False

    @property
    def entries(self) -> tuple[str, ...]:
        return tuple(self._entries)

    @property
    def shown(self) -> str:
        return " → ".join("▶" + entry[1:] if entry.startswith("w") else entry
                          for entry in self._entries)

    def handle(self, key: str) -> ParkChoice:
        accept = key in ("\r", "\n", " ", "p")
        if self.confirming:
            entries = self.entries
            self.open()
            return ParkChoice(ParkAction.RUN if accept else ParkAction.CANCEL,
                              entries if accept else (), confirmed=True)
        if key == "w":
            self._take_next = True
            return ParkChoice(ParkAction.TAKE_DIGIT)
        if key.isdigit():
            self._entries.append(("w" if self._take_next else "") + key)
            self._take_next = False
            return ParkChoice(ParkAction.UPDATE)
        if accept:
            if len(self._entries) >= 2:
                self.confirming = True
                return ParkChoice(ParkAction.CONFIRM)
            entries = self.entries or ("0",)
            self.open()
            return ParkChoice(ParkAction.RUN, entries)
        self.open()
        return ParkChoice(ParkAction.CANCEL)
