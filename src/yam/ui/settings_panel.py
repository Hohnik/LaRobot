"""Live-settings selection and key handling, independent of device ownership.

The caller applies values to active devices, renders their status and persists
settings. A mode key closes the panel without also requesting a mode change.
"""
from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Callable, Collection, Mapping

from yam.settings import LIVE_ORDER, adjust, live_lines, one_line


class SettingsAction(Enum):
    STAY = "stay"
    CLOSE = "close"
    QUIT = "quit"


class SettingsPanel:
    def __init__(self, *, values: Callable[[], Mapping[str, float]],
                 apply: Callable[[str, float], None], save: Callable[[], None],
                 show_status: Callable[[], None], emit: Callable[[str], None],
                 builtin: Mapping[str, float], settings_file: Path,
                 mode_keys: Collection[str]) -> None:
        self._values, self._apply, self._save = values, apply, save
        self._show_status, self._emit = show_status, emit
        self._builtin, self._initial = dict(builtin), dict(values())
        self._settings_file, self._mode_keys = settings_file, frozenset(mode_keys)
        self.selected = LIVE_ORDER[0]

    def show(self) -> None:
        for line in live_lines(self._values(), self.selected, self._builtin):
            self._emit(line)

    def _show_selected(self, *, before: float | None = None) -> None:
        self._emit(one_line(self.selected, float(self._values()[self.selected]),
                            before=before, builtin=self._builtin))

    def handle(self, key: str) -> SettingsAction:
        if key in ("\x1b[A", "\x1b[B"):
            step = -1 if key == "\x1b[A" else 1
            self.selected = LIVE_ORDER[(LIVE_ORDER.index(self.selected) + step) % len(LIVE_ORDER)]
            self._show_selected()
        elif key == "n":
            self._emit("\n  ⭐ SETTINGS closed. The values are live; press n then s "
                       "to write them to the file.\n")
            return SettingsAction.CLOSE
        elif key in "123456789":
            index = int(key) - 1
            if index < len(LIVE_ORDER):
                self.selected = LIVE_ORDER[index]
            self._show_selected()
        elif key in ("+", "=", "-"):
            before = float(self._values()[self.selected])
            self._apply(self.selected, adjust(self.selected, before, key != "-"))
            self._show_selected(before=before)
            self._show_status()
        elif key == "0":
            for name, value in self._initial.items():
                self._apply(name, value)
            self._emit("\n  ⭐ back to the values this session started with.\n")
            self.show()
        elif key == "s":
            self._save()  # Failure must propagate; never announce a save that failed.
            self._emit(f"\n  ⭐ SAVED to {self._settings_file.parent.name}/"
                       f"{self._settings_file.name}. Every later session starts with these.\n")
            return SettingsAction.CLOSE
        elif key in self._mode_keys or key in ("\r", "\n", " "):
            self._emit("\n  ⭐ leaving SETTINGS. The values are live; nothing was "
                       "written to the file.\n     Press n then s to make them "
                       "permanent. Press the mode key again to change mode.\n")
            return SettingsAction.CLOSE
        elif key == "q":
            self._emit("\n  ⭐ SETTINGS closed — over to the quit menu. The values "
                       "stay live; they were not saved.\n")
            return SettingsAction.QUIT
        elif key == "?":
            self.show()
        else:
            shown = {"\x1b[C": "right arrow", "\x1b[D": "left arrow"}.get(
                key, repr(key) if key.isprintable() else "that key")
            self._emit(f"\n  ({shown} does nothing here — 1-9 or up/down to pick, "
                       "-/+ to change, 0 revert, s save, q quit, n or t/g/h to leave)\n")
        return SettingsAction.STAY
