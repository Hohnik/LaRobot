"""Non-blocking single-key input, so a control loop can be steered live.

A 100 Hz loop cannot call `input()` — that blocks until Enter and the arm would
stop being commanded, which past 400 ms means the motors damp themselves. So the
terminal goes into cbreak mode and `select` polls stdin with a zero timeout:
keys are picked up between control cycles and cost nothing when none are pressed.

⚠️ Always use it as a context manager. It puts the terminal into a raw-ish mode,
and if the process exits without restoring, the user's shell is left without echo
or line editing — recoverable with `stty sane`, but rude.
"""

from __future__ import annotations

import select
import sys
import termios
import tty
from typing import Any


class KeyReader:
    """Reads single keypresses without blocking and without waiting for Enter."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._saved: Any = None
        self.enabled = False

    def __enter__(self) -> KeyReader:
        try:
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self.enabled = True
        except Exception:  # noqa: BLE001
            # Not a tty (piped, or run from a harness). Degrade to "no keys ever
            # pressed" rather than failing — the loop still runs, it just cannot
            # be steered interactively.
            self.enabled = False
        return self

    def __exit__(self, *exc: Any) -> None:
        if self._fd is not None and self._saved is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            except Exception:  # noqa: BLE001, S110
                pass

    def get(self) -> str | None:
        """Return one pending keypress, or None. Never blocks."""
        if not self.enabled:
            return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except Exception:  # noqa: BLE001
            return None
        return None

    def drain(self) -> list[str]:
        """Return every pending keypress. Never blocks."""
        keys = []
        while True:
            k = self.get()
            if k is None:
                return keys
            keys.append(k)
