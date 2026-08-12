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

import os
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
        """Return one pending keypress, or None. Never blocks.

        ⛔⭐ THE BUG THIS FIXES, AND IT SWALLOWED KEYS THAT STOP THE ARM.

        This used to `select()` on `sys.stdin` and then call `sys.stdin.read(1)`.
        Those two do not see the same thing. `sys.stdin` is a `TextIOWrapper` over a
        `BufferedReader`: `read(1)` returns one character but the buffer underneath
        it pulls **everything available** off the file descriptor first. `select()`
        then asks the *descriptor* whether data is waiting, the descriptor says no —
        and the remaining keys sit in Python's buffer, invisible, until the next
        keystroke happens to arrive.

        Reproduced on a pty on 2026-08-12: type `p`, `g`, `h` together and
        `drain()` — whose docstring promises *every* pending keypress — returns
        `['p']`. The other two are **lost**, not delayed.

        ⚠️ Why this matters more here than in most programs: these keys are `h` for
        HOLD, `q` for the quit-and-consent flow, and "any key" to stop a park in
        motion. A burst of keys is exactly what someone produces when they want the
        arm to stop, and it is exactly the case that was broken. It also explains
        modes appearing to change by themselves, which Julien saw and described as
        the session *"kind of going back into a mode"*: a swallowed key surfaces
        later, next to input that had nothing to do with it.

        The fix is to read the descriptor directly, so `select` and the read are
        talking about the same buffer.
        """
        if not self.enabled or self._fd is None:
            return None
        try:
            if select.select([self._fd], [], [], 0)[0]:
                data = os.read(self._fd, 1)
                return data.decode("utf-8", errors="replace") if data else None
        except Exception:  # noqa: BLE001
            return None
        return None

    def drain(self) -> list[str]:
        """Return every pending keypress, oldest first. Never blocks.

        ⚠️ An escape sequence arrives as its bytes — an arrow key is `ESC`, `[`, `A`
        — so a caller that treats bare `ESC` as an action will see one. That is
        pre-existing and unchanged; it is noted here because this function now
        genuinely returns everything, where before the tail was being eaten.
        """
        keys = []
        while True:
            k = self.get()
            if k is None:
                return keys
            keys.append(k)
