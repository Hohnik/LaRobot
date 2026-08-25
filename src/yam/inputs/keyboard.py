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

import codecs
import os
import select
import sys
import termios
import tty
from typing import Any

# CSI/SS3 final bytes: an escape sequence ends at the first byte in this range.
_FINAL_LO, _FINAL_HI = "\x40", "\x7e"


def tokenise(text: str) -> list[str]:
    """Split decoded terminal input into ONE STRING PER KEYPRESS.

    ⛔⭐ WHY THIS IS NOT JUST `list(text)`, AND IT MATTERS ON THIS RIG. An arrow key
    sends three characters — `ESC` `[` `A` — and in `teleop_session.py` **`[` is a
    bound key**: it shrinks the gripper step in the drive modes and shortens the ease
    ramp while a run is being typed. So pressing ↑ silently changed a motion
    parameter and then printed `(key 'A' does nothing)`, which reads as the program
    being confused rather than as the operator having pressed an unsupported key.

    That is this repo's signature failure mode — a confident, plausible, wrong
    result with no exception raised (working contract rule 6). An escape sequence now
    comes back as a single non-printable token, which the session's
    `k.isprintable()` filter drops in silence, as it should.

    ⚠️ A lone `ESC` (the Escape key) is returned as `"\\x1b"`, unchanged: callers
    that treat it as "cancel" keep working. A truncated sequence is returned whole
    rather than split, so `[` can never leak out of one.
    """
    tokens: list[str] = []
    i = 0
    while i < len(text):
        if text[i] == "\x1b" and i + 1 < len(text) and text[i + 1] in "[O":
            j = i + 2
            while j < len(text) and not (_FINAL_LO <= text[j] <= _FINAL_HI):
                j += 1
            tokens.append(text[i : j + 1])
            i = j + 1
            continue
        tokens.append(text[i])
        i += 1
    return tokens


class KeyReader:
    """Reads single keypresses without blocking and without waiting for Enter."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._saved: Any = None
        self.enabled = False
        # ⭐ A UTF-8 decoder that SURVIVES BETWEEN CALLS, because a non-ASCII key is
        # more than one byte and the two halves can arrive in different reads.
        self._decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        self._queue: list[str] = []

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

    def _refill(self) -> None:
        """Pull everything the descriptor holds, decode it, tokenise it. Never blocks.

        ⛔⭐ ONE BYTE AT A TIME COULD NOT SEE A GERMAN KEY, AND THAT IS WHY THIS
        CHANGED. Julien, 2026-08-12, on the ease-ramp keys: *"I don't like the fact
        that the brackets are used because I have a German keyboard, and they're
        awkward to reach. Maybe ä and ö could be used."*

        `ö` is **two bytes** in UTF-8 (`0xC3 0xB6`) and `ä` is `0xC3 0xA4`. The old
        reader did `os.read(fd, 1)` and then `bytes.decode(errors="replace")` on that
        single byte — so each half decoded independently to `U+FFFD`, and **pressing
        `ö` produced two replacement characters and no key at all.** Not a mapping
        that was missing: a key the program was structurally incapable of seeing.

        ⚠️ The incremental decoder is an instance attribute rather than a local for a
        specific reason: a two-byte character can be **split across two reads**, so a
        decoder created per call would emit two replacement characters for a key that
        arrived intact. It has to remember the half-finished sequence between calls,
        which is exactly what an incremental decoder is for.
        """
        if not self.enabled or self._fd is None:
            return
        chunks: list[bytes] = []
        try:
            while select.select([self._fd], [], [], 0)[0]:
                data = os.read(self._fd, 64)
                if not data:
                    break
                chunks.append(data)
        except Exception:  # noqa: BLE001
            pass
        if chunks:
            self._queue.extend(tokenise(self._decoder.decode(b"".join(chunks))))

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
        self._refill()
        return self._queue.pop(0) if self._queue else None

    def drain(self) -> list[str]:
        """Return every pending keypress, oldest first. Never blocks.

        ⭐ One entry per KEYPRESS, so a non-ASCII key (`ö`, `ä`) is one token and an
        arrow key is one token rather than three — see `tokenise` for why the second
        of those was quietly changing a motion parameter.
        """
        self._refill()
        keys, self._queue = self._queue, []
        return keys
