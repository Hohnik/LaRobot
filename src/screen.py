"""⭐ One live status line at the bottom, messages scrolling above it.

    screen = StatusLine()
    screen.set("[TELEOP] t=12.0s  hottest 44°C")   # repaints in place
    screen.say("⭐ MODE: PARK")                     # scrolls above, status stays put

⛔ WHAT THIS FIXES. Julien, 2026-08-12, after changing the park speed six times while
choosing a run: *"can we make the print a bit nicer so that it only prints the things
multiple times where something actively changes … that seems to be more of a bug."*

He was right that it looked like a bug. The session had two kinds of output fighting
each other: a once-a-second status written with `\\r` and no newline, and ordinary
`print()` calls with newlines. Every message landed in the middle of a half-drawn
status line, and every knob change reprinted a whole two-line block — so tapping `+`
six times produced six copies of the plan interleaved with six status lines, and the
one thing he actually wanted to compare (the speed) was scattered down the screen.

**The rule this enforces: exactly one line is live, and it is always the last one.**
Anything transient — the status, the run plan being edited, park progress — is a
`set()` and repaints in place. Anything that is a record of something happening — a
mode change, a saved pose, a warning — is a `say()` and scrolls above.

⚠️ Deliberately not curses. This has to interleave with a 100 Hz control loop, with
`KeyReader` holding the terminal in cbreak mode, and with tracebacks from a vendor
SDK's threads. Two escape codes are auditable at 3am; a screen library that owns the
terminal is not, and it would fight the one thing that must never break — the arm's
loop continuing to run while the display misbehaves.
"""

from __future__ import annotations

import shutil
import sys
from typing import Any

CLEAR_LINE = "\r\x1b[K"


class StatusLine:
    """Owns the bottom **two** lines. Everything else scrolls above them.

    ⭐ TWO LINES, NOT ONE, AND JULIEN DIAGNOSED WHY HIMSELF: *"I can now see the line
    that it gets overwritten by is the normal line that always shows the seconds and
    the temperature… so maybe that could just be a line below, and the e cycling and
    the speed adjustment could be a line above when I'm still editing."*

    Exactly right. With one live line the once-a-second status and the thing being
    edited were fighting for the same row, so a knob change flashed up and vanished
    half a second later. They are different kinds of information and they need
    different rows:

    - **`status`** (bottom) — the heartbeat: time, temperatures, joint angles. Always
      there, always changing on its own.
    - **`hint`** (above it) — what *you* are doing right now: the run being typed, the
      speed you just changed, the ease profile you just cycled. Silent when there is
      nothing to say.
    """

    def __init__(self, stream: Any = None, width: int | None = None) -> None:
        self.stream = stream if stream is not None else sys.stdout
        self._width = width
        self._current = ""
        self._hint = ""
        self._rows = 0          # how many rows the live block currently occupies

    def width(self) -> int:
        if self._width is not None:
            return self._width
        try:
            return max(40, shutil.get_terminal_size(fallback=(100, 30)).columns - 1)
        except Exception:  # noqa: BLE001
            return 100

    def _fit(self, text: str) -> str:
        """⚠️ Truncate rather than wrap. A wrapped status line scrolls the terminal by
        a row every time it is redrawn, which turns a stationary readout into a
        flickering waterfall — the exact complaint this class exists to answer."""
        text = text.replace("\n", " ")
        limit = self.width()
        return text if len(text) <= limit else text[: limit - 1] + "…"

    def _rewind(self) -> str:
        """Escape sequence that puts the cursor back at the top of the live block."""
        return "\r" + (f"\x1b[{self._rows - 1}A" if self._rows > 1 else "")

    def _paint(self) -> None:
        rows = [r for r in (self._hint, self._current) if r]
        out = [self._rewind()]
        for i, row in enumerate(rows):
            out.append("\x1b[K" + self._fit(row))
            if i < len(rows) - 1:
                out.append("\n")
        # ⚠️ If the block just shrank, the old bottom row is still on screen. Wipe
        # every row the previous paint used, or a stale line survives underneath.
        for _ in range(max(0, self._rows - len(rows))):
            out.append("\n\x1b[K")
        if self._rows > len(rows):
            out.append(f"\x1b[{self._rows - len(rows)}A")
        self._rows = len(rows)
        self.stream.write("".join(out))
        self.stream.flush()

    def set(self, text: str) -> None:
        """Repaint the bottom line — the heartbeat. Cheap enough to call every cycle."""
        self._current = text
        self._paint()

    def hint(self, text: str = "") -> None:
        """Repaint the line above the status — what the operator is doing right now.

        ⭐ This is where a changed value goes. `linear speed → 0.188 m/s` printed as a
        message six times in a row is six rows of scrollback saying the same word;
        as a hint it is one row whose number changes, which is what Julien asked for.
        Pass `""` to clear it.
        """
        self._hint = text
        self._paint()

    def say(self, text: str = "") -> None:
        """Print a message ABOVE the live block, then repaint the block under it."""
        self.stream.write(self._rewind() + "\x1b[K" + text + "\n")
        for _ in range(max(0, self._rows - 1)):
            self.stream.write("\x1b[K\n")
        if self._rows > 1:
            self.stream.write(f"\x1b[{self._rows - 1}A")
        self._rows = 0
        self._paint()

    def clear(self) -> None:
        """Drop the live block entirely — used before long blocks of plain output."""
        self._current = self._hint = ""
        self._paint()

    def done(self) -> None:
        """End the live block so ordinary printing can resume on a fresh row."""
        if self._rows:
            self.stream.write("\n")
        self._current = self._hint = ""
        self._rows = 0
        self.stream.flush()
