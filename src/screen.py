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
    """Owns the bottom line. Everything else scrolls above it."""

    def __init__(self, stream: Any = None, width: int | None = None) -> None:
        self.stream = stream if stream is not None else sys.stdout
        self._width = width
        self._current = ""

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

    def set(self, text: str) -> None:
        """Repaint the live line in place. Cheap enough to call every cycle."""
        self._current = text
        self.stream.write(CLEAR_LINE + self._fit(text))
        self.stream.flush()

    def say(self, text: str = "") -> None:
        """Print a message ABOVE the live line, then repaint the live line under it."""
        self.stream.write(CLEAR_LINE + text + "\n")
        if self._current:
            self.stream.write(self._fit(self._current))
        self.stream.flush()

    def clear(self) -> None:
        """Drop the live line entirely — used before long blocks of plain output."""
        self._current = ""
        self.stream.write(CLEAR_LINE)
        self.stream.flush()

    def done(self) -> None:
        """End the live line so ordinary printing can resume on a fresh row."""
        if self._current:
            self.stream.write("\n")
        self._current = ""
        self.stream.flush()
