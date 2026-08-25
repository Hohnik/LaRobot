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
import unicodedata
from typing import Any

CLEAR_LINE = "\r\x1b[K"

# ⚠️ VARIATION SELECTOR-16 turns a text symbol into an emoji, and terminals draw an
# emoji TWO columns wide. `⚠️` is `⚠` + VS16, so it is not one column, and the
# selector itself is not a column at all.
_VS16 = "️"
_ZERO_WIDTH = ("︎", "​", "‌", "‍")


def char_width(text: str, i: int) -> int:
    """Columns occupied by `text[i]`, given what follows it.

    ⛔ Needs the whole string because width is not a property of a character alone:
    `⚠` is one column and `⚠` + VS16 is two.
    """
    ch = text[i]
    if unicodedata.combining(ch) or ch == _VS16 or ch in _ZERO_WIDTH:
        return 0
    if unicodedata.east_asian_width(ch) in ("W", "F"):
        return 2
    # ⚠️ VS16 forces emoji presentation, which is two columns whatever the base
    # character's width class says. `⚠` (U+26A0) is `N`, *neutral* — not ambiguous —
    # so a rule keyed on `A` missed the single most common symbol in this codebase.
    # Measured, after asserting the wrong thing first.
    if i + 1 < len(text) and text[i + 1] == _VS16:
        return 2
    return 1


def display_width(text: str) -> int:
    """How many terminal COLUMNS `text` occupies. ⛔ Not `len()`.

    ⛔⭐ WHY THIS EXISTS. The truncation that stops the live block wrapping was
    measuring `len()`, and this repo's status lines are full of `⭐ ⛔ ⚠️ →` — every
    one of which Python counts as a single character and a terminal draws as one or
    two columns. So a line "fitted" to 99 columns could be 105 columns wide, wrap,
    and desynchronise the cursor arithmetic by a row. That is the same class of
    defect as measuring a claim with something that cannot distinguish it from its
    opposite (working contract rule 5).

    ⚠️ `east_asian_width` returns `A` (ambiguous) for `→ · ° ⚠` — genuinely
    terminal-dependent, one column in most, two in a CJK locale. Treated as one,
    except when VS16 forces emoji presentation. **So this can still undercount in a
    CJK-configured terminal**, which is why `width()` also keeps a spare column.
    """
    return sum(char_width(text, i) for i in range(len(text)))


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
        #: ⭐ A LIST, because the status is one row PER ARM since 2026-08-14. With one arm
        #: it holds one string and behaves exactly as the single `_current` it replaced.
        #: ⚠️ Empty strings are dropped when painting, so a blank row never eats a line.
        self._status: list[str] = []
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
        flickering waterfall — the exact complaint this class exists to answer.

        ⛔ Measured in COLUMNS, not characters — see `display_width`."""
        text = text.replace("\n", " ")
        limit = self.width()
        if display_width(text) <= limit:
            return text
        kept, width = [], 0
        for i in range(len(text)):
            w = char_width(text, i)
            if width + w > limit - 1:       # keep a column for the ellipsis
                break
            kept.append(text[i])
            width += w
        return "".join(kept) + "…"

    def _rewind(self) -> str:
        """Escape sequence that puts the cursor back at the top of the live block."""
        return "\r" + (f"\x1b[{self._rows - 1}A" if self._rows > 1 else "")

    def _paint(self) -> None:
        rows = [r for r in (self._hint, *self._status) if r]
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
        self.set_rows([text])

    def set_rows(self, rows: list[str]) -> None:
        """Repaint the heartbeat as SEVERAL rows — one per arm. Bottom-most is last.

        ⭐ WHY THIS EXISTS. Two arms have two modes, two temperatures and two poses, and
        cramming them into one row makes the row unreadable at exactly the moment there is
        most to read. ROADMAP §6.1 step 2 asks for one row per arm.

        ⚠️ The hint stays ABOVE all of them, because it is what the operator is doing
        rather than what an arm is doing.

        ⛔ Order is the caller's and is never sorted here. It is the order the arms were
        given on `--arms`, so the row an operator looks at does not move between sessions.
        """
        self._status = list(rows)
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
        """Print a message ABOVE the live block, then repaint the block under it.

        ⛔⭐ EVERY LINE OF THE MESSAGE GETS ITS OWN CLEARED ROW, AND THAT IS THE FIX
        FOR WHAT JULIEN SAW ON 2026-08-12. He reported *"hold sometimes gets
        overwritten"* and output that looked duplicated. Both were this function.

        It used to clear **one** row and then write `text` — but every caller in
        `teleop_session.py` writes `print("\\n⭐ MODE: HOLD\\n")`, because `print` is
        shadowed and those ~60 call sites were written for the builtin. So the payload
        carried newlines: the first line landed on the cleared row, and the SECOND
        line landed on the status row **without clearing it**, overwriting only its
        first few columns. That is the visible signature in his paste, on every
        message he sent back:

            ⭐ MODE: GUIDE — arm is weightless°C  jaw   33°C  q [-0.49 …]
            ⭐ MODE: TELEOP — SpaceMouse drivesC  jaw   33°C  q [-0.49 …]
              run cancelled.6.0s  hottest   39°C

        The tail after each message is the surviving right-hand end of the status row.
        ⭐ **And `⭐ MODE: HOLD` is the shortest banner in the program**, so it
        overwrote the least and looked, correctly, like the one that got eaten.

        The worse half is invisible: `_rows` was then wrong by the number of embedded
        newlines, so the next `_rewind()` moved the cursor to the wrong row and the
        live block was repainted somewhere it had not been — leaving the previous copy
        on screen. **That is the "duplicate print": two copies of the same block whose
        only difference is the timestamp**, which is exactly what he pasted
        (`t= 198.0s` and `t= 256.0s`, 58 seconds apart, identical otherwise).

        ⚠️ The test that was supposed to cover this passed a SINGLE-LINE message —
        `say("⭐ MODE: PARK")` — which no call site in the program produces. A test
        written against an interface rather than against its callers is the same
        defect as a guard never re-derived against the thing it guards (working
        contract rule 7). The tests below now use the real payload shape.
        """
        # ⭐ TRUNCATE ONLY WHEN THERE IS A LIVE BLOCK TO PROTECT. Wrapping is what
        # desynchronises the row count — but only because the next `_rewind()` has to
        # move back over a known number of rows. With nothing live there is nothing to
        # rewind over, so a long line may wrap harmlessly.
        #
        # ⚠️ This is not a micro-optimisation: the whole startup plan (`--arm`, the axis
        # map, the workspace box, HELP) is printed through this before the loop begins,
        # and truncating THAT would silently cut the information the operator is reading
        # to decide whether to press `--yes`. Losing a column of a temperature readout
        # is cosmetic; losing the end of "map scope: SHARED — edits affect BOTH arms" is
        # not.
        live = self._rows > 0 or any(self._status) or bool(self._hint)
        lines = [self._fit(line) if live else line for line in text.split("\n")]
        out = [self._rewind()]
        for line in lines:
            out.append("\x1b[K" + line + "\n")
        # If the message is SHORTER than the block it displaced, the block's lower
        # rows still hold stale text. Wipe exactly those, then come back.
        stale = max(0, self._rows - len(lines))
        for _ in range(stale):
            out.append("\x1b[K\n")
        if stale:
            out.append(f"\x1b[{stale}A")
        self._rows = 0
        self.stream.write("".join(out))
        self._paint()

    def clear(self) -> None:
        """Drop the live block entirely — used before long blocks of plain output."""
        self._status, self._hint = [], ""
        self._paint()

    def done(self) -> None:
        """End the live block so ordinary printing can resume on a fresh row."""
        if self._rows:
            self.stream.write("\n")
        self._status, self._hint = [], ""
        self._rows = 0
        self.stream.flush()
