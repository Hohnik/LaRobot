#!/usr/bin/env python3
"""Tests for `KeyReader`, on a real pty. No hardware.

    uv run scripts/test_keyboard.py

⛔ WHY THIS FILE EXISTS. These keys are `h` for HOLD, `q` for the quit-and-consent
flow, and "any key" to stop a park while the arm is moving. On 2026-08-12 it turned
out that typing three keys quickly delivered **one** and silently dropped the rest:
`select()` was asking the file descriptor whether data was waiting while
`sys.stdin.read(1)` was pulling it into Python's own buffer, where the descriptor
could not see it.

A burst of keys is exactly what someone produces when they want the arm to stop, so
that was the broken case. It also explains modes seeming to change by themselves —
a swallowed key surfaces later, beside input that had nothing to do with it.

A pty is used rather than a mock because the bug lived in the seam between two real
objects. A fake stdin would have "passed" while the arm still ate keystrokes.
"""

from __future__ import annotations

import os
import pty
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.inputs.keyboard import KeyReader  # noqa: E402


class FakeTerminal:
    """A real pty, with `sys.stdin` pointed at its slave end."""

    def __enter__(self):  # noqa: ANN204
        self.master, slave = pty.openpty()
        self._real_stdin = sys.stdin
        sys.stdin = os.fdopen(slave, "r")
        return self

    def type(self, text: str) -> None:
        os.write(self.master, text.encode())
        time.sleep(0.05)          # let the pty deliver

    def __exit__(self, *exc: object) -> None:
        sys.stdin.close()
        sys.stdin = self._real_stdin
        os.close(self.master)


def test_a_burst_of_keys_is_delivered_WHOLE() -> None:
    """⛔⭐ THE REGRESSION. Three keys typed together used to come back as one, and
    the other two were gone — not queued, gone. `drain()` promises every pending
    keypress; this is that promise, tested."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("pgh")
        assert keys.drain() == ["p", "g", "h"], "keys typed together were dropped"


def test_nothing_pending_returns_nothing_and_does_not_block() -> None:
    with FakeTerminal() as term, KeyReader() as keys:
        start = time.perf_counter()
        assert keys.drain() == []
        assert keys.get() is None
        assert time.perf_counter() - start < 0.5, "a non-blocking read blocked"
        term.type("x")
        assert keys.drain() == ["x"]


def test_keys_arriving_between_drains_are_not_lost() -> None:
    """The control loop drains once per cycle. Anything typed between two cycles
    must survive until the next one."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("a")
        assert keys.drain() == ["a"]
        term.type("bc")
        assert keys.drain() == ["b", "c"]
        assert keys.drain() == []


def test_a_long_burst_survives() -> None:
    """Someone hammering a key because the arm is doing something they dislike."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("q" * 32)
        assert keys.drain() == ["q"] * 32


def test_an_arrow_key_is_ONE_token_and_never_leaks_a_bracket() -> None:
    """⛔⭐ THIS TEST USED TO ASSERT THE BUG. It pinned `["\\x1b", "[", "A"]` and called
    it *"documented, not desired"* — but `[` is a **bound key** in
    `teleop_session.py`: it shrinks the gripper step, and the ease ramp while a run is
    being typed. So every ↑ silently halved a motion parameter and then printed
    `(key 'A' does nothing)`.

    ⚠️ Writing a defect into a test does not make it safe; it makes it permanent. The
    "documented, not desired" wording was the tell — a test recording behaviour nobody
    wanted, next to a comment explaining why it was fine."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("\x1b[A")
        assert keys.drain() == ["\x1b[A"]
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("\x1b[A\x1b[B\x1b[C\x1b[D\x1bOP")
        assert keys.drain() == ["\x1b[A", "\x1b[B", "\x1b[C", "\x1b[D", "\x1bOP"]
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("h\x1b[1;2At")
        assert keys.drain() == ["h", "\x1b[1;2A", "t"], "parameter bytes must be consumed"


def test_a_bare_escape_still_arrives_as_a_key() -> None:
    """Esc cancels a pending park, so it must not be swallowed while the reader waits to
    see whether a sequence follows."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("\x1b")
        assert keys.drain() == ["\x1b"]


def test_a_GERMAN_KEY_ARRIVES_AT_ALL() -> None:
    """⛔⭐ Julien asked for `ö`/`ä` instead of `[`/`]` because AltGr+8 and AltGr+9 are a
    three-finger chord on a QWERTZ layout. The reader could not have seen them: `ö` is
    two bytes (`0xC3 0xB6`), and `os.read(fd, 1)` decoded each half separately into a
    replacement character. **Two keys pressed, no key received, no error raised.**"""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("öä")
        assert keys.drain() == ["ö", "ä"], "a two-byte key must arrive as one key"
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("pöhät")
        assert keys.drain() == ["p", "ö", "h", "ä", "t"], "mixed with ASCII in one burst"


def test_a_multibyte_key_SPLIT_ACROSS_TWO_READS_survives() -> None:
    """⚠️ The reason the decoder is an instance attribute and not a local. The two bytes
    of `ö` can land in different reads; a fresh decoder per call would emit two
    replacement characters for a key that arrived intact."""
    with FakeTerminal() as term, KeyReader() as keys:
        os.write(term.master, b"\xc3")
        time.sleep(0.05)
        assert keys.drain() == [], "half a character is not a keypress"
        os.write(term.master, b"\xb6")
        time.sleep(0.05)
        assert keys.drain() == ["ö"], "the halves must be rejoined across drains"


def test_not_a_terminal_degrades_to_no_keys_rather_than_crashing() -> None:
    """Run from a harness or with input piped, the loop must still run — it just
    cannot be steered. ⛔ Crashing here would take down a session holding a raised
    arm because nobody was at the keyboard."""
    reader = KeyReader()
    with reader as keys:
        if keys.enabled:          # a real tty under the test runner; skip
            return
        assert keys.get() is None
        assert keys.drain() == []


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
