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
sys.path.insert(0, str(REPO / "src"))

from keyboard import KeyReader  # noqa: E402


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


def test_an_escape_sequence_arrives_as_its_bytes() -> None:
    """⚠️ Documented, not desired: an arrow key is ESC [ A, so a caller that treats
    bare ESC as an action will see one. Pre-existing and unchanged — recorded here so
    the next person meets it in a test rather than on the arm."""
    with FakeTerminal() as term, KeyReader() as keys:
        term.type("\x1b[A")
        assert keys.drain() == ["\x1b", "[", "A"]


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
