#!/usr/bin/env python3
"""Tests for the SAVE-prompt decision, `save_slot_action`. No hardware.

    uv run scripts/test_save_slot.py

⭐⭐ WHY THIS FILE EXISTS. This decision is what stands between minutes of Julien's
hand-guided work and an empty folder, and it had **no test at all** until 2026-08-17. Every
change to it was verified by him recording something on live hardware and pressing keys.

⛔ It also had a real dead end, which he found the first time the guard ever fired for him:
*"when I pressed save seven, then the guard came up. But then when I pressed a different
number, I wanted to save it on, it's still discarded."* The old rule was "the same digit
confirms, anything else discards", so the only exits were **overwrite the take you are
protecting** or **lose the new recording**. The obvious third thing a person wants — put it
somewhere else — was the one thing it would not do.

⚠️ The asymmetry that makes this worth testing: an overwrite destroys old work, a discard
destroys new work, and **a recording cannot be re-taken identically**. So both outcomes must
be deliberate acts, and neither may be the default result of aiming at a busy slot.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

from teleop_session import save_slot_action  # noqa: E402

BUSY = "5.2s on B,G, live:B:teleop+G:teleop, recorded 2026-08-17T12:11"


def test_a_digit_on_a_FREE_slot_saves_immediately() -> None:
    """⭐ No confirmation for a free slot. A prompt that asks when nothing is at risk trains
    the reader to hit the key twice without looking, which is how the guard stops working."""
    assert save_slot_action("8", True, False, None, None) == ("save", "8")


def test_a_digit_on_an_OCCUPIED_slot_asks_first() -> None:
    """⛔ Five separate overwrites destroyed the only copy of something before this existed
    (docs/FINDINGS.md §33.2, §34.7)."""
    action = save_slot_action("7", True, False, None, BUSY)
    assert action[0] == "ask" and action[1] == "7"
    assert action[2] is False, "nothing was being asked before, so this is not a re-aim"


def test_the_SAME_digit_again_replaces() -> None:
    assert save_slot_action("7", True, True, "7", BUSY) == ("save", "7")


def test_a_DIFFERENT_digit_while_asking_RE_AIMS_at_a_free_slot() -> None:
    """⛔⭐⭐ THE EXACT THING JULIEN ASKED FOR. He pressed 7, the guard fired, he pressed 8
    meaning "put it there instead", and the old code threw the recording away."""
    assert save_slot_action("8", True, True, "7", None) == ("save", "8")


def test_a_DIFFERENT_digit_while_asking_ASKS_AGAIN_if_that_slot_is_also_busy() -> None:
    """⭐ Re-aiming must not become a way to overwrite by accident. Slot 8 being occupied too
    means the guard fires again, for 8 this time."""
    action = save_slot_action("8", True, True, "7", BUSY)
    assert action[0] == "ask" and action[1] == "8"
    assert action[2] is True, "the message should say it re-aimed"


def test_re_aiming_TWICE_still_works() -> None:
    """⚠️ There is no limit on how many times he may change his mind, and each step keeps the
    recording alive."""
    a = save_slot_action("8", True, True, "7", BUSY)      # 7 -> 8, also busy
    assert a[0] == "ask" and a[1] == "8"
    b = save_slot_action("9", True, True, "8", None)      # 8 -> 9, free
    assert b == ("save", "9")


def test_a_NON_digit_discards_and_says_what_was_KEPT() -> None:
    """⭐ The only way to discard is a deliberate non-digit. When a slot was being protected,
    the caller is told which one survived, so "discarded" is never ambiguous."""
    assert save_slot_action("x", True, True, "7", BUSY) == ("discard", "7")


def test_a_NON_digit_with_nothing_being_protected_just_discards() -> None:
    assert save_slot_action("x", True, False, None, None) == ("discard", None)


def test_ENTER_and_SPACE_discard_rather_than_confirming() -> None:
    """⛔ Neither is a digit, so neither can silently overwrite. ⚠️ This matters because
    Enter CONFIRMS in the `l` and `p` prompts, so a hand trained on those would reach for it
    here — and here it must destroy the new take rather than the old one."""
    for key in ("\\r", "\\n", " "):
        assert save_slot_action(key, True, True, "7", BUSY) == ("discard", "7")


def test_no_recording_to_save_can_never_write_anything() -> None:
    """⛔ `has_take=False` means the recording was already saved or discarded. A stray digit
    then must not write, and must not resurrect anything."""
    assert save_slot_action("8", False, False, None, None) == ("discard", None)
    assert save_slot_action("7", False, True, "7", BUSY) == ("discard", "7")


def test_slot_0_is_a_normal_recording_slot_here() -> None:
    """⚠️ `0` is the BASE POSE slot for the `s` key, which is a different thing entirely.
    For recordings the prompt says 0-9 and 0 is an ordinary slot, so it must not be special
    cased into a refusal."""
    assert save_slot_action("0", True, False, None, None) == ("save", "0")
    assert save_slot_action("0", True, False, None, BUSY)[0] == "ask"


def test_every_digit_behaves_the_same_way() -> None:
    for d in "0123456789":
        assert save_slot_action(d, True, False, None, None) == ("save", d)
        assert save_slot_action(d, True, False, None, BUSY)[0] == "ask"


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
