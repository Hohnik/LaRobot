#!/usr/bin/env python3
"""Tests for the status line, and for the easing profiles. No hardware.

    uv run scripts/test_screen.py

⛔ WHY. Julien changed the park speed six times while choosing a run and got six
copies of a two-line plan interleaved with six status lines: *"that seems to be more
of a bug."* The fix is a rule — one live line, always last; everything else scrolls
above — and a rule is worth pinning.
"""

from __future__ import annotations

import io
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from motion import EASINGS, Easing, easing_factor  # noqa: E402
from screen import CLEAR_LINE, StatusLine  # noqa: E402


def fresh(width: int = 80):  # noqa: ANN201
    buf = io.StringIO()
    return buf, StatusLine(stream=buf, width=width)


# ------------------------------------------------------- the status line ----


def test_repainting_the_status_never_adds_a_line() -> None:
    """⛔ THE COMPLAINT, as a test. Six knob changes must leave ONE line on screen,
    not six."""
    buf, screen = fresh()
    for speed in (0.40, 0.50, 0.62, 0.78, 0.98, 1.22):
        screen.set(f"RUN 1 → 2 → 3   speed {speed:.2f} rad/s")
    assert buf.getvalue().count("\n") == 0, "repainting the status scrolled the terminal"
    assert "1.22" in buf.getvalue()


def test_a_message_scrolls_above_and_the_status_is_repainted_under_it() -> None:
    buf, screen = fresh()
    screen.set("[TELEOP] t=12.0s")
    screen.say("⭐ MODE: PARK")
    out = buf.getvalue()
    assert out.count("\n") == 1, "a message should produce exactly one new row"
    assert out.endswith("[TELEOP] t=12.0s"), "the status must be repainted last"
    assert out.index("MODE: PARK") < out.rindex("[TELEOP]"), "the message must be above"


def test_every_write_starts_by_clearing_the_line() -> None:
    """Without the clear, a shorter status leaves the tail of the longer one behind —
    which is how a readout ends up saying `1.22 rad/ss`."""
    buf, screen = fresh()
    screen.set("a very long status line indeed")
    screen.set("short")
    assert buf.getvalue().endswith(CLEAR_LINE + "short")


def test_a_long_status_is_truncated_not_wrapped() -> None:
    """⚠️ A wrapped status scrolls the terminal by a row on every repaint, turning a
    stationary readout into a flickering waterfall."""
    buf, screen = fresh(width=40)
    screen.set("x" * 200)
    painted = buf.getvalue().split(CLEAR_LINE)[-1]
    assert len(painted) <= 40
    assert painted.endswith("…")


def test_a_newline_inside_a_status_cannot_break_the_layout() -> None:
    """Callers build status text with f-strings; one stray \\n would permanently
    desynchronise the live line from the cursor."""
    buf, screen = fresh()
    screen.set("line one\nline two")
    assert buf.getvalue().count("\n") == 0


def test_done_ends_the_line_only_if_something_was_on_it() -> None:
    buf, screen = fresh()
    screen.done()
    assert buf.getvalue() == "", "nothing was live, so nothing to end"
    screen.set("live")
    screen.done()
    assert buf.getvalue().endswith("\n")


# ------------------------------------------------------------- easing ----


RAMP = 0.2


def test_easing_none_runs_at_full_speed_from_the_first_step() -> None:
    """⛔ Julien on Ctrl-C: *"I want it to move into its parking position quickly and
    swiftly, without the excessive starting and pausing."* A shutdown move should
    leave immediately."""
    none = EASINGS[0]
    assert none.name == "none"
    assert easing_factor(none, 0.0, 1.0, RAMP) == 1.0
    assert easing_factor(none, 1.0, 0.0, RAMP) == 1.0


def test_ease_in_softens_only_the_start() -> None:
    e = Easing("in", True, False)
    assert easing_factor(e, 0.0, 1.0, RAMP) < 1.0
    assert easing_factor(e, 1.0, 0.0, RAMP) == 1.0, "the stop should stay hard"


def test_ease_out_softens_only_the_stop() -> None:
    e = Easing("out", False, True)
    assert easing_factor(e, 0.0, 1.0, RAMP) == 1.0, "the start should be immediate"
    assert easing_factor(e, 0.0, 0.0, RAMP) < 1.0


def test_the_s_curve_is_gentler_than_the_straight_ramp_in_the_early_part() -> None:
    """Smoothstep removes the corner in ACCELERATION too — the difference between
    "ease" and "ease with a Bézier handle" in an editor."""
    linear = Easing("both", True, True)
    scurve = Easing("s", True, True, smooth=True)
    assert (easing_factor(scurve, 0.05, 1.0, RAMP)
            < easing_factor(linear, 0.05, 1.0, RAMP))


def test_no_profile_ever_exceeds_full_speed() -> None:
    """⛔ The safety direction: scaling a step DOWN cannot overshoot, because
    advance_park_command already clamps to the distance remaining. Scaling UP would
    break that guarantee."""
    for e in EASINGS:
        for travelled in (0.0, 0.05, 0.5, 10.0):
            for remaining in (0.0, 0.05, 0.5, 10.0):
                assert easing_factor(e, travelled, remaining, RAMP) <= 1.0


def test_no_profile_ever_reaches_zero() -> None:
    """Without the floor the arm creeps for ever and the stall detector wrongly calls
    it an obstruction."""
    for e in EASINGS:
        assert easing_factor(e, 0.0, 0.0, RAMP) >= 0.15


def test_a_zero_ramp_disables_easing_for_every_profile() -> None:
    for e in EASINGS:
        assert easing_factor(e, 0.0, 1.0, 0.0) == 1.0


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
