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

from yam.motion import EASINGS, Easing, easing_factor  # noqa: E402
from yam.ui.screen import CLEAR_LINE, StatusLine, display_width  # noqa: E402


def fresh(width: int = 80):  # noqa: ANN201
    buf = io.StringIO()
    return buf, StatusLine(stream=buf, width=width)


def rows_of(out: str) -> list[str]:
    """What each terminal ROW ends up holding, replaying the escape codes we emit.

    ⛔⭐ WHY A REPLAY AND NOT A SUBSTRING CHECK. The bug Julien hit was invisible to
    `assert "MODE: HOLD" in out` — the text WAS in the output. What was wrong was
    *where it landed*: on the status row, without clearing it, so the row read
    `⭐ MODE: GUIDE — arm is weightless°C  jaw   33°C  q [-0.49 …]`. Only a model of
    the screen can catch that, and the tests that missed it were asserting on the
    stream. Handles the four codes this module emits: `\\r`, `\\x1b[K`, `\\x1b[NA`
    and `\\n`.
    """
    grid: list[list[str]] = [[]]
    row, col = 0, 0
    i = 0
    while i < len(out):
        if out.startswith("\x1b[K", i):
            del grid[row][col:]
            i += 3
        elif out[i] == "\r":
            col = 0
            i += 1
        elif out[i] == "\n":
            row += 1
            col = 0
            while len(grid) <= row:
                grid.append([])
            i += 1
        elif out.startswith("\x1b[", i) and out.find("A", i) > 0:
            end = out.index("A", i)
            row = max(0, row - int(out[i + 2 : end] or "1"))
            col = 0
            i = end + 1
        else:
            while len(grid[row]) < col:
                grid[row].append(" ")
            if col < len(grid[row]):
                grid[row][col] = out[i]
            else:
                grid[row].append(out[i])
            col += 1
            i += 1
    return ["".join(r) for r in grid]


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


def test_a_MULTI_LINE_message_never_lands_on_the_status_row() -> None:
    """⛔⭐ THE BUG JULIEN REPORTED AS *"hold sometimes gets overwritten"*, 2026-08-12.

    Every mode banner in `teleop_session.py` is `print("\\n⭐ MODE: HOLD\\n")`, so the
    payload reaching `say()` carries newlines. It used to clear ONE row and then write
    the lot: line 2 landed on the status row without clearing it and overwrote only its
    first columns, which is why his paste reads
    `⭐ MODE: GUIDE — arm is weightless°C  jaw   33°C …`.
    """
    buf, screen = fresh()
    screen.set("[HOLD    ] t= 108.0s  hottest   40°C  jaw   33°C  q [-0.05  0.02]")
    screen.say("\n⭐ MODE: HOLD\n")
    rows = rows_of(buf.getvalue())
    banner = [r for r in rows if "MODE: HOLD" in r]
    assert banner, "the banner vanished entirely"
    assert banner[0].strip() == "⭐ MODE: HOLD", f"status text welded on: {banner[0]!r}"
    assert rows[-1].startswith("[HOLD"), "the status must be repainted last, intact"
    assert sum("hottest" in r for r in rows) == 1, "the status row exists twice"


def test_the_live_block_is_not_duplicated_when_messages_and_hints_interleave() -> None:
    """⛔⭐ THE "DUPLICATE PRINT". His paste has the same three rows twice, differing
    only in the timestamp (`t= 198.0s`, `t= 256.0s`) — 58 seconds apart, so not a
    copy-paste artefact. `_rows` had desynchronised from the real cursor by the number
    of newlines inside the messages, so `_rewind()` moved to the wrong row and the block
    was repainted where it had never been, leaving the earlier copy on screen."""
    buf, screen = fresh()
    screen.set("[HOLD    ] t= 198.0s  hottest   40°C")
    screen.hint("RUN 1 → 2 → 3 · speed 1.22 (-/+) · Enter=go")
    screen.say("\n⭐ MODE: PARK → 1 → 2 → 3\n")
    screen.set("[PARK    ] t= 200.0s  hottest   40°C")
    screen.say("  ⭐ slot 1 in 2.6s → next 2")
    screen.set("[PARK    ] t= 202.0s  hottest   41°C")
    rows = rows_of(buf.getvalue())
    live = [r for r in rows if r.startswith("[")]
    assert len(live) == 1, f"the status row survives more than once: {live}"
    assert live[0].startswith("[PARK    ] t= 202.0s"), "the surviving row is stale"
    assert sum("Enter=go" in r for r in rows) <= 1, "the run plan row was duplicated"


def test_two_arms_get_a_row_each_in_the_order_given() -> None:
    """⭐ ROADMAP §6.1 step 2: one status row per arm. Order is the caller's — the order
    the arms were named on `--arms` — so a row does not move between sessions."""
    buf, screen = fresh()
    screen.set_rows(["[B TELEOP] t=12.0s  hottest 44°C", "[G HOLD  ]          hottest 41°C"])
    rows = [r for r in rows_of(buf.getvalue()) if r.strip()]
    assert len(rows) == 2, f"expected one row per arm, got {rows}"
    assert rows[0].startswith("[B TELEOP]") and rows[1].startswith("[G HOLD")


def test_a_hint_stays_above_every_arm_row() -> None:
    """The hint is what the OPERATOR is doing; the arm rows are what the arms are doing."""
    buf, screen = fresh()
    screen.set_rows(["[B TELEOP] t=12.0s", "[G HOLD  ]"])
    screen.hint("RUN 1 → 2 · Enter=go")
    rows = [r for r in rows_of(buf.getvalue()) if r.strip()]
    assert rows[0].startswith("RUN 1"), f"the hint is not on top: {rows}"
    assert rows[-1].startswith("[G HOLD"), "the last arm must be the bottom row"


def test_dropping_from_two_rows_to_one_leaves_no_stale_row_behind() -> None:
    """⛔ The shrink case, which is how a dead arm's row would otherwise stay on screen
    reporting a temperature nobody is reading any more."""
    buf, screen = fresh()
    screen.set_rows(["[B TELEOP] t=12.0s  hottest 44°C", "[G HOLD  ]  hottest 41°C"])
    screen.set_rows(["[B TELEOP] t=13.0s  hottest 44°C"])
    rows = [r for r in rows_of(buf.getvalue()) if r.strip()]
    assert rows == ["[B TELEOP] t=13.0s  hottest 44°C"], f"a stale row survived: {rows}"


def test_one_row_behaves_exactly_as_the_single_line_it_replaced() -> None:
    """⚠️ The N=1 guarantee: `set` is `set_rows([text])`, so nothing about a one-arm
    session's display changed when the status became a list."""
    buf_a, screen_a = fresh()
    buf_b, screen_b = fresh()
    screen_a.set("[HOLD    ] t= 108.0s  hottest   40°C")
    screen_b.set_rows(["[HOLD    ] t= 108.0s  hottest   40°C"])
    assert buf_a.getvalue() == buf_b.getvalue()


def test_a_multi_line_message_over_TWO_arm_rows_keeps_the_block_intact() -> None:
    """⛔ The `say()` defect of 2026-08-12, re-derived against the multi-row block rather
    than assumed to still hold. Every banner in the session carries newlines, and with two
    arm rows there are two rows for the second line of a message to land on."""
    buf, screen = fresh()
    screen.set_rows(["[B TELEOP] t=12.0s  hottest 44°C", "[G HOLD  ]  hottest 41°C"])
    screen.say("\n⭐ MODE: HOLD\n")
    rows = rows_of(buf.getvalue())
    assert sum("hottest 44" in r for r in rows) == 1, "arm B's row exists twice"
    assert sum("hottest 41" in r for r in rows) == 1, "arm G's row exists twice"
    banner = [r for r in rows if "MODE: HOLD" in r]
    assert banner and banner[0].strip() == "⭐ MODE: HOLD", f"welded: {banner}"
    live = [r for r in rows if r.startswith("[")]
    assert len(live) == 2 and live[0].startswith("[B "), f"the block is not intact: {live}"


def test_the_startup_plan_is_NOT_truncated_because_nothing_is_live_yet() -> None:
    """⭐ The whole `--arm` plan and HELP go through `say()` before the loop starts.
    Truncating those would cut the information he reads to decide whether to pass
    `--yes` — and with no live block there is nothing for a wrap to desynchronise."""
    buf, screen = fresh(width=40)
    screen.say("  map scope   : SHARED — edits here affect BOTH arms")
    assert "affect BOTH arms" in buf.getvalue(), "the startup plan was truncated"


def test_a_message_longer_than_the_terminal_is_truncated_not_wrapped() -> None:
    """⚠️ A wrapped MESSAGE desynchronises the row count exactly as an embedded newline
    does — the class counts one row, the terminal consumes two. `say()` never truncated,
    and the PARK banner is ~110 characters wide."""
    buf, screen = fresh(width=60)
    screen.set("[PARK] t=1.0s")
    screen.say("⭐ MODE: PARK → slot 0, 0.98 rad of travel at 0.78 rad/s, "
               "corners smooth. Press h or t to stop.")
    for row in rows_of(buf.getvalue()):
        assert display_width(row) <= 60, f"row is {display_width(row)} columns: {row!r}"


def test_width_is_measured_in_COLUMNS_not_characters() -> None:
    """⛔ `len()` counts `⭐` as one; a terminal draws it as two. A line "fitted" to the
    width could therefore still wrap — and every line in this program has emoji in it."""
    assert display_width("⭐") == 2
    assert display_width("⛔") == 2
    assert display_width("⚠️") == 2, "VS16 makes it emoji-presentation, so two columns"
    assert display_width("abc") == 3
    assert display_width("→") == 1, "ambiguous-width, one column outside a CJK locale"
    buf, screen = fresh(width=10)
    screen.set("⭐⭐⭐⭐⭐⭐")
    painted = buf.getvalue().split(CLEAR_LINE)[-1]
    assert display_width(painted) <= 10, f"{display_width(painted)} columns: {painted!r}"


def test_an_empty_message_is_one_blank_row() -> None:
    """`print()` with no arguments is a real call site — it must not silently do nothing
    and must not consume two rows."""
    buf, screen = fresh()
    screen.set("[HOLD] t=1.0s")
    screen.say("")
    assert rows_of(buf.getvalue())[-1] == "[HOLD] t=1.0s"
    assert buf.getvalue().count("\n") == 1


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
