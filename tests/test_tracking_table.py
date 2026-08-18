#!/usr/bin/env python3
"""Tests for the playback tracking table, `flat_joint_names` and `tracking_table`.

    uv run tests/test_tracking_table.py

⭐⭐ WHY THIS FILE EXISTS. Julien's 2026-08-17 two-arm playback printed a tracking table
with **six named rows and six rows labelled just "joint"**, for a recording of **14
joints**. Two rows were missing and nothing said so, and no row said which arm it belonged
to — so the six named rows read as "the arm" when they were only arm B.

⛔ Both faults were invisible with one arm, which is why nothing caught them for four days:
at N=1 the flat sample index and the per-arm joint index are the same number. That is the
"correct at N=1 by construction" signature this project now watches for.

⚠️ The correct code already existed **eight lines below** the wrong code, building per-arm
names for the saved JSON, with a comment explaining exactly why it was necessary. Two
copies of one expression is what let them drift, so there is now one function and this
file tests it.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "apps"))  # ⛔ the app script is not a package; a test OF it imports it as a file
sys.path.insert(0, str(REPO / "scripts"))

from teleop_session import flat_joint_names, tracking_table  # noqa: E402


def rows_from(top_speeds: list[float]) -> list:
    """A `TrackingLog.rows()` shape with the given top speed per joint."""
    return [(i, 0.05, 0.3, top, 0.04) for i, top in enumerate(top_speeds)]


# ------------------------------------------------------------------------- the names


def test_two_arms_get_FOURTEEN_names_and_every_one_says_which_arm() -> None:
    names = flat_joint_names(["B", "G"], 7, 14)
    assert len(names) == 14, f"expected 14 names, got {len(names)}"
    assert all(n.startswith("B ") for n in names[:7]), names[:7]
    assert all(n.startswith("G ") for n in names[7:]), names[7:]


def test_the_SECOND_arm_s_joints_are_NAMED_not_called_joint() -> None:
    """⛔⭐⭐ THE EXACT DEFECT FROM HIS LOG. `YAM_JOINTS` holds keys 1-7. The old code did
    `YAM_JOINTS.get(flat_index + 1)`, so arm G's flat indices 7-13 became keys 8-14, missed,
    and fell back to "joint" for all seven."""
    names = flat_joint_names(["B", "G"], 7, 14)
    assert names[7] == "G base_yaw", f"arm G's first joint is named {names[7]!r}"
    assert names[12] == "G gripper_twist", f"got {names[12]!r}"
    assert names[13] == "G gripper_jaws", f"got {names[13]!r}"
    bare = [n for n in names if n.endswith(" joint")]
    assert not bare, f"these rows are still anonymous: {bare}"


def test_both_arms_use_the_SAME_joint_names() -> None:
    """⭐ The arms are identical hardware, so joint 3 is `elbow_pitch` on both. Only the
    prefix distinguishes them."""
    names = flat_joint_names(["B", "G"], 7, 14)
    assert names[2] == "B elbow_pitch" and names[9] == "G elbow_pitch"


def test_one_arm_still_works_and_is_still_prefixed() -> None:
    """⚠️ The one-arm case must not regress, and it should still say which arm — a table
    that names the arm only when there are two is a table whose format changes under you."""
    names = flat_joint_names(["B"], 7, 7)
    assert len(names) == 7 and names[0] == "B base_yaw" and names[6] == "B gripper_jaws"


def test_the_total_clips_the_list() -> None:
    assert len(flat_joint_names(["B", "G"], 7, 12)) == 12


def test_three_arms_would_also_work() -> None:
    """⭐ Nothing here assumes two. The restructure's whole point was N arms."""
    names = flat_joint_names(["B", "G", "X"], 7, 21)
    assert names[14] == "X base_yaw" and len(names) == 21


# ------------------------------------------------------------------------- the table


def test_HIS_LOG_reproduced__12_rows_shown_and_the_2_missing_ones_are_NAMED() -> None:
    """⛔⭐⭐ THE WHOLE POINT. His recording had 14 joints and the table printed 12 rows
    with no explanation. Both grippers had barely moved, because he never touched them.

    ⚠️ Leaving them out is correct: a joint that did not move says nothing about tracking.
    **Saying nothing about leaving them out is not**, because a reader counting rows would
    conclude the recorder had lost two joints."""
    speeds = [0.56, 0.36, 0.62, 0.49, 0.45, 0.24, 0.0,      # arm B, gripper still
              0.22, 0.48, 0.37, 0.23, 0.25, 0.31, 0.0]      # arm G, gripper still
    lines = tracking_table(rows_from(speeds), flat_joint_names(["B", "G"], 7, 14), 0.15)

    shown = [ln for ln in lines if "worst lag" in ln]
    assert len(shown) == 12, f"expected 12 rows of data, got {len(shown)}"

    note = [ln for ln in lines if "too slow to rate" in ln]
    assert len(note) == 1, f"the missing joints are not reported: {lines[-1]!r}"
    assert "2 of 14" in note[0], f"the note miscounts: {note[0]!r}"
    assert "B gripper_jaws" in note[0] and "G gripper_jaws" in note[0], (
        f"the note does not name which joints are missing: {note[0]!r}")


def test_every_shown_row_says_which_arm() -> None:
    speeds = [0.5] * 14
    lines = tracking_table(rows_from(speeds), flat_joint_names(["B", "G"], 7, 14), 0.15)
    shown = [ln for ln in lines if "worst lag" in ln]
    assert len(shown) == 14
    assert sum(1 for ln in shown if ln.strip().startswith("B ")) == 7
    assert sum(1 for ln in shown if ln.strip().startswith("G ")) == 7


def test_no_note_when_every_joint_moved() -> None:
    """⚠️ The note must not appear when there is nothing to report, or it becomes noise
    that gets skipped, and then it will be skipped on the run where it matters."""
    lines = tracking_table(rows_from([0.5] * 14),
                           flat_joint_names(["B", "G"], 7, 14), 0.15)
    assert not [ln for ln in lines if "too slow to rate" in ln]


def test_all_joints_still_gives_a_readable_answer_rather_than_an_empty_table() -> None:
    """⚠️ A playback where nothing moved should say so, not print a bare heading."""
    lines = tracking_table(rows_from([0.0] * 14),
                           flat_joint_names(["B", "G"], 7, 14), 0.15)
    assert not [ln for ln in lines if "worst lag" in ln]
    assert "ALL 14 joints" in lines[-1], lines[-1]
    assert "nothing actually moved" in lines[-1], lines[-1]


def test_the_note_STAYS_SHORT_because_the_screen_painter_truncates_it() -> None:
    """⛔⭐⭐ FOUND BY ACTUALLY RUNNING A `--sim` SESSION, and unfindable by reading.

    The first version listed every unmoved joint. In a simulated playback where nothing
    moved, that was all fourteen names on one line, and `src/yam/ui/screen.py`'s painter cut it off
    with an ellipsis: the log shows `B base_yaw, B shoulder_pit…` and nothing more. **A note
    whose entire job is to say which joints are missing, truncated before it says so, is
    worse than no note, because it looks answered.**

    ⚠️ So the length is now part of the contract, not an accident of how many joints
    happened to be still.
    """
    # The all-still case says so instead of listing anything.
    everything = tracking_table(rows_from([0.0] * 14),
                                flat_joint_names(["B", "G"], 7, 14), 0.15)
    assert len(everything[-1]) < 110, f"{len(everything[-1])} chars: {everything[-1]!r}"
    assert "base_yaw" not in everything[-1], "it is still listing names"

    # A middling case caps the list and counts the rest.
    speeds = [0.5, 0.5, 0.5] + [0.0] * 11
    partial = tracking_table(rows_from(speeds),
                             flat_joint_names(["B", "G"], 7, 14), 0.15)
    note = partial[-1]
    assert len(note) < 110, f"{len(note)} chars, still too long: {note!r}"
    assert "+9 more" in note, note
    assert "11 of 14" in note, note


def test_a_short_list_of_missing_joints_is_shown_IN_FULL() -> None:
    """⭐ His real case: two grippers still out of fourteen. Both must be named, because
    capping is a defence against length, not a reason to withhold a two-item list."""
    speeds = [0.5] * 6 + [0.0] + [0.5] * 6 + [0.0]
    note = tracking_table(rows_from(speeds),
                          flat_joint_names(["B", "G"], 7, 14), 0.15)[-1]
    assert "B gripper_jaws" in note and "G gripper_jaws" in note, note
    assert "more" not in note, f"a 2-item list should not be capped: {note!r}"
    assert len(note) < 110, f"{len(note)} chars: {note!r}"


def test_a_short_name_list_does_not_crash_the_table() -> None:
    """⚠️ Defensive: if the layout and the tracking log ever disagree on length, a
    diagnostic must not be the thing that raises. The arm is in HOLD when this prints."""
    lines = tracking_table(rows_from([0.5] * 14), ["B base_yaw"], 0.15)
    assert len(lines) == 14
    assert "joint 13" in lines[13], lines[13]


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
