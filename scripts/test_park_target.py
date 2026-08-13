#!/usr/bin/env python3
"""Tests for `yam_robot.park_target_from()`. No hardware, no device.

    uv run scripts/test_park_target.py

⛔ WHY THIS FILE EXISTS. The bug it covers could only be found by reading, and its
consequence was the arm being **released while raised**: a saved 7-joint park pose
against a 6-DoF `--no-gripper` robot raised `ValueError` inside the control loop,
which escaped past the "the arm is HOLDING, press g or d" consent flow and landed
in `finally`, where the motors are disabled. FINDINGS §0's rule is that this stack
fails by lying rather than crashing; this one crashed, and the crash *was* the
hazard. A fix for that should not itself rest on my reading being right.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))

from yam_robot import (  # noqa: E402
    advance_park_command,
    check_grasp,
    park_slots,
    park_speed_factor,
    park_target_from,
    park_verdict,
    resolve_park_legs,
    with_park_slot,
)

N_ARM = 6
GRIPPER_MIN, GRIPPER_MAX = 0.02, 0.98


def clamp(v: float) -> float:
    return float(np.clip(v, GRIPPER_MIN, GRIPPER_MAX))


# The real saved pose, 2026-08-10.
REAL_PARK = [0.8932249942778672, 1.3876173037308313, 0.7917524986648363,
             0.8031967650873586, 0.7684824902723744, -1.2911039902342285,
             0.03661992298190788]


def test_matching_lengths_reproduce_the_saved_pose() -> None:
    measured = np.zeros(7)
    target, warn = park_target_from(measured, REAL_PARK, N_ARM, clamp)
    assert warn is None, warn
    assert np.allclose(target[:N_ARM], REAL_PARK[:N_ARM])
    assert len(target) == 7


def test_seven_joint_pose_on_a_six_dof_robot_does_not_raise() -> None:
    """⭐ The crash. `--no-gripper` gives 6 DoF; the saved pose has 7."""
    measured = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    target, warn = park_target_from(measured, REAL_PARK, N_ARM, clamp)
    assert len(target) == 6, target
    assert np.allclose(target, REAL_PARK[:6])
    assert warn is not None and "7 joints" in warn and "6" in warn, warn
    # And the thing that actually broke must now work: a subtraction.
    delta = target - measured
    assert delta.shape == (6,)


def test_six_joint_pose_on_a_seven_dof_robot_keeps_the_measured_gripper() -> None:
    """The mirror case: a pose saved in a --no-gripper session, replayed with the
    gripper enabled. The jaw target must come from reality, never be invented."""
    measured = np.array([0.0] * 6 + [0.44])
    saved = REAL_PARK[:6]
    target, warn = park_target_from(measured, saved, N_ARM, clamp)
    assert len(target) == 7
    assert np.allclose(target[:6], saved)
    assert np.isclose(target[6], 0.44), "the measured jaw position must be preserved"
    assert warn is not None and "6 joints" in warn, warn


def test_gripper_is_clamped_off_the_stops() -> None:
    """PARK used to bypass the clamp, so a pose saved with the jaws on a stop drove
    them back onto it and held there — the stall that cooked motor 7."""
    for jaw, expect in ((0.0, GRIPPER_MIN), (1.0, GRIPPER_MAX), (-0.3, GRIPPER_MIN), (1.4, GRIPPER_MAX)):
        measured = np.zeros(7)
        saved = [0.0] * 6 + [jaw]
        target, _ = park_target_from(measured, saved, N_ARM, clamp)
        assert np.isclose(target[6], expect), f"jaw {jaw} should clamp to {expect}, got {target[6]}"


def test_a_safe_gripper_value_is_left_alone() -> None:
    measured = np.zeros(7)
    target, _ = park_target_from(measured, REAL_PARK, N_ARM, clamp)
    assert np.isclose(target[6], REAL_PARK[6]), "0.0366 is inside the band and must not move"


def test_no_clamp_means_no_gripper_handling() -> None:
    measured = np.zeros(7)
    saved = [0.0] * 6 + [1.0]
    target, _ = park_target_from(measured, saved, None, None)
    assert np.isclose(target[6], 1.0), "without a clamp the value passes through unchanged"


def test_measured_is_not_mutated() -> None:
    """The caller keeps using its measured array; aliasing it would be a live bug."""
    measured = np.array([0.1] * 7)
    before = measured.copy()
    park_target_from(measured, REAL_PARK, N_ARM, clamp)
    assert np.allclose(measured, before)


def test_lists_and_arrays_are_both_accepted() -> None:
    a, _ = park_target_from([0.0] * 7, REAL_PARK, N_ARM, clamp)
    b, _ = park_target_from(np.zeros(7), np.asarray(REAL_PARK), N_ARM, clamp)
    assert np.allclose(a, b)


def test_park_speed_step_is_finite_for_every_shape() -> None:
    """What the control loop actually does with the result, for both shapes."""
    for n in (6, 7):
        measured = np.zeros(n)
        target, _ = park_target_from(measured, REAL_PARK, N_ARM, clamp)
        step = np.clip(target - measured, -0.004, 0.004)
        assert step.shape == (n,)
        assert np.all(np.isfinite(step))


# --------------------------------------------------- the park trajectory ----


def test_advance_moves_the_command_toward_the_target() -> None:
    cmd = advance_park_command([0.0, 0.0], [1.0, -1.0], 0.1)
    assert np.allclose(cmd, [0.1, -0.1]), cmd


def test_advance_never_overshoots() -> None:
    cmd = advance_park_command([0.95, -0.95], [1.0, -1.0], 0.1)
    assert np.allclose(cmd, [1.0, -1.0]), cmd


def test_advance_converges_even_when_the_arm_is_stuck() -> None:
    """⭐ THE REGRESSION, stated as a test.

    Old PARK re-anchored to the MEASURED pose every cycle, so if the arm did not
    move the command never got more than ONE step ahead — 0.004 rad — and the
    controller's error term, and therefore its torque, stayed at that forever.
    A trajectory must be able to run ahead of an arm that has not started yet.
    """
    target = np.array([1.2, -0.8, 0.5])
    cmd = np.zeros(3)
    stuck = np.zeros(3)                      # the arm never moves at all
    step = 0.40 * 0.01                       # PARK_SPEED * dt
    for _ in range(2000):                    # 20 s at 100 Hz
        cmd = advance_park_command(cmd, target, step)
    assert np.allclose(cmd, target, atol=1e-9), cmd
    assert float(np.max(np.abs(cmd - stuck))) > 1.0, "must run ahead of a stuck arm"


def test_the_old_formula_provably_could_not_converge() -> None:
    """Proof the diagnosis is mechanical, not a story. Reproduces the old code."""
    target = np.array([1.2, -0.8, 0.5])
    measured = np.zeros(3)                   # a stuck arm
    step = 0.40 * 0.01
    commanded = measured
    for _ in range(2000):
        commanded = measured + np.clip(target - measured, -step, step)
    lead = float(np.max(np.abs(commanded - measured)))
    assert np.isclose(lead, step), lead
    assert lead < 0.005, (
        f"after 20 s the old command was still only {lead:.4f} rad ahead of the arm "
        "— that is the entire bug"
    )


def test_advance_does_not_run_away_from_an_arm_that_follows() -> None:
    target = np.array([1.0])
    cmd = np.zeros(1)
    measured = np.zeros(1)
    for _ in range(400):
        cmd = advance_park_command(cmd, target, 0.004)
        measured = measured + (cmd - measured) * 0.5      # a well-behaved follower
    assert np.allclose(cmd, target, atol=1e-9)
    assert float(np.max(np.abs(cmd - measured))) < 0.01


def test_advance_handles_an_already_reached_target() -> None:
    assert np.allclose(advance_park_command([1.0, 2.0], [1.0, 2.0], 0.1), [1.0, 2.0])


# ------------------------------------------- when is a park finished? ----

TOL, SETTLED = 0.02, 0.06


def test_the_exact_knife_edge_seen_on_hardware() -> None:
    """⛔⭐ THE REGRESSION, with the real numbers. Two consecutive sessions on the
    same arm and the same pose: 0.020 rad reported "PARK reached", 0.021 reported
    "PARK STALLED". The tolerance is 0.02 — the arm was landing either side of it by
    a thousandth of a radian, which is the controller's steady-state error, not a
    fault. Both must now finish."""
    assert park_verdict(0.020, True, TOL, SETTLED) == "settled"
    assert park_verdict(0.021, True, TOL, SETTLED) == "settled"
    assert park_verdict(0.019, False, TOL, SETTLED) == "arrived"


def test_still_closing_the_gap_is_not_a_verdict_yet() -> None:
    assert park_verdict(0.5, False, TOL, SETTLED) == "moving"
    assert park_verdict(0.03, False, TOL, SETTLED) == "moving"


def test_stopping_FAR_from_the_target_is_still_blocked() -> None:
    """⛔ The case that must not be softened away. Loosening the tolerance would have
    made an obstructed arm look parked — and the obstruction might be a hand."""
    assert park_verdict(0.2, True, TOL, SETTLED) == "blocked"
    assert park_verdict(SETTLED, True, TOL, SETTLED) == "blocked", "the band is exclusive"


def test_arrival_does_not_depend_on_having_stopped() -> None:
    """Reaching the pose while still moving is arrival, not a stall."""
    assert park_verdict(0.001, False, TOL, SETTLED) == "arrived"
    assert park_verdict(0.001, True, TOL, SETTLED) == "arrived"


def test_the_two_thresholds_each_do_one_job() -> None:
    """The whole point of splitting them: one threshold was deciding both "close
    enough to stop" and "close enough to trust", and those are different questions."""
    assert SETTLED > TOL, "the settled band must be looser than clean arrival"
    for err in (0.021, 0.03, 0.05):
        assert park_verdict(err, True, TOL, SETTLED) == "settled"
        assert park_verdict(err, False, TOL, SETTLED) == "moving", (
            "the same error means 'keep going' while progress is still being made")


# --------------------------------------------------- saved pose slots ----


LEGACY = {"B": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.5]}


def test_the_pose_already_on_the_rig_still_loads() -> None:
    """⛔⭐ config/park_pose.json is MEASURED CALIBRATION and it exists on the arm
    right now, in the pre-slots shape `{"B": [q…]}`. A format change that quietly
    dropped it would cost bench time to recreate — and `q p d`, the hands-free
    shutdown, depends on it. A bare list reads as the `default` slot."""
    slots = park_slots(LEGACY, "B")
    assert slots == {"default": LEGACY["B"]}
    assert park_slots(LEGACY, "G") == {}, "an arm with nothing saved has no slots"


def test_saving_a_slot_migrates_the_legacy_file_without_losing_it() -> None:
    updated = with_park_slot(LEGACY, "B", "3", [1.0] * 7)
    slots = park_slots(updated, "B")
    assert slots["default"] == LEGACY["B"], "the original pose must survive the migration"
    assert slots["3"] == [1.0] * 7


def test_saving_does_not_mutate_what_it_was_given() -> None:
    """So a caller can compare before and after and only write when something
    changed — the axis-map file was once overwritten with mangled values, and the
    lesson taken from it was to make "did this actually change?" answerable."""
    before = {"B": [0.0] * 7}
    after = with_park_slot(before, "B", "1", [9.0] * 7)
    assert before == {"B": [0.0] * 7}, "the input was mutated"
    assert park_slots(after, "B")["1"] == [9.0] * 7


def test_slots_are_per_arm() -> None:
    data = with_park_slot(with_park_slot({}, "B", "1", [1.0]), "G", "1", [2.0])
    assert park_slots(data, "B")["1"] == [1.0]
    assert park_slots(data, "G")["1"] == [2.0], "arms must not share slots"


def test_a_slot_can_be_overwritten() -> None:
    data = with_park_slot(with_park_slot({}, "B", "2", [1.0]), "B", "2", [5.0])
    assert park_slots(data, "B")["2"] == [5.0]


# ---------------------------------------- smoothing between the poses ----

RAMP, FLOOR = 0.2, 0.15


def test_the_move_eases_in_and_out() -> None:
    """A constant-rate park starts and stops with a jerk. With sequences that jerk
    lands at every waypoint, in the middle of a motion someone is watching."""
    start = park_speed_factor(0.0, 1.0, RAMP)
    middle = park_speed_factor(0.5, 0.5, RAMP)
    end = park_speed_factor(1.0, 0.0, RAMP)
    assert start < middle, "it should ramp up from rest"
    assert end < middle, "it should ramp down into the target"
    assert middle == 1.0, "the middle of a long move runs at full speed"


def test_it_never_reaches_zero() -> None:
    """⚠️ Without a floor the factor hits zero at both ends and the arm creeps for
    ever — which the stall detector would eventually, and wrongly, call an
    obstruction."""
    assert park_speed_factor(0.0, 1.0, RAMP) >= FLOOR
    assert park_speed_factor(1.0, 0.0, RAMP) >= FLOOR
    assert park_speed_factor(0.0, 0.0, RAMP) >= FLOOR


def test_it_never_exceeds_full_speed() -> None:
    """⛔ The safety-relevant direction. Scaling the step DOWN cannot overshoot,
    because advance_park_command already clamps to the distance remaining. Scaling it
    UP would break that guarantee, so the factor must never exceed 1."""
    for travelled in (0.0, 0.1, 1.0, 100.0):
        for remaining in (0.0, 0.1, 1.0, 100.0):
            assert park_speed_factor(travelled, remaining, RAMP) <= 1.0


def test_a_short_hop_becomes_a_triangle_not_a_special_case() -> None:
    """A move shorter than two ramps never reaches full speed. That is correct, and
    it falls out of the min() rather than needing a branch."""
    peak = max(park_speed_factor(d, 0.1 - d, RAMP) for d in (0.0, 0.05, 0.1))
    assert peak < 1.0, "a 0.1 rad move should never reach full speed with a 0.2 ramp"


def test_a_zero_ramp_disables_smoothing_entirely() -> None:
    """`--no-smooth` is exactly this, so the old constant-rate behaviour stays
    reachable with one flag."""
    for travelled, remaining in ((0.0, 1.0), (0.5, 0.5), (1.0, 0.0)):
        assert park_speed_factor(travelled, remaining, 0.0) == 1.0


BASE = [0.0] * 7
SLOTS = {"1": [1.0] * 7, "3": [3.0] * 7}


def test_zero_always_means_the_base_pose() -> None:
    """⛔⭐ Julien's ruling: Ctrl-C parks and then RELEASES the motors, so the pose it
    picks must be one that is safe to be let go in. `0` is that pose, and saving a
    waypoint mid-task must never move it."""
    legs, missing = resolve_park_legs(["0"], BASE, SLOTS)
    assert legs == [("0", BASE)] and missing == []


def test_a_sequence_keeps_the_order_it_was_typed() -> None:
    legs, missing = resolve_park_legs(["1", "3", "0"], BASE, SLOTS)
    assert [n for n, _ in legs] == ["1", "3", "0"]
    assert missing == []


def test_an_empty_slot_is_SKIPPED_never_substituted() -> None:
    """⛔ The tempting alternative — fall back to the base when a waypoint is empty —
    would send the arm somewhere nobody asked for, mid-sequence, while it is being
    watched. A pose the arm MOVES TO is never a default."""
    legs, missing = resolve_park_legs(["1", "7", "3"], BASE, SLOTS)
    assert [n for n, _ in legs] == ["1", "3"], "an empty slot must not become a move"
    assert missing == ["7"]


def test_a_repeated_slot_is_visited_twice() -> None:
    """`p 1 2 1 Enter` is the obvious way to type a there-and-back."""
    legs, _ = resolve_park_legs(["1", "3", "1"], BASE, SLOTS)
    assert [n for n, _ in legs] == ["1", "3", "1"]


def test_no_base_saved_and_zero_requested_is_reported_not_invented() -> None:
    legs, missing = resolve_park_legs(["0", "1"], None, SLOTS)
    assert [n for n, _ in legs] == ["1"]
    assert missing == ["0"]


def test_the_legs_are_copies_so_a_run_cannot_edit_the_saved_pose() -> None:
    legs, _ = resolve_park_legs(["1"], BASE, SLOTS)
    legs[0][1][0] = 99.0
    assert SLOTS["1"][0] == 1.0, "running a sequence mutated the saved waypoint"


def test_junk_in_the_file_is_ignored_rather_than_crashing_the_session() -> None:
    """⚠️ This file is hand-editable and lives in git. A malformed entry must not
    take down a session that is holding a raised arm."""
    data = {"B": {"1": [1.0, 2.0], "2": "not a pose", "3": [], "4": None}}
    assert park_slots(data, "B") == {"1": [1.0, 2.0]}


# ------------------------------------------- did the gripper grab something? ----
#
# ⭐ Two plans need the robot to know whether it succeeded with nobody watching: throwing
# failed episodes out of a dataset, and ENPIRE's reset-run-verify-improve loop. The jaws are
# position controlled and their reading is already normalised 0 closed to 1 open, and
# calibration closes them onto themselves, so 0 means empty. That makes this free.


def test_jaws_stopping_short_of_closed_means_something_is_in_them() -> None:
    """A 3 cm object in a 96 mm stroke leaves the jaws about a third open."""
    got = check_grasp(commanded=0.0, measured=0.31, settled=True)
    assert got.holding
    assert got.confident
    assert abs(got.gap - 0.31) < 1e-9


def test_jaws_closing_all_the_way_means_they_are_empty() -> None:
    got = check_grasp(commanded=0.0, measured=0.01, settled=True)
    assert not got.holding
    assert got.confident
    assert "empty" in got.why


def test_it_refuses_to_answer_before_the_jaws_have_STOPPED_moving() -> None:
    """⛔ Mid-close looks identical to holding a wide object. Answering then would label a
    failed grab as a success, which is the one outcome that poisons a dataset."""
    got = check_grasp(commanded=0.0, measured=0.40, settled=False)
    assert not got.confident
    assert not got.holding


def test_it_refuses_to_answer_when_the_jaws_were_never_told_to_close() -> None:
    """Commanding 0.5 and measuring 0.5 carries no information about an object."""
    got = check_grasp(commanded=0.5, measured=0.5, settled=True)
    assert not got.confident
    assert "not closed" in got.why


def test_a_few_millimetres_of_gap_is_noise_rather_than_an_object() -> None:
    """The controller never lands exactly on its target, and 0.03 of a 96 mm stroke is
    about 3 mm. Below that the gap is steady-state error."""
    assert not check_grasp(commanded=0.0, measured=0.02, settled=True).holding
    assert check_grasp(commanded=0.0, measured=0.05, settled=True).holding


def test_a_negative_gap_is_reported_as_zero_rather_than_as_a_negative_object() -> None:
    """The jaws can read slightly past their commanded position. A negative width is not a
    thing, and letting it through would make any later averaging nonsense."""
    got = check_grasp(commanded=0.05, measured=0.01, settled=True)
    assert not got.holding
    assert got.gap == 0.0


def test_every_answer_explains_itself() -> None:
    """⭐ The verdict goes into episode metadata, and a bare False months later is unreadable."""
    for args in ((0.0, 0.31, True), (0.0, 0.01, True), (0.0, 0.4, False), (0.5, 0.5, True)):
        assert check_grasp(*args).why, args


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except AssertionError as exc:
            failed.append((name, str(exc)))
            print(f"  ✗ {name}\n      {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
