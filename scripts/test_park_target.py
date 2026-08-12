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
    park_slots,
    park_target_from,
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


def test_junk_in_the_file_is_ignored_rather_than_crashing_the_session() -> None:
    """⚠️ This file is hand-editable and lives in git. A malformed entry must not
    take down a session that is holding a raised arm."""
    data = {"B": {"1": [1.0, 2.0], "2": "not a pose", "3": [], "4": None}}
    assert park_slots(data, "B") == {"1": [1.0, 2.0]}


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
