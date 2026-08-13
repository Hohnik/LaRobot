#!/usr/bin/env python3
"""Tests for hand-taught movement recording and playback. No hardware.

    uv run scripts/test_recording.py

⛔ WHY THESE EXIST BEFORE THE ARM EVER SEES THIS. Session 4 is the standing warning in
this repo: three changes passed 34 tests, three dry runs and a simulated loop, and the
first hardware contact produced three failures, one of which dropped 4.3 kg. Playback
commands a path a hand taught, so the failure mode is the arm moving where nobody is
expecting it. Everything decidable without hardware is decided here.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from recording import (  # noqa: E402
    Sample, Trajectory, TrackingLog, replay_step, safe_time_scale,
)


def straight_line(n: int = 11, hz: float = 10.0) -> Trajectory:
    """Joint 0 moves 0 → 1 rad at a constant rate. Every other joint stays put."""
    traj = Trajectory(meta={"arm": "B", "method": "guide", "nominal_hz": hz})
    for i in range(n):
        traj.append(i / hz, (i / (n - 1), 0.0, 0.0, 0.0, 0.0, 0.0, 0.035))
    return traj


# ------------------------------------------------------------------ building ----


def test_a_recording_keeps_its_samples_in_order() -> None:
    traj = straight_line()
    assert len(traj) == 11
    assert traj.n_joints == 7
    assert abs(traj.duration - 1.0) < 1e-9


def test_time_going_BACKWARDS_is_refused() -> None:
    """⛔ A control loop should never produce this. If it does, every calculation here
    returns confident nonsense instead of raising, which is the failure mode this repo
    is named after ([FINDINGS §0])."""
    traj = straight_line(3)
    try:
        traj.append(0.0, (0.0,) * 7)
    except ValueError as exc:
        assert "forwards" in str(exc)
    else:
        raise AssertionError("a backwards sample was accepted")


def test_the_same_timestamp_twice_is_refused() -> None:
    """Two samples at one instant make the speed calculation divide by zero."""
    traj = Trajectory()
    traj.append(1.0, (0.0,) * 7)
    try:
        traj.append(1.0, (0.1,) * 7)
    except ValueError:
        pass
    else:
        raise AssertionError("a duplicate timestamp was accepted")


def test_the_joint_count_cannot_CHANGE_mid_recording() -> None:
    """⛔ A 6-vs-7 joint mismatch is exactly what raised, escaped the control loop and
    DROPPED A RAISED ARM once before ([FINDINGS §11], park_target_from). Refuse it where
    it starts rather than where it hurts."""
    traj = Trajectory()
    traj.append(0.0, (0.0,) * 7)
    try:
        traj.append(0.1, (0.0,) * 6)
    except ValueError as exc:
        assert "joint" in str(exc)
    else:
        raise AssertionError("a recording changed joint count silently")


# ------------------------------------------------------------------- reading ----


def test_an_empty_recording_reports_nothing_rather_than_crashing() -> None:
    empty = Trajectory()
    assert len(empty) == 0
    assert empty.duration == 0.0
    assert empty.n_joints == 0
    assert empty.start_pose() is None
    assert empty.max_joint_speed() == 0.0


def test_the_start_pose_is_where_playback_has_to_begin() -> None:
    """⛔ The safety point of the whole feature. If the arm is elsewhere when playback
    starts, commanding the first pose is a jump. A caller must park to this first."""
    traj = straight_line()
    assert traj.start_pose() == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.035)


def test_a_pose_between_two_samples_is_interpolated() -> None:
    traj = straight_line()
    mid = traj.pose_at(0.55)
    assert abs(mid[0] - 0.55) < 1e-9, mid
    assert mid[6] == 0.035, "an unmoving joint must not drift"


def test_asking_beyond_either_end_is_CLAMPED_not_extrapolated() -> None:
    """⛔ Extrapolating past the end of a recording invents a pose nobody taught, and
    commands it to 4.3 kg of arm."""
    traj = straight_line()
    assert traj.pose_at(-5.0) == traj.start_pose()
    assert traj.pose_at(99.0) == traj.samples[-1].q


def test_interpolation_is_correct_on_UNEVEN_sample_spacing() -> None:
    """⚠️ The real case. A 100 Hz loop does not deliver evenly spaced samples, so a
    binary search plus a fraction has to handle a ragged clock."""
    traj = Trajectory()
    for t, x in [(0.0, 0.0), (0.1, 1.0), (0.9, 2.0), (1.0, 3.0)]:
        traj.append(t, (x,))
    assert abs(traj.pose_at(0.05)[0] - 0.5) < 1e-9
    assert abs(traj.pose_at(0.5)[0] - 1.5) < 1e-9
    assert abs(traj.pose_at(0.95)[0] - 2.5) < 1e-9


def test_the_fastest_joint_speed_is_measured_not_guessed() -> None:
    """⭐ The number a caller checks a speed multiplier against, because this rig has no
    emergency stop and a hand-taught path is only known safe at the speed it was taught."""
    traj = straight_line(n=11, hz=10.0)          # 1 rad over 1.0 s, 10 equal steps
    assert abs(traj.max_joint_speed() - 1.0) < 1e-9


def test_a_single_fast_step_dominates_the_speed_report() -> None:
    traj = Trajectory()
    traj.append(0.0, (0.0,))
    traj.append(1.0, (0.1,))                     # slow
    traj.append(1.1, (1.0,))                     # 0.9 rad in 0.1 s = 9 rad/s
    assert abs(traj.max_joint_speed() - 9.0) < 1e-6


# ------------------------------------------------- padding at the end (§30.1) ----


def wobbling_tail(move_s: float, pad_s: float, wobble: float = 0.035,
                  hz: float = 100.0) -> Trajectory:
    """A real movement, then a stretch of the wobble a held arm actually produces.

    ⭐ The wobble alternates sign so the arm goes nowhere overall, which is what the
    padded tails in Julien's recordings look like: a flat non-zero speed floor and no net
    displacement.

    ⚠️ The offset is `wobble / hz` on alternate samples, so the step between two samples
    is `wobble / hz` and the resulting speed is exactly `wobble`. Offsetting by ±that
    instead doubles the step and doubles the speed, which is how the first version of this
    fixture reported no padding at all.
    """
    traj = Trajectory(meta={"arm": "B", "method": "live:guide"})
    n_move, n_pad = int(move_s * hz), int(pad_s * hz)
    for i in range(n_move):                       # 1.0 rad/s on joint 0
        traj.append(i / hz, (i / hz, 0.0))
    rest = (n_move - 1) / hz if n_move else 0.0
    for i in range(n_pad):
        traj.append((n_move + i) / hz, (rest + (wobble / hz if i % 2 else 0.0), 0.0))
    return traj


def test_a_padded_tail_is_measured_even_though_it_is_not_motionless() -> None:
    """⛔ The defect this exists for. A held arm wobbles at ~0.035 rad/s, so the padding
    is never still and a zero-speed test finds none of it."""
    traj = wobbling_tail(move_s=3.0, pad_s=4.0)
    assert traj.trailing_still_seconds() > 3.9, "4 s of padding must be seen"
    assert traj.trailing_still_seconds(still=0.01) < 0.05, (
        "and a threshold under the wobble floor is how it stayed invisible"
    )


def test_a_clean_recording_reports_no_padding() -> None:
    traj = straight_line(n=101, hz=100.0)
    assert traj.trailing_still_seconds() < 0.02


def test_a_recording_that_never_moves_is_padding_end_to_end() -> None:
    traj = wobbling_tail(move_s=0.0, pad_s=2.0)
    assert abs(traj.trailing_still_seconds() - traj.duration) < 1e-9


def test_padding_needs_two_samples_to_exist() -> None:
    traj = Trajectory()
    assert traj.trailing_still_seconds() == 0.0
    traj.append(0.0, (0.0,))
    assert traj.trailing_still_seconds() == 0.0


# ---------------------------------------------------------------- reshaping ----


def test_resampling_gives_an_evenly_spaced_clock() -> None:
    """⭐ Needed because ABC's training file has one row per timestep and no timestamp
    column, so the rows must genuinely be evenly spaced ([ROADMAP §9.2])."""
    traj = Trajectory()
    for t, x in [(0.0, 0.0), (0.1, 1.0), (0.9, 2.0), (1.0, 3.0)]:
        traj.append(t, (x,))
    out = traj.resampled(10.0)
    gaps = [b.t - a.t for a, b in zip(out.samples, out.samples[1:])]
    assert all(abs(g - 0.1) < 1e-9 for g in gaps), gaps
    assert out.meta["resampled_hz"] == 10.0


def test_resampling_never_LOSES_the_end_of_the_movement() -> None:
    """⚠️ When the grid does not land on the final sample, dropping it quietly shortens
    every playback by up to one step, and the arm stops short of where the hand left it."""
    traj = Trajectory()
    traj.append(0.0, (0.0,))
    traj.append(0.35, (1.0,))                    # 0.35 s is not a multiple of 0.1
    out = traj.resampled(10.0)
    assert abs(out.samples[-1].t - 0.35) < 1e-9
    assert abs(out.samples[-1].q[0] - 1.0) < 1e-9


def test_resampling_preserves_the_shape_of_the_path() -> None:
    traj = straight_line(n=11, hz=10.0)
    out = traj.resampled(50.0)
    for s in out.samples:
        assert abs(s.q[0] - s.t) < 1e-6, "a straight line resampled must stay straight"


def test_a_zero_or_negative_rate_is_refused() -> None:
    traj = straight_line()
    for bad in (0.0, -10.0):
        try:
            traj.resampled(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"hz={bad} was accepted")


def test_resampling_a_recording_too_short_to_move_returns_it_unchanged() -> None:
    one = Trajectory()
    one.append(0.0, (0.0,) * 7)
    assert len(one.resampled(30.0)) == 1


def test_time_scaling_speeds_the_path_up_without_changing_it() -> None:
    traj = straight_line()
    fast = traj.time_scaled(2.0)
    assert abs(fast.duration - traj.duration / 2) < 1e-9
    assert abs(fast.max_joint_speed() - traj.max_joint_speed() * 2) < 1e-6
    assert fast.samples[-1].q == traj.samples[-1].q, "the path itself must be identical"


def test_time_scaling_by_a_non_positive_factor_is_refused() -> None:
    traj = straight_line()
    for bad in (0.0, -1.0):
        try:
            traj.time_scaled(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"factor={bad} was accepted")


# ------------------------------------------------------ one playback cycle ----
#
# ⛔ THIS BRANCH COMMANDS THE ARM, which is why the decision lives in a pure function and
# is tested here rather than only in the session script. Session 4's lesson runs the other
# way (reading does not find what hardware knows), and this is its pair: code that reaches
# the motors and has never been executed is the shape that dropped 4.3 kg.


def test_a_playback_advances_by_dt_when_the_arm_is_keeping_up() -> None:
    traj = straight_line()
    on_track = traj.pose_at(0.5)
    step = replay_step(traj, 0.5, on_track, dt=0.01, speed=1.0)
    assert abs(step.cursor - 0.51) < 1e-9
    assert not step.held
    assert not step.finished
    assert step.lag < 1e-9


def test_the_speed_multiplier_scales_the_clock_not_the_path() -> None:
    traj = straight_line()
    on_track = traj.pose_at(0.2)
    step = replay_step(traj, 0.2, on_track, dt=0.01, speed=2.0)
    assert abs(step.cursor - 0.22) < 1e-9
    assert step.target == traj.pose_at(0.2), "the pose asked for must not change with speed"


def test_the_CLOCK_IS_HELD_when_the_arm_has_fallen_behind() -> None:
    """⛔ The safety rule of the playback loop. Advancing while the arm is behind widens
    the gap, and when whatever was holding it back lets go, the arm crosses the whole
    accumulated gap at once. That is a lurch, not a replay."""
    traj = straight_line()
    behind = tuple(x - 0.5 for x in traj.pose_at(0.5))       # 0.5 rad behind on every joint
    step = replay_step(traj, 0.5, behind, dt=0.01, speed=1.0, max_lag=0.15)
    assert step.held, "the clock advanced while the arm was 0.5 rad behind"
    assert step.cursor == 0.5, "a held clock must not move at all"
    assert abs(step.lag - 0.5) < 1e-9


def test_a_held_clock_can_never_report_finished_early() -> None:
    """Otherwise a stuck arm would be announced as a completed playback."""
    traj = straight_line()
    behind = tuple(x - 1.0 for x in traj.samples[-1].q)
    step = replay_step(traj, traj.duration - 0.001, behind, dt=0.01)
    assert step.held
    assert not step.finished


def test_the_gripper_is_LEFT_OUT_of_the_lag_measurement_when_asked() -> None:
    """⛔ The jaws legitimately sit far from their commanded value while closing on an
    object. Counting that as "the arm cannot follow" would stall every playback that grips
    anything, which is every playback worth recording."""
    traj = straight_line()
    q = list(traj.pose_at(0.5))
    q[6] -= 0.4                                              # jaws well off target
    all_joints = replay_step(traj, 0.5, q, dt=0.01, max_lag=0.15)
    arm_only = replay_step(traj, 0.5, q, dt=0.01, max_lag=0.15, n_compare=6)
    assert all_joints.held, "counting the gripper, this looks stuck"
    assert not arm_only.held, "ignoring the gripper, the arm is following fine"


def test_reaching_the_end_reports_finished() -> None:
    traj = straight_line()
    step = replay_step(traj, traj.duration - 0.005, traj.samples[-1].q, dt=0.01)
    assert step.finished


def test_playing_an_empty_recording_is_refused() -> None:
    try:
        replay_step(Trajectory(), 0.0, (0.0,) * 7, dt=0.01)
    except ValueError as exc:
        assert "empty" in str(exc)
    else:
        raise AssertionError("an empty recording was played")


def test_a_non_positive_speed_is_refused() -> None:
    traj = straight_line()
    for bad in (0.0, -1.0):
        try:
            replay_step(traj, 0.0, traj.start_pose(), dt=0.01, speed=bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"speed={bad} was accepted")


def test_a_whole_playback_run_reaches_the_end_and_follows_the_path() -> None:
    """⭐ The loop as the session actually runs it, with an arm that tracks perfectly.
    Checks the two things that matter: it terminates, and it commands the taught path."""
    traj = straight_line(n=21, hz=20.0)
    cursor, measured, cycles = 0.0, traj.start_pose(), 0
    seen = []
    while cycles < 10_000:
        step = replay_step(traj, cursor, measured, dt=0.01, speed=1.0, n_compare=6)
        cursor, measured = step.cursor, step.target
        seen.append(step.target[0])
        cycles += 1
        if step.finished:
            break
    assert step.finished, "the playback never terminated"
    assert 95 <= cycles <= 105, f"a 1.0 s recording at 0.01 s per cycle took {cycles}"
    assert seen == sorted(seen), "the commanded path must move monotonically here"
    assert abs(seen[-1] - 1.0) < 0.02, "it must arrive at the taught end pose"


# --------------------------------------------------- the playback speed cap ----


def test_a_slow_recording_reports_headroom_above_1x() -> None:
    """A recording taught at 0.5 rad/s against a 1.5 rad/s cap has 3x of headroom."""
    assert abs(safe_time_scale(0.5, 1.5) - 3.0) < 1e-9


def test_a_recording_FASTER_than_the_cap_reports_BELOW_1x() -> None:
    """⛔ THE CHANGE OF 2026-08-13, and it came from a real run. This used to be floored at
    1.0, which expressed the policy "1x is always allowed" inside a function whose job is
    to measure. The floor hid the one fact the session needed: the recording is faster than
    any planned motion here is permitted to be. Every hand-taught recording reported
    "max 1.00x" and then played back slower than 1x anyway, and nothing on screen could
    explain why."""
    assert safe_time_scale(1.5, 1.5) == 1.0
    assert abs(safe_time_scale(3.0, 1.5) - 0.5) < 1e-9
    assert safe_time_scale(2.67, 1.5) < 1.0, "a hand-taught 2.67 rad/s cannot run at 1x"


def test_a_recording_that_never_moved_has_unlimited_headroom() -> None:
    assert safe_time_scale(0.0, 1.5) == float("inf")


def test_a_non_positive_cap_is_refused() -> None:
    for bad in (0.0, -1.0):
        try:
            safe_time_scale(1.0, bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"cap={bad} was accepted")


def test_the_cap_and_the_measurement_agree_end_to_end() -> None:
    """The two halves used together: measure what was taught, derive the multiplier, and
    confirm the scaled recording really does respect the cap."""
    traj = straight_line(n=11, hz=10.0)          # 1.0 rad/s taught
    allowed = safe_time_scale(traj.max_joint_speed(), 1.5)
    assert abs(allowed - 1.5) < 1e-9
    assert traj.time_scaled(allowed).max_joint_speed() <= 1.5 + 1e-9


def test_a_PERCENTILE_ignores_a_single_noisy_sample_and_the_MAX_does_not() -> None:
    """⭐ Measured from his own recordings on 2026-08-13: recording 4 has a maximum of
    3.31 rad/s and a 99th percentile of 2.36, so one sample was dragging the maximum up by
    40%. At 100 Hz a single noisy reading of 0.033 rad does that, and a weightless arm
    being pushed by hand is exactly where such a reading comes from."""
    traj = Trajectory()
    for i in range(100):
        traj.append(i / 100.0, (i * 0.005,))     # a steady 0.5 rad/s
    traj.append(1.0, (traj.samples[-1].q[0] + 0.05,))   # one 5 rad/s spike
    assert traj.max_joint_speed() > 4.0, "the maximum must still report the spike honestly"
    assert traj.joint_speed(95) < 1.0, "the percentile must ignore it"
    assert abs(traj.joint_speed(50) - 0.5) < 0.01


def test_the_percentile_of_a_steady_recording_equals_its_speed() -> None:
    traj = straight_line(n=11, hz=10.0)          # a constant 1.0 rad/s
    for pct in (50, 95, 99, 100):
        assert abs(traj.joint_speed(pct) - 1.0) < 1e-9, pct


def test_the_percentile_of_an_empty_or_single_sample_recording_is_zero() -> None:
    assert Trajectory().joint_speed(99) == 0.0
    one = Trajectory([Sample(0.0, (0.0,))])
    assert one.joint_speed(99) == 0.0


# -------------------------------------------------------------------- files ----


def test_a_recording_survives_a_round_trip_to_disk() -> None:
    traj = straight_line()
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "taught.json"
        traj.save(path)
        back = Trajectory.load(path)
    assert len(back) == len(traj)
    assert back.meta["arm"] == "B"
    assert back.n_joints == 7
    for a, b in zip(traj.samples, back.samples):
        assert abs(a.t - b.t) < 1e-5
        assert all(abs(x - y) < 1e-5 for x, y in zip(a.q, b.q))


def test_saving_an_EMPTY_recording_is_refused() -> None:
    """⛔ A file that looks like a demonstration and contains no movement survives into a
    dataset and trains on nothing. Failing loudly here is the whole point."""
    with tempfile.TemporaryDirectory() as tmp:
        for traj in (Trajectory(), Trajectory([Sample(0.0, (0.0,) * 7)])):
            try:
                traj.save(Path(tmp) / "x.json")
            except ValueError as exc:
                assert "no movement" in str(exc)
            else:
                raise AssertionError("an empty recording was written to disk")


def test_the_file_is_rows_not_objects_so_it_stays_readable() -> None:
    """⭐ 30 000 samples as named objects is megabytes of repeated field names. Rows keep
    it diffable in git, which is the only reason this is JSON rather than a blob."""
    traj = straight_line()
    data = json.loads(json.dumps(traj.to_dict()))
    assert isinstance(data["samples"][0], list)
    assert len(data["samples"][0]) == 8, "one timestamp plus seven joints"


def test_metadata_survives_and_the_expected_field_names_are_documented() -> None:
    """⭐ Provenance is a stated requirement, not a nicety: he wants to reproduce every
    recording and connect it to papers ([ROADMAP §6.6])."""
    for name in ("arm", "commit", "recorded_at", "nominal_hz", "method"):
        assert name in Trajectory.META_FIELDS
    traj = straight_line()
    traj.meta["commit"] = "deadbee"
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "t.json"
        traj.save(path)
        assert Trajectory.load(path).meta["commit"] == "deadbee"


def test_a_long_recording_is_handled_without_a_linear_scan_per_lookup() -> None:
    """A five-minute hand-taught movement at 100 Hz is 30 000 samples, and playback asks
    for a pose every cycle. The lookup is a binary search for that reason."""
    traj = Trajectory()
    for i in range(30_000):
        traj.append(i / 100.0, (i / 30_000.0,) * 7)
    assert abs(traj.duration - 299.99) < 1e-6
    assert abs(traj.pose_at(150.0)[0] - 0.5) < 1e-3


# --------------------------------------------- how well each joint kept up ----


def test_tracking_records_the_worst_lag_and_the_speed_at_that_moment() -> None:
    """⭐ Answers Julien's question about arm speed with no new motion. Every playback
    already commands a path and already measures how far behind the arm is."""
    log = TrackingLog(2)
    log.observe((0.10, 0.0), (0.00, 0.0), (0.05, 0.0), 0.01)   # joint 0: 10 rad/s, lag 0.05
    log.observe((0.12, 0.0), (0.10, 0.0), (0.11, 0.0), 0.01)   # slower, smaller lag
    rows = log.rows()
    assert abs(rows[0][1] - 0.05) < 1e-9, "worst lag"
    assert abs(rows[0][2] - 10.0) < 1e-9, "the speed when the worst lag happened"
    assert rows[1][1] == 0.0, "a joint that never moved has no lag"


def test_tracking_records_the_top_speed_separately_from_the_worst_lag() -> None:
    """⭐ The two pairs answer different questions. Where the limit is, and whether a
    given speed is usable at all."""
    log = TrackingLog(1)
    log.observe((0.30,), (0.00,), (0.29,), 0.01)   # 30 rad/s, tiny lag
    log.observe((0.35,), (0.30,), (0.15,), 0.01)   # 5 rad/s, big lag
    joint, worst_lag, speed_then, top_speed, lag_then = log.rows()[0]
    assert abs(top_speed - 30.0) < 1e-9
    assert abs(lag_then - 0.01) < 1e-9
    assert abs(worst_lag - 0.20) < 1e-9
    assert abs(speed_then - 5.0) < 1e-9


def test_tracking_ignores_a_zero_or_negative_timestep() -> None:
    """A cycle with no elapsed time would divide by zero and report an infinite speed."""
    log = TrackingLog(1)
    log.observe((0.1,), (0.0,), (0.0,), 0.0)
    assert log.cycles == 0
    assert log.rows()[0][3] == 0.0


def test_tracking_handles_a_shorter_measurement_than_the_target() -> None:
    """A 6-joint robot playing a 7-joint recording must not raise inside the loop."""
    log = TrackingLog(7)
    log.observe((0.1,) * 7, (0.0,) * 7, (0.0,) * 6, 0.01)
    assert log.cycles == 1
    assert log.rows()[6][1] == 0.0, "the joint with no measurement stays empty"


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
