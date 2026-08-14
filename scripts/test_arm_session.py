#!/usr/bin/env python3
"""Tests for `ArmSession` — one arm's state and mode machine. No hardware.

    uv run scripts/test_arm_session.py

⛔ WHY THIS FILE MATTERS MORE THAN MOST. `ArmSession` is the extraction that makes
bimanual possible, and it is the largest restructure this repo will have had. The
whole point of pulling one arm's state out of `teleop_session.main()` is that it
then becomes testable *without hardware* — so every behaviour worth trusting on two
arms is pinned here first, on a fake robot, before either arm is ever driven by it.

The fake below is deliberately not clever. It records what was commanded and can be
told to fail, because the interesting cases are the failures: a chain that dies
mid-park, a park that stops short, a gripper clamp that must not fire on entry.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from arm_session import ArmSelector, ArmSession, ParkLeg, parse_arms  # noqa: E402

#: The real thing, so a rename of an arm cannot leave these tests passing against a
#: rig that no longer has that arm.
from yam_can import ARM_SERIALS, DEFAULT_ARM  # noqa: E402

N_ARM = 6


class FakeChain:
    def __init__(self, running=True, states=None, raises=False):  # noqa: ANN001
        self.running = running
        self._states = states or []
        self._raises = raises

    def read_states(self):  # noqa: ANN201
        if self._raises:
            raise RuntimeError("CAN read failed")
        return self._states


class FakeState:
    def __init__(self, temp_mos=30.0, temp_rotor=30.0, eff=0.0, vel=0.0):  # noqa: ANN001
        self.temp_mos, self.temp_rotor = temp_mos, temp_rotor
        self.eff, self.vel = eff, vel        # torque and velocity, for the stall guard


class FakeRobot:
    """An arm that moves exactly as commanded, unless told otherwise."""

    def __init__(self, q=None, dofs=7, follow=True):  # noqa: ANN001
        self.q = np.array(q if q is not None else [0.0] * dofs, dtype=float)
        self.commands: list[np.ndarray] = []
        self.motor_chain = FakeChain()
        self.follow = follow          # False = a stuck arm, for the blocked case
        self.gravity_calls = 0
        self.resyncs = 0

    def get_joint_pos(self):  # noqa: ANN201
        return self.q.copy()

    def command_joint_pos(self, q) -> None:  # noqa: ANN001
        q = np.asarray(q, dtype=float)
        self.commands.append(q.copy())
        if self.follow:
            self.q = q.copy()

    def num_dofs(self) -> int:
        return len(self.q)

    def enter_gravity_comp_idle(self) -> None:
        self.gravity_calls += 1

    def resync(self) -> None:
        self.resyncs += 1


def run_park(arm, robot, seconds=30.0, dt=0.01):  # noqa: ANN001, ANN201
    """Drive the park loop until it reaches a verdict. Returns (verdict, error, t)."""
    t, step = 0.0, None
    while t < seconds:
        step = arm.step_path(t, dt)
        if step.verdict != "moving":
            return step.verdict, step.err, t
        t += dt
    return "timeout", (step.err if step else float("inf")), t


def to(pose, name="0"):  # noqa: ANN001, ANN201
    """One leg, since most tests drive a single waypoint."""
    return [ParkLeg(name, pose)]


# ------------------------------------------------------------- liveness ----


def test_a_dead_chain_is_detected() -> None:
    """⛔ The check that was missing when the loop commanded a corpse for 64 seconds
    while printing healthy numbers."""
    robot = FakeRobot()
    arm = ArmSession(robot, name="B")
    assert arm.alive() is True
    robot.motor_chain.running = False
    assert arm.alive() is False
    robot.motor_chain = None
    assert arm.alive() is False, "no chain at all must read as not alive"


def test_a_failed_temperature_read_reports_blind_not_cold() -> None:
    """⛔ FINDINGS §24.1: a thrown read used to become 0 °C and disarm the stop."""
    robot = FakeRobot()
    robot.motor_chain = FakeChain(raises=True)
    arm = ArmSession(robot, name="B")
    verdict, hottest, jaw = arm.read_thermal()
    assert hottest is None and jaw is None, "a failed read is not a temperature"
    assert verdict.warning is not None and "BLIND" in verdict.warning


def test_temperatures_are_read_per_arm() -> None:
    """With two arms this is per-arm state; a shared guard would hide one arm's
    gripper behind the other arm's shoulder."""
    robot = FakeRobot()
    robot.motor_chain = FakeChain(states=[FakeState(40)] * 6 + [FakeState(33)])
    arm = ArmSession(robot, name="B")
    _, hottest, jaw = arm.read_thermal()
    assert hottest == 40.0 and jaw == 33.0


# ------------------------------------------------- the gripper stall guard ----


def stalling_arm(eff=2.0, vel=0.0, jaw_pos=0.31):  # noqa: ANN001, ANN201
    """An arm whose jaws are pushing hard and not moving."""
    robot = FakeRobot(q=[0.0] * 6 + [jaw_pos])
    robot.motor_chain = FakeChain(
        states=[FakeState()] * 6 + [FakeState(eff=eff, vel=vel)])
    arm = ArmSession(robot, name="B")
    return robot, arm


def test_a_stalled_gripper_is_released_to_where_the_jaws_actually_ARE() -> None:
    """⛔ Motor 7 was cooked three times. Pushing at full current without moving is the
    worst thermal case there is: full current, no motion, no cooling."""
    robot, arm = stalling_arm(jaw_pos=0.31)
    arm.read_thermal()
    assert arm.gripper_stall_release(0.0) is None, "one cycle is not a stall yet"
    arm.read_thermal()
    assert arm.gripper_stall_release(0.2) is None, "0.4s has not passed"
    arm.read_thermal()
    released = arm.gripper_stall_release(0.5)
    assert released is not None and abs(released - 0.31) < 1e-9, (
        "it must release to the MEASURED jaw position, so the command stops pushing")
    assert arm.stall_since is None, "the timer must reset after a release"


def test_a_gripper_that_is_MOVING_is_not_stalled() -> None:
    """Closing hard on something while still travelling is normal, not a stall."""
    _, arm = stalling_arm(eff=2.0, vel=0.5)
    for i in range(200):
        arm.read_thermal()
        assert arm.gripper_stall_release(i * 0.01) is None
    assert arm.stall_since is None


def test_a_gentle_gripper_is_not_stalled_however_long_it_holds() -> None:
    _, arm = stalling_arm(eff=0.2, vel=0.0)
    for i in range(200):
        arm.read_thermal()
        assert arm.gripper_stall_release(i * 0.01) is None


def test_the_stall_guard_reports_NOTHING_when_it_cannot_see() -> None:
    """⛔ Same rule as the thermal guard: a failed read is not a reading. FINDINGS §24.1
    is the case where a thrown read silently became 0 °C and disarmed the stop."""
    robot, arm = stalling_arm()
    arm.read_thermal()
    arm.gripper_stall_release(0.0)
    assert arm.stall_since is not None, "it was stalling a moment ago"
    robot.motor_chain = FakeChain(raises=True)
    arm.read_thermal()
    assert arm.gripper_stall_release(1.0) is None
    assert arm.stall_since is None, "a stall cannot be judged if it cannot be seen"


def test_the_stall_guard_is_silent_on_a_six_motor_arm() -> None:
    """⚠️ `--no-gripper` leaves six motors, so indexing the seventh would raise inside
    the control loop. That exact shape of bug once dropped 4.3 kg."""
    robot = FakeRobot(q=[0.0] * 6, dofs=6)
    robot.motor_chain = FakeChain(states=[FakeState()] * 6)
    arm = ArmSession(robot, name="B")
    arm.read_thermal()
    assert arm.gripper_stall_release(5.0) is None


def test_the_stall_guard_says_nothing_before_any_thermal_read() -> None:
    _, arm = stalling_arm()
    assert arm.gripper_stall_release(9.0) is None


# ---------------------------------------------------------------- modes ----


def test_entering_teleop_does_NOT_move_the_gripper() -> None:
    """⛔ Clamping on entry is a COMMAND TO MOVE, and nobody asked for it. An earlier
    version clamped here, so jaws sitting outside the band were driven the instant
    teleop began — into a mechanical stop, when the limits were also mis-framed."""
    robot = FakeRobot(q=[0.0] * 6 + [0.995])      # outside the [0.02, 0.98] band
    arm = ArmSession(robot, name="B")
    arm.enter_teleop()
    assert arm.gripper_value == 0.995, "the jaws were clamped on entry — that is a move"
    assert robot.commands[-1][N_ARM] == 0.995, "and it was commanded"


def test_every_mode_change_re_reads_reality() -> None:
    """⛔ A mode change must never carry cached state. `prev_q` surviving a hand-guide
    is what made the arm snap back to a pose from minutes earlier."""
    robot = FakeRobot()
    arm = ArmSession(robot, name="B")
    arm.enter_teleop()
    robot.q = np.array([0.5] * 6 + [0.3])         # hand-guided somewhere else
    arm.enter_hold()
    assert np.allclose(arm.prev_q, [0.5] * 6), "prev_q was stale after a mode change"


def test_guide_records_a_drift_reference_and_measures_against_it() -> None:
    """⛔ In GUIDE, kp = 0 and gravity compensation is the ONLY thing holding 4.3 kg
    up. On 2026-08-10 the arm sank to its stops over 33 s while the readout showed a
    calm temperature, because nothing was measuring the quantity going wrong."""
    robot = FakeRobot()
    arm = ArmSession(robot, name="B")
    assert arm.enter_guide() is None
    assert robot.gravity_calls == 1
    assert arm.guide_drift() == 0.0
    robot.q = np.array([0.0, 0.09, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert abs(arm.guide_drift() - 0.09) < 1e-9


def test_a_missing_zero_gravity_api_is_reported_not_assumed() -> None:
    """⛔ The first attempt guessed at three method names that did not exist, so GUIDE
    silently never worked while the banner announced "the arm is weightless"."""
    class OldSDK(FakeRobot):
        enter_gravity_comp_idle = None      # the method simply is not there

    arm = ArmSession(OldSDK(), name="B")
    warning = arm.enter_guide()
    assert warning is not None and "NOT weightless" in warning
    assert arm.guide_ref is not None, "the drift reference is still taken, so the "
    "operator can see the arm is NOT holding itself"


# ----------------------------------------------------------------- park ----


def test_a_park_reaches_its_target() -> None:
    robot = FakeRobot(q=[1.0] * 6 + [0.5])
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([0.0] * 6 + [0.5]), t=0.0)
    verdict, err, _ = run_park(arm, robot)
    assert verdict in ("arrived", "settled"), verdict
    assert err < 0.06


def test_a_stuck_arm_is_BLOCKED_not_quietly_finished() -> None:
    """⛔ The case that must never be softened away by the settled band — the thing
    in the way might be a hand."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5], follow=False)
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([0.0] * 6 + [0.5]), t=0.0)
    verdict, err, _ = run_park(arm, robot, seconds=10.0)
    assert verdict == "blocked", verdict
    assert err > 0.5, "it never moved, so the error should still be the whole distance"


def test_completion_is_judged_from_the_MEASUREMENT_not_the_command() -> None:
    """⛔ The command always arrives first. Judging it declares success while the arm
    is still travelling — a real bug that hid for two sessions."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5], follow=False)
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([0.0] * 6 + [0.5]), t=0.0)
    for i in range(400):
        arm.step_path(i * 0.01, 0.01)
    assert arm.park_s > 0.0, "the cursor should have advanced before the arm fell behind"
    assert float(np.max(np.abs(arm.park_cmd - robot.q))) > 0.1, (
        "the command has run well ahead of the arm, which is the whole trap")
    # ⚠️ Run to a real verdict rather than sampling one instant. The stall timer starts
    # when the cursor stops advancing, not when the park starts, so a fixed t is a
    # guess about easing and path length. An earlier version of this test guessed 4.5 s
    # and read "moving", which is the stall timer working correctly.
    verdict, _, _ = run_park(arm, robot, seconds=20.0)
    assert verdict == "blocked", f"the arm never moved, so this is not arrival: {verdict}"


def test_the_park_eases_in_rather_than_starting_at_full_speed() -> None:
    """The move must not start at full speed, or every run begins with a jerk."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5])
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([0.0] * 6 + [0.5]), t=0.0)
    before = arm.park_cmd.copy()
    arm.step_path(0.0, 0.01)
    first = float(np.max(np.abs(arm.park_cmd - before)))
    for i in range(1, 80):                 # travel past the ramp
        arm.step_path(i * 0.01, 0.01)
    mid_before = arm.park_cmd.copy()
    arm.step_path(0.8, 0.01)
    mid = float(np.max(np.abs(arm.park_cmd - mid_before)))
    assert first < mid, f"first step {first:.5f} was not slower than mid-move {mid:.5f}"


def test_switching_off_smoothing_removes_the_ramp_not_the_blending() -> None:
    """⚠️ `--no-smooth` is about the speed profile only. Blending is the shape the arm
    follows and easing is the speed along it; they are independent axes."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5])
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([0.0] * 6 + [0.5]), t=0.0, smooth=False)
    before = arm.park_cmd.copy()
    arm.step_path(0.0, 0.01)
    first = float(np.max(np.abs(arm.park_cmd - before)))
    for i in range(1, 80):
        arm.step_path(i * 0.01, 0.01)
    mid_before = arm.park_cmd.copy()
    arm.step_path(0.8, 0.01)
    mid = float(np.max(np.abs(arm.park_cmd - mid_before)))
    assert abs(first - mid) < 1e-9, "with no easing the rate should be constant"


def test_park_with_no_target_refuses_rather_than_crashing() -> None:
    arm = ArmSession(FakeRobot(), name="B")
    assert arm.step_path(0.0, 0.01).verdict == "blocked"


# ---------------------------------------------- one blended path, N waypoints ----


def test_the_path_does_NOT_stop_at_an_intermediate_waypoint() -> None:
    """⛔⭐ THE WHOLE POINT OF BLENDING, and the behaviour that replaced a queue of
    separate legs on 2026-08-12. Julien: *"instead of moving and then jittering ninety
    degrees to the next side, in a smooth curve it would go to the next point."* The
    earlier model drove to each waypoint and stopped dead. **A test that asserted a
    fresh ease-in at every leg used to live here; it asserted the wrong thing.**"""
    robot = FakeRobot(q=[0.0] * 7)
    arm = ArmSession(robot, name="B")
    arm.begin_path([ParkLeg("1", [1.0] * 6 + [0.5]),
                    ParkLeg("2", [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.5])], t=0.0)
    passed_at = None
    slowest_after = None
    for i in range(4000):
        t = i * 0.01
        before = arm.park_s
        step = arm.step_path(t, 0.01)
        if step.leg_passed == "1":
            passed_at = t
        if passed_at is not None and t - passed_at < 0.10:
            moved = arm.park_s - before
            slowest_after = moved if slowest_after is None else min(slowest_after, moved)
        if step.verdict != "moving":
            break
    assert passed_at is not None, "waypoint 1 was never reported"
    assert slowest_after is not None and slowest_after > 0.0, (
        "the cursor stopped at the intermediate waypoint — that is the old behaviour")


def test_waypoints_are_reported_in_order_with_the_one_coming_next() -> None:
    robot = FakeRobot(q=[0.0] * 7)
    arm = ArmSession(robot, name="B")
    arm.begin_path([ParkLeg("1", [0.4] * 6 + [0.5]),
                    ParkLeg("2", [0.8] * 6 + [0.5]),
                    ParkLeg("3", [1.2] * 6 + [0.5])], t=0.0)
    seen = []
    for i in range(6000):
        step = arm.step_path(i * 0.01, 0.01)
        if step.leg_passed:
            seen.append((step.leg_passed, step.next_leg))
        if step.verdict != "moving":
            break
    assert seen == [("1", "2"), ("2", "3"), ("3", None)], seen


def test_arrival_is_gated_on_the_CURSOR_not_on_the_error() -> None:
    """⛔ A run like `p 1 2 1` ends where it began, so the distance to the final target
    is near zero at t=0. Judging arrival on the error alone would declare the whole
    sequence finished before the arm had moved at all."""
    # ⚠️ The gripper column counts toward the error like any other joint, so the start
    # pose has to match the final leg there too or the run does not actually begin at
    # its own target. An earlier version of this test used a gripper of 0.0 against a
    # target of 0.5 and measured an error of 0.5, which tested nothing.
    robot = FakeRobot(q=[0.0] * 6 + [0.5])
    arm = ArmSession(robot, name="B")
    arm.begin_path([ParkLeg("1", [1.0] * 6 + [0.5]),
                    ParkLeg("2", [0.0] * 6 + [0.5])], t=0.0)
    first = arm.step_path(0.0, 0.01)
    assert first.err < 0.02, f"this run must start at its own final target, got {first.err:.3f}"
    assert first.verdict == "moving", "it must not report arrival before moving"
    assert first.remaining > 1.0, "there is a whole out-and-back path still ahead"


def test_the_two_park_clocks_are_kept_apart() -> None:
    """⛔ FINDINGS §34.3. Sharing one clock reported a 4.4 s park as "reached in 0.0s",
    because the last waypoint is passed at the very end of the path."""
    robot = FakeRobot(q=[0.0] * 7)
    arm = ArmSession(robot, name="B")
    arm.begin_path([ParkLeg("1", [0.5] * 6 + [0.5]),
                    ParkLeg("2", [1.0] * 6 + [0.5])], t=0.0)
    after_first = None
    for i in range(4000):
        t = i * 0.01
        step = arm.step_path(t, 0.01)
        if step.leg_passed == "1":
            after_first = t
        if step.verdict != "moving":
            assert step.total_seconds > 0.5, "the total must cover the whole run"
            assert after_first is not None
            assert step.leg_seconds < step.total_seconds, (
                "the leg clock must have been reset at the waypoint")
            assert abs(step.settling_seconds - step.leg_seconds) < 1e-9
            return
    raise AssertionError("the park never finished")


def test_the_cursor_WAITS_when_the_arm_falls_behind() -> None:
    """⭐ The trajectory is a shape. A command racing ahead while the arm cuts its own
    corner is not the shape anyone chose."""
    robot = FakeRobot(q=[0.0] * 7, follow=False)
    arm = ArmSession(robot, name="B")
    arm.begin_path(to([2.0] * 6 + [0.5]), t=0.0)
    for i in range(200):
        arm.step_path(i * 0.01, 0.01)
    stalled_at = arm.park_s
    for i in range(200, 300):
        arm.step_path(i * 0.01, 0.01)
    assert arm.park_s == stalled_at, "the cursor kept going while the arm was stuck"
    assert stalled_at < arm.park_path.length, "it should not have reached the end"


def test_abandoning_a_run_reports_how_much_path_was_dropped() -> None:
    """⛔ An arm that resumes a trajectory after the operator pressed HOLD is doing
    something nobody asked for — and it must say so, not just stop."""
    arm = ArmSession(FakeRobot(q=[0.0] * 7), name="B")
    arm.begin_path(to([1.0] * 6 + [0.5]), t=0.0)
    dropped = arm.abandon_path()
    assert dropped > 0.9, f"most of the path was still ahead, got {dropped:.3f}"
    assert arm.park_path is None and arm.park_s == 0.0
    assert arm.abandon_path() == 0.0, "abandoning twice must be harmless"


# ------------------------------------------------------- two arms at once ----


def test_two_sessions_share_no_state() -> None:
    """⭐ THE WHOLE POINT. If any of this were class-level or shared, one arm's mode,
    gripper value or park target would silently follow the other — with two 4.3 kg
    arms on one desk."""
    a = ArmSession(FakeRobot(q=[0.0] * 7), name="B")
    b = ArmSession(FakeRobot(q=[1.0] * 7), name="G")
    a.enter_teleop()
    a.gripper_value = 0.9
    a.begin_path(to([0.5] * 6 + [0.5]), t=0.0)
    a.park_speed = 0.9
    b.enter_hold()
    assert b.mode == "hold" and a.mode == "park"
    assert b.gripper_value != 0.9
    assert b.park_path is None, "one arm's path must not become the other's"
    assert b.park_speed != 0.9, "the live knobs are per arm too"
    assert b.thermal is not a.thermal, "a shared thermal guard would hide one arm"
    assert not np.shares_memory(a.prev_q, b.prev_q)


def test_each_arm_keeps_its_own_frame() -> None:
    """Per-frame maps are already per-arm; the frame itself must be too, or pressing
    `v` on one arm silently re-interprets the other arm's puck."""
    a = ArmSession(FakeRobot(), name="B", frame="world")
    b = ArmSession(FakeRobot(), name="G", frame="tool")
    assert (a.frame, b.frame) == ("world", "tool")



# ── the restructure's one silent hazard, pinned 2026-08-14 (FINDINGS §50) ─────


def test_the_class_defaults_to_hold_which_is_why_the_script_must_override_it() -> None:
    """⛔⭐⭐ THE PAIR OF FACTS THAT MAKES THE HAZARD VISIBLE.

    `ArmSession` defaults to `hold`, which is right for a class that may be built before
    anyone has chosen a mode. The script has already chosen one, from `--start-mode`, and
    `build_robot()` has already acted on it by deciding `zero_gravity`.

    ⛔ So if the script ever stops assigning `arm.mode = start_mode`, then
    `--start-mode guide` builds a WEIGHTLESS robot and runs the loop believing it is in
    HOLD. Nothing raises. The arm hangs from gravity compensation while the screen says
    HOLD, which is the defect class FINDINGS §0 exists for.
    """
    arm = ArmSession(FakeRobot(), name="B")
    assert arm.mode == "hold", "the class default changed; re-read FINDINGS §50"


def test_the_script_still_hands_its_start_mode_to_the_class() -> None:
    """⚠️ A source check, because this line lives in `main()` and no headless test can
    reach it. If it is ever deleted the failure is silent, so the check is cheap
    insurance rather than a real test."""
    src = (REPO / "scripts" / "teleop_session.py").read_text()
    assert "arm.mode = start_mode" in src, (
        "the script no longer passes --start-mode to the ArmSession, so a guide start "
        "would run as HOLD with a weightless arm"
    )
    assert "zero_gravity=(start_mode ==" in src, (
        "build_robot no longer reads start_mode; if it reads arm.mode instead it runs "
        "before the object exists"
    )

def _rejected(single, spec) -> str:  # noqa: ANN001
    """The message `parse_arms` refuses with, or a failure if it accepted the input."""
    try:
        got = parse_arms(single, spec, ARM_SERIALS, DEFAULT_ARM)
    except ValueError as exc:
        return str(exc)
    raise AssertionError(f"parse_arms({single!r}, {spec!r}) returned {got} instead of "
                         "refusing")


def test_no_flags_means_the_default_arm() -> None:
    assert parse_arms(None, None, ARM_SERIALS, DEFAULT_ARM) == [DEFAULT_ARM]


def test_the_one_arm_spelling_still_works() -> None:
    """⭐ `--arm G` is what every other script here takes and what Julien types. It must
    keep working unchanged now that `--arms` exists."""
    assert parse_arms("G", None, ARM_SERIALS, DEFAULT_ARM) == ["G"]


def test_the_list_spelling_keeps_the_order_it_was_given() -> None:
    """⚠️ Order is not cosmetic: it decides which arm is asked for its puck first, and
    which one the `a` selector starts on."""
    assert parse_arms(None, "B,G", ARM_SERIALS, DEFAULT_ARM) == ["B", "G"]
    assert parse_arms(None, "G,B", ARM_SERIALS, DEFAULT_ARM) == ["G", "B"]


def test_spaces_around_a_name_are_forgiven() -> None:
    assert parse_arms(None, "B, G", ARM_SERIALS, DEFAULT_ARM) == ["B", "G"]


def test_both_spellings_are_fine_when_they_agree() -> None:
    assert parse_arms("G", "G", ARM_SERIALS, DEFAULT_ARM) == ["G"]


def test_the_two_spellings_may_not_disagree() -> None:
    """⛔ Refused rather than resolved by a precedence rule nobody would remember."""
    assert "disagree" in _rejected("B", "G")


def test_the_same_arm_may_not_appear_twice() -> None:
    """⛔ Two `ArmSession` objects over one CAN bus would command the same motors twice
    a cycle from two sets of cached state, and nothing would raise."""
    assert "more than once" in _rejected(None, "B,B")


def test_an_unknown_arm_is_named_in_the_refusal() -> None:
    """⚠️ The message has to say what the rig actually has. A bare "invalid choice" sent
    Julien to the source once already."""
    message = _rejected(None, "B,X")
    assert "X" in message and "B" in message and "G" in message


def test_an_empty_entry_is_refused_rather_than_dropped() -> None:
    """`--arms B,` is a typo, and silently reading it as one arm would hide it."""
    assert "empty entry" in _rejected(None, "B,")
    assert "empty entry" in _rejected(None, "")


def test_the_selector_starts_on_the_first_arm() -> None:
    """⚠️ Not on BOTH. A session that opens with both arms selected would make the first
    `g` anyone presses weightless on 8.6 kg."""
    assert ArmSelector(["B", "G"]).label == "B"
    assert ArmSelector(["B", "G"]).names() == ["B"]


def test_the_selector_visits_each_arm_before_BOTH() -> None:
    sel = ArmSelector(["B", "G"])
    assert [sel.cycle() for _ in range(3)] == ["G", "BOTH", "B"]


def test_BOTH_means_every_arm() -> None:
    sel = ArmSelector(["B", "G"])
    sel.cycle(); sel.cycle()
    assert sel.label == "BOTH"
    assert sel.names() == ["B", "G"]


def test_one_arm_has_nothing_to_cycle_and_says_so() -> None:
    """⭐ No invented BOTH for a single arm: BOTH of one arm is that arm, and a key that
    appears to change something while changing nothing is FINDINGS §17.1."""
    sel = ArmSelector(["B"])
    assert sel.only_one()
    assert sel.cycle() == "B"
    assert sel.names() == ["B"]


def test_a_selector_needs_an_arm() -> None:
    try:
        ArmSelector([])
    except ValueError:
        return
    raise AssertionError("an empty session was accepted")


def _dry_run(*flags: str):  # noqa: ANN202
    """Run the session script with no `--yes`, and return the finished process.

    ⚠️ Safe by construction: without `--yes` the script prints its plan and returns
    before it opens the SpaceMouse or builds a robot, so nothing is energised and no
    device is touched. That is also why this is the only way to reach `main()` at all
    from a headless test.
    """
    return subprocess.run(  # noqa: S603
        [sys.executable, str(REPO / "scripts" / "teleop_session.py"), *flags],
        capture_output=True, text=True, timeout=120, check=False)


def test_one_arm_still_dry_runs_under_both_spellings() -> None:
    for flags in (("--arm", "B"), ("--arms", "B"), ()):
        done = _dry_run(*flags)
        assert done.returncode == 0, f"{flags} exited {done.returncode}: {done.stderr[-400:]}"
        assert "DRY RUN" in done.stdout, f"{flags} printed no dry-run line"


def test_two_arms_now_START_where_they_used_to_be_refused() -> None:
    """⭐⭐ THE DELETED TEST, REPLACED RATHER THAN REMOVED — and this is the moment it was
    written for.

    Until 2026-08-14 `--arms B,G` was refused, because the script below `ArmSession` was
    single-arm: one puck, one axis map, one robot, one park pose. A test pinned that refusal
    **and said in its own docstring that finishing step 2 would mean deliberately deleting
    it**, so the refusal could neither outlive its reason nor quietly vanish before it
    (FINDINGS §52.1).

    ⛔ It is deleted now, and this test replaces it with the opposite claim: two arms PLAN,
    each with its own axis map and its own park pose. ⚠️ The dry run stops before anything is
    built, so this proves the plan and the flags, never the hardware.
    """
    done = _dry_run("--arms", "B,G", "--start-mode", "hold")
    assert done.returncode == 0, f"two arms were refused: {done.stderr[-400:]}"
    for expected in ("ARM         : B", "ARM         : G",
                     "axis map B", "axis map G", "park pose B", "park pose G"):
        assert expected in done.stdout, f"the plan is missing {expected!r}"


def test_two_arms_may_not_START_weightless() -> None:
    """⛔ ROADMAP §6's ruling: two arms going weightless on a first run is the worst possible
    first run. `g` reaches the same state, but only after the operator has deliberately
    selected BOTH and pressed a key at a rig they are watching."""
    done = _dry_run("--arms", "B,G", "--start-mode", "guide")
    assert done.returncode != 0, "two arms were allowed to start weightless"
    assert "refused" in done.stderr and "falling arm" in done.stderr, (
        f"the refusal no longer explains itself: {done.stderr[-300:]}")


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
