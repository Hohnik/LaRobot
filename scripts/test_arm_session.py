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

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from arm_session import ArmSession, ParkLeg  # noqa: E402

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
    def __init__(self, temp_mos=30.0, temp_rotor=30.0):  # noqa: ANN001
        self.temp_mos, self.temp_rotor = temp_mos, temp_rotor


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
    t = 0.0
    while t < seconds:
        verdict, err = arm.step_park(t, dt)
        if verdict != "moving":
            return verdict, err, t
        t += dt
    return "timeout", err, t


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
    arm.begin_park([0.0] * 6 + [0.5], t=0.0)
    verdict, err, _ = run_park(arm, robot)
    assert verdict in ("arrived", "settled"), verdict
    assert err < 0.06


def test_a_stuck_arm_is_BLOCKED_not_quietly_finished() -> None:
    """⛔ The case that must never be softened away by the settled band — the thing
    in the way might be a hand."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5], follow=False)
    arm = ArmSession(robot, name="B")
    arm.begin_park([0.0] * 6 + [0.5], t=0.0)
    verdict, err, _ = run_park(arm, robot, seconds=10.0)
    assert verdict == "blocked", verdict
    assert err > 0.5, "it never moved, so the error should still be the whole distance"


def test_completion_is_judged_from_the_MEASUREMENT_not_the_command() -> None:
    """⛔ The command always arrives first. Judging it declares success while the arm
    is still travelling — a real bug that hid for two sessions."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5], follow=False)
    arm = ArmSession(robot, name="B")
    arm.begin_park([0.0] * 6 + [0.5], t=0.0)
    for i in range(400):
        arm.step_park(i * 0.01, 0.01)
    assert float(np.max(np.abs(arm.park_cmd[:N_ARM]))) < 0.5, "the command should have advanced"
    verdict, err = arm.step_park(4.5, 0.01)
    assert verdict == "blocked", "the arm never moved, so this is not arrival"


def test_the_park_eases_in_rather_than_starting_at_full_speed() -> None:
    """The first step of a leg must be slower than a mid-move step, or every waypoint
    in a sequence is a jerk."""
    robot = FakeRobot(q=[1.0] * 6 + [0.5])
    arm = ArmSession(robot, name="B")
    arm.begin_park([0.0] * 6 + [0.5], t=0.0)
    before = arm.park_cmd.copy()
    arm.step_park(0.0, 0.01)
    first = float(np.max(np.abs(arm.park_cmd - before)))
    for i in range(1, 80):                 # travel past the ramp
        arm.step_park(i * 0.01, 0.01)
    mid_before = arm.park_cmd.copy()
    arm.step_park(0.8, 0.01)
    mid = float(np.max(np.abs(arm.park_cmd - mid_before)))
    assert first < mid, f"first step {first:.5f} was not slower than mid-move {mid:.5f}"


def test_park_with_no_target_refuses_rather_than_crashing() -> None:
    arm = ArmSession(FakeRobot(), name="B")
    verdict, _ = arm.step_park(0.0, 0.01)
    assert verdict == "blocked"


# ------------------------------------------------------------ sequences ----


def test_legs_run_in_order_and_then_stop() -> None:
    robot = FakeRobot(q=[0.0] * 7)
    arm = ArmSession(robot, name="B")
    arm.park_queue = [ParkLeg("1", [0.2] * 6 + [0.5]), ParkLeg("2", [0.4] * 6 + [0.5])]
    first = arm.next_leg(t=0.0)
    assert first.name == "1"
    second = arm.next_leg(t=1.0)
    assert second.name == "2"
    assert arm.next_leg(t=2.0) is None, "the run should end, not loop"


def test_each_leg_gets_its_own_ease_in() -> None:
    """Not just the first — otherwise every waypoint after the first starts abruptly."""
    robot = FakeRobot(q=[0.0] * 7)
    arm = ArmSession(robot, name="B")
    arm.park_queue = [ParkLeg("1", [1.0] * 6 + [0.5])]
    arm.next_leg(t=0.0)
    for i in range(80):
        arm.step_park(i * 0.01, 0.01)
    arm.park_queue = [ParkLeg("2", [2.0] * 6 + [0.5])]
    arm.next_leg(t=1.0)
    assert np.allclose(arm.park_start, arm.park_cmd), (
        "park_start must be re-taken per leg, or the ramp thinks it is mid-move")


def test_abandoning_the_queue_reports_how_many_were_dropped() -> None:
    """⛔ An arm that resumes a queued trajectory after the operator pressed HOLD is
    doing something nobody asked for — and it must say so, not just stop."""
    arm = ArmSession(FakeRobot(), name="B")
    arm.park_queue = [ParkLeg("1", [0.0] * 7), ParkLeg("2", [0.0] * 7)]
    assert arm.abandon_queue() == 2
    assert arm.park_queue == []
    assert arm.abandon_queue() == 0


# ------------------------------------------------------- two arms at once ----


def test_two_sessions_share_no_state() -> None:
    """⭐ THE WHOLE POINT. If any of this were class-level or shared, one arm's mode,
    gripper value or park target would silently follow the other — with two 4.3 kg
    arms on one desk."""
    a = ArmSession(FakeRobot(q=[0.0] * 7), name="B")
    b = ArmSession(FakeRobot(q=[1.0] * 7), name="G")
    a.enter_teleop()
    a.gripper_value = 0.9
    a.park_queue = [ParkLeg("1", [0.0] * 7)]
    b.enter_hold()
    assert b.mode == "hold" and a.mode == "teleop"
    assert b.gripper_value != 0.9
    assert b.park_queue == []
    assert b.thermal is not a.thermal, "a shared thermal guard would hide one arm"
    assert not np.shares_memory(a.prev_q, b.prev_q)


def test_each_arm_keeps_its_own_frame() -> None:
    """Per-frame maps are already per-arm; the frame itself must be too, or pressing
    `v` on one arm silently re-interprets the other arm's puck."""
    a = ArmSession(FakeRobot(), name="B", frame="world")
    b = ArmSession(FakeRobot(), name="G", frame="tool")
    assert (a.frame, b.frame) == ("world", "tool")


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
