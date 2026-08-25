#!/usr/bin/env python3
"""Tests for the jaw pause — a run splits where only the jaws move, holds, and waits.

    uv run tests/test_jaw_pause.py

⛔ WHY THIS FEATURE IS SAFETY-ADJACENT AND NOT COSMETIC. The pause is where a grab happens: the arm stops at the object, the jaws are commanded, and the run waits for them. Get the wait wrong in one direction and the run resumes with the jaws mid-travel (the object is shoved or dropped); get it wrong in the other and a jammed gripper stops the run for ever while pushing at full stall current — which is how motor 7 was cooked three times. So the wait is MEASURED (the jaws are still) with a bounded timeout, and both directions are pinned here.

⭐ The fake robot's jaw moves at a finite rate and can be blocked by an "object" at a chosen width, because the interesting cases are exactly the ones where the jaw does NOT reach its command: a successful grab and a jam look identical to a position check, and only the stillness rule tells them apart from mid-travel.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.session import (  # noqa: E402
    JAW_MIN_WAIT,
    JAW_SETTLE_SECONDS,
    JAW_TIMEOUT_SECONDS,
    ArmSession,
    ParkLeg,
)

N_ARM = 6
DT = 1.0 / 90.0


class FakeChain:
    running = True

    def read_states(self):  # noqa: ANN201
        return []


class JawRobot:
    """An arm whose ARM joints follow exactly and whose JAW moves at a finite rate.

    `jaw_floor` is an object between the jaws: the measured jaw can never go below it, exactly like a physical grab. `jitter=True` makes the jaw wobble for ever after arriving, which is the only way to reach the pause timeout.
    """

    def __init__(self, q=None, jaw_rate=2.0, jaw_floor=None, jitter=False,
                 arm_rate=None, arm_floor=0.0):  # noqa: ANN001
        self.q = np.array(q if q is not None else [0.0] * 7, dtype=float)
        self.commands: list[np.ndarray] = []
        self.motor_chain = FakeChain()
        self.jaw_rate = jaw_rate
        self.jaw_floor = jaw_floor
        self.jitter = jitter
        self._flip = 1.0
        # ⭐ The arm side can trail too: `arm_rate` (rad/s, None = follows exactly) and
        # `arm_floor` (each joint sticks once within this of its command — stiction, the
        # measured 0.02-0.08 rad floor of FINDINGS §69.2 in its simplest form).
        self.arm_rate = arm_rate
        self.arm_floor = arm_floor

    def get_joint_pos(self):  # noqa: ANN201
        return self.q.copy()

    def command_joint_pos(self, cmd) -> None:  # noqa: ANN001
        cmd = np.asarray(cmd, dtype=float)
        self.commands.append(cmd.copy())
        if self.arm_rate is None and self.arm_floor == 0.0:
            self.q[:N_ARM] = cmd[:N_ARM]
        else:
            delta = cmd[:N_ARM] - self.q[:N_ARM]
            rate = self.arm_rate if self.arm_rate is not None else 1e9
            step = np.clip(delta, -rate * DT, rate * DT)
            self.q[:N_ARM] += np.where(np.abs(delta) > self.arm_floor, step, 0.0)
        if len(cmd) > N_ARM and len(self.q) > N_ARM:
            gap = cmd[N_ARM] - self.q[N_ARM]
            step = np.clip(gap, -self.jaw_rate * DT, self.jaw_rate * DT)
            jaw = self.q[N_ARM] + step
            if self.jaw_floor is not None:
                jaw = max(jaw, self.jaw_floor)
            if self.jitter and abs(gap) < 0.05:
                self._flip = -self._flip
                jaw += 0.01 * self._flip
            self.q[N_ARM] = jaw

    def num_dofs(self) -> int:
        return len(self.q)


def make_arm(robot):  # noqa: ANN001, ANN201
    return ArmSession(robot, "B")


def drive(arm, t, until, max_seconds=30.0):  # noqa: ANN001, ANN201
    """Step the park until `until(ps)` is true; returns (t, that step, all steps)."""
    steps = []
    deadline = t + max_seconds
    while t < deadline:
        t += DT
        ps = arm.step_path(t, DT)
        steps.append(ps)
        if until(ps):
            return t, ps, steps
    raise AssertionError(f"never reached the condition; last verdict "
                         f"{steps[-1].verdict if steps else 'none'}")


#: A grab saved the natural way: approach with the jaws open, close on the spot, lift.
#: Leg w2→w3 changes only the jaws, so the run must split exactly once.
GRAB_LEGS = [
    ParkLeg("1", [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9]),   # above the object, open
    ParkLeg("2", [0.3, 0.4, 0.0, 0.0, 0.0, 0.0, 0.9]),   # at the object, open
    ParkLeg("3", [0.3, 0.4, 0.0, 0.0, 0.0, 0.0, 0.1]),   # at the object, closed
    ParkLeg("4", [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]),   # lifted, closed
]


def test_a_jaws_only_leg_splits_the_run() -> None:
    robot = JawRobot(q=[0, 0, 0, 0, 0, 0, 0.9])
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    assert arm.park_stops == 1, f"one jaws-only leg must queue one stop, got {arm.park_stops}"
    assert arm.park_total_length > arm.park_path.length, \
        "the queued segment must count toward the total travel"


def test_a_plain_run_is_one_segment_and_never_pauses() -> None:
    robot = JawRobot(q=[0, 0, 0, 0, 0, 0, 0.5])
    arm = make_arm(robot)
    arm.begin_path([ParkLeg("1", [0.3, 0, 0, 0, 0, 0, 0.5]),
                    ParkLeg("2", [0.3, 0.4, 0, 0, 0, 0, 0.5])], t=0.0)
    assert arm.park_stops == 0
    _, ps, steps = drive(arm, 0.0, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert ps.verdict in ("arrived", "settled"), ps.verdict
    assert all(p.verdict != "jaws" for p in steps), "a plain run must never enter the jaw phase"


def test_the_arm_holds_while_the_jaws_travel() -> None:
    robot = JawRobot(q=[0, 0, 0, 0, 0, 0, 0.9])
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, first, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    assert first.verdict == "jaws" and first.jaw_name == "3"
    assert abs(first.jaw_target - 0.1) < 1e-9
    n_before = len(robot.commands)
    t, done, steps = drive(arm, t, lambda p: p.jaw_done)
    pause_cmds = robot.commands[n_before:]
    arms_held = {tuple(np.round(c[:N_ARM], 9)) for c in pause_cmds}
    assert len(arms_held) == 1, "the ARM must hold one pose for the whole pause"
    assert all(abs(c[N_ARM] - 0.1) < 1e-9 for c in pause_cmds), \
        "every pause cycle must command the leg's jaw value"
    assert sum(1 for p in steps if p.jaw_started) == 0, "jaw_started fires exactly once"


def test_the_wait_is_measured_not_timed() -> None:
    slow, fast = JawRobot(q=[0] * 6 + [0.9], jaw_rate=0.5), JawRobot(q=[0] * 6 + [0.9], jaw_rate=4.0)
    waits = []
    for robot in (slow, fast):
        arm = make_arm(robot)
        arm.begin_path(GRAB_LEGS, t=0.0)
        t, _, _ = drive(arm, 0.0, lambda p: p.jaw_started)
        _, done, _ = drive(arm, t, lambda p: p.jaw_done)
        assert not done.jaw_timed_out
        waits.append(done.jaw_seconds)
    travel = 0.8  # stroke from 0.9 to 0.1
    assert waits[0] > waits[1] + 0.5, \
        f"a slower jaw must be waited on longer: {waits}"
    assert waits[0] >= travel / 0.5, "the wait cannot be shorter than the jaw's own travel"
    assert abs(waits[1] - max(travel / 4.0 + JAW_SETTLE_SECONDS, JAW_MIN_WAIT)) < 0.15, \
        f"the fast wait should be travel + the settle window, got {waits[1]:.2f}"


def test_a_grab_reports_holding() -> None:
    robot = JawRobot(q=[0] * 6 + [0.9], jaw_floor=0.3)   # an object 0.3 of the stroke wide
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, _, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    _, done, _ = drive(arm, t, lambda p: p.jaw_done)
    assert done.grasp is not None and done.grasp.confident
    assert done.grasp.holding, "jaws stopped 0.2 short of the command — that IS an object"
    assert abs(done.grasp.gap - 0.2) < 0.02


def test_an_empty_close_reports_empty_and_the_run_finishes() -> None:
    robot = JawRobot(q=[0] * 6 + [0.9])
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, _, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    t, done, _ = drive(arm, t, lambda p: p.jaw_done)
    assert done.grasp is not None and done.grasp.confident and not done.grasp.holding
    _, end, _ = drive(arm, t, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled"), end.verdict


def test_an_opening_leg_stays_silent_about_grasping() -> None:
    legs = [ParkLeg("1", [0.3, 0, 0, 0, 0, 0, 0.1]),
            ParkLeg("2", [0.3, 0, 0, 0, 0, 0, 0.9])]     # only the jaws OPEN
    robot = JawRobot(q=[0] * 6 + [0.1])
    arm = make_arm(robot)
    arm.begin_path(legs, t=0.0)
    t, _, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    _, done, _ = drive(arm, t, lambda p: p.jaw_done)
    assert done.grasp is not None and not done.grasp.confident, \
        "an opening command says nothing about an object, and the check must say so"


def test_a_jaw_that_never_settles_times_out_and_the_run_continues() -> None:
    robot = JawRobot(q=[0] * 6 + [0.9], jitter=True)
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, _, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    t, done, _ = drive(arm, t, lambda p: p.jaw_done, max_seconds=JAW_TIMEOUT_SECONDS + 2)
    assert done.jaw_timed_out, "a for-ever-moving jaw must stop gating the run"
    assert abs(done.jaw_seconds - JAW_TIMEOUT_SECONDS) < 0.1
    assert done.grasp is not None and not done.grasp.confident, \
        "a timed-out close was never settled, so the grasp check must refuse to answer"
    _, end, _ = drive(arm, t, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled"), end.verdict


def test_err_and_lag_ignore_a_held_object() -> None:
    """⛔ The regression this whole design guards: a SUCCESSFUL grab must not read as a blocked park. The held object keeps the jaw 0.2 from its command for the entire lift, and the run must still arrive."""
    robot = JawRobot(q=[0] * 6 + [0.9], jaw_floor=0.3)
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, done, _ = drive(arm, 0.0, lambda p: p.jaw_done)
    _, end, steps = drive(arm, t, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled"), \
        f"a run that grabbed something must still finish, got {end.verdict}"
    assert end.err < 0.06, "the ARM error must be judged without the jaw gap"


def test_a_latched_jaw_block_is_honoured_by_the_park() -> None:
    """The stall guard latched a block (an object) before the run, and the run's poses keep the jaws closed. Every park command must hold at the block instead of pushing into the object at 90 Hz — the exact §58.2 shape the old park had. ⚠️ A run that OPENS the jaws clears the latch on purpose (that is `hold_jaw`'s designed release), so this test never opens them."""
    robot = JawRobot(q=[0] * 6 + [0.4], jaw_floor=0.4)
    arm = make_arm(robot)
    arm.block_jaw_at(0.4)
    legs = [ParkLeg("1", [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]),
            ParkLeg("2", [0.3, 0.4, 0.0, 0.0, 0.0, 0.0, 0.1])]   # closed throughout
    arm.begin_path(legs, t=0.0)
    _, end, _ = drive(arm, 0.0, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled"), end.verdict
    closing = [c for c in robot.commands if len(c) > N_ARM and c[N_ARM] < 0.39]
    assert not closing, "no command may push the jaws past the latched block"
    assert arm.jaw_block is not None, "nothing opened the jaws, so the latch must survive"


def test_abandoning_counts_the_queued_segments_too() -> None:
    robot = JawRobot(q=[0] * 6 + [0.9])
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    total = arm.park_total_length
    dropped = arm.abandon_path()
    assert abs(dropped - total) < 1e-9, \
        f"abandoning at the start must report the WHOLE run: {dropped} vs {total}"
    assert arm.park_stops == 0 and arm.park_jaw is None, "the queue and the pause must clear"


def test_the_preview_count_matches_the_run() -> None:
    robot = JawRobot(q=[0] * 6 + [0.9])
    arm = make_arm(robot)
    preview = arm.count_gripper_stops(GRAB_LEGS)
    arm.begin_path(GRAB_LEGS, t=0.0)
    assert preview == arm.park_stops == 1, (preview, arm.park_stops)


def test_six_joint_poses_never_split() -> None:
    """`--no-gripper` saves 6-value poses; there is no jaw to pause for."""
    robot = JawRobot(q=[0.0] * 6)
    arm = make_arm(robot)
    legs = [ParkLeg("1", [0.3, 0, 0, 0, 0, 0]), ParkLeg("2", [0.3, 0.4, 0, 0, 0, 0])]
    assert arm.count_gripper_stops(legs) == 0
    arm.begin_path(legs, t=0.0)
    assert arm.park_stops == 0
    _, end, steps = drive(arm, 0.0, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled")
    assert all(p.verdict != "jaws" for p in steps)


def test_mixed_leg_advice_can_be_silenced_for_replay_parks() -> None:
    legs = [ParkLeg("start", [0.5, 0.2, 0, 0, 0, 0, 0.2])]   # arm AND jaws both change
    robot = JawRobot(q=[0] * 6 + [0.9])
    arm = make_arm(robot)
    with_advice = arm.begin_path(legs, t=0.0)
    assert any("moves the arm AND the gripper" in w for w in with_advice), \
        "a mixed leg must be reported on a waypoint run"
    arm2 = make_arm(JawRobot(q=[0] * 6 + [0.9]))
    without = arm2.begin_path(legs, t=0.0, mixed_leg_advice=False)
    assert not any("moves the arm AND the gripper" in w for w in without), \
        "a replay park cannot act on waypoint advice, so it must not print it"


def test_the_jaws_wait_for_a_trailing_arm_to_settle() -> None:
    """His 2026-08-18 grab missed by millimetres: the jaws closed while the arm was still creeping. With a trailing arm, the pause must not open until the arm is within tolerance of the split waypoint, and the reported offset must say where it actually settled."""
    robot = JawRobot(q=[0] * 6 + [0.9], arm_rate=0.5)
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    _, first, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    assert first.jaw_arm_off is not None and first.jaw_arm_off < 0.02, \
        f"the jaws started while the arm was {first.jaw_arm_off} rad from the split pose"


def test_a_sticking_arm_does_not_gate_the_jaws_for_ever() -> None:
    """Stiction leaves each joint short of its command permanently. The gate must give up after the settle window and report the offset honestly, never hang the run."""
    robot = JawRobot(q=[0] * 6 + [0.9], arm_rate=2.0, arm_floor=0.04)
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    t, first, _ = drive(arm, 0.0, lambda p: p.jaw_started)
    assert first.jaw_arm_off is not None and 0.02 <= first.jaw_arm_off <= 0.05, \
        f"a sticking arm should report its floor as the offset, got {first.jaw_arm_off}"
    _, end, _ = drive(arm, t, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict == "settled", \
        f"a floor inside the settled band must still finish the run, got {end.verdict}"


def test_an_exact_arm_pays_nothing_for_the_gate() -> None:
    """An arm already at the split pose must start the jaws on the very next cycle after the cursor finishes — the gate is free when there is nothing to settle."""
    robot = JawRobot(q=[0] * 6 + [0.9])
    arm = make_arm(robot)
    arm.begin_path(GRAB_LEGS, t=0.0)
    _, _, steps = drive(arm, 0.0, lambda p: p.jaw_started)
    mark_i = max(i for i, p in enumerate(steps) if p.leg_passed == "2")
    start_i = len(steps) - 1
    assert start_i - mark_i <= 1, \
        f"{start_i - mark_i} cycles between the split waypoint and the jaws starting"
    assert steps[-1].jaw_arm_off is not None and steps[-1].jaw_arm_off < 1e-6


def test_a_first_leg_that_only_moves_the_jaws_pauses_immediately() -> None:
    """`p 3` where slot 3 is the current pose with closed jaws: the run is one pause."""
    robot = JawRobot(q=[0.3, 0.4, 0, 0, 0, 0, 0.9], jaw_floor=0.25)
    arm = make_arm(robot)
    arm.begin_path([ParkLeg("3", [0.3, 0.4, 0, 0, 0, 0, 0.1])], t=0.0)
    assert arm.park_stops == 1
    t, first, _ = drive(arm, 0.0, lambda p: p.jaw_started, max_seconds=1.0)
    t, done, _ = drive(arm, t, lambda p: p.jaw_done)
    assert done.grasp is not None and done.grasp.holding
    _, end, _ = drive(arm, t, lambda p: p.verdict in ("arrived", "settled", "blocked"))
    assert end.verdict in ("arrived", "settled")


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
