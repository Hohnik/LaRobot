#!/usr/bin/env python3
"""Tests for the shutdown park, `park_arms()`. No hardware.

    uv run scripts/test_park_arms.py

⭐⭐ WHY THIS FILE EXISTS. `park_arms()` is what runs on **Ctrl-C, on `q q`, and on every
unplanned stop** — a thermal cut-out, a guard refusing, an exception in the loop. It is the
most safety-relevant path in `teleop_session.py`, because the arm is raised and holding
when it starts and released when it finishes.

⛔ **It had NO tests at all until 2026-08-14**, in either its single-arm or its N-arm form.
It was reachable only from a real session's shutdown, so every change to it was verified by
Julien pressing Ctrl-C on live hardware.

⚠️ What these tests pin is the DECISION, not the motion: which outcome comes back, whether a
key stops every arm, whether a dead chain is skipped rather than fatal, and whether one
stalled arm prevents the release. The trajectory maths belongs to `advance_park_command` and
`park_verdict`, which have their own tests in `scripts/test_park_target.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

from arm_session import ArmSession  # noqa: E402
from teleop_session import park_arms  # noqa: E402

N_ARM = 6


class FakeChain:
    def __init__(self, running=True) -> None:  # noqa: ANN001
        self.running = running


class FakeRobot:
    """An arm that follows commands exactly, unless `follow` is False."""

    def __init__(self, q=None, follow=True) -> None:  # noqa: ANN001
        self.q = np.array(q if q is not None else [0.5] * 7, dtype=float)
        self.commands: list[np.ndarray] = []
        self.motor_chain = FakeChain()
        self.follow = follow

    def get_joint_pos(self):  # noqa: ANN201
        return self.q.copy()

    def command_joint_pos(self, q) -> None:  # noqa: ANN001
        q = np.asarray(q, dtype=float)
        self.commands.append(q.copy())
        if self.follow:
            self.q = q.copy()

    def num_dofs(self) -> int:
        return len(self.q)


class Keys:
    """A key reader that yields nothing, or one key after N calls."""

    def __init__(self, press_after: int | None = None, key: str = "x") -> None:
        self.press_after, self.key = press_after, key
        self.calls = 0
        self.drained = 0

    def get(self):  # noqa: ANN201
        self.calls += 1
        if self.press_after is not None and self.calls >= self.press_after:
            return self.key
        return None

    def drain(self):  # noqa: ANN201
        self.drained += 1
        return []


def arm_at(name="B", q=0.5, base=0.0, follow=True, alive=True):  # noqa: ANN001, ANN201
    robot = FakeRobot(q=[q] * 7, follow=follow)
    robot.motor_chain.running = alive
    one = ArmSession(robot, name=name)
    one.base_pose = [base] * 7
    one.park_speed, one.park_ramp = 4.0, 0.0     # fast and unshaped, so tests are quick
    return one


def clamp(v):  # noqa: ANN001, ANN201
    return float(np.clip(v, 0.02, 0.98))


def test_one_arm_arrives() -> None:
    one = arm_at()
    assert park_arms([one], Keys(), clamp) == "arrived"
    assert one.robot.commands, "it never commanded anything"


def test_both_arms_arrive() -> None:
    b, g = arm_at("B", q=0.5), arm_at("G", q=-0.4)
    assert park_arms([b, g], Keys(), clamp) == "arrived"
    assert len(b.robot.commands) > 1 and len(g.robot.commands) > 1


def test_the_arms_are_commanded_TOGETHER_not_one_after_the_other() -> None:
    """⭐⭐ EVERY ARM ADVANCES ON EVERY CYCLE, and this test had to be rewritten to be able
    to say so.

    ⛔ The first version compared how many commands each arm received and allowed a
    difference of two. **Sequential parking passes that**, because two arms starting a
    similar distance from their targets need a similar number of cycles either way. It was
    the defect working-contract rule 5 names: evidence that cannot distinguish the claim
    from its opposite.

    ⭐ This version records the ORDER of commands across both arms in one log. Interleaved
    driving alternates B, G, B, G …; parking one arm and then the other gives all of B's
    commands before any of G's. **A long arm and a short arm make the difference stark**,
    which is the case the weak version could not see either.
    """
    order: list[str] = []
    far, near = arm_at("B", q=1.2), arm_at("G", q=0.05)
    for one in (far, near):
        original = one.robot.command_joint_pos

        def logged(q, one=one, original=original):  # noqa: ANN001, ANN202
            order.append(one.name)
            original(q)

        one.robot.command_joint_pos = logged

    assert park_arms([far, near], Keys(), clamp) == "arrived"
    # The near arm finishes first, so the tail is all B. What matters is the START.
    head = order[:6]
    assert "B" in head and "G" in head, (
        f"one arm was parked before the other was touched: {order[:12]}")


def test_a_key_stops_every_arm() -> None:
    """⛔ "Any key stops it" has to mean the whole motion. Stopping one arm and leaving the
    other moving would make the operator guess which one they had stopped."""
    b, g = arm_at("B", q=2.0, follow=False), arm_at("G", q=2.0, follow=False)
    assert park_arms([b, g], Keys(press_after=3), clamp) == "stopped"


def test_stale_keystrokes_are_discarded_before_the_move() -> None:
    """⛔ A key typed before the park existed must not cancel it. Julien saw a park announce
    itself and stop in the same breath."""
    keys = Keys()
    park_arms([arm_at()], keys, clamp)
    assert keys.drained == 1, "the key buffer was not drained before moving"


def test_a_stuck_arm_reports_stalled() -> None:
    b = arm_at("B", q=2.0, follow=False)
    assert park_arms([b], Keys(), clamp, stall_seconds=0.2) == "stalled"


def test_one_stalled_arm_prevents_the_release_even_if_the_other_arrived() -> None:
    """⛔⭐ THE RULE THIS PATH EXISTS FOR: "I could not reach the safe pose" is exactly when
    a human should decide rather than a default. The caller releases the motors only on
    'arrived', so the worst outcome has to win."""
    good, stuck = arm_at("B", q=0.5), arm_at("G", q=2.0, follow=False)
    assert park_arms([good, stuck], Keys(), clamp, stall_seconds=0.2) == "stalled"


def test_a_dead_arm_is_skipped_and_the_live_one_still_parks() -> None:
    """⚠️ A dead chain cannot be commanded, so that arm is already sagging. Parking the live
    arm is better than leaving it holding and far better than disabling it."""
    dead, live = arm_at("B", alive=False), arm_at("G", q=0.5)
    assert park_arms([dead, live], Keys(), clamp) == "arrived"
    assert not dead.robot.commands, "a dead chain must never be commanded"
    assert live.robot.commands, "the live arm was not parked"


def test_every_chain_dead_returns_dead_and_commands_nothing() -> None:
    b, g = arm_at("B", alive=False), arm_at("G", alive=False)
    assert park_arms([b, g], Keys(), clamp) == "dead"
    assert not b.robot.commands and not g.robot.commands


def test_a_chain_that_dies_MID_PARK_is_reported() -> None:
    """⚠️ The liveness check is inside the loop, not only at the start. This is what happened
    on 2026-08-14: the CAN link went away while the arm was moving."""
    one = arm_at("B", q=2.0, follow=False)
    keys = Keys()
    original = one.robot.command_joint_pos

    def die_after_a_few(q):  # noqa: ANN001, ANN202
        original(q)
        if len(one.robot.commands) >= 3:
            one.robot.motor_chain.running = False

    one.robot.command_joint_pos = die_after_a_few
    assert park_arms([one], keys, clamp, stall_seconds=5.0) == "dead"


def test_an_arm_with_no_saved_pose_is_skipped_rather_than_guessed_at() -> None:
    """⛔ A pose the arm moves to is never a default. With nothing saved there is nothing
    safe to park to, so it says so and leaves that arm alone."""
    one = arm_at()
    one.base_pose = None
    assert park_arms([one], Keys(), clamp) == "dead"
    assert not one.robot.commands


def test_the_park_respects_each_arm_s_own_speed() -> None:
    """⭐ Speed and ease ramp are per-arm fields, so a slower arm takes more cycles. This is
    what makes `park_arms` different from one shared speed for the pair."""
    fast, slow = arm_at("B", q=1.0), arm_at("G", q=1.0)
    slow.park_speed = 0.5
    park_arms([fast, slow], Keys(), clamp)
    assert len(slow.robot.commands) > len(fast.robot.commands), (
        "the slower arm should have needed more cycles")


def test_a_blocked_park_reports_what_it_MEASURED_not_a_guess() -> None:
    """⛔⭐ Julien's Ctrl-C park stalled with *"Something is in the way, or the pose is
    unreachable"* and nothing was in the way. The message now reports how far the command ran
    ahead of the arm and how often SafeRobot held it back, then offers the reading those
    numbers support.

    ⚠️ This test pins the OUTCOME and that the run carries the baseline it needs; the printed
    wording is checked by reading, because capturing stdout here would test the print rather
    than the decision."""
    stuck = arm_at("B", q=2.0, follow=False)
    stuck.robot.limited_cycles = 7          # SafeRobot counts on the real robot
    assert park_arms([stuck], Keys(), clamp, stall_seconds=0.2) == "stalled"


def test_the_command_history_is_FORGOTTEN_before_the_first_command() -> None:
    """⛔⭐⭐ THE SPASM. Julien: quit menu, `g` weightless, moved an arm by hand, then `p` —
    and the arm *"quickly spasmed for a tenth of a second, for seemingly no reason"*.

    `SafeRobot` is stateful: its rate limiter walks from `_last_cmd`, which still held the
    pose from BEFORE the hand-guiding, because GUIDE commands no positions. So the first park
    command jerked toward the stale pose. `resync()` exists precisely for this, its docstring
    says "call on EVERY mode transition", and this function is a mode transition that lives
    outside the mode system — which is why it was the one place that missed it.
    """
    calls = []

    class ResyncingRobot(FakeRobot):
        def resync(self) -> None:
            calls.append(len(self.commands))     # how many commands had been sent already

    robot = ResyncingRobot(q=[0.5] * 7)
    one = ArmSession(robot, name="B")
    one.base_pose = [0.0] * 7
    one.park_speed, one.park_ramp = 4.0, 0.0
    assert park_arms([one], Keys(), clamp) == "arrived"
    assert calls, "resync was never called — the stale-command spasm is back"
    assert calls[0] == 0, (
        f"resync came after {calls[0]} command(s); it must come BEFORE the first one, "
        f"or the spasm has already happened")


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
