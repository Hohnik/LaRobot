#!/usr/bin/env python3
"""Tests for the jaw block latch, `ArmSession.hold_jaw`. No hardware.

    uv run tests/test_jaw_block.py

⭐⭐ WHY THIS EXISTS, and it is docs/ROADMAP.md §8.2 item 29. The gripper stall release
already worked: when the jaws push hard without moving, the command is backed off to the
measured jaw position so the motor stops fighting the object. ⛔ **But it was undone on the
very next cycle.**

Julien's 2026-08-17 log is the evidence, and the numbers tell the whole story:

    ⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.152
    ⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.151
    ⚠️  ARM G GRIPPER STALLED (14 times now) (+1.03 Nm, not moving) — released to 0.150
    ⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.147

⭐ Each release moved the command by about a thousandth of a radian, and each next cycle
MIRROR copied the leader's jaw straight back over it. **A one-cycle correction against a
source that re-commands at 90 Hz can only nibble.** Motor 7 has been cooked three times by
exactly this shape of problem, and pushing hard while not moving is the worst thermal case
there is: full current, no motion, no cooling.

⚠️ `teleop_session.py` already carried a comment describing this precisely, so the diagnosis
existed for days and the fix did not. That is the same pattern as the tracking-table names:
the right answer written down next to the wrong behaviour.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.session import JAW_CLEAR_MARGIN, ArmSession  # noqa: E402


class FakeRobot:
    def __init__(self) -> None:
        self.q = np.zeros(7)

    def get_joint_pos(self):  # noqa: ANN201
        return self.q.copy()

    def command_joint_pos(self, q) -> None:  # noqa: ANN001
        self.q = np.asarray(q, dtype=float).copy()

    def num_dofs(self) -> int:
        return 7


def arm() -> ArmSession:
    return ArmSession(FakeRobot(), name="G")


# ------------------------------------------------------------------ nothing latched


def test_with_nothing_latched_the_command_passes_straight_through() -> None:
    """⭐ The latch must be invisible until a stall actually happens. Every jaw command in
    the session goes through this now, so a default that altered anything would change how
    the gripper behaves in normal use."""
    one = arm()
    for wanted in (0.0, 0.25, 0.5, 0.9, 1.0):
        assert one.hold_jaw(wanted) == wanted


# ------------------------------------------------------------------- the latch holds


def test_a_command_pushing_FURTHER_CLOSED_is_held_at_the_block() -> None:
    """⛔⭐⭐ THE DEFECT FROM HIS LOG. Bigger is more open, so a smaller number is a harder
    squeeze. The leader kept asking for smaller and the follower kept obeying."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.10) == 0.152, "it is still being pushed closed"
    assert one.hold_jaw(0.0) == 0.152
    assert one.hold_jaw(-5.0) == 0.152


def test_the_block_SURVIVES_repeated_pushes() -> None:
    """⛔⭐⭐ THIS IS THE WHOLE POINT. His log shows 0.152, 0.151, 0.150, 0.147 — the command
    creeping closed one cycle at a time. Ninety pushes a second must leave it exactly where
    it was."""
    one = arm()
    one.block_jaw_at(0.152)
    for _ in range(900):                      # ten seconds at 90 Hz
        assert one.hold_jaw(0.05) == 0.152
    assert one.jaw_block == 0.152, "the block drifted"


def test_holding_AT_the_block_is_allowed_so_the_grip_is_kept() -> None:
    """⛔ Releasing entirely would drop whatever is being held. The block value IS the
    measured position where the jaws stalled, so commanding it keeps the grip and stops the
    pushing."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.152) == 0.152
    assert one.jaw_block == 0.152, "commanding the block exactly should not clear it"


# ------------------------------------------------------------------ the latch clears


def test_OPENING_clears_the_latch_and_is_obeyed() -> None:
    """⭐ Anything moving away from the obstruction is evidence it is no longer being pushed
    into. The object may have been put down, or the leader's hand may have opened."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.4) == 0.4
    assert one.jaw_block is None, "the latch did not clear"


def test_after_clearing_a_close_command_is_obeyed_again() -> None:
    """⚠️ A latch that stayed shut after opening would eventually be the reason the jaws
    refused to work for a reason nobody could see."""
    one = arm()
    one.block_jaw_at(0.152)
    one.hold_jaw(0.6)                          # opened, latch clears
    assert one.hold_jaw(0.05) == 0.05, "the jaws are still refusing to close"


def test_a_second_stall_can_latch_again_at_a_new_place() -> None:
    """⭐ Picking up a bigger object next time must latch at the new position rather than the
    old one."""
    one = arm()
    one.block_jaw_at(0.152)
    one.hold_jaw(0.8)                          # cleared
    one.block_jaw_at(0.44)
    assert one.hold_jaw(0.1) == 0.44


def test_the_tiniest_opening_does_NOT_clear_the_latch() -> None:
    """⛔⭐⭐ THIS TEST USED TO ASSERT THE OPPOSITE, AND ARGUED FOR IT. It said *"deliberately
    a strict inequality rather than a tolerance"*, reasoning that a tolerance would let a
    command a hair below the block through.

    ⚠️ **That was the wrong risk to weigh.** A hair BELOW the block is harmless: the jaws are
    still not pushing. A hair ABOVE it disarms the whole mechanism, and a jaw position read
    off a motor jitters by thousandths every cycle — the leader in MIRROR is a hand-held arm
    being squeezed, so its measured jaw never sits still.

    ⛔ His 2026-08-17 log is what exposed it: three stalls at 0.117, 0.098 and 0.104, where
    the latch should have held after the first. ⭐ **A test can encode a mistake as
    confidently as code can, and a docstring defending it makes the mistake harder to see.**
    """
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.1520001) == 0.152, "sensor noise cleared the latch"
    assert one.jaw_block == 0.152
    assert one.hold_jaw(0.152 + JAW_CLEAR_MARGIN * 0.9) == 0.152, "still inside the margin"
    assert one.jaw_block == 0.152


def test_JITTER_AROUND_the_block_never_clears_it() -> None:
    """⛔⭐⭐ HIS SCENARIO, AS NOISE RATHER THAN AS INTENT. A measured jaw wandering a few
    thousandths either side of the block must leave the latch alone for as long as it takes.
    Nine hundred cycles is ten seconds at his loop rate."""
    one = arm()
    one.block_jaw_at(0.117)
    rng = np.random.default_rng(17)
    for _ in range(900):
        jitter = float(rng.normal(0.0, 0.004))       # a few thousandths, both directions
        one.hold_jaw(0.117 + jitter)
    assert one.jaw_block == 0.117, (
        "the latch was cleared by noise, which is what made the stall recur on hardware")


def test_a_DELIBERATE_open_still_clears_it_in_one_press() -> None:
    """⭐ The margin must not make the latch sticky. One gripper-step press is 0.02 and the
    puck-button rate moves faster, so a real open has to clear it immediately."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.152 + JAW_CLEAR_MARGIN + 0.01) > 0.152
    assert one.jaw_block is None


def test_clearing_RECORDS_where_it_cleared_from() -> None:
    """⭐ So the session can say it once. A latch that silently comes and goes is
    indistinguishable from a latch that never worked, and that ambiguity is exactly what his
    log left behind."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.jaw_unblocked_from is None
    one.hold_jaw(0.6)
    assert one.jaw_unblocked_from == 0.152


def test_a_hair_MORE_closed_does_NOT_count_as_opening() -> None:
    """⚠️ Held at the block, which is harmless: the jaws are not pushing at the block."""
    one = arm()
    one.block_jaw_at(0.152)
    assert one.hold_jaw(0.1519999) == 0.152
    assert one.jaw_block == 0.152


# ------------------------------------------------------------ the scenario end to end


def test_HIS_SCENARIO__leader_squeezes_while_follower_holds_something() -> None:
    """⛔⭐⭐ THE FULL SEQUENCE FROM HIS LOG, as a story.

    He hand-guided arm B in GUIDE while arm G mirrored it. Arm G's jaws already had hold of
    something. He squeezed arm B's jaws. Arm B's jaw position went below what arm G could
    reach, so every cycle told arm G to squeeze harder into an object that would not move.
    """
    one = arm()
    leader_jaw = 0.30
    stalled_at = 0.152
    releases = 0

    for cycle in range(600):
        # The leader's jaws close steadily, as a hand squeezing does.
        leader_jaw = max(0.0, leader_jaw - 0.001)
        commanded = one.hold_jaw(leader_jaw)
        # The follower's jaws physically cannot go below where the object is.
        if commanded < stalled_at - 1e-9:
            releases += 1                      # a stall the old code would have reported
            one.block_jaw_at(stalled_at)

    assert releases <= 1, (
        f"the stall fired {releases} times. It should latch on the FIRST one and never "
        f"again, which is the whole fix")
    assert one.jaw_block == stalled_at

    # And when he lets go of the leader's jaws, the follower is free at once.
    assert one.hold_jaw(0.6) == 0.6
    assert one.jaw_block is None


def test_the_OLD_behaviour_would_fail_that_scenario() -> None:
    """⭐ Proof the test above has teeth: without a latch, the same loop stalls repeatedly."""
    stalled_at = 0.152
    leader_jaw = 0.30
    releases = 0
    for _ in range(600):
        leader_jaw = max(0.0, leader_jaw - 0.001)
        commanded = leader_jaw                 # no latch: the leader wins every cycle
        if commanded < stalled_at - 1e-9:
            releases += 1
    assert releases > 100, (
        f"the unlatched version only stalled {releases} times, so the scenario is not "
        f"reproducing the problem it claims to")


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
