#!/usr/bin/env python3
"""Tests for `src/yam/mirror.py`. No hardware, no simulation, no device.

    uv run scripts/test_mirror.py

⛔ WHY THESE MATTER MORE THAN USUAL. Mirror mode's one real hazard is a **sudden
large motion**: the two arms do not start in the same pose, so a naive
implementation commands the follower straight to the leader's angles and it jumps
across the gap. `MirrorLink` was written with no robot handle and no I/O precisely so
that the engagement behaviour — the part that could produce that jump — is testable
without an arm in the room.

Every change in this repo that skipped that step failed on first contact with the
hardware, three times (FINDINGS §11). This is the cheap half of not doing it again.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.mirror import (  # noqa: E402
    MIRROR_SIGNS,
    STUCK_SPEED,
    MirrorLink,
    follower_target,
    gap,
    pick_pair,
)

DT = 0.01
LEADER = np.array([0.5, 1.2, 0.8, -0.3, 0.4, -0.9, 0.2])   # 6 joints + gripper


# ------------------------------------------------------------- the mapping ----


def test_copy_reproduces_the_leader_exactly() -> None:
    assert np.allclose(follower_target(LEADER, "copy"), LEADER)


def test_copy_is_the_default() -> None:
    """Julien's arms stand side by side, so unchanged copying is correct."""
    assert np.allclose(follower_target(LEADER), follower_target(LEADER, "copy"))
    assert MirrorLink().mode == "copy"


def test_mirror_negates_only_the_reflecting_joints() -> None:
    out = follower_target(LEADER, "mirror")
    for i in range(6):
        expect = LEADER[i] * MIRROR_SIGNS[i]
        assert np.isclose(out[i], expect), f"joint {i + 1}: {out[i]} != {expect}"


def test_the_gripper_is_never_mirrored() -> None:
    """A gripper has no handedness — copying how far it is open is what anyone
    would expect, and negating it would command it through its own zero."""
    out = follower_target(LEADER, "mirror")
    assert np.isclose(out[6], LEADER[6]), "the gripper opening must be copied, not reflected"


def test_an_unknown_mode_is_refused() -> None:
    for bad in ("flip", "reverse", ""):
        try:
            follower_target(LEADER, bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"mode {bad!r} should have been refused")


def test_mirroring_twice_returns_the_original() -> None:
    """Reflection is its own inverse; a sign table that failed this would be wrong."""
    assert np.allclose(follower_target(follower_target(LEADER, "mirror"), "mirror"), LEADER)


# --------------------------------------------------------- safe engagement ----


def test_it_never_jumps_across_the_initial_gap() -> None:
    """⭐ THE HAZARD. The arms start far apart; the follower must ramp, not leap."""
    leader = np.array([0.9, 1.4, 0.8, 0.8, 0.7, -1.2])
    follower = np.zeros(6)                               # ~1.4 rad away on some joints
    link = MirrorLink(align_speed=0.30)
    prev = follower.copy()
    worst_step = 0.0
    for _ in range(500):
        cmd = link.step(leader, follower, DT)
        assert cmd is not None
        # ⭐ The limit that must hold is the FOLLOW rate once engaged, and the ALIGN
        # rate before that. Checking against the align rate throughout would fail
        # for the right reason only by accident.
        worst_step = max(worst_step, float(np.max(np.abs(cmd - prev))))
        prev = cmd.copy()
        follower = cmd.copy()                            # a perfectly obedient arm
    allowed = max(0.30, link.follow_speed) * DT
    assert worst_step <= allowed * 1.01, (
        f"the follower was commanded {worst_step:.4f} rad in one cycle "
        f"({worst_step / DT:.2f} rad/s); the align speed allows {allowed:.4f}"
    )


def test_it_starts_aligning_and_only_then_follows() -> None:
    leader = np.array([0.9, 1.4, 0.8, 0.8, 0.7, -1.2])
    follower = np.zeros(6)
    link = MirrorLink()
    assert link.state == "aligning"
    assert "ALIGNING" in link.status(leader, follower)
    for _ in range(2000):
        cmd = link.step(leader, follower, DT)
        if cmd is not None:
            follower = cmd.copy()
        if link.state == "following":
            break
    assert link.state == "following", "alignment never completed against a still leader"
    assert gap(follower, leader) <= link.engage_tolerance


def test_following_tracks_the_leader_one_for_one() -> None:
    link = MirrorLink()
    leader = np.zeros(6)
    follower = np.zeros(6)
    for _ in range(50):                                   # already aligned
        follower = link.step(leader, follower, DT)
    assert link.state == "following"
    leader = np.array([0.10, 0.05, 0.0, 0.0, 0.0, 0.0])   # the leader is hand-guided
    # ⚠️ It CONVERGES on the leader rather than snapping to it in one cycle. The
    # follow rate limit is what removes the discontinuity at engagement, and 1.0
    # rad/s matches SafeRobot's cap anyway, so nothing is lost — a faster command
    # would be clipped one layer down regardless.
    for _ in range(200):
        follower = link.step(leader, follower, DT)
    assert np.allclose(follower, leader, atol=1e-6), f"never caught up: {follower}"


def test_it_stops_rather_than_chasing_a_follower_that_cannot_keep_up() -> None:
    """⛔ A large gap while following means blocked, faulted or at a limit. Chasing
    it would hold a motor against whatever is stopping it — the stall that cooked
    motor 7 three times."""
    link = MirrorLink(max_gap=0.35)
    leader = np.zeros(6)
    follower = np.zeros(6)
    for _ in range(10):
        follower = link.step(leader, follower, DT)
    assert link.state == "following"
    stuck = follower.copy()                               # the follower stops moving
    leader = np.array([1.0, 0, 0, 0, 0, 0])               # the leader runs away
    cmd = link.step(leader, stuck, DT)
    assert cmd is None, "it must stop commanding, not keep chasing"
    assert link.state == "stopped"
    assert "behind" in (link.stop_reason or "")
    assert "STOPPED" in link.status(leader, stuck)


def test_stopped_stays_stopped() -> None:
    """It must not silently re-engage once the fault clears — that would be a
    surprise motion with nobody expecting it."""
    link = MirrorLink(max_gap=0.2)
    leader, follower = np.zeros(6), np.zeros(6)
    for _ in range(5):
        follower = link.step(leader, follower, DT)
    link.step(np.array([1.0, 0, 0, 0, 0, 0]), follower, DT)     # trip it
    assert link.state == "stopped"
    for _ in range(20):
        assert link.step(np.zeros(6), np.zeros(6), DT) is None


def test_mirror_mode_also_engages_safely() -> None:
    """The ramp must protect the mirrored case too, where the gap is often larger."""
    leader = np.array([0.9, 1.4, 0.8, 0.8, 0.7, -1.2])
    follower = np.zeros(6)
    link = MirrorLink(mode="mirror", align_speed=0.30)
    prev, worst = follower.copy(), 0.0
    for _ in range(400):
        cmd = link.step(leader, follower, DT)
        if cmd is None:
            break
        worst = max(worst, float(np.max(np.abs(cmd - prev))))
        prev, follower = cmd.copy(), cmd.copy()
    assert worst <= 0.30 * DT * 1.01, f"mirrored engagement jumped {worst:.4f} rad in a cycle"


def test_gap_is_worst_joint_not_average() -> None:
    """One badly-placed joint must not be hidden by five well-placed ones."""
    assert np.isclose(gap(np.zeros(6), np.array([0, 0, 0, 0, 0, 0.9])), 0.9)


def test_a_seven_dof_follower_and_six_dof_leader_do_not_crash() -> None:
    """The length-mismatch class that dropped a raised arm once already."""
    link = MirrorLink()
    cmd = link.step(np.zeros(6), np.zeros(7), DT)
    assert cmd is not None and len(cmd) == 7


# ------------------------------------------------------- who leads, who follows ----


def refused(names, selected):  # noqa: ANN001, ANN201
    """The message `pick_pair` refuses with, or a failure if it accepted the input."""
    try:
        got = pick_pair(names, selected)
    except ValueError as exc:
        return str(exc)
    raise AssertionError(f"pick_pair({names}, {selected}) returned {got} instead of refusing")


def test_the_selected_arm_leads() -> None:
    """⭐ The operator selects the arm they are about to put their hands on, which is how
    every other aimed key in the session already works."""
    assert pick_pair(["B", "G"], ["B"]) == ("B", "G")
    assert pick_pair(["B", "G"], ["G"]) == ("G", "B")


def test_BOTH_selected_is_refused_rather_than_guessed() -> None:
    """⛔ "Both arms lead" has no meaning, and picking the first would engage a motion on
    whichever arm happened to come first in `--arms`."""
    assert "select ONE arm" in refused(["B", "G"], ["B", "G"])


def test_one_arm_cannot_mirror() -> None:
    assert "exactly two arms" in refused(["B"], ["B"])


def test_an_arm_outside_the_session_is_refused() -> None:
    assert "not in this session" in refused(["B", "G"], ["X"])


# ------------------------------------------- the stop names what it measured ----


def run_until_stopped(link, leader_step, follower_follows=True, cycles=400):  # noqa: ANN001, ANN201
    """Drive the link with a leader that moves `leader_step` rad per cycle."""
    lead = LEADER.copy()
    follow = LEADER.copy()
    for _ in range(cycles):
        cmd = link.step(lead, follow, DT)
        if cmd is None:
            return link
        if follower_follows:
            follow = np.asarray(cmd, dtype=float).copy()
        lead = lead + np.array([0, 0, 0, 0, 0, leader_step, 0])
    return link


def test_a_leader_moving_faster_than_the_limit_is_NAMED_as_the_cause() -> None:
    """⛔⭐ THE FIX FOR THE MESSAGE JULIEN SAW. On its first real run MIRROR stopped twice
    saying *"It is blocked, at a joint limit, or faulted."* **None of the three was true** —
    he was hand-guiding the leader's wrist faster than the follower may move. The message now
    reports which joint opened the gap and how fast the leader was moving it, then says which
    explanation the numbers support."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    # 0.03 rad per 10 ms cycle is 3.0 rad/s, three times the follower's limit.
    run_until_stopped(link, leader_step=0.03)
    assert link.state == "stopped"
    assert link.stop_joint == 5, f"joint 6 opened the gap, got index {link.stop_joint}"
    assert link.stop_leader_speed is not None and link.stop_leader_speed > 1.0
    assert link.stop_cause == "follow_limit", f"cause was {link.stop_cause}"
    assert "could not keep up" in link.stop_detail
    assert "joint 6" in link.stop_reason


def test_a_STUCK_follower_is_named_differently_from_a_fast_leader() -> None:
    """⭐ The other half of the same measurement: if the leader is moving slowly and the gap
    still opens, the follower is not moving, and the message says so."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    # The leader creeps at 0.5 rad/s, well inside the limit, and the follower never moves.
    run_until_stopped(link, leader_step=0.005, follower_follows=False)
    assert link.state == "stopped"
    assert link.stop_leader_speed is not None and link.stop_leader_speed < 1.0
    assert link.stop_cause == "stuck", f"cause was {link.stop_cause}"
    assert "blocked, at a joint limit, or faulted" in link.stop_detail


def test_the_row_warns_BEFORE_the_gap_trips() -> None:
    """⭐ Julien's first run gave no notice: the row read "tracking 0.34 rad behind" one second
    and the link was gone the next. Past 70% of that joint's OWN limit it says so.

    ⚠️ THIS TEST USED TO USE JOINT 6 AND HAD TO CHANGE. It offset index 5 by 0.30 and called
    that "86% of the limit", which was true when all six joints shared 0.35. Joint 6's limit is
    now 0.35 × 4.0 = 1.4, so 0.30 is 21% of it and the row is right to stay quiet. ⭐ Using
    joint 3, whose multiplier is 1.00, tests the warning without depending on the scaling.
    """
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    link.state = "following"
    near = LEADER.copy()
    near[2] += 0.30                      # joint 3, limit 0.35, so 86% of it
    assert "near its 0.35 limit" in link.status(near, LEADER)
    fine = LEADER.copy()
    fine[2] += 0.10                      # 29% of it
    assert "near its" not in link.status(fine, LEADER)


def test_the_row_does_NOT_cry_wolf_about_a_wrist_with_rope_to_spare() -> None:
    """⛔⭐ THE POINT OF THE PER-JOINT WARNING. The same 0.30 rad on joint 6 is a fifth of its
    limit. Warning about it would train him to ignore the line, and then it is ignored on the
    run where the elbow is the one in trouble."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    link.state = "following"
    wrist = LEADER.copy()
    wrist[5] += 0.30
    assert "near its" not in link.status(wrist, LEADER)


def test_a_STALLED_joint_stops_on_the_TIGHT_limit_not_the_scaled_one() -> None:
    """⛔⭐⭐⭐ A SAFETY CONSEQUENCE OF THE SCALING THAT I MISSED FIRST TIME.

    Julien's 2026-08-17 log: joint 4 stopped at **0.869 rad** against its scaled 0.87 limit,
    having moved **0.01 rad/s**, with `SafeRobot` clipping the command on **1115 cycles**. That
    is about 12 seconds of a motor pushing against something that would not move, and the
    session's hottest readings were 45 and 46 °C, the highest recorded.

    ⭐ The multiplier is right for a joint that is LAGGING and wrong for one that is STALLED.
    So a follower that is not moving is stopped on the unscaled `max_gap`.
    """
    import numpy as np

    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    stuck = np.zeros(7)
    leader = np.zeros(7)
    leader[3] = 0.5                      # joint 4, scaled limit 0.87, unscaled 0.35
    # Engage, then hold the follower completely still while the leader sits out there.
    link.step(leader, stuck, 0.011)
    link.state = "following"
    out = None
    for _ in range(40):
        out = link.step(leader, stuck, 0.011)
        if out is None:
            break
    assert link.state == "stopped", "a completely stalled joint 4 never stopped the mirror"
    assert link.stop_cause == "stuck", f"cause was {link.stop_cause!r}"
    assert link.stop_gap < 0.87, (
        f"it waited until {link.stop_gap:.3f} rad, which is the SCALED limit. A stalled joint "
        f"should stop at the tight {0.35} one")
    assert "NOT MOVING" in (link.stop_reason or ""), link.stop_reason


def test_an_ARM_that_cannot_track_is_named_as_the_hardware_and_not_the_software() -> None:
    """⛔⭐⭐ THE THIRD CAUSE, and it is the one Julien met at `--max-speed 5`. The message
    said *"blocked, at a joint limit, or faulted"* and his answer was *"the robot was never
    blocked by anything. It just, like, didn't kind of catch up at high speeds."*

    ⭐ He was right, and the reason is one layer down: `SafeRobot` clips every command to
    **0.25 rad from the measured position**, so the follower's command can never run further
    ahead than that however high `max_speed` goes. Past a certain leader speed the follower is
    tracking as hard as it can and still losing ground.

    ⭐ Here the follower is given a generous allowance (5 rad/s) and made to move at a third
    of the leader's speed — moving, but not fast enough. **That is neither of the other two
    causes**, and calling it either one sends the reader looking for an obstruction that is
    not there.
    """
    link = MirrorLink(follow_speed=5.0, max_gap=0.35)
    lead = LEADER.copy()
    follow = LEADER.copy()
    # ⚠️ The follower moves at a FIXED 1.0 rad/s, which is what an arm at its physical limit
    # looks like, while the leader is asked for 2.5. The first version of this test let the
    # follower close a third of the gap per cycle, which keeps up easily — it never tripped,
    # and the failure was in the test rather than in the code.
    follower_limit = 0.01                                   # 1.0 rad/s at 100 Hz
    for _ in range(400):
        cmd = link.step(lead, follow, DT)
        if cmd is None:
            break
        follow = follow + np.clip(np.asarray(cmd, dtype=float) - follow,
                                  -follower_limit, follower_limit)
        lead = lead + np.array([0, 0, 0, 0, 0, 0.025, 0])    # 2.5 rad/s, inside 5.0
    assert link.state == "stopped", "the gap never opened; the scenario does not model a slow arm"
    assert link.stop_cause == "tracking", f"cause was {link.stop_cause}"
    assert link.stop_follower_speed is not None and link.stop_follower_speed > STUCK_SPEED
    assert "the ARM itself could not track that fast" in link.stop_detail


def test_the_reason_and_the_detail_are_SEPARATE_so_neither_gets_truncated() -> None:
    """⛔ `StatusLine.say()` truncates each line to the terminal width while a live block is on
    screen. Julien's first high-speed run lost the end of the stop message to an ellipsis, and
    the lost half was the part that named the cause."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    run_until_stopped(link, leader_step=0.03)
    assert link.stop_reason and link.stop_detail
    assert len(link.stop_reason) < 110, f"{len(link.stop_reason)} chars is a truncation risk"
    assert len(link.stop_detail) < 140, f"{len(link.stop_detail)} chars is a truncation risk"


def test_ALIGNING_gives_up_instead_of_chasing_a_moving_leader() -> None:
    """⛔⭐⭐ THE HEADER OF `src/yam/mirror.py` CLAIMED THIS FROM 2026-08-11 AND THE CODE DID NOT DO
    IT. It said *"alignment reports its progress and gives up rather than chasing forever,
    exactly like PARK's stall detector"* — and the gap check only ever ran in the `following`
    state. A leader that kept moving during ALIGNING had the follower chasing it indefinitely.

    ⚠️ Nothing raised, and Julien never hit it because he held the leader still as the plan
    line asks. **A design note written in the present tense reads afterwards as a description
    of the code.**
    """
    link = MirrorLink(align_speed=0.3, align_stall_seconds=0.2)
    lead = LEADER.copy()
    # ⚠️ THE FOLLOWER STARTS BEHIND AND THE LEADER MOVES AWAY. The first version of this test
    # put the follower 1.0 rad ABOVE the leader and then moved the leader UP, so the leader
    # closed the gap itself and alignment converged — the test passed nothing. Printing the
    # per-cycle gap is what showed it. **A dynamics scenario written from intuition needs its
    # trace read once before its assertion is trusted.**
    follow = LEADER - 1.0
    for _ in range(400):
        cmd = link.step(lead, follow, DT)
        if cmd is None:
            break
        follow = np.asarray(cmd, dtype=float).copy()
        lead = lead + 0.02                   # the leader runs away at 2 rad/s
    assert link.state == "stopped", "it chased a moving leader forever"
    assert link.stop_cause == "align_stalled", f"cause was {link.stop_cause}"
    assert "hold the leader STILL" in link.stop_detail


def test_ALIGNING_still_converges_when_the_leader_is_held_still() -> None:
    """⚠️ The give-up must not fire on a normal engagement. This is the case Julien actually
    runs: the leader is held, the follower closes a gap of a couple of tenths."""
    link = MirrorLink(align_speed=0.3, align_stall_seconds=0.2)
    lead = LEADER.copy()
    follow = LEADER + 0.2
    for _ in range(400):
        cmd = link.step(lead, follow, DT)
        assert cmd is not None, f"it gave up while converging: {link.stop_reason}"
        follow = np.asarray(cmd, dtype=float).copy()
        if link.state == "following":
            break
    assert link.state == "following"



# ============================================================ the catch-up correction
#
# ⭐⭐ WHY THESE EXIST. Julien, 2026-08-17: *"I can move the mirrored robot about in a maybe
# two centimetre diameter sphere around the position it should actually be at… when I try to
# pick up something from the table, sometimes my guiding robot is already moving into the
# table whilst my mirror robot isn't even far enough down."*
#
# ⭐ Measured on his hardware: 0.024 rad of joint error is 11 mm at the tip in the extended
# pose his log shows, so the 2 cm sphere is what those numbers predict.


DROOP = 0.024          # rad — the standing error his status row reported


def _drooping_follower(link, leader, cycles=400, dt=0.011, droop=DROOP):
    """Run `cycles` of mirroring against a follower that always sits `droop` short.

    ⭐ THIS IS THE WHOLE POINT OF THE FIXTURE. A position-controlled arm settles where its
    motor force balances gravity and friction, which is always short of the command. So the
    fake follower reports `command − droop`, never the command. **A fake that reports the
    command exactly cannot show this problem at all**, which is the same trap the old fake
    robots had (docs/FINDINGS.md §59.0).

    Returns the final measured follower pose.
    """
    import numpy as np

    measured = np.zeros(7)
    for _ in range(cycles):
        cmd = link.step(leader, measured, dt)
        if cmd is None:
            continue
        measured = np.asarray(cmd, dtype=float).copy()
        measured[:6] -= droop          # the arm sits short of whatever it was told
    return measured


def test_WITHOUT_catchup_the_follower_stays_short_forever() -> None:
    """⛔⭐⭐ THE DEFECT, AS A TEST. The command converges to the leader's angles exactly, so
    the follower sits at `leader − droop` and no part of the loop ever reads that back."""
    import numpy as np

    leader = np.full(7, 0.5)
    link = MirrorLink(follow_speed=2.0, catchup=0.0)
    measured = _drooping_follower(link, leader)
    err = float(np.max(np.abs(leader[:6] - measured[:6])))
    assert err > DROOP * 0.9, (
        f"expected the droop to persist at about {DROOP} rad; got {err:.4f}")


def test_WITH_catchup_the_follower_ACTUALLY_ARRIVES() -> None:
    """✅⭐⭐ THE FIX. The bias grows until the follower reaches the leader, and the residual
    should be a small fraction of the droop rather than the whole of it."""
    import numpy as np

    leader = np.full(7, 0.5)
    link = MirrorLink(follow_speed=2.0, catchup=3.0)
    measured = _drooping_follower(link, leader)
    err = float(np.max(np.abs(leader[:6] - measured[:6])))
    assert err < DROOP * 0.2, (
        f"the follower is still {err:.4f} rad short, which is {err / DROOP:.0%} of the "
        f"original droop. The correction is not closing it")


def test_the_bias_NEVER_exceeds_its_clamp_even_against_a_blocked_follower() -> None:
    """⛔⭐⭐ THE SAFETY PROPERTY. A blocked follower never closes its error, so an unclamped
    integral grows forever and the arm lurches when the block clears. This runs a follower
    that cannot move at all for twenty seconds."""
    import numpy as np

    leader = np.full(7, 1.0)
    link = MirrorLink(follow_speed=2.0, catchup=5.0, max_bias=0.06, max_gap=99.0)
    stuck = np.zeros(7)
    for _ in range(2000):
        link.step(leader, stuck, 0.011)      # the follower never moves
    assert link.worst_bias() <= 0.06 + 1e-9, (
        f"the bias wound up to {link.worst_bias():.4f} rad against its 0.06 clamp")


def test_the_clamp_stays_well_under_SafeRobot_s_lag_limit() -> None:
    """⚠️ The bias makes the command sit further from the measured pose, which is exactly what
    `SafeRobot.max_lag` limits. If the clamp approached that, the correction would simply be
    clipped away and the feature would silently do nothing."""
    from yam.mirror import DEFAULT_MAX_BIAS
    from yam.robot import SAFE_MAX_LAG

    assert DEFAULT_MAX_BIAS < SAFE_MAX_LAG * 0.5, (
        f"a {DEFAULT_MAX_BIAS} rad bias against a {SAFE_MAX_LAG} rad lag clip leaves too "
        f"little room")


def test_a_FAST_leader_does_not_accumulate_any_bias() -> None:
    """⛔⭐ While the leader moves, part of the error is honest lag that disappears by itself.
    Integrating it would push the follower PAST the leader every time it stopped."""
    import numpy as np

    link = MirrorLink(follow_speed=5.0, catchup=5.0, catchup_below=0.25, max_gap=99.0)
    measured = np.zeros(7)
    leader = np.zeros(7)
    for _ in range(300):
        leader = leader + 0.02              # ~1.8 rad/s, far above catchup_below
        cmd = link.step(leader, measured, 0.011)
        if cmd is not None:
            measured = np.asarray(cmd, dtype=float).copy()
            measured[:6] -= DROOP
    # ⚠️ Not exactly zero, and the reason is worth knowing. The leader's speed estimate is
    # smoothed, so for the first few cycles of a fast motion it still reads slow and a sliver
    # of bias accumulates. Measured at 0.0011 rad, which is about 0.5 mm at the tip.
    # ⭐ What matters is that a fast leader DECAYS it rather than freezing it, so the sliver
    # bleeds away instead of one being added at every slow-to-fast transition.
    assert link.worst_bias() < 0.002, (
        f"a fast leader accumulated {link.worst_bias():.4f} rad of bias, which is more than "
        f"the smoothing lag can explain")


def test_a_bias_accumulated_while_SLOW_decays_once_the_leader_moves_fast() -> None:
    """⛔⭐⭐ THE LEAK, AND A TEST FOUND WHY IT IS NEEDED. Without it a fast leader freezes the
    bias, so every slow-to-fast transition adds a sliver and a long session of reaching and
    sweeping could creep to the clamp for a reason nobody chose."""
    import numpy as np

    leader = np.full(7, 0.5)
    link = MirrorLink(follow_speed=2.0, catchup=3.0, max_gap=99.0)
    _drooping_follower(link, leader)                 # build a real bias while slow
    settled = link.worst_bias()
    assert settled > 0.005, f"no bias to decay ({settled:.4f}); the test proves nothing"

    measured = np.full(7, 0.5)
    fast = leader.copy()
    for _ in range(400):
        fast = fast + 0.02                           # ~1.8 rad/s, far above catchup_below
        cmd = link.step(fast, measured, 0.011)
        if cmd is not None:
            measured = np.asarray(cmd, dtype=float).copy()
    assert link.worst_bias() < settled * 0.2, (
        f"the bias only fell from {settled:.4f} to {link.worst_bias():.4f}; it is being "
        f"frozen rather than decayed")


def test_catchup_is_OFF_by_default() -> None:
    """⛔⭐ It changes what a 4.3 kg arm does, so it is opt-in. Julien raises limits and enables
    control changes; the agent adds the flag and recommends."""
    from yam.mirror import DEFAULT_CATCHUP

    assert DEFAULT_CATCHUP == 0.0
    assert MirrorLink().catchup == 0.0


def test_the_bias_is_RESET_when_the_link_re_engages() -> None:
    """⛔ Carrying control state across an engagement is the same class of bug as the stale
    `prev_q` that snapped this arm on 2026-08-10."""
    import numpy as np

    leader = np.full(7, 0.5)
    link = MirrorLink(follow_speed=2.0, catchup=3.0)
    _drooping_follower(link, leader)
    assert link.worst_bias() > 0.0, "no bias accumulated, so this test proves nothing"

    fresh = MirrorLink(follow_speed=2.0, catchup=3.0)
    fresh.step(leader, np.zeros(7), 0.011)
    assert fresh.worst_bias() == 0.0, "a new link started with a bias already in place"



# ==================================================== the per-joint gap scaling
#
# ⭐⭐ WHY. Every mirror stop in Julien's 2026-08-17 logs was on joint 5 (`wrist_roll`) or
# joint 6 (`gripper_twist`), the two joints that barely move the tip. The only way to tolerate
# a flicked wrist was to raise the threshold for the SHOULDER as well, and he reached
# `--mirror-gap 1.335` doing it. At the elbow's measured 0.418 m/rad that allows 56 cm of tip
# error on a limit whose purpose is noticing that the arm has gone somewhere wrong.


def test_a_WRIST_gap_that_used_to_stop_it_now_does_not() -> None:
    """⛔⭐⭐ HIS EXACT STOP. Joint 5 fell 0.364 rad behind against a 0.35 threshold. Joint 5's
    own limit is 0.35 x 4.0 = 1.4, so that flick should no longer interrupt him."""
    import numpy as np
    from yam.mirror import worst_scaled_joint

    follower = np.zeros(7)
    target = np.zeros(7)
    target[4] = 0.364                      # joint 5 is index 4
    j, g, limit = worst_scaled_joint(follower, target, 0.35)
    assert j == 4 and abs(g - 0.364) < 1e-9
    assert limit > 1.3, f"joint 5's limit is only {limit:.2f}"
    assert g < limit, "his 0.364 rad wrist flick would still stop the mirror"


def test_the_SAME_gap_on_the_ELBOW_still_stops_it() -> None:
    """⛔⭐⭐ THE OTHER HALF, AND THE ONE THAT MATTERS FOR SAFETY. The elbow moves the tip
    0.418 m per radian, so it gets no extra rope at all."""
    import numpy as np
    from yam.mirror import worst_scaled_joint

    follower = np.zeros(7)
    target = np.zeros(7)
    target[2] = 0.364                      # joint 3, the elbow
    j, g, limit = worst_scaled_joint(follower, target, 0.35)
    assert j == 2
    assert limit <= 0.36, f"the elbow was given {limit:.2f}, which is extra rope"
    assert g > limit, "a 0.364 rad ELBOW error must still stop the mirror"


def test_it_names_the_joint_CLOSEST_TO_ITS_OWN_LIMIT_not_the_biggest_gap() -> None:
    """⭐ Those are different questions once the thresholds differ. A 0.9 rad wrist error
    against a 1.4 rad wrist limit is further from stopping than a 0.4 rad elbow error against
    0.35. Reporting the largest raw gap would name the wrist and send him after the wrong
    flag."""
    import numpy as np
    from yam.mirror import worst_scaled_joint

    follower = np.zeros(7)
    target = np.zeros(7)
    target[4] = 0.9                        # big, but well inside the wrist's limit
    target[2] = 0.4                        # smaller, but over the elbow's limit
    j, g, limit = worst_scaled_joint(follower, target, 0.35)
    assert j == 2, f"it named joint {j + 1}; the elbow is the one in trouble"
    assert g > limit


def test_the_weights_put_the_SHOULDER_JOINTS_at_about_1x() -> None:
    """⚠️ Joints 1-3 carry the arm through space, so their thresholds must be essentially
    unchanged from what he has been running. A silent loosening there would be the opposite of
    what this change is for."""
    from yam.mirror import GAP_WEIGHTS

    for j in range(3):
        assert GAP_WEIGHTS[j] <= 1.3, f"joint {j + 1} got {GAP_WEIGHTS[j]}x, which is a real loosening"


def test_the_wrist_multiplier_is_CAPPED_rather_than_following_the_measurement() -> None:
    """⚠️⚠️ Pure tip-displacement scaling would give joint 6 about 6.6x, and tip position is
    the wrong basis for task accuracy even though it is the right one for danger. 1.4 rad on
    the gripper twist is the gripper rotated 80° from where it should be, which ruins a grasp
    while barely moving the tip. So the cap is deliberate."""
    from yam.mirror import GAP_WEIGHTS

    assert max(GAP_WEIGHTS) <= 4.0, f"a multiplier of {max(GAP_WEIGHTS)}x is too much rope"
    assert GAP_WEIGHTS[5] == 4.0, "the gripper twist should be at the cap"


def test_an_all_zero_gap_stops_nothing() -> None:
    import numpy as np
    from yam.mirror import worst_scaled_joint

    j, g, limit = worst_scaled_joint(np.zeros(7), np.zeros(7), 0.35)
    assert g == 0.0 and g < limit


def test_a_SHORT_pose_vector_does_not_crash_the_check() -> None:
    """⚠️ A 6-DoF follower tracking a 7-DoF leader has bitten this file before."""
    import numpy as np
    from yam.mirror import worst_scaled_joint

    j, g, limit = worst_scaled_joint(np.zeros(3), np.zeros(3), 0.35)
    assert 0 <= j < 3 and limit > 0


def test_his_gripper_twist_stop_would_now_pass_at_the_DEFAULT_gap() -> None:
    """⭐⭐ THE PRACTICAL POINT. He reached `--mirror-gap 1.335` chasing joint 6, and joint 6
    then fell 1.369 behind anyway. At the DEFAULT 0.35 the wrist's own limit is 1.4, so the
    same flick passes without the elbow's threshold being touched at all."""
    import numpy as np
    from yam.mirror import worst_scaled_joint

    follower = np.zeros(7)
    target = np.zeros(7)
    target[5] = 1.369                      # joint 6
    j, g, limit = worst_scaled_joint(follower, target, 0.35)
    assert j == 5
    assert g < limit, (
        f"joint 6 at {g:.3f} still exceeds its {limit:.2f} limit at the default gap")


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
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
