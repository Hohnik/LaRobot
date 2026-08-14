#!/usr/bin/env python3
"""Tests for `src/mirror.py`. No hardware, no simulation, no device.

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
sys.path.insert(0, str(REPO / "src"))

from mirror import (  # noqa: E402
    MIRROR_SIGNS,
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
    assert "could not keep up" in link.stop_reason
    assert "joint 6" in link.stop_reason


def test_a_STUCK_follower_is_named_differently_from_a_fast_leader() -> None:
    """⭐ The other half of the same measurement: if the leader is moving slowly and the gap
    still opens, the follower is not moving, and the message says so."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    # The leader creeps at 0.5 rad/s, well inside the limit, and the follower never moves.
    run_until_stopped(link, leader_step=0.005, follower_follows=False)
    assert link.state == "stopped"
    assert link.stop_leader_speed is not None and link.stop_leader_speed < 1.0
    assert "blocked, at a joint limit, or faulted" in link.stop_reason


def test_the_row_warns_BEFORE_the_gap_trips() -> None:
    """⭐ Julien's first run gave no notice: the row read "tracking 0.34 rad behind" one second
    and the link was gone the next. Past 70% of the limit it says so."""
    link = MirrorLink(follow_speed=1.0, max_gap=0.35)
    link.state = "following"
    near = LEADER.copy()
    near[5] += 0.30                      # 86% of the limit
    assert "near the 0.35 limit" in link.status(near, LEADER)
    fine = LEADER.copy()
    fine[5] += 0.10                      # 29% of the limit
    assert "near the" not in link.status(fine, LEADER)


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
