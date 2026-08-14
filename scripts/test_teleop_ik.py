#!/usr/bin/env python3
"""Regression tests for the cartesian teleop loop. Simulation only, no hardware.

    uv run scripts/test_teleop_ik.py

⛔ WHY THESE EXIST. Julien, 2026-08-11: *"the inverse kinematics being weird and not
working as intended, specifically when the robot gets into weird positions, and then
it starts moving very, very incoherently."*

Measured cause: commanding **pure rotation** moved the **tool point** up to 44 cm.
A wrist joint hits its limit, the orientation goal keeps integrating anyway, and the
QP — which trades `position_cost` against `orientation_cost` — starts moving the tool
point to partially satisfy an orientation it can never reach. Full account in
FINDINGS §18.

These tests reproduce the loop faithfully, including the per-cycle joint-step clamp,
the joint-limit clamp and the workspace box, because **the bug only appears when the
clamps interact with the IK.** Testing `CartesianTeleop` in isolation would have
missed it entirely.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))

from teleop import (  # noqa: E402
    FLOOR_LIMIT,
    REACH_LIMIT,
    CartesianTeleop,
    clamp_to_workspace,
    effective_limits,
    workspace_room,
)
from yam_can import YAM_JOINTS  # noqa: E402

DT = 0.01
N_ARM = 6
MAX_JOINT_STEP = 0.015
JOINT_LIMIT_MARGIN = 0.08
# ⚠️ `WORKSPACE_BOX = 0.30` used to be here, and `drive()` below applied it. The script
# replaced that cube with a reach sphere plus a floor on 2026-08-14, and a simulation
# that keeps applying the old limit is a copy of a design the real loop no longer has.
# That is exactly what happened to `ArmSession` for a day while all its tests passed
# (FINDINGS §36.2). `drive()` now imports the real clamp instead of imitating it.

LO = np.array([YAM_JOINTS[i][1] for i in range(1, N_ARM + 1)]) + JOINT_LIMIT_MARGIN
HI = np.array([YAM_JOINTS[i][2] for i in range(1, N_ARM + 1)]) - JOINT_LIMIT_MARGIN

PARK = np.array([0.893, 1.388, 0.792, 0.803, 0.768, -1.291])
REACHING = np.array([0.0, 1.2, 0.6, 0.0, 0.0, 0.0])
FOLDED = np.array([0.3, 0.5, 2.0, -0.5, 0.5, 0.5])


def drive(start: np.ndarray, twist: np.ndarray, seconds: float = 10.0, **kw):
    """Reproduce teleop_session's TELEOP branch. Returns (worst tool wander,
    degrees rotated, worst translational lead, worst rotational lead)."""
    tp = CartesianTeleop(**kw)
    tp.reset(start)
    home = tp.ee_position().copy()
    prev = start.copy()
    r0 = tp.configuration.get_transform_frame_to_world(tp.ee_site, "site").rotation().as_matrix()
    wander = lead_m = lead_r = 0.0

    for _ in range(int(seconds / DT)):
        q_target = tp.step(twist, DT)
        ee = tp.ee_position()
        lim_r, lim_f = effective_limits(home, REACH_LIMIT, FLOOR_LIMIT)
        allowed = clamp_to_workspace(ee, lim_r, lim_f)
        if not np.allclose(allowed, ee):
            import mink  # noqa: PLC0415
            tp.target = mink.SE3.from_rotation_and_translation(
                rotation=tp.target.rotation(), translation=allowed,
            )
        q = prev + np.clip(q_target - prev, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        prev = np.clip(q, LO, HI)

        wander = max(wander, float(np.linalg.norm(tp.ee_position() - home)))
        a, b = tp.lead()
        lead_m, lead_r = max(lead_m, a), max(lead_r, b)

    r1 = tp.configuration.get_transform_frame_to_world(tp.ee_site, "site").rotation().as_matrix()
    d = r1 @ r0.T
    turned = float(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1))))
    return wander, turned, lead_m, lead_r


ROLL = np.array([0.0, 0.0, 0.0, 0.6, 0.0, 0.0])
YAW = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.6])
PUSH_X = np.array([0.12, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_pure_rotation_does_not_move_the_tool_point() -> None:
    """⭐ THE REGRESSION. Was 0.40-0.56 m of unrequested translation."""
    for name, start in (("park", PARK), ("reaching", REACHING), ("folded", FOLDED)):
        for label, tw in (("roll", ROLL), ("yaw", YAW)):
            wander, _, _, _ = drive(start, tw)
            assert wander < 0.02, (
                f"{name}/{label}: commanding pure rotation moved the tool point "
                f"{wander:.3f} m — it must stay put"
            )


def test_rotation_still_actually_happens() -> None:
    """The fix must not have bought stillness by refusing to rotate."""
    for name, start in (("park", PARK), ("reaching", REACHING)):
        _, turned, _, _ = drive(start, ROLL)
        assert turned > 45.0, f"{name}: only {turned:.1f}° of rotation in 10 s"


def test_translation_is_unaffected() -> None:
    wander, _, _, _ = drive(PARK, PUSH_X)
    assert wander > 0.25, f"pushing +X for 10 s should still travel, got {wander:.3f} m"


def test_the_goal_never_runs_away_from_the_arm() -> None:
    """Anti-windup: the target may lead reality, but only by a bounded amount.

    ⚠️ The tolerance is 10%, not exact, and the reason is worth knowing: the clamp is
    applied inside `step()` BEFORE the IK solve, and `integrate_inplace()` then moves
    the achieved pose. So a lead measured after the solve can sit a hair past the
    limit — 0.250060 against 0.25 — without anything being wrong. What would be wrong
    is *growth*, which `test_the_lead_does_not_grow_with_time` covers.
    """
    d = CartesianTeleop()
    for start in (PARK, REACHING, FOLDED):
        for tw in (ROLL, YAW, PUSH_X):
            _, _, lead_m, lead_r = drive(start, tw)
            assert lead_m <= d.max_lead_m * 1.1, f"translational lead {lead_m:.4f} m"
            assert lead_r <= d.max_lead_rad * 1.1, f"rotational lead {lead_r:.4f} rad"


def test_the_lead_does_not_grow_with_time() -> None:
    """⭐ The property that distinguishes 'bounded' from 'winding up slowly'.

    An unbounded integrator looks fine over 5 s and ruins you over 60. Measured:
    the worst lead is identical at 10 s and 80 s (0.250060 rad), so the clamp holds
    rather than merely slowing the runaway down.
    """
    _, _, m_short, r_short = drive(PARK, ROLL, seconds=10.0)
    _, _, m_long, r_long = drive(PARK, ROLL, seconds=60.0)
    assert r_long <= r_short * 1.02 + 1e-6, (
        f"rotational lead grew from {r_short:.6f} to {r_long:.6f} over 6x the time — "
        "that is windup, not a bound"
    )
    assert m_long <= m_short * 1.02 + 1e-6, (m_short, m_long)


def test_the_old_cost_ratio_reproduces_the_bug() -> None:
    """Proof the diagnosis is mechanical: restore the old orientation_cost and the
    tool point wanders again. If this ever stops failing, the cause moved."""
    wander, _, _, _ = drive(PARK, ROLL, orientation_cost=0.5)
    assert wander > 0.1, (
        f"expected the OLD orientation_cost=0.5 to still wander, got {wander:.3f} m — "
        "the diagnosis may no longer hold"
    )


def test_small_rotations_are_unchanged() -> None:
    """Normal operation must not have been traded away for the edge case."""
    _, turned, _, _ = drive(PARK, np.array([0.0, 0.0, 0.0, 0.15, 0.0, 0.0]))
    assert turned > 45.0, f"a gentle roll should still turn the wrist, got {turned:.1f}°"


def test_lead_is_zero_at_rest() -> None:
    tp = CartesianTeleop()
    tp.reset(PARK)
    m, r = tp.lead()
    assert m < 1e-6 and r < 1e-6, (m, r)


# ------------------------------------------------------- control frames ----


def test_world_frame_is_unchanged_by_default() -> None:
    """The default must still be world — this is the behaviour tuned on hardware."""
    tp = CartesianTeleop()
    assert tp.frame == "world"
    raw = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert np.allclose(tp._twist_to_world(raw), raw), "world frame must be a passthrough"


def test_an_unknown_frame_is_refused() -> None:
    try:
        CartesianTeleop(frame="gripper")
    except ValueError as exc:
        assert "frame must be one of" in str(exc)
    else:
        raise AssertionError("an unknown frame name must be refused, not silently ignored")


def test_tool_frame_turns_with_the_wrist_and_world_does_not() -> None:
    """⭐ THE POINT OF THE FEATURE. Same puck push, two different wrist orientations:
    in WORLD the tool goes the same way both times; in TOOL it follows the wrist."""
    push = np.array([0.12, 0.0, 0.0, 0.0, 0.0, 0.0])
    turned = PARK.copy()
    turned[5] += 1.2                      # rotate the wrist about its own axis

    dirs = {}
    for frame in ("world", "tool"):
        for label, start in (("straight", PARK), ("turned", turned)):
            tp = CartesianTeleop(frame=frame)
            tp.reset(start)
            before = tp.ee_position().copy()
            for _ in range(100):
                tp.step(push, DT)
            d = tp.ee_position() - before
            dirs[(frame, label)] = d / (np.linalg.norm(d) + 1e-12)

    def angle(a, b):
        return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1))))

    world_shift = angle(dirs[("world", "straight")], dirs[("world", "turned")])
    tool_shift = angle(dirs[("tool", "straight")], dirs[("tool", "turned")])
    assert world_shift < 10.0, f"world-frame motion should NOT follow the wrist, moved {world_shift:.1f}°"
    assert tool_shift > 25.0, f"tool-frame motion SHOULD follow the wrist, only moved {tool_shift:.1f}°"


def test_every_frame_still_moves_the_arm() -> None:
    """A frame that transforms the twist to nothing would be a silent dead control."""
    for frame in ("world", "tool", "camera"):
        tp = CartesianTeleop(frame=frame)
        tp.reset(PARK)
        before = tp.ee_position().copy()
        for _ in range(100):
            tp.step(np.array([0.12, 0.0, 0.0, 0.0, 0.0, 0.0]), DT)
        moved = float(np.linalg.norm(tp.ee_position() - before))
        assert moved > 0.05, f"frame {frame!r} moved the tool only {moved:.4f} m in 1 s"


def test_frame_note_describes_each_frame() -> None:
    for frame in ("world", "tool", "camera"):
        note = CartesianTeleop(frame=frame).frame_note()
        assert note and frame.upper() in note.upper()
    assert "⚠️" in CartesianTeleop(frame="camera").frame_note(), (
        "the camera frame is the MODELLED D405 mount and must say so — a hand-mounted "
        "webcam does not share it"
    )



# ── the workspace limit, replaced 2026-08-14 (FINDINGS §43) ──────────────────


def test_a_pose_inside_the_limits_is_returned_untouched() -> None:
    """Every park pose on record is inside, so a normal session never clamps."""
    for pose in ([0.111, 0.0, 0.174], [0.110, -0.003, 0.179], [0.211, 0.223, 0.306]):
        out = clamp_to_workspace(np.array(pose), REACH_LIMIT, FLOOR_LIMIT)
        assert np.allclose(out, pose), (pose, out)


def test_a_pose_beyond_the_reach_is_pulled_back_along_the_same_direction() -> None:
    """⭐ Pulled straight in, so the direction the operator was driving is preserved.
    Clamping per axis instead would slide the tip sideways, which is the kind of motion
    nobody asked for (FINDINGS §18)."""
    far = np.array([0.6, 0.0, 0.45])          # 0.75 m out
    out = clamp_to_workspace(far, 0.60, 0.05)
    assert abs(np.linalg.norm(out) - 0.60) < 1e-9
    assert np.allclose(out / np.linalg.norm(out), far / np.linalg.norm(far))


def test_the_floor_stops_the_tip_going_below_the_base() -> None:
    """⛔ THE REASON THE FLOOR EXISTS. The old cube bounded the tip above z=0.175 as a
    side effect of being a cube. A bare sphere has no floor, and this arm can put its
    tip at z=-0.377, below its own base."""
    out = clamp_to_workspace(np.array([0.2, 0.0, -0.30]), 0.60, 0.05)
    assert out[2] == 0.05, out


def test_reach_and_floor_can_both_bite_at_once() -> None:
    out = clamp_to_workspace(np.array([0.7, 0.0, -0.30]), 0.60, 0.05)
    assert np.linalg.norm([out[0], out[1]]) <= 0.60 + 1e-9
    assert out[2] == 0.05


def test_an_arm_that_starts_OUTSIDE_is_not_yanked_inward() -> None:
    """⛔⭐⭐ THE ONE REAL HAZARD IN SWAPPING A MOVING LIMIT FOR A FIXED ONE.

    The old cube re-centred on the arm at TELEOP entry, so it could never be entered
    from outside. A fixed limit can be. Clamping to 0.60 when the arm sits at 0.65
    would command it inward the instant TELEOP starts, and nobody asked for that.
    """
    start = np.array([0.65, 0.0, 0.20])       # 0.680 m out
    reach, floor = effective_limits(start, 0.60, 0.05)
    assert reach > 0.60, "the limit must open to include the starting pose"
    assert np.allclose(clamp_to_workspace(start, reach, floor), start), "it was yanked"


def test_a_start_BELOW_the_floor_lowers_the_floor_rather_than_lifting_the_arm() -> None:
    start = np.array([0.2, 0.0, -0.10])
    reach, floor = effective_limits(start, 0.60, 0.05)
    assert floor <= -0.10
    assert np.allclose(clamp_to_workspace(start, reach, floor), start)


def test_a_normal_start_leaves_both_limits_exactly_as_configured() -> None:
    reach, floor = effective_limits(np.array([0.110, -0.003, 0.179]), 0.60, 0.05)
    assert (reach, floor) == (0.60, 0.05)


def test_no_starting_pose_yet_means_the_configured_limits() -> None:
    assert effective_limits(None, 0.60, 0.05) == (0.60, 0.05)


def test_the_limit_does_NOT_shrink_back_during_a_session() -> None:
    """⚠️ Deliberate. A limit that moves mid-session is what was wrong with the cube,
    so widening for the starting pose is a one-time decision and holds all session."""
    start = np.array([0.65, 0.0, 0.20])
    reach, _ = effective_limits(start, 0.60, 0.05)
    # the arm comes in; the limit is still derived from the same starting pose
    again, _ = effective_limits(start, 0.60, 0.05)
    assert again == reach


def test_the_readout_reports_distance_out_and_height_above_the_floor() -> None:
    out, up = workspace_room(np.array([0.219, 0.029, 0.475]), 0.60, 0.05)
    assert abs(out - 0.524) < 0.002, out      # his measured pose, FINDINGS §41.1
    assert abs(up - 0.425) < 1e-9, up


def test_the_clamp_never_returns_a_position_outside_the_limits_it_was_given() -> None:
    rng = np.random.default_rng(20260814)
    for _ in range(400):
        ee = rng.uniform(-1.0, 1.0, 3)
        out = clamp_to_workspace(ee, 0.60, 0.05)
        assert np.linalg.norm(out) <= 0.60 + 1e-9 or out[2] == 0.05
        assert out[2] >= 0.05 - 1e-12


def test_a_widened_limit_leaves_ROOM_and_does_not_sit_on_the_wall() -> None:
    """⛔⭐⭐ THE KNIFE EDGE, and it cost a real regression before the margin existed.

    The first version widened the limit to exactly the starting distance, so an arm
    starting outside sat precisely on the wall and the clamp fired every cycle. A
    position clamp fights the orientation task in the QP — written down in
    `_limit_lead`'s own notes — and commanding pure roll from the FOLDED pose then
    moved the tool point 0.178 m, against under 0.002 m with room to spare.
    """
    start = np.array([-0.183, -0.075, 0.577])      # the FOLDED pose, 0.610 m out
    reach, _ = effective_limits(start, REACH_LIMIT, FLOOR_LIMIT)
    room = reach - float(np.linalg.norm(start))
    assert room >= 0.04, f"only {room:.3f} m of room past the start — the clamp will chatter"
    assert np.allclose(clamp_to_workspace(start, reach, FLOOR_LIMIT), start)


def test_the_widening_margin_is_at_least_one_lead_length() -> None:
    """⭐ The reason 0.05 is the number: the goal is already allowed to lead the arm by
    `max_lead_m`, so a limit closer than that sits inside the controller's own slack."""
    from teleop import LIMIT_WIDEN_MARGIN  # noqa: PLC0415
    assert LIMIT_WIDEN_MARGIN >= CartesianTeleop().max_lead_m


def test_starting_inside_adds_no_margin_at_all() -> None:
    """⚠️ Otherwise every normal session would quietly run a wider limit than configured."""
    for pose in ([0.211, 0.223, 0.306], [0.110, -0.003, 0.179], [0.251, 0.0, 0.209]):
        assert effective_limits(np.array(pose), 0.60, 0.05) == (0.60, 0.05)


def test_the_DEFAULT_floor_cannot_block_a_pick_off_the_desk() -> None:
    """⛔⭐⭐ JULIEN'S REQUIREMENT, and the floor was set wrong for about an hour.

    It shipped at +0.05 m, and he caught it before it ever ran: *"the bottom floor five
    centimeter thing… sounds problematic because then I can't really pick anything up
    from the table anymore."* A floor above the base plane stops the tip short of
    anything lying on the desk, and picking things off the desk is what the rig is for.

    A limit that forbids the task is worse than no limit, because it gets switched off.
    """
    assert FLOOR_LIMIT <= 0.0, (
        f"the default floor is {FLOOR_LIMIT:+.3f} m, above the base plane — that blocks "
        "every pick off the desk"
    )


def test_the_DEFAULT_floor_still_bounds_a_gross_excursion() -> None:
    """⚠️ The other half. The floor is not desk protection, because the desk height has
    never been measured (ROADMAP §8.4). It bounds the 0.377 m below-base excursion this
    arm is otherwise capable of, and it must keep doing that."""
    assert FLOOR_LIMIT > -0.30, f"a floor at {FLOOR_LIMIT} bounds nothing useful"
    out = clamp_to_workspace(np.array([0.2, 0.0, -0.377]), REACH_LIMIT, FLOOR_LIMIT)
    assert out[2] == FLOOR_LIMIT
    assert out[2] > -0.30


def test_every_park_pose_clears_the_default_floor() -> None:
    """The measured park poses. If a limit ever obstructed a park, `q p d` would break.

    ⚠️ This asked for 0.20 m of clearance while the floor sat at −0.10, and it failed the
    moment Julien moved the floor to the base plane: the lowest park pose is at z = 0.174,
    so it clears a zero floor by 0.174 rather than 0.20. **The test was pinning the old
    floor value through a margin, instead of pinning the property it cares about** — that
    a park is never obstructed. 0.15 m is that property, with room to spare.
    """
    for z in (0.174, 0.179, 0.306):
        assert z - FLOOR_LIMIT > 0.15, f"park pose at z={z} is only {z - FLOOR_LIMIT:.2f} above the floor"

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




# --------------------------------------------------------- speed throttle ----

SAFEROBOT_CAP = 1.0   # rad/s — what SafeRobot actually allows through


def _cycles_over_cap(v: float, throttle: bool, cycles: int = 200):
    tp = CartesianTeleop(max_joint_rate=0.9 if throttle else 1e9)
    tp.reset(PARK)
    prev = PARK.copy()
    over, lead = 0, 0.0
    for _ in range(cycles):
        q = tp.step(np.array([v, 0, 0, 0, 0, 0]), DT)
        if float(np.max(np.abs(q - prev))) / DT > SAFEROBOT_CAP:
            over += 1
        prev = q.copy()
        lead = max(lead, tp.lead()[0])
    return over, lead, tp.speed_scale


def test_the_throttle_stops_the_rate_limiter_fighting_the_solver() -> None:
    """⭐ THE FIX. Without it, 86 of 200 cycles at 0.25 m/s asked for more joint
    speed than SafeRobot allows, so the arm lagged by construction."""
    for v in (0.25, 0.4, 0.6):
        without, _, _ = _cycles_over_cap(v, throttle=False)
        with_, lead, _ = _cycles_over_cap(v, throttle=True)
        assert without > 20, f"{v} m/s: expected the un-throttled case to saturate, got {without}"
        assert with_ <= 3, f"{v} m/s: still saturating on {with_} cycles with the throttle on"
        assert lead < 0.02, f"{v} m/s: command ran {lead:.3f} m ahead of the arm"


def test_low_speed_is_completely_unaffected() -> None:
    """Normal driving must not be slowed down to fix an edge case."""
    over, lead, scale = _cycles_over_cap(0.12, throttle=True)
    assert over == 0 and lead < 0.005
    assert scale == 1.0, f"the throttle engaged at a speed that never needed it (scale {scale})"


def test_the_throttle_recovers_when_the_arm_can_follow_again() -> None:
    """It must not latch. Drive into the workspace edge, then come back."""
    tp = CartesianTeleop()
    tp.reset(PARK)
    for _ in range(200):                        # out to the edge; scale collapses
        tp.step(np.array([0.4, 0, 0, 0, 0, 0]), DT)
    assert tp.speed_scale < 0.5, f"expected a throttle near the edge, got {tp.speed_scale}"
    for _ in range(300):                        # come back toward the middle
        tp.step(np.array([-0.2, 0, 0, 0, 0, 0]), DT)
    assert tp.speed_scale > 0.9, f"the throttle latched: still {tp.speed_scale:.2f} back in open space"


def test_reset_clears_the_throttle() -> None:
    """A mode change re-seeds from reality and must not inherit a stale throttle."""
    tp = CartesianTeleop()
    tp.reset(PARK)
    for _ in range(200):
        tp.step(np.array([0.6, 0, 0, 0, 0, 0]), DT)
    assert tp.speed_scale < 1.0
    tp.reset(PARK)
    assert tp.speed_scale == 1.0


def test_the_throttle_costs_time_near_the_edge_but_not_reach() -> None:
    """⚠️ The distinction this test exists to pin down.

    An earlier version compared reach after a FIXED number of cycles and failed —
    correctly, because the throttle deliberately slows the approach to the workspace
    edge, so at any fixed moment the throttled arm is behind. That is the feature
    working, not a defect. What must not change is where the arm can eventually GET.

    Measured: both converge on 0.5194 m. The throttle trades time, not workspace.
    """
    def reach(throttle, cycles):
        tp = CartesianTeleop(max_joint_rate=0.9 if throttle else 1e9)
        tp.reset(PARK)
        home = tp.ee_position().copy()
        for _ in range(cycles):
            tp.step(np.array([0.4, 0, 0, 0, 0, 0]), DT)
        return float(np.linalg.norm(tp.ee_position() - home))

    assert reach(True, 200) < reach(False, 200), "the throttle should slow the approach"
    settled_on, settled_off = reach(True, 2000), reach(False, 2000)
    assert abs(settled_on - settled_off) < 0.005, (
        f"given time, reach must be unchanged: {settled_on:.4f} vs {settled_off:.4f}"
    )


if __name__ == "__main__":
    sys.exit(main())
