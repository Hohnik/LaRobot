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

from teleop import CartesianTeleop  # noqa: E402
from yam_can import YAM_JOINTS  # noqa: E402

DT = 0.01
N_ARM = 6
MAX_JOINT_STEP = 0.015
JOINT_LIMIT_MARGIN = 0.08
WORKSPACE_BOX = 0.30

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
        if np.any(np.abs(ee - home) > WORKSPACE_BOX):
            import mink  # noqa: PLC0415
            tp.target = mink.SE3.from_rotation_and_translation(
                rotation=tp.target.rotation(),
                translation=np.clip(ee, home - WORKSPACE_BOX, home + WORKSPACE_BOX),
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
