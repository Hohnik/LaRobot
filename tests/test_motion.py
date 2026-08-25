#!/usr/bin/env python3
"""Tests for `JointPath` — the blended waypoint path. No hardware.

    uv run tests/test_motion.py

⛔ WHY THESE MATTER. This decides the shape the arm moves through when running a
saved sequence, and it is the first thing in this project that moves the arm along a
plan rather than under a hand or a puck. Two properties are safety properties rather
than aesthetics, and both are pinned below:

- a blended corner must stay **inside the joint range the waypoints already span**,
  so rounding can never invent a limit violation;
- the path **length must be exact**, because cursor speed is derived from it and the
  stall thresholds are calibrated in rad/s.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.motion import JointPath, _dist, plan_gripper_stops  # noqa: E402


def corner_path(blend: float) -> JointPath:
    """A right-angle turn in two joints: out along joint 0, then along joint 1."""
    return JointPath([np.array([0.0, 0.0]), np.array([1.0, 0.0]),
                      np.array([1.0, 1.0])], blend=blend)


def sample(path: JointPath, n: int = 200):  # noqa: ANN201
    return [path.point_at(path.length * i / n) for i in range(n + 1)]


def test_a_straight_path_measures_its_real_length() -> None:
    """Chebyshev, so length is the largest single-joint move — which makes cursor
    speed mean "the fastest joint's rad/s", exactly what PARK_SPEED has always meant."""
    path = JointPath([np.array([0.0, 0.0]), np.array([1.0, 0.5])])
    assert abs(path.length - 1.0) < 1e-9


def test_the_ends_are_exactly_the_first_and_last_waypoint() -> None:
    path = corner_path(0.2)
    assert np.allclose(path.point_at(0.0), [0.0, 0.0])
    assert np.allclose(path.point_at(path.length), [1.0, 1.0])
    assert np.allclose(path.point_at(-5.0), [0.0, 0.0]), "before the start clamps"
    assert np.allclose(path.point_at(1e6), [1.0, 1.0]), "past the end clamps"


def test_without_blending_the_path_passes_through_every_waypoint() -> None:
    """⛔ blend=0 must reproduce the old behaviour EXACTLY, so `--no-smooth` and the
    'sharp' corner mode are a genuine escape hatch rather than an approximation."""
    path = corner_path(0.0)
    pts = sample(path, 400)
    assert min(_dist(p, [1.0, 0.0]) for p in pts) < 1e-6, "the corner was cut anyway"
    assert abs(path.length - 2.0) < 1e-9


def test_blending_rounds_the_corner_and_shortens_the_path() -> None:
    """A cut corner is a shorter route. If the length did not drop, nothing rounded."""
    sharp, smooth = corner_path(0.0), corner_path(0.3)
    assert smooth.length < sharp.length
    assert min(_dist(p, [1.0, 0.0]) for p in sample(smooth)) > 0.02, (
        "the path still goes through the sharp corner")


def test_a_bigger_blend_rounds_more() -> None:
    lengths = [corner_path(b).length for b in (0.0, 0.1, 0.3)]
    assert lengths[0] > lengths[1] > lengths[2], lengths


def test_blending_cannot_leave_the_joint_range_the_waypoints_span() -> None:
    """⛔⭐ THE SAFETY PROPERTY. A quadratic Bézier stays inside the convex hull of
    its three control points, and all three sit on the original straight segments —
    so a rounded corner can never take a joint beyond where the waypoints already go,
    and cannot invent a joint-limit violation."""
    waypoints = [np.array([0.0, 0.0, 0.5]), np.array([1.0, 0.0, 0.5]),
                 np.array([1.0, 1.0, 0.2]), np.array([0.2, 0.8, 0.9])]
    lo = np.min(np.array(waypoints), axis=0)
    hi = np.max(np.array(waypoints), axis=0)
    for p in sample(JointPath(waypoints, blend=0.4), 500):
        assert np.all(p >= lo - 1e-9) and np.all(p <= hi + 1e-9), (
            f"the blended path reached {p}, outside the waypoints' own range")


def test_the_direction_never_jumps_on_a_blended_corner() -> None:
    """⭐ THIS IS THE JITTER JULIEN DESCRIBED. On a sharp corner the direction of
    travel reverses in one step; blended, it turns gradually. Measured as the largest
    change in unit direction between consecutive samples."""
    def worst_turn(path: JointPath) -> float:
        pts = sample(path, 300)
        dirs = []
        for a, b in zip(pts, pts[1:]):
            d = np.asarray(b) - np.asarray(a)
            n = np.linalg.norm(d)
            if n > 1e-12:
                dirs.append(d / n)
        return max(float(np.linalg.norm(y - x)) for x, y in zip(dirs, dirs[1:]))

    assert worst_turn(corner_path(0.3)) < 0.5 * worst_turn(corner_path(0.0)), (
        "blending did not soften the direction change")


def test_the_blend_cannot_eat_a_short_segment() -> None:
    """⚠️ Clipped to half the shorter neighbour, so two corners can never overlap and
    swallow the waypoint between them — which would silently drop a pose the operator
    asked the arm to visit."""
    waypoints = [np.array([0.0, 0.0]), np.array([1.0, 0.0]),
                 np.array([1.05, 0.0]), np.array([1.05, 1.0])]
    path = JointPath(waypoints, blend=5.0)          # absurdly large on purpose
    assert min(_dist(p, waypoints[1]) for p in sample(path, 500)) < 0.05
    assert min(_dist(p, waypoints[2]) for p in sample(path, 500)) < 0.05


def test_repeated_waypoints_do_not_produce_NaN() -> None:
    """`p 1 1 2 Enter` is easy to type by accident, and a zero-length segment has no
    direction — the unit vector would be NaN and every later sample poisoned."""
    path = JointPath([np.array([0.0, 0.0]), np.array([0.0, 0.0]),
                      np.array([1.0, 0.0])], blend=0.2)
    assert np.all(np.isfinite(path.point_at(path.length / 2)))
    assert abs(path.length - 1.0) < 1e-9


def test_a_single_waypoint_is_a_zero_length_path() -> None:
    path = JointPath([np.array([0.3, 0.4])], blend=0.2)
    assert path.length == 0.0
    assert np.allclose(path.point_at(0.0), [0.3, 0.4])
    assert np.allclose(path.point_at(10.0), [0.3, 0.4])


def test_the_cursor_advances_monotonically_along_the_path() -> None:
    """The command must never step backwards — the arm would reverse mid-move."""
    path = corner_path(0.2)
    seen = [path.point_at(s) for s in np.linspace(0, path.length, 300)]
    travelled = [0.0]
    for a, b in zip(seen, seen[1:]):
        travelled.append(travelled[-1] + _dist(a, b))
    assert all(y >= x - 1e-12 for x, y in zip(travelled, travelled[1:]))
    # ⚠️ Re-sampling a CURVE coarsely always under-measures it: each chord cuts inside
    # the arc, and Chebyshev obeys the triangle inequality. So this is bounded, not
    # exact — the real length is the finely-sampled one the path itself measured.
    assert travelled[-1] <= path.length + 1e-9, "resampling cannot exceed the true length"
    assert travelled[-1] > 0.98 * path.length, "resampling lost more than 2%"


def test_waypoint_arrival_marks_are_in_order() -> None:
    """⭐ How the session says "now heading for slot 3" during a continuous run. The
    honest objection to blending was that "which pose is the arm at" becomes vague;
    this answers it instead of dropping it."""
    path = JointPath([np.array([0.0, 0.0]), np.array([1.0, 0.0]),
                      np.array([1.0, 1.0]), np.array([0.0, 1.0])], blend=0.2)
    marks = path.arrival_lengths()
    assert marks == sorted(marks), marks
    assert marks[0] == 0.0
    assert abs(marks[-1] - path.length) < 1e-9


# --------------------------------- where a run must stop so the jaws can move ----
#
# ⛔ Blending means the arm curves THROUGH a waypoint without stopping, which is the smooth
# motion Julien confirmed on the arm. The gripper is another number in the same vector, so a
# blended corner between "jaws open" and "jaws closed" closes them mid-move. Every grab
# depends on getting this split right.

# ⛔ NORMALISED, 0 closed to 1 open. Not raw motor radians: the SDK normalises joint 7
# against the calibrated limits, so a saved pose holds a fraction of the stroke. Every real
# recording on this rig reads 0.036 for the nearly-closed jaws, which is how it was checked.
OPEN, SHUT = 0.90, 0.04


def grab_run():  # noqa: ANN201
    """The natural way to save a pick: approach, arrive, close, lift."""
    return [
        [0.0, 0.0, 0, 0, 0, 0, OPEN],       # 0 where the arm is now
        [0.5, 0.5, 0, 0, 0, 0, OPEN],       # 1 above the object
        [0.5, 0.8, 0, 0, 0, 0, OPEN],       # 2 at the object
        [0.5, 0.8, 0, 0, 0, 0, SHUT],       # 3 jaws closed, arm still
        [0.5, 0.3, 0, 0, 0, 0, SHUT],       # 4 lifted
    ]


def test_a_grab_splits_exactly_where_the_jaws_close() -> None:
    """⭐ The whole point. The arm blends through the approach, stops at the object, closes,
    then blends away. It arrives at the object EXACTLY, because the corner it would have
    rounded is now the end of a segment."""
    plan = plan_gripper_stops(grab_run(), gripper_index=6)
    assert plan.segments == [[0, 1, 2], [3, 4]], plan.segments
    assert plan.gripper_legs == [(2, 3)], plan.gripper_legs
    assert plan.warnings == []


def test_a_run_that_never_touches_the_gripper_stays_ONE_blended_path() -> None:
    """⛔ Otherwise this change would quietly undo the smooth motion he confirmed."""
    poses = [[0, 0, 0, 0, 0, 0, OPEN], [0.5, 0, 0, 0, 0, 0, OPEN], [1.0, 0.4, 0, 0, 0, 0, OPEN]]
    plan = plan_gripper_stops(poses, gripper_index=6)
    assert plan.segments == [[0, 1, 2]]
    assert plan.gripper_legs == []


def test_moving_the_arm_and_the_jaws_together_WARNS_instead_of_guessing() -> None:
    """⚠️ Both readings are defensible: close while approaching, or stop and close. Guessing
    wrong on 4.3 kg is worse than saying so, so today's behaviour is kept and reported."""
    poses = [[0, 0, 0, 0, 0, 0, OPEN], [0.5, 0.8, 0, 0, 0, 0, SHUT]]
    plan = plan_gripper_stops(poses, gripper_index=6)
    assert plan.gripper_legs == [], "it must not split a leg that also moves the arm"
    assert len(plan.warnings) == 1
    assert "arm AND the gripper" in plan.warnings[0]


def test_a_tiny_gripper_difference_is_not_a_jaw_movement() -> None:
    """A pose re-saved in the same place differs by sensor noise. Splitting on that would
    stop the arm at every waypoint, which is the behaviour this replaced."""
    poses = [[0, 0, 0, 0, 0, 0, OPEN], [0.5, 0, 0, 0, 0, 0, OPEN - 0.01]]
    assert plan_gripper_stops(poses, gripper_index=6).gripper_legs == []


def test_two_grabs_in_one_run_both_split() -> None:
    """Pick something up, put it down: two gripper legs, three segments."""
    poses = [
        [0.0, 0.0, 0, 0, 0, 0, OPEN],
        [0.5, 0.8, 0, 0, 0, 0, OPEN],
        [0.5, 0.8, 0, 0, 0, 0, SHUT],       # close
        [1.0, 0.8, 0, 0, 0, 0, SHUT],
        [1.0, 0.8, 0, 0, 0, 0, OPEN],       # open again
        [1.0, 0.3, 0, 0, 0, 0, OPEN],
    ]
    plan = plan_gripper_stops(poses, gripper_index=6)
    assert plan.segments == [[0, 1], [2, 3], [4, 5]], plan.segments
    assert plan.gripper_legs == [(1, 2), (3, 4)]


def test_a_six_joint_robot_with_no_gripper_is_handled() -> None:
    """⛔ --no-gripper produces 6-value poses. A length assumption here is the same class of
    bug that once raised inside the control loop and dropped a raised arm."""
    poses = [[0.0] * 6, [0.5] * 6]
    plan = plan_gripper_stops(poses, gripper_index=6)
    assert plan.segments == [[0, 1]]
    assert plan.gripper_legs == []
    assert plan.warnings == []


def test_a_single_pose_run_is_not_split() -> None:
    assert plan_gripper_stops([[0.0] * 7], gripper_index=6).segments == [[0]]
    assert plan_gripper_stops([], gripper_index=6).segments == [[]]


def test_every_waypoint_appears_exactly_once_across_the_segments() -> None:
    """⭐ The invariant that stops a waypoint being skipped or run twice. A skipped waypoint
    is an arm going somewhere nobody asked for."""
    plan = plan_gripper_stops(grab_run(), gripper_index=6)
    seen = [i for seg in plan.segments for i in seg]
    assert sorted(seen) == list(range(5)), seen
    assert len(seen) == len(set(seen)), "a waypoint appears twice"


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
