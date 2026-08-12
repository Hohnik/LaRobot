#!/usr/bin/env python3
"""Tests for `JointPath` — the blended waypoint path. No hardware.

    uv run scripts/test_motion.py

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
sys.path.insert(0, str(REPO / "src"))

from motion import JointPath, _dist  # noqa: E402


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
