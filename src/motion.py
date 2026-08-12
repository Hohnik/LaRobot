"""⭐ Joint-space paths that curve THROUGH waypoints instead of stopping at each.

    path = JointPath([q_now, q1, q2, q3], blend=0.15)
    q = path.point_at(s)          # s in radians of max-joint travel

⛔ WHAT THIS FIXES, and it is a correction of something I built wrongly on
2026-08-12. Julien asked for smooth motion between saved poses. I implemented a
trapezoidal **speed ramp along each leg** — ease in, cruise, ease out — which makes a
single move gentler but leaves the arm **coming to a full stop at every waypoint**.
His description of what he actually wanted is precise:

    *"if it does a ninety degree turn, then instead of moving and then jittering
    ninety degrees to the next side, in a smooth curve it would go to the next point,
    and then in a smooth curve it would move on to connect to the next point — so
    that we have one smooth motion of specific waypoints."*

That is **corner blending**, not speed shaping. The two are independent and both are
wanted: the ramp decides *how fast* the cursor moves, this decides *what shape* it
follows.

WHY JOINT SPACE AND NOT CARTESIAN
---------------------------------
The waypoints are recorded joint poses, so a joint-space path needs no IK, cannot hit
a singularity, and — see `point_at` — provably stays inside the joint range the
waypoints already occupy. A Cartesian blend would look smoother in the world at the
cost of an IK solve per sample and a singularity risk on every corner, for poses that
were never Cartesian to begin with.

WHY MAX-NORM
------------
Distance between two poses is `max |Δjoint|` (Chebyshev), everywhere. That is
deliberate: it makes "cursor speed" mean **the fastest-moving joint's speed in
rad/s**, which is exactly what `PARK_SPEED` has always meant, so the existing feel
and the existing stall thresholds carry over unchanged.

⚠️ ONE REAL BEHAVIOUR CHANGE, stated because it will be felt. `advance_park_command`
moves *every* joint at up to the step size independently, so joints with less to do
arrive early and the arm's pose drifts through a shape nobody chose. A path moves all
joints in proportion, so **they arrive together** and the motion is the straight line
(or blended curve) between poses. Same duration — the longest joint still sets it —
but a different, and predictable, shape.
"""

from __future__ import annotations

import numpy as np

# How finely each rounded corner is sampled. 12 is smooth to the eye at these speeds
# and keeps a 10-waypoint path in the low hundreds of points, which is nothing.
CORNER_SAMPLES = 12


def _dist(a, b) -> float:  # noqa: ANN001
    """Chebyshev distance — the largest single-joint move. See the module docstring."""
    return float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))


class JointPath:
    """A polyline through joint-space waypoints, with rounded corners.

    `blend` is how far, in radians of max-joint travel, the path may leave a corner
    early to curve into the next segment. `blend=0` reproduces the old behaviour
    exactly: straight to each waypoint, sharp turn, straight to the next.

    ⭐ EACH CORNER IS A QUADRATIC BÉZIER, which buys three properties worth having:

    1. **Velocity direction is continuous** at both joins, which is precisely the
       "jitter" being removed — the arm never reverses or snaps direction.
    2. ⛔ **The curve stays inside the convex hull of its control points**, and all
       three lie on the original straight segments. So a blended path can never take
       a joint outside the range the waypoints themselves span — it cannot invent a
       joint-limit violation, and `test_blending_cannot_leave_the_joint_range_the_
       waypoints_span` pins that.
    3. The blend distance is clipped to **half** the shorter adjoining segment, so
       neighbouring corners can never overlap and eat a waypoint.

    ⚠️ The path is sampled to a polyline at construction and then measured exactly,
    rather than parameterised analytically. Arc length of a Bézier has no closed form,
    and a wrong length would make the cursor speed wrong — which is a *safety* number
    here, not a cosmetic one, because the stall thresholds are calibrated in rad/s.
    Sampling makes the length exactly what the arm will travel.
    """

    def __init__(self, waypoints, blend: float = 0.0,
                 corner_samples: int = CORNER_SAMPLES) -> None:  # noqa: ANN001
        pts = [np.asarray(w, dtype=float) for w in waypoints]
        # ⚠️ Consecutive duplicates have no direction, so they would produce a
        # zero-length unit vector and a NaN corner. Dropping them is not cosmetic.
        cleaned = [pts[0]] if pts else []
        for p in pts[1:]:
            if _dist(p, cleaned[-1]) > 1e-9:
                cleaned.append(p)
        self.waypoints = cleaned
        self.blend = max(0.0, float(blend))

        self.points: list[np.ndarray] = self._build(corner_samples)
        self._cum = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self._cum.append(self._cum[-1] + _dist(a, b))

    # ----------------------------------------------------------- building ----

    def _build(self, corner_samples: int) -> list[np.ndarray]:
        pts = self.waypoints
        if len(pts) < 3 or self.blend <= 0.0:
            return list(pts)

        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            a, p, b = pts[i - 1], pts[i], pts[i + 1]
            in_len, out_len = _dist(a, p), _dist(p, b)
            # Half the shorter neighbour, so two corners can never overlap.
            radius = min(self.blend, 0.5 * in_len, 0.5 * out_len)
            if radius <= 1e-9:
                out.append(p)
                continue
            start = p + (a - p) * (radius / in_len)     # leave the corner early
            end = p + (b - p) * (radius / out_len)      # rejoin after it
            out.append(start)
            for k in range(1, corner_samples):
                t = k / corner_samples
                out.append((1 - t) ** 2 * start + 2 * (1 - t) * t * p + t ** 2 * end)
            out.append(end)
        out.append(pts[-1])
        return out

    # ------------------------------------------------------------ queries ----

    @property
    def length(self) -> float:
        """Total travel, in radians of the fastest joint."""
        return self._cum[-1] if self._cum else 0.0

    def point_at(self, s: float):  # noqa: ANN201
        """The pose at arc length `s`, clamped to the ends."""
        if not self.points:
            raise ValueError("an empty path has no points")
        if s <= 0 or len(self.points) == 1:
            return self.points[0].copy()
        if s >= self.length:
            return self.points[-1].copy()
        i = int(np.searchsorted(self._cum, s, side="right")) - 1
        i = min(max(i, 0), len(self.points) - 2)
        span = self._cum[i + 1] - self._cum[i]
        t = 0.0 if span <= 0 else (s - self._cum[i]) / span
        return self.points[i] + (self.points[i + 1] - self.points[i]) * t

    def arrival_lengths(self) -> list[float]:
        """Arc length at which the path is nearest each original waypoint.

        ⭐ This is how the session reports *"now heading for slot 3"* during a
        continuous run. Without it a blended path has no notion of "which waypoint am
        I at", which was the honest objection to blending in the first place — so it
        is answered rather than dropped.
        """
        marks = []
        for w in self.waypoints:
            best_i = min(range(len(self.points)), key=lambda i: _dist(self.points[i], w))
            marks.append(self._cum[best_i])
        return marks
