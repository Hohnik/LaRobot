#!/usr/bin/env python3
"""Tests for the two-arm distance measurement, `src/yam/collision.py`. No hardware.

    uv run tests/test_collision.py

⭐⭐ THE TEST THAT MATTERS MOST IS `test_the_JOINT_ANGLES_actually_change_the_answer`.
Everything else could pass with `link_points()` ignoring its argument entirely and
returning the model's rest pose every time — and a collision measurement that silently
ignores where the arms are pointing would be the most dangerous possible version of this
file, because it would return confident, stable, plausible numbers forever.

⚠️ This module measures and never refuses, so there is no "does it stop the arm" test to
write. The margin is Julien's decision (docs/FINDINGS.md §58.4 item 4).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

from yam.collision import (  # noqa: E402
    ArmGeometry,
    BasePose,
    closest_approach,
    describe,
    reach_spheres_can_touch,
)
from yam.teleop import REACH_LIMIT  # noqa: E402

GEOM = ArmGeometry()          # built once; loading the XML is the slow part
ZERO = np.zeros(7)


def facing(sep: float) -> tuple[BasePose, BasePose]:
    """Two arms `sep` metres apart across a desk, facing each other."""
    return BasePose(0.0, 0.0, 0.0, 0.0), BasePose(sep, 0.0, 0.0, 180.0)


# ------------------------------------------------------------- the anti-tautology test


def test_the_JOINT_ANGLES_actually_change_the_answer() -> None:
    """⛔⭐⭐ THE ONE DEFECT THAT WOULD MAKE THIS FILE WORSE THAN USELESS.

    If `link_points()` ignored `q_arm` and always returned the rest pose, every other test
    here would still pass and the module would report a confident, stable, wrong number
    forever. Two clearly different poses must give clearly different distances.
    """
    a, b = facing(0.9)
    folded = closest_approach(ZERO, ZERO, a, b, geometry=GEOM)

    # ⭐ JOINT INDEX 2, AND THE INDEX WAS MEASURED RATHER THAN ASSUMED. The first version
    # of this test used index 1 and failed with "changed by only 2.13 cm", which looked
    # like the module ignoring its input. It was not: index 1 is a shoulder PITCH that
    # lifts the tip 163 mm vertically and moves it **3 mm** horizontally, so it barely
    # changes the gap between two arms standing side by side. Index 2 is the elbow and
    # moves the tip 260 mm in x. ⚠️ A test can fail for being wrong about the machine.
    reaching = np.zeros(7)
    reaching[2] = 1.2
    out = closest_approach(reaching, ZERO, a, b, geometry=GEOM)

    assert abs(out.distance - folded.distance) > 0.05, (
        f"posing the arm changed the distance by only "
        f"{abs(out.distance - folded.distance) * 100:.2f} cm "
        f"({folded.distance:.3f} -> {out.distance:.3f}). The joint angles are probably "
        f"being ignored, which is the worst possible defect in this file")


def test_moving_the_OTHER_arm_also_changes_it() -> None:
    """⚠️ Same trap, second argument. A function that reads only its first arm would pass
    the test above."""
    a, b = facing(0.9)
    base = closest_approach(ZERO, ZERO, a, b, geometry=GEOM)
    reaching = np.zeros(7)
    reaching[2] = 1.2                     # the elbow; see the note in the test above
    moved = closest_approach(ZERO, reaching, a, b, geometry=GEOM)
    assert abs(moved.distance - base.distance) > 0.05, (
        "posing the SECOND arm did not change the distance")


# --------------------------------------------------------------------- the basic shape


def test_further_apart_means_more_clearance() -> None:
    last = -99.0
    for sep in (0.5, 0.7, 1.0, 1.4):
        a, b = facing(sep)
        d = closest_approach(ZERO, ZERO, a, b, geometry=GEOM).distance
        assert d > last, f"clearance did not grow from {last:.3f} at {sep} m ({d:.3f})"
        last = d


def test_bases_on_top_of_each_other_report_an_OVERLAP() -> None:
    """⚠️ Negative means the bounding spheres intersect, not that metal is touching."""
    a, b = facing(0.05)
    c = closest_approach(ZERO, ZERO, a, b, geometry=GEOM)
    assert c.spheres_overlap and c.distance < 0.0
    assert "OVERLAP" in describe(c)


def test_the_answer_is_symmetric_when_the_arms_are_swapped() -> None:
    """⭐ Distance is a property of the pair. An asymmetry would mean one arm's radii were
    being applied to the other, which is the kind of index slip that reads as plausible."""
    a, b = facing(0.8)
    q = np.zeros(7)
    q[1] = 0.7
    one = closest_approach(q, ZERO, a, b, geometry=GEOM).distance
    other = closest_approach(ZERO, q, b, a, geometry=GEOM).distance
    assert abs(one - other) < 1e-9, f"{one} vs {other} — swapping the arms changed it"


def test_which_bodies_are_closest_is_NAMED() -> None:
    """⭐ A report that says "12 cm" is far less useful than one that says which parts.
    The names come from the model, so they match what a person sees on the bench."""
    a, b = facing(0.8)
    c = closest_approach(ZERO, ZERO, a, b, geometry=GEOM)
    assert c.body_a in GEOM.body_names and c.body_b in GEOM.body_names
    assert c.body_a != "world" and c.body_b != "world", (
        "the closest body should be a real part, not the world frame")


def test_the_base_separation_is_carried_in_the_answer() -> None:
    """⚠️ Every distance reading is meaningless without it, so it travels with it."""
    a, b = facing(0.85)
    assert abs(closest_approach(ZERO, ZERO, a, b, geometry=GEOM).base_separation
               - 0.85) < 1e-9


# ------------------------------------------------------------------ the geometry basics


def test_yaw_CHANGES_the_answer() -> None:
    """⭐ If yaw were dropped, two arms back to back would read exactly the same as two
    arms reaching toward each other, and the measurement would be meaningless.

    ⛔⭐⭐ THIS TEST ORIGINALLY ASSERTED SOMETHING FALSE, and finding out why taught me the
    shape of the arm. It claimed "facing away must be roomier than facing each other", and
    the measurement said the opposite: 0.265 m facing away against 0.377 m facing each
    other. **The code was right.** At the rest pose `link3` sits at **x = −0.244 m with a
    bounding radius of 0.197 m** — a large body sticking 24 cm out the BACK of the arm. So
    turning an arm around does not move it away from its neighbour; it swings that rear
    link toward it.

    ⚠️ The lesson is about testing, not about geometry: I asserted an intuition about a
    machine I had not measured, and a red test looked briefly like a defect in the code.
    ⭐ **So this now pins what actually matters — that yaw is not ignored** — and leaves
    which orientation is roomier to be measured per bench rather than assumed.
    """
    reaching = np.zeros(7)
    reaching[2] = 1.0
    toward = closest_approach(
        reaching, reaching, BasePose(0, 0, 0, 0), BasePose(0.9, 0, 0, 180)).distance
    away = closest_approach(
        reaching, reaching, BasePose(0, 0, 0, 180), BasePose(0.9, 0, 0, 0)).distance
    assert abs(away - toward) > 0.05, (
        f"turning both arms around changed the clearance by only "
        f"{abs(away - toward) * 100:.2f} cm ({toward:.3f} vs {away:.3f}) — yaw is "
        f"probably being dropped")


def test_to_world_rotates_and_translates() -> None:
    pts = np.array([[1.0, 0.0, 0.0]])
    out = BasePose(0.0, 0.0, 0.0, 90.0).to_world(pts)[0]
    assert abs(out[0]) < 1e-9 and abs(out[1] - 1.0) < 1e-9, f"90° yaw gave {out}"
    shifted = BasePose(2.0, 3.0, 0.5).to_world(pts)[0]
    assert np.allclose(shifted, [3.0, 3.0, 0.5]), f"translation gave {shifted}"


def test_height_counts_toward_the_separation() -> None:
    """⚠️ Two bases at the same x,y but different heights are not in the same place."""
    assert abs(BasePose(0, 0, 0).distance_to(BasePose(0, 0, 0.4)) - 0.4) < 1e-9


# ------------------------------------------------------------- the cheap analytic bound


def test_the_reach_spheres_bound_matches_the_reach_limit() -> None:
    """⭐⭐ The answer that may close the whole question: beyond twice the reach limit a
    collision is geometrically impossible while that limit is enforced."""
    just_inside = BasePose(2.0 * REACH_LIMIT - 0.01, 0, 0)
    just_outside = BasePose(2.0 * REACH_LIMIT + 0.01, 0, 0)
    origin = BasePose(0, 0, 0)
    assert reach_spheres_can_touch(origin, just_inside) is True
    assert reach_spheres_can_touch(origin, just_outside) is False


def test_the_bound_reads_the_SHARED_reach_constant() -> None:
    """⛔ A hard-coded 0.60 here would silently disagree with the session the moment
    Julien changed `--reach`. This pins that it comes from the same constant."""
    origin = BasePose(0, 0, 0)
    edge = BasePose(2.0 * REACH_LIMIT, 0, 0)
    assert reach_spheres_can_touch(origin, edge) is True
    # And an explicitly passed reach overrides it, which is what --reach would do.
    assert reach_spheres_can_touch(origin, edge, reach=REACH_LIMIT / 2) is False


def test_a_base_pose_cannot_be_conjured_without_a_measurement() -> None:
    """⛔⭐ `BasePose` has no default x or y ON PURPOSE. Nothing in the repo records where
    the bases are, and a default would be an unmeasured number quietly deciding whether
    two 4.3 kg arms may occupy the same space."""
    try:
        BasePose()  # type: ignore[call-arg]
    except TypeError:
        pass
    else:
        raise AssertionError("BasePose() was constructible with no measurement at all")


def test_the_estimate_is_CONSERVATIVE_not_optimistic() -> None:
    """⭐ Bounding spheres are larger than the parts inside them, so the reported gap must
    never exceed the true centre-to-centre distance of the closest pair. Being wrong in
    this direction is the only acceptable way for a safety measurement to be wrong."""
    a, b = facing(0.9)
    c = closest_approach(ZERO, ZERO, a, b, geometry=GEOM)
    pts_a = a.to_world(GEOM.link_points(ZERO))
    pts_b = b.to_world(GEOM.link_points(ZERO))
    centre_min = float(np.min(np.linalg.norm(
        pts_a[:, None, :] - pts_b[None, :, :], axis=2)))
    assert c.distance <= centre_min + 1e-9, (
        f"reported {c.distance:.4f} m of clearance but the closest body ORIGINS are only "
        f"{centre_min:.4f} m apart — the estimate is optimistic, which it must never be")


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
