#!/usr/bin/env python3
"""How close can the two arms get? Answers it from ONE tape-measure reading.

    uv run scripts/check_collision.py --separation 0.9
    uv run scripts/check_collision.py --separation 0.9 --yaw-b 180   # facing each other

⭐⭐ THE QUESTION THIS SETTLES. docs/ROADMAP.md §8.2 item 25: nothing in this project knows
where the other arm is, and MIRROR mode is the first mode where an arm moves with nobody's
hand on it. **But each arm is already confined to a sphere of 0.60 m around its own base**
by a limit Julien chose, and two such spheres cannot intersect if the bases are more than
1.20 m apart. So the whole question may already be closed by a limit that exists — and one
measurement decides which.

⛔ IT MOVES NOTHING AND ENERGISES NOTHING. It is arithmetic on the shipped MuJoCo model.

⚠️ WHAT IT NEEDS FROM A HUMAN: the distance between the two arm bases, and which way each
one faces. **Nothing in the repo records either.** It is not derivable from anything the
software can see.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from collision import (  # noqa: E402
    ArmGeometry,
    BasePose,
    closest_approach,
    describe,
    reach_spheres_can_touch,
)
from teleop import REACH_LIMIT  # noqa: E402

#: Representative poses. ⚠️ NOT a search over everything the arms can do — a handful of
#: readable cases. Joint indices are measured, not assumed: index 0 is the base yaw, index
#: 2 is the elbow and the biggest horizontal mover (260 mm at 1 rad).
POSES = {
    "both at rest (all joints zero)": (0, 0.0, 0, 0.0),
    "both elbows out": (2, 1.2, 2, 1.2),
    "A's elbow out, B at rest": (2, 1.2, 0, 0.0),
    "both yawed toward each other": (0, 0.8, 0, -0.8),
    "both yawed and elbows out": (2, 1.2, 0, 0.8),
}


def pose(joint: int, value: float) -> np.ndarray:
    q = np.zeros(7)
    q[joint] = value
    return q


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--separation", type=float, required=True,
                    help="metres between the two arm BASES (a tape-measure reading)")
    ap.add_argument("--yaw-a", type=float, default=0.0,
                    help="degrees arm A is turned about vertical (default 0)")
    ap.add_argument("--yaw-b", type=float, default=0.0,
                    help="degrees arm B is turned; 180 = facing each other")
    ap.add_argument("--height-b", type=float, default=0.0,
                    help="metres arm B's base sits above arm A's (default 0, same desk)")
    ap.add_argument("--reach", type=float, default=REACH_LIMIT,
                    help=f"the reach limit in force (default {REACH_LIMIT})")
    args = ap.parse_args()

    a = BasePose(0.0, 0.0, 0.0, args.yaw_a)
    b = BasePose(args.separation, 0.0, args.height_b, args.yaw_b)

    print(f"\nTWO-ARM CLEARANCE — bases {a.distance_to(b):.3f} m apart, "
          f"arm A yawed {args.yaw_a:g}°, arm B yawed {args.yaw_b:g}°.")
    print("Arithmetic on the shipped MuJoCo model. Nothing was moved or energised.\n")

    # ---- the cheap analytic answer first, because it may end the discussion
    touching = reach_spheres_can_touch(a, b, reach=args.reach)
    print(f"  REACH SPHERES ({args.reach:.2f} m each, so clear beyond "
          f"{2 * args.reach:.2f} m):")
    if not touching:
        print(f"    ✓ CANNOT TOUCH. The bases are {a.distance_to(b):.3f} m apart, more "
              f"than {2 * args.reach:.2f} m,\n"
              f"      so while the reach limit is enforced a collision is geometrically\n"
              f"      impossible and NO new limit is needed.")
        print("    ⛔ EXCEPT IN GUIDE MODE. Hand-guiding is not subject to the reach\n"
              "       limit, because nothing can stop a hand. This clears TELEOP,\n"
              "       MIRROR's follower and playback, not two arms guided by hand.")
    else:
        print(f"    ⚠️ CAN TOUCH. {a.distance_to(b):.3f} m is within "
              f"{2 * args.reach:.2f} m, so the reach limit alone\n"
              f"      does not prevent the two arms occupying the same space.")

    # ---- then the per-pose measurement
    print("\n  CLEARANCE AT SOME REPRESENTATIVE POSES (conservative — see below):")
    geom = ArmGeometry()
    worst = None
    for label, (ja, va, jb, vb) in POSES.items():
        c = closest_approach(pose(ja, va), pose(jb, vb), a, b, geometry=geom)
        mark = "⛔" if c.spheres_overlap else "  "
        print(f"    {mark} {label:32s} {describe(c)}")
        if worst is None or c.distance < worst.distance:
            worst = c

    if worst is not None:
        print(f"\n  WORST OF THOSE: {describe(worst)}")

    print("""
  ⚠️ HOW TO READ THESE NUMBERS.
     Each body is given the bounding-sphere radius the MuJoCo model declares, and
     bounding spheres are LARGER than the parts inside them. So every figure above
     UNDER-reports the real clearance: treat it as "at least this much", never as
     "the gap". For a safety measurement that is the right direction to be wrong.

  ⛔ AND WHAT IT DOES NOT MODEL, all of which make it optimistic:
     the gripper jaws are posed shut whatever they really are; anything the arms are
     HOLDING does not exist, and a two-arm handover is exactly when they are closest;
     the desk, mounts and camera cables are absent; and this is a snapshot, so two arms
     5 cm apart and closing fast read the same as two arms 5 cm apart and stopped.

  ⭐ These five poses are a readable sample, NOT a search over everything the arms can
     reach. A pose worse than all of them almost certainly exists.

  ⛔ NOTHING HERE REFUSES ANY MOVEMENT, on purpose. The margin is Julien's to set
     (docs/FINDINGS.md §58.4 item 4). This exists so the decision is made against
     measurements instead of against a guess.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
