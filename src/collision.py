"""How close are the two arms to each other? The MEASUREMENT only — it refuses nothing.

⭐⭐ WHY THIS EXISTS. docs/ROADMAP.md §8.2 item 25, and docs/HANDOFF.md's entry block
states the problem in one line: **nothing knows where the other arm is.** Every limit in
this project — the 0.60 m reach sphere, the floor, the workspace refusals — is computed
per arm and relative to **that arm's own base**, and no code anywhere knows a second arm
exists. MIRROR mode is what made this matter, because it is the first mode where an arm
moves with **nobody's hand on it**, so the operator is currently the only thing standing
between the two arms reaching into the same space.

⛔⭐⭐ THIS MODULE DELIBERATELY DOES NOT REFUSE ANYTHING, AND THAT IS NOT TIMIDITY.
docs/FINDINGS.md §58.4 item 4 ruled it: *"Needs Julien's decision on the margin, so build
the measurement first and report it before refusing anything."* A margin invented here
would become a limit he never chose, and this project already has one of those in its
history — the ±0.30 m cube that quietly stopped him at 71% of the arm's reach for days
(docs/FINDINGS.md §41.1). **Measure, report, let him set the number.**

⛔⭐⭐⭐ AND ONE PHYSICAL NUMBER IS MISSING, WITHOUT WHICH NONE OF THIS CAN RUN.
**Nothing in the repo records where the two bases are relative to each other.** It is not
in any document, any config file, or any model. It cannot be derived, computed, or
guessed from anything the software can see: it is a tape-measure reading. So
`BasePose` has **no default** and every function here requires one.

⭐⭐ THE GOOD NEWS, AND IT MAY MAKE THE WHOLE PROBLEM DISAPPEAR. Each arm is already
confined to a sphere of `REACH_LIMIT` (0.60 m) around its own base by a limit Julien
himself chose. **Two spheres of radius 0.60 m cannot intersect if their centres are more
than 1.20 m apart.** So if the bases are further apart than that, the existing reach
limit already makes a collision impossible and no new limit is needed at all —
`reach_spheres_can_touch()` answers that from one measurement. ⚠️ If they are closer, the
per-pose measurement below is what tells him how much room there really is.

⭐ HOW THE PER-POSE DISTANCE IS COMPUTED, and why it errs the safe way. Each arm's link
positions come from the same MuJoCo model and the same `mink.Configuration` the IK
already uses, so the geometry is the shipped model rather than a hand-written skeleton.
Each body is then given the **bounding-sphere radius the model itself declares**
(`geom_rbound`), and the distance between two bodies is the distance between their
origins minus both radii.

⚠️ **Bounding spheres are bigger than the parts they contain**, so this **under-reports
clearance**: it will say the arms are closer than they physically are, never further. For
a safety measurement that is the correct direction to be wrong in. ⛔ It is therefore not
the number to quote as "the gap", only as "at least this much".

⛔ FOUR THINGS IT DOES NOT MODEL, all of which make it optimistic in one specific way:

1. ⛔ **The jaws are not posed.** The IK model's gripper joint is left at zero, exactly as
   `src/teleop.py` does, so an open gripper's tips are in the wrong place by a few cm.
2. ⛔ **Nothing the arms are HOLDING exists.** A grasped object can be much larger than
   the gripper, and a two-arm handover is precisely when they are closest.
3. ⛔ **The desk, the cameras' cables and the mounts are absent.**
4. ⛔ **It is a snapshot, not a prediction.** Two arms 5 cm apart and closing fast is a
   very different situation from two arms 5 cm apart and stationary, and this returns the
   same number for both.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np

#: ⭐ The reach limit each arm is already held to, read from the same constant the
#: session enforces so the two can never drift apart.
from teleop import DEFAULT_MODEL, N_ARM_JOINTS, REACH_LIMIT  # noqa: E402


@dataclass(frozen=True)
class BasePose:
    """Where one arm's base sits in a shared room frame.

    ⛔⭐ THERE IS NO DEFAULT ON PURPOSE. This is a tape-measure reading and nothing in the
    repo records it. A plausible-looking default here would be a number nobody measured
    silently deciding whether two 4.3 kg arms are allowed to occupy the same space.

    ⭐ The simple case is the one to use: both arms bolted to the same desk, both upright,
    so `z = 0` for both and the only real measurements are how far apart they are and
    which way each one faces.

    `x`, `y`, `z` are metres. `yaw_deg` rotates the arm about the vertical axis, so two
    arms facing each other across a desk differ by 180°.
    """

    x: float
    y: float
    z: float = 0.0
    yaw_deg: float = 0.0

    def to_world(self, points: np.ndarray) -> np.ndarray:
        """Base-frame points -> room frame."""
        a = math.radians(self.yaw_deg)
        rot = np.array([[math.cos(a), -math.sin(a), 0.0],
                        [math.sin(a), math.cos(a), 0.0],
                        [0.0, 0.0, 1.0]])
        return points @ rot.T + np.array([self.x, self.y, self.z])

    def distance_to(self, other: BasePose) -> float:
        return float(np.linalg.norm(
            np.array([self.x, self.y, self.z]) - np.array([other.x, other.y, other.z])))


@dataclass
class Closest:
    """The answer, with enough detail to be reported rather than just obeyed."""

    #: Metres between the two arms' nearest surfaces, by the conservative estimate.
    #: ⚠️ May be NEGATIVE, which means the bounding spheres overlap — a warning that they
    #: are very close, not proof that metal is touching metal.
    distance: float
    #: Which body of each arm was closest, so a message can name the part.
    body_a: str
    body_b: str
    #: Both arms' base separation, carried along because every reading needs it.
    base_separation: float

    @property
    def spheres_overlap(self) -> bool:
        return self.distance < 0.0


@lru_cache(maxsize=4)
def _model(path: str) -> tuple[Any, list[str], np.ndarray]:
    """`(model, body_names, body_radius)`. Cached: loading the XML costs milliseconds and
    a 100 Hz loop must not pay it twice a cycle."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(path)
    names, radii = [], []
    for i in range(model.nbody):
        names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or f"body{i}")
        start = model.body_geomadr[i]
        count = model.body_geomnum[i]
        # ⭐ `geom_rbound` is the model's OWN bounding-sphere radius for each geom, so the
        # size comes from the shipped mesh rather than from a number I chose. A body with
        # no geometry (`world`, and some mounting frames) gets 0.0 and so contributes
        # only its origin.
        radii.append(float(max((model.geom_rbound[g]
                                for g in range(start, start + count)), default=0.0)))
    return model, names, np.array(radii)


class ArmGeometry:
    """One arm's link positions, from the same model and solver the IK already uses."""

    def __init__(self, model_path: Path = DEFAULT_MODEL) -> None:
        import mink

        self.model, self.body_names, self.body_radius = _model(str(model_path))
        self.configuration = mink.Configuration(self.model)

    def link_points(self, q_arm: Any) -> np.ndarray:
        """World positions of every body, for measured joint angles, in the BASE frame.

        ⚠️ Only the first `N_ARM_JOINTS` (6) are used and the gripper is left at zero,
        which is exactly what `src/teleop.py` does. So the jaw tips are posed as if the
        gripper were closed however open it really is.
        """
        q = np.zeros(self.model.nq)
        q_arm = np.asarray(q_arm, dtype=float)
        n = min(len(q_arm), N_ARM_JOINTS)
        q[:n] = q_arm[:n]
        self.configuration.update(q)
        return np.array(self.configuration.data.xpos, dtype=float).copy()


def closest_approach(
    q_a: Any,
    q_b: Any,
    pose_a: BasePose,
    pose_b: BasePose,
    *,
    geometry: ArmGeometry | None = None,
) -> Closest:
    """The conservative minimum distance between two posed arms. Refuses nothing.

    ⚠️ `pose_a` and `pose_b` are required. See `BasePose`: nothing in the repo knows where
    the bases are, and inventing it is the one thing this module must not do.
    """
    geom = geometry or ArmGeometry()
    pts_a = pose_a.to_world(geom.link_points(q_a))
    pts_b = pose_b.to_world(geom.link_points(q_b))
    radius = geom.body_radius

    # Every body against every body. 15 x 15 is 225 pairs, which at 100 Hz is nothing,
    # and being exhaustive avoids a "we only check the gripper" blind spot.
    deltas = pts_a[:, None, :] - pts_b[None, :, :]
    centre_gap = np.linalg.norm(deltas, axis=2)
    surface_gap = centre_gap - radius[:, None] - radius[None, :]

    i, j = np.unravel_index(int(np.argmin(surface_gap)), surface_gap.shape)
    return Closest(
        distance=float(surface_gap[i, j]),
        body_a=geom.body_names[i],
        body_b=geom.body_names[j],
        base_separation=pose_a.distance_to(pose_b),
    )


def reach_spheres_can_touch(pose_a: BasePose, pose_b: BasePose,
                            reach: float = REACH_LIMIT) -> bool:
    """⭐⭐ THE CHEAP ANSWER, AND IT MAY CLOSE THE WHOLE QUESTION.

    Each arm is already confined to a sphere of `reach` metres about its own base by a
    limit Julien chose. Two such spheres cannot intersect when their centres are more
    than `2 · reach` apart. **If this returns False, the existing reach limit already
    makes a collision geometrically impossible and no new limit is needed.**

    ⚠️ It assumes the reach limit is actually in force, which it is in TELEOP and
    playback. ⛔ **GUIDE mode is hand-guiding and the reach limit cannot stop a hand**, so
    a False here does not license two arms being hand-guided into each other.
    """
    return pose_a.distance_to(pose_b) <= 2.0 * reach


def describe(closest: Closest) -> str:
    """One line for a status row or a report. ⚠️ States the estimate's direction of error,
    because a bare number invites being read as the true gap."""
    if closest.spheres_overlap:
        return (f"⛔ {closest.body_a} and {closest.body_b} OVERLAP by "
                f"{-closest.distance * 100:.1f} cm of bounding sphere (bases "
                f"{closest.base_separation:.2f} m apart) — conservative, so check by eye")
    return (f"{closest.distance * 100:.1f} cm or more between {closest.body_a} and "
            f"{closest.body_b} (bases {closest.base_separation:.2f} m apart)")
