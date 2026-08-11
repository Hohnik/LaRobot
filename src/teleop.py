"""Cartesian teleoperation: a 6-DoF twist in, joint targets out.

This is the piece `docs/Setup-Plan.md` §4.2 says we have to build ourselves, and
the single largest deviation from the papers. ABC/ENPIRE/RoboTTT all teleoperate
with GELLO leader arms, which are passive copies of the robot and therefore hand
over **joint angles** directly. A SpaceMouse hands over a **cartesian twist** of
the end effector instead, and there is no fixed mapping between the two — it
depends on the arm's current configuration. Computing it is inverse kinematics:

    twist  →  integrate into a target EE pose  →  IK  →  joint targets  →  arm

⭐ Deliberately robot-agnostic. It talks to anything exposing I2RT's `Robot`
interface, which `get_yam_robot(sim=True)` and `get_yam_robot(sim=False)` both
do. The same code therefore drives the simulated arm and the real one, and
moving between them is a flag rather than a rewrite — so a sign error costs
nothing the first time it happens.

WHY THE TARGET POSE IS INTEGRATED, NOT COMMANDED DIRECTLY
---------------------------------------------------------
A SpaceMouse reports *deflection*, i.e. a velocity command, not a position.
Releasing it returns to zero, which must mean "stop", not "go home". So the
target pose is state that we integrate the twist into: push and the target
drifts, release and it stays where it was. This also makes a deadman trivial —
zero deflection integrates nothing.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mink
import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
I2RT_MODELS = REPO_ROOT / "third_party" / "i2rt" / "i2rt" / "robot_models" / "arm"

# The 4310 gripper variant, matching the measured hardware (README §6.0), and the
# only YAM model that ships an end-effector site for IK to aim at.
DEFAULT_MODEL = I2RT_MODELS / "yam" / "v1" / "yam_linear_4310_d405.xml"
DEFAULT_EE_SITE = "tcp_site"

N_ARM_JOINTS = 6  # joints 1-6; the gripper is commanded separately, not by IK


class CartesianTeleop:
    """Integrates a twist into an EE pose and solves IK for joint targets.

    Costs are the tuning surface. `position_cost` and `orientation_cost` trade
    translation accuracy against rotation accuracy; `posture_cost` is a weak pull
    toward a reference configuration that resolves the redundancy and, more
    importantly, keeps the solver away from singular configurations where joint
    velocities blow up.
    """

    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL,
        ee_site: str = DEFAULT_EE_SITE,
        position_cost: float = 1.0,
        # ⛔⭐ 0.05, NOT 0.5. Lowered 10x on 2026-08-11 after measuring, and the
        # measurement is counter-intuitive enough to be worth keeping:
        #
        #   pos:ori    pure-roll tool wander    rotation achieved
        #   1.0:0.5          0.443 m                  7.9 deg     <- the old default
        #   1.0:0.2          0.034 m                129.5 deg
        #   1.0:0.05         0.002 m                134.6 deg     <- now
        #   1.0:0.01         0.000 m                 18.2 deg
        #
        # The old default was the WORST OF BOTH: it wandered 44 cm *and* achieved
        # the least rotation. A higher orientation cost produced LESS rotation,
        # because the effort went into satisfying an unreachable orientation by
        # translating, which drags the arm into a configuration that can rotate
        # even less. Verified at three different starting poses; small rotations
        # are unaffected and translation reach is unchanged (0.319 -> 0.320 m).
        #
        # The priority this encodes, in one line: **never sacrifice where the tool
        # IS to chase where it POINTS.** A wrist that cannot turn should simply not
        # turn — it should not drag the whole arm across the desk.
        orientation_cost: float = 0.05,
        posture_cost: float = 1e-2,
        lm_damping: float = 1.0,
        solver: str = "daqp",
        damping: float = 1e-3,
        max_lead_m: float = 0.05,
        max_lead_rad: float = 0.25,
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.ee_site = ee_site
        self.solver = solver
        self.damping = damping

        self.configuration = mink.Configuration(self.model)
        self.ee_task = mink.FrameTask(
            frame_name=ee_site,
            frame_type="site",
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping,
        )
        self.posture_task = mink.PostureTask(self.model, cost=posture_cost)
        self.tasks = [self.ee_task, self.posture_task]

        # Joint limits enforced inside the QP, so the solver never proposes a
        # velocity that would drive a joint past its stop. Cheaper and safer than
        # clamping afterwards, which would silently distort the requested motion.
        try:
            self.limits: list = [mink.ConfigurationLimit(self.model)]
        except Exception:  # noqa: BLE001
            self.limits = []

        # ⛔⭐ ANTI-WINDUP. How far the integrated goal may run ahead of the pose the
        # arm has actually achieved. This is `SafeRobot.max_lag` one layer up, and it
        # exists for a measured reason — see `step()`.
        self.max_lead_m = max_lead_m
        self.max_lead_rad = max_lead_rad

        self.target: mink.SE3 | None = None

    def reset(self, q_arm: np.ndarray) -> None:
        """Seed the IK state from the arm's *measured* joint positions.

        Always seed from reality rather than from zero: the IK's notion of where
        the arm is has to match where it actually is, or the very first solve
        commands a jump.
        """
        q = np.zeros(self.model.nq)
        n = min(len(q_arm), N_ARM_JOINTS)
        q[:n] = np.asarray(q_arm)[:n]
        self.configuration.update(q)
        self.posture_task.set_target_from_configuration(self.configuration)
        self.target = self.configuration.get_transform_frame_to_world(self.ee_site, "site")

    def step(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """Advance by one control cycle. Returns joint targets for joints 1-6.

        `twist` is [vx, vy, vz, wx, wy, wz] — linear m/s and angular rad/s,
        expressed in the world frame.
        """
        if self.target is None:
            raise RuntimeError("reset() must be called with the arm's measured joint positions first")

        twist = np.asarray(twist, dtype=float)
        lin, ang = twist[:3] * dt, twist[3:] * dt

        # World-frame integration: rotation pre-multiplies, so a twist means the
        # same thing regardless of how the gripper happens to be oriented. Body
        # frame would be more natural to a hand holding the puck, and is a
        # deliberate later choice — not something to leave ambiguous now.
        self.target = mink.SE3.from_rotation_and_translation(
            rotation=mink.SO3.exp(ang).multiply(self.target.rotation()),
            translation=self.target.translation() + lin,
        )
        self._limit_lead()
        self.ee_task.set_target(self.target)

        vel = mink.solve_ik(
            self.configuration,
            self.tasks,
            dt,
            self.solver,
            damping=self.damping,
            limits=self.limits,
        )
        self.configuration.integrate_inplace(vel, dt)
        return np.array(self.configuration.q[:N_ARM_JOINTS], dtype=float)

    def _limit_lead(self) -> None:
        """Stop the integrated goal running away from the pose actually achieved.

        ⛔⭐ THE BUG THIS FIXES — Julien, 2026-08-11: *"the inverse kinematics being
        weird and not working as intended, specifically when the robot gets into
        weird positions, and then it starts moving very, very incoherently."*

        `step()` advances `self.target` by the twist **unconditionally**. It never
        asks whether the arm followed. Measured in simulation, commanding pure roll
        at 0.6 rad/s from the park pose:

            t=4 s   target-vs-achieved gap 0.0004 m   tool point moved 0.000 m
            t=8 s   target-vs-achieved gap 0.238  m   tool point moved 0.238 m
            t=12 s                                    tool point moved 0.290 m

        **A pure rotation command moved the tool point 41 cm.** The chain is:

        1. A wrist joint hits its limit — the tight ones are ±1.5708. (Confirmed:
           the IK-vs-command gap pins at exactly 0.0800 rad, which *is*
           `JOINT_LIMIT_MARGIN`, so a joint is clamped at the margin while the IK
           believes it is at the true limit.)
        2. The orientation goal keeps integrating anyway, so it runs arbitrarily
           far past anything reachable.
        3. The QP now holds an impossible orientation target, and because
           `position_cost` (1.0) and `orientation_cost` (0.5) are traded against
           each other, it starts **moving the tool point to partially satisfy the
           unreachable rotation**.
        4. The workspace box then re-clamps translation, which fights the
           orientation task — hence the oscillation in the measurement above.

        So the arm does something the operator never asked for, in a direction that
        has no relation to the puck. That is exactly "incoherent".

        ⭐ The fix is the same idea as `SafeRobot.max_lag`, one layer up: a goal may
        only lead reality by a bounded amount. Translation and rotation are limited
        separately because they fail independently — the workspace box already
        happened to bound translation, which is why only rotation misbehaved.

        ⚠️ This deliberately does NOT try to detect singularities or predict
        reachability. It does not need to: whatever the reason the arm cannot
        follow — joint limit, singularity, rate limiter, a stalled motor — the
        symptom is the same, an unclosable gap, and bounding the gap bounds every
        one of those cases with no model of why.
        """
        if self.target is None:
            return
        achieved = self.configuration.get_transform_frame_to_world(self.ee_site, "site")

        lin = self.target.translation() - achieved.translation()
        dist = float(np.linalg.norm(lin))
        if dist > self.max_lead_m:
            lin = lin * (self.max_lead_m / dist)

        # Rotational lead, as an axis-angle in the world frame.
        d_rot = self.target.rotation().multiply(achieved.rotation().inverse())
        log = d_rot.log()
        angle = float(np.linalg.norm(log))
        if angle > self.max_lead_rad:
            d_rot = mink.SO3.exp(log * (self.max_lead_rad / angle))

        self.target = mink.SE3.from_rotation_and_translation(
            rotation=d_rot.multiply(achieved.rotation()),
            translation=achieved.translation() + lin,
        )

    def lead(self) -> tuple[float, float]:
        """How far the goal is currently ahead of the achieved pose: (metres, rad).

        Exposed so the session can show it. A goal that sits pinned at the limit is
        the signature of an arm that cannot follow, and that is worth seeing rather
        than inferring from the arm behaving oddly.
        """
        if self.target is None:
            return 0.0, 0.0
        achieved = self.configuration.get_transform_frame_to_world(self.ee_site, "site")
        d = float(np.linalg.norm(self.target.translation() - achieved.translation()))
        a = float(np.linalg.norm(self.target.rotation().multiply(achieved.rotation().inverse()).log()))
        return d, a

    def ee_position(self) -> np.ndarray:
        """Where the IK believes the end effector currently is."""
        return self.configuration.get_transform_frame_to_world(self.ee_site, "site").translation()


def scripted_twist(t: float, speed: float = 0.04) -> np.ndarray:
    """A slow horizontal circle, for validating IK with no input device involved.

    Debugging IK and debugging a HID decoder at the same time is how you end up
    unable to tell which one is wrong.
    """
    return np.array([speed * np.cos(t * 0.8), speed * np.sin(t * 0.8), 0.0, 0.0, 0.0, 0.0])
