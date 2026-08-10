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
        orientation_cost: float = 0.5,
        posture_cost: float = 1e-2,
        lm_damping: float = 1.0,
        solver: str = "daqp",
        damping: float = 1e-3,
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

    def ee_position(self) -> np.ndarray:
        """Where the IK believes the end effector currently is."""
        return self.configuration.get_transform_frame_to_world(self.ee_site, "site").translation()


def scripted_twist(t: float, speed: float = 0.04) -> np.ndarray:
    """A slow horizontal circle, for validating IK with no input device involved.

    Debugging IK and debugging a HID decoder at the same time is how you end up
    unable to tell which one is wrong.
    """
    return np.array([speed * np.cos(t * 0.8), speed * np.sin(t * 0.8), 0.0, 0.0, 0.0, 0.0])
