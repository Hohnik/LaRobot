"""Cartesian velocity commands to six arm-joint targets, without device I/O.

reset() seeds the internal model from measured joints at mode entry. step()
integrates a tool-pose target and solves IK from that evolving model; it does
not refresh the model from encoders each cycle. SafeRobot separately limits
commands against measured joints. The caller applies workspace constraints
and commands the gripper outside this solver.

Zero input stops advancing the target; it does not return to a home pose.
Current contracts are below. Historical experiments and replaced explanations
are preserved in docs/archive/ik-and-mode-source-notes.md.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mink
import mujoco
import numpy as np

from yam import REPO_ROOT  # ⛔ anchored ONCE in yam/__init__.py, never per file
I2RT_MODELS = REPO_ROOT / "third_party" / "i2rt" / "i2rt" / "robot_models" / "arm"

# The 4310 gripper variant, matching the measured hardware (README §6.0), and the
# only YAM model that ships an end-effector site for IK to aim at.
DEFAULT_MODEL = I2RT_MODELS / "yam" / "v1" / "yam_linear_4310_d405.xml"
DEFAULT_EE_SITE = "tcp_site"

N_ARM_JOINTS = 6  # joints 1-6; the gripper is commanded separately, not by IK

# Supported input frames. All targets are integrated in world coordinates;
# tool and camera rotate the input using the internal model's orientation.
FRAMES = {
    "world": None,              # base frame: +X out from the base, +Y left, +Z up
    "tool": ("tcp_site", "site"),
    "camera": ("camera", "body"),
}


class CartesianTeleop:
    """Integrate a tool-pose target and solve inverse kinematics (IK).

    Task costs trade position, orientation and a weak reference-posture preference.
    These are solver objectives; they do not guarantee reachability or avoid contact.
    """

    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL,
        ee_site: str = DEFAULT_EE_SITE,
        position_cost: float = 1.0,
        # Prefer translation when orientation cannot be satisfied. The measured tuning
        # comparison is preserved in the source-note archive; keep this default unchanged.
        orientation_cost: float = 0.05,
        posture_cost: float = 1e-2,
        lm_damping: float = 1.0,
        solver: str = "daqp",
        damping: float = 1e-3,
        max_lead_m: float = 0.05,
        max_lead_rad: float = 0.25,
        frame: str = "world",
        max_joint_rate: float = 0.9,
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

        # Request model joint limits inside the solver. Construction failure retains
        # the existing fallback without IK limits; downstream guards remain separate.
        try:
            self.limits: list = [mink.ConfigurationLimit(self.model)]
        except Exception:  # noqa: BLE001
            self.limits = []

        # Bound integrated target lead over the IK model, independently of the
        # encoder-based SafeRobot.max_lag guard.
        self.max_lead_m = max_lead_m
        self.max_lead_rad = max_lead_rad

        # Input frame, converted by _twist_to_world before integration.
        if frame not in FRAMES:
            raise ValueError(f"frame must be one of {sorted(FRAMES)}, got {frame!r}")
        self.frame = frame

        # Request less input when IK demands high joint rates. The default 0.9 rad/s
        # is below SafeRobot's 1.0 command-rate ceiling; neither measures physical speed.
        self.max_joint_rate = max_joint_rate
        self.speed_scale = 1.0
        # Last IK step's requested rate before the next cycle's scale adjustment.
        # Report the value without assigning an unmeasured cause such as full reach.
        self.requested_rate = 0.0

        self.target: mink.SE3 | None = None

    def reset(self, q_arm: np.ndarray) -> None:  # noqa: D401
        """Seed the model, target and reference posture from measured arm joints.

        Call on mode entry so old targets and speed throttling cannot survive hand-guiding.
        """
        q = np.zeros(self.model.nq)
        n = min(len(q_arm), N_ARM_JOINTS)
        q[:n] = np.asarray(q_arm)[:n]
        self.configuration.update(q)
        self.speed_scale = 1.0        # a mode change must not inherit a throttle
        self.requested_rate = 0.0
        self.posture_task.set_target_from_configuration(self.configuration)
        self.target = self.configuration.get_transform_frame_to_world(self.ee_site, "site")

    def step(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """Advance the model and return six joint targets in radians.

        twist is [vx, vy, vz, wx, wy, wz], in m/s and rad/s in self.frame.
        dt is seconds. Convert to world coordinates, integrate the pose target,
        bound its lead over the model, then solve and integrate joint velocities.
        The resulting requested rate adjusts the next cycle's input scale.
        """
        if self.target is None:
            raise RuntimeError("reset() must be called with the arm's measured joint positions first")

        twist = np.asarray(twist, dtype=float)
        twist = self._twist_to_world(twist) * self.speed_scale
        lin, ang = twist[:3] * dt, twist[3:] * dt

        # Input is already world-relative. Pre-multiplication applies that angular
        # increment regardless of the current tool orientation.
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
        q_before = np.array(self.configuration.q[:N_ARM_JOINTS], dtype=float)
        self.configuration.integrate_inplace(vel, dt)
        q_after = np.array(self.configuration.q[:N_ARM_JOINTS], dtype=float)
        self._apply_speed_scale(q_after - q_before, dt)
        return q_after

    def _apply_speed_scale(self, joint_step: np.ndarray, dt: float) -> None:
        """Adjust the next input scale from this step's requested joint rate.

        Reduce immediately above max_joint_rate; recover by 5% per cycle otherwise.
        The scale has a 0.05 minimum. This is neither an encoder measurement nor a
        hard cap on the returned joints; SafeRobot applies downstream command limits.
        """
        if dt <= 0:
            return
        requested = float(np.max(np.abs(joint_step))) / dt
        self.requested_rate = requested
        if requested > self.max_joint_rate:
            self.speed_scale = max(0.05, self.speed_scale * self.max_joint_rate / requested)
        elif self.speed_scale < 1.0:
            self.speed_scale = min(1.0, self.speed_scale * 1.05)

    def _twist_to_world(self, twist: np.ndarray) -> np.ndarray:
        """Rotate linear and angular velocity from the selected frame into world.

        tool uses the model's TCP orientation. camera uses its D405 mounting transform.
        An unmeasured physical camera mount does not establish that transform; use world
        until the intended control frame is verified. Integration remains world-relative.
        """
        spec = FRAMES[self.frame]
        if spec is None:
            return twist
        name, kind = spec
        rot = self.configuration.get_transform_frame_to_world(name, kind).rotation().as_matrix()
        out = np.empty(6)
        out[:3] = rot @ twist[:3]
        out[3:] = rot @ twist[3:]
        return out

    FRAME_NOTES = {
            "world": "WORLD — fixed to the desk; 'forward' does not turn with the wrist",
            "tool": "TOOL — attached to the gripper; 'forward' is where the gripper points",
            "camera": "CAMERA — the MODELLED D405 optical frame (⚠️ wrong for a hand-mounted webcam)",
    }

    def frame_note(self) -> str:
        """One line describing what the puck's directions currently mean."""
        return self.FRAME_NOTES[self.frame]

    def _limit_lead(self) -> None:
        """Bound target translation and rotation relative to the internal model pose.

        This limits integrated goals that IK cannot satisfy. No physical measurement
        enters here, so it cannot diagnose contact or a stalled motor. SafeRobot's
        measured-joint lag guard is independent. Bounds are metres and radians.
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
        """Return target-to-model translation and rotation gaps, in metres and radians.

        These solver diagnostics do not measure physical tracking error or contact.
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


# Configured tip-target bounds in the arm's base frame, in metres.
# The floor stays at the base plane by Julien's decision: raising it prevents
# table-level tasks, lowering it permits targets below the table. The physical
# table height still needs measurement. These limits do not model link collisions.
REACH_LIMIT = 0.60          # m from the base
FLOOR_LIMIT = 0.0           # m — the base plane, which is the desk to within a plate


# When entry starts outside a boundary, allow one default lead-length beyond
# it. Widening exactly to the entry pose caused repeated clamping in prior tests.
LIMIT_WIDEN_MARGIN = 0.05


def effective_limits(home_ee: np.ndarray | None, reach: float,
                     floor: float) -> tuple[float, float]:
    """Include the TELEOP entry position with clearance if it exceeds a limit.

    An outside start widens that boundary by LIMIT_WIDEN_MARGIN beyond the entry
    pose, avoiding an unsolicited inward command. Inside starts keep the configured
    limits. The caller retains home_ee for the session; boundaries do not shrink
    as the arm moves back inward. Units are metres in the arm's base frame.
    """
    if home_ee is None:
        return reach, floor
    start = np.asarray(home_ee, dtype=float)
    dist, height = float(np.linalg.norm(start)), float(start[2])
    # ⚠️ The margin applies only when the limit actually has to open. An arm starting
    # inside keeps the configured limits exactly, which is every normal session.
    out = reach if dist <= reach else dist + LIMIT_WIDEN_MARGIN
    low = floor if height >= floor else height - LIMIT_WIDEN_MARGIN
    return out, low


def clamp_to_workspace(ee: np.ndarray, reach: float, floor: float) -> np.ndarray:
    """Project the target onto the reach sphere, then raise it to the floor.

    Use effective_limits first to account for an outside entry pose. This only
    constrains the model's tip target; it provides no collision or speed guarantee.
    """
    out = np.asarray(ee, dtype=float).copy()

    dist = float(np.linalg.norm(out))
    if dist > reach and dist > 0.0:
        out *= reach / dist

    if out[2] < floor:
        out[2] = floor

    return out


def workspace_room(ee: np.ndarray, reach: float, floor: float) -> tuple[float, float]:
    """Return base-to-tip distance and height above the configured floor, in metres."""
    ee = np.asarray(ee, dtype=float)
    return float(np.linalg.norm(ee)), float(ee[2] - floor)


def scripted_twist(t: float, speed: float = 0.04) -> np.ndarray:
    """Return a slow horizontal circular velocity for IK checks without a puck."""
    return np.array([speed * np.cos(t * 0.8), speed * np.sin(t * 0.8), 0.0, 0.0, 0.0, 0.0])
