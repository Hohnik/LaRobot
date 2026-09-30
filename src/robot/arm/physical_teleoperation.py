from dataclasses import dataclass

import numpy as np

from robot import CONTROL_HZ
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget


@dataclass
class ArmState:
    """Store a physical arm's target and normalized gripper command."""

    kin: CartesianKinematics
    target: CartesianTarget
    gripper: float
    close_button: int = 0
    open_button: int = 1


def update_arm(
    arm: ArmState,
    measured_joints: np.ndarray,
    velocities: np.ndarray,
    buttons: list[int],
    *,
    dt: float = 1 / CONTROL_HZ,
    gripper_open: float = 1.0,
    gripper_shut: float = 0.0,
    gripper_step: float = 0.05,
    lag_limit: float = 0.07,
) -> np.ndarray:
    """Compute six joint targets and a gripper target from measured arm joints.

    Hardware gripper commands are normalized to [0, 1], unlike simulation
    gripper positions in metres. Keep the commanded value between ticks so
    releasing the button holds the target even while the gripper is moving.
    """
    arm.target.integrate(velocities, dt=dt)

    measured_position = arm.kin.forward(measured_joints)[:3, 3]
    delta = arm.target.position - measured_position
    distance = np.linalg.norm(delta)
    if distance > lag_limit:
        arm.target.position = measured_position + delta / distance * lag_limit

    joints = arm.kin.inverse(
        measured_joints,
        target_position=arm.target.position,
        target_rotation=arm.target.rotation,
        dt=dt,
    )

    if buttons[arm.close_button]:
        arm.gripper -= gripper_step
    elif buttons[arm.open_button]:
        arm.gripper += gripper_step
    arm.gripper = float(np.clip(arm.gripper, gripper_shut, gripper_open))

    return np.concatenate((joints, [arm.gripper]))
