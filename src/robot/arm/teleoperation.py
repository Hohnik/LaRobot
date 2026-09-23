from dataclasses import dataclass

import mujoco
import numpy as np

from robot.environment.simulation import Simulation
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget


@dataclass
class ArmState:
    """Store one arm's kinematics, target, and gripper controls.

    Parameters
    ----------
    kin : CartesianKinematics
        Arm kinematics.
    target : CartesianTarget
        Mutable target pose.
    marker_mocap_id : int
        Target marker index in the simulation's mocap arrays.
    gripper : float
        Current gripper command.
    close_button : int, optional
        Close-button index; defaults to 0.
    open_button : int, optional
        Open-button index; defaults to 1.
    """

    kin: CartesianKinematics
    target: CartesianTarget
    marker_mocap_id: int
    gripper: float
    close_button: int = 0
    open_button: int = 1


def update_arm(
    sim: Simulation,
    arm: ArmState,
    velocities: np.ndarray,
    buttons: np.ndarray,
    *,
    gripper_open: float = 0.0495,
    gripper_shut: float = 0.0,
    gripper_step: float = 0.005,
    lag_limit: float = 0.03,
) -> np.ndarray:
    """Update the arm target, gripper, and marker for one control tick.

    Parameters
    ----------
    sim : Simulation
        Simulation providing measured joints and the marker to update.
    arm : ArmState
        Arm state to update in place.
    velocities : ndarray, shape (6,)
        Linear xyz and angular xyz rates passed to the target integrator.
    buttons : array_like
        Button states indexed by the arm's button mappings.
    gripper_open : float, optional
        Upper gripper command bound; defaults to 0.0495.
    gripper_shut : float, optional
        Lower gripper command bound; defaults to 0.0.
    gripper_step : float, optional
        Gripper command change per tick; defaults to 0.005.
    lag_limit : float, optional
        Maximum position lag in model length units; defaults to 0.03.

    Returns
    -------
    Six joint commands followed by the gripper command.
    ```
    ndarray, shape (7,)
    ```
    """
    measured_joints = sim.data.qpos[arm.kin.qpos_indices]

    arm.target.integrate(velocities)

    # Keep the target within lag_limit of the measured position.
    measured_position = arm.kin.forward(measured_joints)[:3, 3]
    delta = arm.target.position - measured_position
    distance = np.linalg.norm(delta)

    if distance > lag_limit:
        direction = delta / distance
        arm.target.position = measured_position + direction * lag_limit

    joints = arm.kin.inverse(
        measured_joints,
        target_position=arm.target.position,
        target_rotation=arm.target.rotation,
    )

    if buttons[arm.close_button]:
        arm.gripper = max(gripper_shut, arm.gripper - gripper_step)
    elif buttons[arm.open_button]:
        arm.gripper = min(gripper_open, arm.gripper + gripper_step)

    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, arm.target.rotation.ravel())
    sim.data.mocap_pos[arm.marker_mocap_id] = arm.target.position
    sim.data.mocap_quat[arm.marker_mocap_id] = quat

    return np.concatenate((joints, [arm.gripper]))
