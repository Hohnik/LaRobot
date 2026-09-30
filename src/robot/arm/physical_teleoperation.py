import logging
import socket
from contextlib import ExitStack
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import numpy as np

from robot import CONTROL_HZ
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

if TYPE_CHECKING:
    from i2rt.robots.motor_chain_robot import MotorChainRobot

Side = Literal["left", "right"]
logger = logging.getLogger(__name__)


def check_devices(channels: tuple[str, ...], paths: list[str]) -> None:
    """Check Linux SocketCAN interface names and input count before activation.

    Interface existence does not guarantee the bus is up or configured with
    the correct bitrate.

    Parameters
    ----------
    channels : tuple[str, ...]
        CAN interface names for the selected arms.
    paths : list[str]
        Discovered SpaceMouse device paths.

    Raises
    ------
    ValueError
        Two arms use the same interface.
    ConnectionError
        An interface is missing or there are too few SpaceMice.
    """
    if len(set(channels)) != len(channels):
        raise ValueError("Each arm must have its own CAN interface")
    if len(paths) < len(channels):
        raise ConnectionError(f"Need {len(channels)} SpaceMice, but found {len(paths)}")
    for channel in channels:
        try:
            socket.if_nametoindex(channel)
        except OSError as exc:
            raise ConnectionError(f"CAN interface {channel!r} does not exist") from exc


def shutdown_robots(robots: dict[Side, "MotorChainRobot"]) -> None:
    """Enter gravity compensation, prompt for support, then close the arms.

    Closing is attempted for every arm even if the prompt or another close
    fails. Idle failures are logged.

    Parameters
    ----------
    robots : dict[Side, MotorChainRobot]
        Connected arms keyed by side.
    """
    if not robots:
        return

    with ExitStack() as stack:
        for robot in robots.values():
            stack.callback(robot.close)
        for side, robot in robots.items():
            try:
                robot.enter_gravity_comp_idle()
            except Exception:
                logger.exception("Could not idle the %s arm", side)
        input(
            "Teleoperation stopped. Support/place the arms safely, then press "
            "Enter to disable motor torque."
        )


@dataclass
class ArmState:
    """Store a physical arm's target and gripper controls.

    Parameters
    ----------
    kin : CartesianKinematics
        Arm kinematics.
    target : CartesianTarget
        Mutable Cartesian target pose.
    gripper : float
        Gripper command from 0 (closed) to 1 (open).
    close_button : int, optional
        Close-button index; defaults to 0.
    open_button : int, optional
        Open-button index; defaults to 1.
    """

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
    """Update the target and compute one physical control tick.

    Parameters
    ----------
    arm : ArmState
        Target and gripper command to update in place.
    measured_joints : ndarray, shape (6,)
        Measured arm joint positions in radians, excluding the gripper.
    velocities : ndarray, shape (6,)
        Linear xyz (m/s) and angular xyz (rad/s) in the model's world frame.
    buttons : list[int]
        Button states indexed by the arm's button mappings.
    dt : float, optional
        Control timestep in seconds; defaults to 1 / CONTROL_HZ.
    gripper_open : float, optional
        Upper normalized gripper bound; defaults to 1.0.
    gripper_shut : float, optional
        Lower normalized gripper bound; defaults to 0.0.
    gripper_step : float, optional
        Gripper command change per tick; defaults to 0.05.
    lag_limit : float, optional
        Maximum target position lag in metres; defaults to 0.07.

    Returns
    -------
    Six joint targets in radians followed by the normalized gripper command.
    The gripper target is retained when neither button is pressed.
    ```
    ndarray, shape (7,)
    ```
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
