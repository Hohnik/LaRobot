import argparse
import logging
import socket
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.utils import GripperType

from robot import CONTROL_HZ
from robot.arm.physical_teleoperation import ArmState, update_arm
from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 1.0, 0.0
GRIPPER_STEP = 0.05
LAG_LIMIT = 0.07
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.2, 1.5
DT = 1 / CONTROL_HZ
Side = Literal["left", "right"]
logger = logging.getLogger(__name__)


def shutdown_robots(robots: dict[Side, MotorChainRobot]) -> None:
    """Idle all connected arms, then close every arm after manual support."""
    if not robots:
        return

    # ExitStack attempts every close even if another close or the prompt fails.
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


def check_devices(
    args: argparse.Namespace, sides: tuple[Side, ...], channels: dict[str, str]
) -> list[str]:
    """Check interface names and input count before activating any motors."""
    if args.device == "keyboard":
        raise NotImplementedError("Keyboard input is not implemented yet")

    if args.dual and args.left_channel == args.right_channel:
        raise ValueError("Each arm must have its own CAN interface")
    for side in sides:
        try:
            socket.if_nametoindex(channels[side])
        except OSError as exc:
            raise ConnectionError(
                f"CAN interface {channels[side]!r} for the {side} arm does not exist"
            ) from exc

    paths = SpaceMouse.connected_paths()
    if len(paths) < len(sides):
        raise ConnectionError(f"Need {len(sides)} SpaceMice, but found {len(paths)}")
    return paths


def main(args: argparse.Namespace) -> None:
    """Run physical arm teleoperation using one or two SpaceMice.

    CAN interfaces must already be configured and brought up.
    """
    sides: tuple[Side, ...] = ("left", "right") if args.dual else ("left",)
    channels = {"left": args.left_channel, "right": args.right_channel}
    paths = check_devices(args, sides, channels)
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    with ExitStack() as stack:
        arms: dict[Side, ArmState] = {}
        devices: dict[Side, SpaceMouse] = {}
        robots: dict[Side, MotorChainRobot] = {}

        # Open every input before activating either arm.
        for device_index, side in enumerate(sides):
            devices[side] = stack.enter_context(
                SpaceMouse(
                    device_path=paths[device_index],
                    side=side,
                    expo=EXPO,
                    lin_scale=LIN_SCALE,
                    ang_scale=ANG_SCALE,
                )
            )

        # Register cleanup before connecting, including partial initialization.
        stack.callback(shutdown_robots, robots)
        for side in sides:
            print(f"Initializing {side} arm on {channels[side]}", flush=True)
            robots[side] = get_yam_robot(
                channel=channels[side],
                gripper_type=GripperType.LINEAR_4310,
            )

        # read starting poses
        for side in sides:
            measured = robots[side].get_joint_pos()
            kin = CartesianKinematics(model, side=side)
            pose = kin.forward(measured[:6])
            mirrored_buttons = args.dual and side == "left"
            arms[side] = ArmState(
                kin=kin,
                target=CartesianTarget.from_pose(pose=pose),
                gripper=float(np.clip(measured[6], GRIPPER_SHUT, GRIPPER_OPEN)),
                close_button=1 if mirrored_buttons else 0,
                open_button=0 if mirrored_buttons else 1,
            )

        print("Teleoperation running. Press Ctrl+C to stop.", flush=True)
        next_tick = time.perf_counter()
        while True:
            commands = {}
            for side, arm in arms.items():
                velocities, buttons = devices[side].read()
                commands[side] = update_arm(
                    arm,
                    robots[side].get_joint_pos()[:6],
                    velocities,
                    buttons,
                    dt=DT,
                    gripper_open=GRIPPER_OPEN,
                    gripper_shut=GRIPPER_SHUT,
                    gripper_step=GRIPPER_STEP,
                    lag_limit=LAG_LIMIT,
                )

            for side, command in commands.items():
                robots[side].command_joint_pos(command)

            next_tick += DT
            now = time.perf_counter()
            if next_tick <= now:
                next_tick = now + DT
            time.sleep(next_tick - now)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Teleoperate physical YAM arms")
    parser.add_argument(
        "--device",
        "-d",
        choices=["spacemouse", "keyboard"],
        required=True,
        help="Input device to use (keyboard is not implemented yet)",
    )
    parser.add_argument(
        "--dual", action="store_true", help="Control both arms using two SpaceMice"
    )
    parser.add_argument(
        "--left-channel", default="can-left", help="Left arm CAN interface"
    )
    parser.add_argument(
        "--right-channel", default="can-right", help="Right arm CAN interface"
    )
    try:
        main(parser.parse_args())
    except KeyboardInterrupt:
        pass
