# TODO: look for a better way to implement this
import argparse
import time
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np
import viser
from i2rt.robots.get_robot import MotorChainRobot, get_yam_robot
from i2rt.robots.utils import GripperType
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation
from robot.inputs.input import Input
from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 1, 0.0
GRIPPER_STEP = 0.05
LAG_LIMIT = 0.07
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.2, 1.5
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)
DT = 1 / 30

model = mujoco.MjModel.from_xml_path(str(SCENE))


@dataclass
class ArmState:
    robot: MotorChainRobot
    kin: CartesianKinematics
    target: CartesianTarget
    device: Input
    close_button: int = 0
    open_button: int = 1
    gripper: float = GRIPPER_OPEN


def update_arm(arm: ArmState):
    q_measured = arm.robot.get_joint_pos()
    q = q_measured[:6]
    arm.gripper = q_measured[6]

    # spacemouse - where to
    velocities, buttons = arm.device.read()
    arm.target.integrate(velocities, dt=DT)

    kin_position = arm.kin.forward(q)[:3, 3]

    delta = arm.target.position - kin_position
    distance = np.linalg.norm(delta)

    if distance > LAG_LIMIT:
        arm.target.position = kin_position + delta / distance * LAG_LIMIT

    # update joints
    joints = arm.kin.inverse(
        q,
        target_position=arm.target.position,
        target_rotation=arm.target.rotation,
        dt=DT,
    )

    # gripper
    if buttons[arm.close_button]:
        arm.gripper = max(GRIPPER_SHUT, arm.gripper - GRIPPER_STEP)
    elif buttons[arm.open_button]:
        arm.gripper = min(GRIPPER_OPEN, arm.gripper + GRIPPER_STEP)

    return np.append(joints, arm.gripper)


def main(args: argparse.Namespace) -> None:
    if args.device == "keyboard":
        raise NotImplementedError("Keyboard input is not implemented yet")

    sides = ("left", "right") if args.dual else ("left",)

    with ExitStack() as stack:
        arms = {}
        paths = SpaceMouse.connected_paths()

        for device_index, side in enumerate(sides):
            device = stack.enter_context(
                SpaceMouse(
                    device_path=paths[device_index],
                    expo=EXPO,
                    lin_scale=LIN_SCALE,
                    ang_scale=ANG_SCALE,
                    side=side,
                )
            )

            robot: MotorChainRobot = get_yam_robot(
                channel="can0" if side == "left" else "can1",
                gripper_type=GripperType.LINEAR_4310,
            )

            kin = CartesianKinematics(model=model, side=side)
            pose = kin.forward(robot.get_joint_pos()[:6])

            mirrored_buttons = side == "left"
            arms[side] = ArmState(
                robot=robot,
                kin=kin,
                target=CartesianTarget.from_pose(pose=pose),
                device=device,
                close_button=1 if mirrored_buttons else 0,
                open_button=0 if mirrored_buttons else 1,
            )

        try:
            next_tick = time.perf_counter()
            # NOTE: Implementation copied but not ideal for phys setup
            while True:
                for arm in arms.values():
                    arm.robot.command_joint_pos(update_arm(arm))
                next_tick += DT
                time.sleep(max(0.0, next_tick - time.perf_counter()))

        finally:
            for side, arm in arms.items():
                arm.robot.enter_gravity_comp_idle()
            input(f"Program crashed. Press Enter to verify safe position for arm(s).")
            for side, arm in arms.items():
                arm.robot.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device",
        "-d",
        choices=["spacemouse", "keyboard"],
        required=True,
        help="Input device to use (keyboard is not implemented yet)",
    )
    parser.add_argument(
        "--dual",
        action="store_true",
        help="Control both arms using two SpaceMice",
    )
    main(parser.parse_args())
