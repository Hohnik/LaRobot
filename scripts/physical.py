import time
from pathlib import Path

import mujoco
import numpy as np
from i2rt.robots.get_robot import MotorChainRobot, get_yam_robot
from i2rt.robots.utils import GripperType

from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
model = mujoco.MjModel.from_xml_path(str(SCENE))
LAG_LIMIT = 0.07
GRIPPER_STEP = 0.05
GRIPPER_OPEN, GRIPPER_SHUT = 0.049, 0.0

try:
    robot: MotorChainRobot = get_yam_robot(
        channel="can0",
        gripper_type=GripperType.LINEAR_4310,
    )

    kin = CartesianKinematics(model=model, side="left", site_name="tcp")
    joint_indices = kin.qpos_indices
    pose = kin.forward(robot.get_joint_pos()[:6])
    target = CartesianTarget.from_pose(pose)

    path = SpaceMouse.connected_paths()[0]
    device = SpaceMouse(
        device_path=path, side="left", expo=0.6, lin_scale=0.2, ang_scale=1.5
    )

    with device as dev:
        period = 1 / 30
        next_tick = time.perf_counter()

        while True:
            # time
            dt = period

            q_measured = robot.get_joint_pos()
            q = q_measured[:6]
            gripper = q_measured[6]

            # spacemouse - where to
            velocities, buttons = dev.read()
            target.integrate(velocities, dt=dt)

            kin_position = kin.forward(q)[:3, 3]

            delta = target.position - kin_position
            distance = np.linalg.norm(delta)

            if distance > LAG_LIMIT:
                target.position = kin_position + delta / distance * LAG_LIMIT

            # update joints
            joints = kin.inverse(
                q,
                target_position=target.position,
                target_rotation=target.rotation,
                dt=dt,
            )

            # gripper
            if buttons[0]:
                gripper = max(GRIPPER_SHUT, gripper - GRIPPER_STEP)
            elif buttons[1]:
                gripper = min(1, gripper + GRIPPER_STEP)

            robot.command_joint_pos(np.append(joints, gripper))

            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))

finally:
    robot.enter_gravity_comp_idle()
    input("Program crashed. Press Enter to verify safe position.")
    robot.close()
