import time
from pathlib import Path

import mujoco
import numpy as np
from i2rt.robots.get_robot import MotorChainRobot, get_yam_robot
from i2rt.robots.utils import GripperType

from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

robot: MotorChainRobot = get_yam_robot(
    channel="can0",
    gripper_type=GripperType.LINEAR_4310,
)

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
model = mujoco.MjModel.from_xml_path(str(SCENE))
LAG_LIMIT = 0.03
GRIPPER_STEP = 0.005
GRIPPER_OPEN, GRIPPER_SHUT = 0.03, 0.0
NOMINAL_DT = 1 / 30
MAX_DT = 2 * NOMINAL_DT

try:
    kin = CartesianKinematics(model=model, side="left", site_name="tcp")
    joint_indices = kin.qpos_indices
    pose = kin.forward(robot.get_joint_pos()[:6])
    target = CartesianTarget.from_pose(pose)

    path = SpaceMouse.connected_paths()[0]
    device = SpaceMouse(device_path=path, side="left")

    with device as dev:
        last_tick = time.perf_counter()
        while True:
            # time
            now = time.perf_counter()
            elapsed = now - last_tick
            last_tick = now
            dt = min(elapsed, MAX_DT)

            q_measured = robot.get_joint_pos()
            q = q_measured[:6]
            gripper = q_measured[6]

            # spacemouse - where to
            velocities, buttons = dev.read()
            print(velocities, buttons)
            print()
            target.integrate(velocities, dt=dt)

            kin_position = kin.forward(q)[:3, 3]
            print(f"kin_position:\n{kin_position}\n")
            print(f"target_position:\n{target.position}\n")

            delta = target.position - kin_position
            print("delta:", delta)
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
                gripper = min(GRIPPER_OPEN, gripper + GRIPPER_STEP)

            robot.command_joint_pos(np.append(joints, gripper))

    # while True:
    #     q = robot.get_joint_pos()
    #     obs = robot.get_observations()
    #
    #     robot.move_joints()
    #
    #     print("joint pos:", q)
    #     print("joint vel:", obs["joint_vel"])
    #     print("joint eff:", obs["joint_eff"])
    #     # joint pos: [-0.33398184  0.00286107  0.00782025  0.0085832  -0.02613107  0.00820172 0.99899634]
    #     # joint vel: [-0.002442    0.002442   -0.002442   -0.00732601 -0.00732601 -0.00732601]
    #     # joint eff: [-6.83760684e-03  2.01709402e+00  7.26837607e+00  1.87301587e+00 -7.32600733e-03 -2.44200244e-03]
    #     print()
    #
    #     time.sleep(0.5)

finally:
    robot.enter_gravity_comp_idle()
    input("Program crashed. Press Enter to verify safe position.")
    robot.close()
