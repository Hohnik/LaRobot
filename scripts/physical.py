import time
from pathlib import Path

import mujoco
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

try:
    kin = CartesianKinematics(model=model, side="left", site_name="tcp")
    joint_indices = kin.qpos_indices
    pose = kin.forward(robot.get_joint_pos())
    print(pose)
    target = CartesianTarget.from_pose(pose)
    print(target.position, target.rotation)

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
    robot.close()
