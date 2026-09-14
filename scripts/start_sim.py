from pathlib import Path

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation
from robot.inputs.keyboard import Keyboard
from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 0.0495, 0.0
GRIPPER_STEP = 0.005
LAG_LIMIT = 0.03
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.4, 1.5
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)  # Camera


def main(args) -> None:
    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    # NOTE: Values are not perfectly aligned with camera position!!!
    server.initial_camera.position = POSITION
    server.initial_camera.look_at = LOOK_AT
    server.initial_camera.fov = FOV

    view = ViserMujocoScene(server, sim.model, num_envs=1)
    view.camera_tracking_enabled = False

    kin = CartesianKinematics(sim.model, side="left")
    left_joint_incides = kin.qpos_indices

    pose = kin.forward(sim.data.qpos[left_joint_incides])
    target = CartesianTarget.from_pose(pose=pose)

    # marker
    marker_body_id = sim.model.body("target_marker_left").id
    marker_mocap_id = sim.model.body_mocapid[marker_body_id]

    match args.device[0]:
        case "spacemouse":
            device = SpaceMouse(
                device_index=0, expo=EXPO, lin_scale=LIN_SCALE, ang_scale=ANG_SCALE
            )
        case "keyboard":
            device = Keyboard()

    with device as dev:
        gripper = GRIPPER_OPEN
        while True:
            measured_joints = sim.data.qpos[left_joint_incides]

            # spacemouse - where to
            velocities, buttons = dev.read()
            target.integrate(velocities)

            # lag - only move in a radius
            kin_position = kin.forward(measured_joints)[:3, 3]

            delta = target.position - kin_position
            distance = np.linalg.norm(delta)

            if distance > LAG_LIMIT:
                target.position = kin_position + delta / distance * LAG_LIMIT

            # update joints
            joints = kin.inverse(
                measured_joints,
                target_position=target.position,
                target_rotation=target.rotation,
            )

            # gripper
            if buttons[0]:
                gripper = max(GRIPPER_SHUT, gripper - GRIPPER_STEP)
            elif buttons[1]:
                gripper = min(GRIPPER_OPEN, gripper + GRIPPER_STEP)

            # marker
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, target.rotation.ravel())
            sim.data.mocap_pos[marker_mocap_id] = target.position
            sim.data.mocap_quat[marker_mocap_id] = quat

            sim.step(left=np.append(joints, gripper))

            view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(usage="%(prog)s [options]")
    parser.add_argument(
        "--device",
        "-d",
        choices=["spacemouse", "keyboard"],
        nargs=1,
        help="select a input device",
        required=True,
    )
    args = parser.parse_args()
    main(args)
