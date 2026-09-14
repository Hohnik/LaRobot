from pathlib import Path

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation
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


def main() -> None:
    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    # NOTE: Values are not perfectly aligned with camera position!!!
    server.initial_camera.position = POSITION
    server.initial_camera.look_at = LOOK_AT
    server.initial_camera.fov = FOV

    view = ViserMujocoScene(server, sim.model, num_envs=1)
    view.camera_tracking_enabled = False

    kin_left = CartesianKinematics(sim.model, side="left")
    kin_right = CartesianKinematics(sim.model, side="right")
    left_joint_incides = kin_left.qpos_indices
    right_joint_indices = kin_right.qpos_indices

    pose_left = kin_left.forward(sim.data.qpos[left_joint_incides])
    pose_right = kin_right.forward(sim.data.qpos[right_joint_indices])
    target_left = CartesianTarget.from_pose(pose=pose_left)
    target_right = CartesianTarget.from_pose(pose=pose_right)

    # marker left
    marker_body_left_id = sim.model.body("target_marker_left").id
    marker_mocap_left_id = sim.model.body_mocapid[marker_body_left_id]

    # marker right
    marker_body_right_id = sim.model.body("target_marker_right").id
    marker_mocap_right_id = sim.model.body_mocapid[marker_body_right_id]

    dev0 = SpaceMouse(
        device_index=0, expo=EXPO, lin_scale=LIN_SCALE, ang_scale=ANG_SCALE
    )
    dev1 = SpaceMouse(
        device_index=1, expo=EXPO, lin_scale=LIN_SCALE, ang_scale=ANG_SCALE
    )

    with dev0 as dev_left, dev1 as dev_right:
        gripper_left = GRIPPER_OPEN
        gripper_right = GRIPPER_OPEN

        while True:
            measured_joints_left = sim.data.qpos[left_joint_incides]
            measured_joints_right = sim.data.qpos[right_joint_indices]

            # spacemouse - where to
            velocities_left, buttons_left = dev_left.read()
            velocities_right, buttons_right = dev_right.read()
            target_left.integrate(velocities_left)
            target_right.integrate(velocities_right)

            kin_position_left = kin_left.forward(measured_joints_left)[:3, 3]
            kin_position_right = kin_right.forward(measured_joints_right)[:3, 3]

            delta_left = target_left.position - kin_position_left
            delta_right = target_right.position - kin_position_right
            distance_left = np.linalg.norm(delta_left)
            distance_right = np.linalg.norm(delta_right)

            if distance_left > LAG_LIMIT:
                target_left.position = (
                    kin_position_left + delta_left / distance_left * LAG_LIMIT
                )

            if distance_right > LAG_LIMIT:
                target_right.position = (
                    kin_position_right + delta_right / distance_right * LAG_LIMIT
                )

            joints_left = kin_left.inverse(
                measured_joints_left,
                target_position=target_left.position,
                target_rotation=target_left.rotation,
            )
            joints_right = kin_right.inverse(
                measured_joints_right,
                target_position=target_right.position,
                target_rotation=target_right.rotation,
            )

            # gripper
            if buttons_left[1]:
                gripper_left = max(GRIPPER_SHUT, gripper_left - GRIPPER_STEP)
            elif buttons_left[0]:
                gripper_left = min(GRIPPER_OPEN, gripper_left + GRIPPER_STEP)

            if buttons_right[0]:
                gripper_right = max(GRIPPER_SHUT, gripper_right - GRIPPER_STEP)
            elif buttons_right[1]:
                gripper_right = min(GRIPPER_OPEN, gripper_right + GRIPPER_STEP)

            # marker
            quat_left = np.empty(4)
            quat_right = np.empty(4)
            mujoco.mju_mat2Quat(quat_left, target_left.rotation.ravel())
            mujoco.mju_mat2Quat(quat_right, target_right.rotation.ravel())
            sim.data.mocap_pos[marker_mocap_left_id] = target_left.position
            sim.data.mocap_quat[marker_mocap_left_id] = quat_left
            sim.data.mocap_pos[marker_mocap_right_id] = target_right.position
            sim.data.mocap_quat[marker_mocap_right_id] = quat_right

            sim.step(
                left=np.append(joints_left, gripper_left),
                right=np.append(joints_right, gripper_right),
            )

            view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    main()
