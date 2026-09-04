from pathlib import Path

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot import CONTROL_HZ
from robot.environment.simulation import Simulation
from robot.inputs.spacemouse import CartesianTarget, SpaceMouseInput
from robot.kinematics.cartesian import CartesianKinematics

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
LEFT_JOINTS = slice(0, 6)
GRIPPER_OPEN, GRIPPER_SHUT = 0.0495, 0.0
DT = 1 / CONTROL_HZ


def main() -> None:
    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    view = ViserMujocoScene(server, sim.model, num_envs=1)

    kin = CartesianKinematics(sim.model)
    left_joint_incides = kin.left_qpos_indices

    pose = kin.forward(sim.data.qpos[left_joint_incides])
    target = CartesianTarget.from_pose(pose=pose)

    # marker
    marker_body_id = sim.model.body("target_marker").id
    marker_mocap_id = sim.model.body_mocapid[marker_body_id]

    with SpaceMouseInput() as sm:
        gripper = GRIPPER_OPEN
        while True:
            twist, buttons = sm.read()
            target.integrate(twist, DT)
            measured_joints = sim.data.qpos[left_joint_incides]
            joints = kin.inverse(
                measured_joints,
                target_position=target.position,
                target_rotation=target.rotation,
                dt=DT,
            )

            if buttons[0] and gripper >= GRIPPER_SHUT:
                gripper -= 0.001
                pass
            elif buttons[1] and gripper <= GRIPPER_OPEN:
                gripper += 0.001

            # marker
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, target.rotation.ravel())
            sim.data.mocap_pos[marker_mocap_id] = target.position
            sim.data.mocap_quat[marker_mocap_id] = quat

            sim.step(left=np.append(joints, gripper))
            view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    main()
