import argparse
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation
from robot.inputs.input import Input
from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 0.0495, 0.0
GRIPPER_STEP = 0.005
LAG_LIMIT = 0.03
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.4, 1.5
# NOTE: Values are not perfectly aligned with camera position.
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)


@dataclass
class ArmState:
    kin: CartesianKinematics
    target: CartesianTarget
    marker_mocap_id: int
    device: Input
    close_button: int = 0
    open_button: int = 1
    gripper: float = GRIPPER_OPEN


def update_arm(sim: Simulation, arm: ArmState) -> np.ndarray:
    measured_joints = sim.data.qpos[arm.kin.qpos_indices]

    velocities, buttons = arm.device.read()
    arm.target.integrate(velocities)

    # Keep the target within LAG_LIMIT of the measured position.
    measured_position = arm.kin.forward(measured_joints)[:3, 3]
    delta = arm.target.position - measured_position
    distance = np.linalg.norm(delta)

    if distance > LAG_LIMIT:
        arm.target.position = measured_position + delta / distance * LAG_LIMIT

    joints = arm.kin.inverse(
        measured_joints,
        target_position=arm.target.position,
        target_rotation=arm.target.rotation,
    )

    if buttons[arm.close_button]:
        arm.gripper = max(GRIPPER_SHUT, arm.gripper - GRIPPER_STEP)
    elif buttons[arm.open_button]:
        arm.gripper = min(GRIPPER_OPEN, arm.gripper + GRIPPER_STEP)

    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, arm.target.rotation.ravel())
    sim.data.mocap_pos[arm.marker_mocap_id] = arm.target.position
    sim.data.mocap_quat[arm.marker_mocap_id] = quat

    return np.append(joints, arm.gripper)


def main(args: argparse.Namespace) -> None:
    if args.device == "keyboard":
        raise NotImplementedError("Keyboard input is not implemented yet")

    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    server.initial_camera.position = POSITION
    server.initial_camera.look_at = LOOK_AT
    server.initial_camera.fov = FOV

    view = ViserMujocoScene(server, sim.model, num_envs=1)
    view.camera_tracking_enabled = False

    sides = ("left", "right") if args.dual else ("left",)

    with ExitStack() as stack:
        arms = {}

        for device_index, side in enumerate(sides):
            device = stack.enter_context(
                SpaceMouse(
                    device_index=device_index,
                    expo=EXPO,
                    lin_scale=LIN_SCALE,
                    ang_scale=ANG_SCALE,
                )
            )

            kin = CartesianKinematics(sim.model, side=side)
            pose = kin.forward(sim.data.qpos[kin.qpos_indices])
            marker_body_id = sim.model.body(f"target_marker_{side}").id

            # Mirror the left mouse in dual mode for matching thumb actions.
            mirrored_buttons = args.dual and side == "left"
            arms[side] = ArmState(
                kin=kin,
                target=CartesianTarget.from_pose(pose=pose),
                marker_mocap_id=int(sim.model.body_mocapid[marker_body_id]),
                device=device,
                close_button=1 if mirrored_buttons else 0,
                open_button=0 if mirrored_buttons else 1,
            )

        while True:
            commands = {side: update_arm(sim, arm) for side, arm in arms.items()}
            sim.step(**commands)
            view.update_from_mjdata(sim.data)


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
