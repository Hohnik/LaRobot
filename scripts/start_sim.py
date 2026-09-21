import argparse
from contextlib import ExitStack
from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.arm.teleoperation import ArmState, update_arm
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
# NOTE: Values are not perfectly aligned with camera position.
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)


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
                gripper=GRIPPER_OPEN,
                close_button=1 if mirrored_buttons else 0,
                open_button=0 if mirrored_buttons else 1,
            )

        while True:
            commands = {
                side: update_arm(
                    sim,
                    arm,
                    gripper_open=GRIPPER_OPEN,
                    gripper_shut=GRIPPER_SHUT,
                    gripper_step=GRIPPER_STEP,
                    lag_limit=LAG_LIMIT,
                )
                for side, arm in arms.items()
            }
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
