import argparse
from contextlib import ExitStack
from pathlib import Path
from typing import Literal

import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.arm.teleoperation import ArmState, update_arm
from robot.cameras.sim_camera import SimCamera
from robot.environment.simulation import Simulation
from robot.inputs.spacemouse import SpaceMouse
from robot.kinematics.cartesian_kinematics import CartesianKinematics
from robot.kinematics.cartesian_target import CartesianTarget
from robot.recording.recorder import Recorder
from robot.recording.sample import Sample

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 0.0495, 0.0
GRIPPER_STEP = 0.005
LAG_LIMIT = 0.03
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.4, 1.5
# NOTE: Values are not perfectly aligned with camera position.
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)


def main(args: argparse.Namespace) -> None:
    """Run simulated arm teleoperation with a browser viewer.

    Parameters
    ----------
    args : argparse.Namespace
        Input selection (`device`) and two-arm mode (`dual`).
    """
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
        arms: dict[Literal["left", "right"], ArmState] = {}
        devices: dict[Literal["left", "right"], SpaceMouse] = {}

        for device_index, side in enumerate(sides):
            device = stack.enter_context(
                SpaceMouse(
                    device_index=device_index,
                    expo=EXPO,
                    lin_scale=LIN_SCALE,
                    ang_scale=ANG_SCALE,
                )
            )
            devices[side] = device

            kin = CartesianKinematics(sim.model, side=side)
            pose = kin.forward(sim.data.qpos[kin.qpos_indices])
            marker_body_id = sim.model.body(f"target_marker_{side}").id

            # Mirror the left mouse in dual mode for matching thumb actions.
            mirrored_buttons = args.dual and side == "left"
            arms[side] = ArmState(
                kin=kin,
                target=CartesianTarget.from_pose(pose=pose),
                marker_mocap_id=int(sim.model.body_mocapid[marker_body_id]),
                gripper=GRIPPER_OPEN,
                close_button=1 if mirrored_buttons else 0,
                open_button=0 if mirrored_buttons else 1,
            )

        cameras = [
            stack.enter_context(SimCamera(sim, name, 224, 224, 10))
            for _, name in sim.list_cameras()
        ]
        recorder = stack.enter_context(Recorder(ROOT / "data" / "episodes"))

        while True:
            commands = {}
            for side, arm in arms.items():
                velocities, buttons = devices[side].read()
                commands[side] = update_arm(
                    sim,
                    arm,
                    velocities,
                    buttons,
                    gripper_open=GRIPPER_OPEN,
                    gripper_shut=GRIPPER_SHUT,
                    gripper_step=GRIPPER_STEP,
                    lag_limit=LAG_LIMIT,
                )

            action = sim.data.ctrl.astype(np.float32)
            for side, command in commands.items():
                action_slice = slice(0, 7) if side == "left" else slice(7, 14)
                action[action_slice] = command

            sample = Sample(
                timestamp_ns=int(sim.data.time * 1_000_000_000),
                frames=tuple(camera.read() for camera in cameras),
                state=sim.state,
                action=action,
            )
            recorder.record(sample)

            sim.step(**commands)
            view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    _ = parser.add_argument(
        "--device",
        "-d",
        choices=["spacemouse", "keyboard"],
        required=True,
        help="Input device to use (keyboard is not implemented yet)",
        type=str,
    )
    _ = parser.add_argument(
        "--dual",
        action="store_true",
        help="Control both arms using two SpaceMice",
        type=bool,
    )
    main(parser.parse_args())
