import contextlib
from pathlib import Path

import laya
import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot import CONTROL_HZ
from robot.environment.simulation import Simulation
from robot.kinematics.cartesian_kinematics import CartesianKinematics

SCENE = Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml"
BOTTLES = ("bottle_1", "bottle_2", "bottle_3")
OPEN = 0.0495
CLOSED = 0.0
SPEED = 0.12
CARRY_HEIGHT = 1.20
STEPS_PER_DECISION = 8
TOP_CAMERA_POSITION = (0.086, 0.0, 1.6)
TOP_CAMERA_LOOK_AT = (1.086, 0.0, 0.0)

AXES = {
    "x": ("forward", "back"),
    "y": ("left", "right"),
    "z": ("up", "down"),
}
DIRECTION_CACHE = {}
QUESTIONS = {
    axis: {
        "type": "choice",
        "instructions": f"The value of {axis} says where the target is. "
        f"Which direction must the gripper move along {axis}?",
        "criteria": {
            positive: f"move {positive}",
            negative: f"move {negative}",
            "keep": "do not move this axis",
        },
    }
    for axis, (positive, negative) in AXES.items()
}


def is_in_bin(sim: Simulation, bottle: str) -> bool:
    bottle_xy = sim.data.body(bottle).xpos[:2]
    bin_xy = sim.data.body("bin_container").xpos[:2]
    return bool(np.linalg.norm(bottle_xy - bin_xy) < 0.08)


def describe_next_move(sim: Simulation, holding_bottle: bool) -> dict[str, str] | None:
    hand_position = sim.data.site("right_grasp_site").xpos

    if holding_bottle:
        target = sim.data.body("bin_container").xpos.copy()
        target[2] = CARRY_HEIGHT
        if hand_position[2] < CARRY_HEIGHT - 0.03:
            target[:2] = hand_position[:2]  # lift before moving to the bin
    else:
        bottles = [bottle for bottle in BOTTLES if not is_in_bin(sim, bottle)]
        if not bottles:
            return None

        bottle = min(bottles, key=lambda name: sim.data.body(name).xpos[0])
        target = sim.data.body(bottle).xpos + [0.0, 0.0, 0.18]
        if np.linalg.norm(target[:2] - hand_position[:2]) > 0.025:
            target[2] = max(hand_position[2], 1.05)  # move above it first

    difference = target - hand_position
    state = {}
    for (axis, (positive, negative)), distance in zip(AXES.items(), difference):
        if abs(distance) < 0.01:
            state[axis] = "hold"
        else:
            direction = positive if distance > 0 else negative
            state[axis] = f"{abs(round(distance * 100))} cm {direction}"

    reached_target = np.linalg.norm(difference) < 0.02
    state["hand"] = "open" if holding_bottle == reached_target else "close"
    return state


def choose_action(agent, state: dict[str, str]) -> tuple[np.ndarray, float]:
    directions = tuple(state[axis].split()[-1] for axis in AXES)

    if directions not in DIRECTION_CACHE:
        movement = {axis: state[axis] for axis in AXES}
        answers = agent.predict(movement, QUESTIONS)["answers"]
        print(answers)
        print()
        DIRECTION_CACHE[directions] = np.array(
            [
                {positive: 1.0, negative: -1.0}.get(answers[axis]["choice"], 0.0)
                for axis, (positive, negative) in AXES.items()
            ]
        )

    gripper = OPEN if state["hand"] == "open" else CLOSED
    return DIRECTION_CACHE[directions], gripper


def main() -> None:
    sim = Simulation(str(SCENE))

    server = viser.ViserServer()
    server.initial_camera.position = TOP_CAMERA_POSITION
    server.initial_camera.look_at = TOP_CAMERA_LOOK_AT
    server.initial_camera.fov = np.radians(60)
    view = ViserMujocoScene(server, sim.model, num_envs=1)
    view.camera_tracking_enabled = False

    agent = laya.load(device="mps", subfolder="multilingual")
    kinematics = CartesianKinematics(sim.model, side="right", site_name="grasp")

    hand = sim.data.site("right_grasp_site")
    rotation = hand.xmat.reshape(3, 3).copy()
    gripper_target = OPEN

    while True:
        holding_bottle = gripper_target == CLOSED and sim.state[-1] > 0.004
        state = describe_next_move(sim, holding_bottle)
        if state is None:
            print("All bottles are in the bin.")
            return

        direction, gripper_target = choose_action(agent, state)
        goal = hand.xpos.copy()

        for _ in range(STEPS_PER_DECISION):
            goal += direction * SPEED / CONTROL_HZ
            joints = kinematics.inverse(
                sim.data.qpos[kinematics.qpos_indices], goal, rotation
            )
            sim.step(right=np.append(joints, gripper_target))

        view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        main()
