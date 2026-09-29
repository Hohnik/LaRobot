#Fully AI Written Test
from pathlib import Path

import mujoco
import numpy as np

from robot.environment.simulation import Simulation
from robot.evaluation.put_bottles import (
    BOTTLE_NAMES,
    PlacementEvaluator,
)

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets" / "put_bottles" / "put_bottle.xml"


def _move_bottle_center(
    sim: Simulation,
    evaluator: PlacementEvaluator,
    bottle_name: str,
    target_in_bin: np.ndarray,
) -> None:
    body_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_BODY,
        bottle_name,
    )
    joint_id = int(sim.model.body_jntadr[body_id])
    qpos_address = int(sim.model.jnt_qposadr[joint_id])

    bin_position = sim.data.xpos[evaluator._bin_body_id]
    bin_rotation = sim.data.xmat[
        evaluator._bin_body_id
    ].reshape(3, 3)

    target_world = (
        bin_position
        + bin_rotation @ target_in_bin
    )
    translation = (
        target_world
        - sim.data.subtree_com[body_id]
    )

    sim.data.qpos[
        qpos_address : qpos_address + 3
    ] += translation

    mujoco.mj_forward(sim.model, sim.data)


def test_initial_scene_has_no_bottles_in_bin():
    sim = Simulation(str(SCENE))
    evaluator = PlacementEvaluator(sim.model)

    result = evaluator.evaluate(sim.data)

    assert result.num_inside == 0
    assert result.progress == 0.0
    assert not result.success


def test_one_bottle_in_bin_is_detected():
    sim = Simulation(str(SCENE))
    evaluator = PlacementEvaluator(sim.model)

    middle_height = (
        evaluator._bin_bottom_height
        + evaluator._bin_top_height
    ) / 2

    _move_bottle_center(
        sim,
        evaluator,
        "bottle_1",
        np.array([0.0, middle_height, 0.0]),
    )

    result = evaluator.evaluate(sim.data)

    assert result.num_inside == 1
    assert result.inside_names == ("bottle_1",)
    assert not result.success


def test_all_bottles_in_bin_is_success():
    sim = Simulation(str(SCENE))
    evaluator = PlacementEvaluator(sim.model)

    middle_height = (
        evaluator._bin_bottom_height
        + evaluator._bin_top_height
    ) / 2

    for bottle_name in BOTTLE_NAMES:
        _move_bottle_center(
            sim,
            evaluator,
            bottle_name,
            np.array([0.0, middle_height, 0.0]),
        )

    result = evaluator.evaluate(sim.data)

    assert result.num_inside == len(BOTTLE_NAMES)
    assert result.progress == 1.0
    assert result.success


def test_bottle_outside_radius_is_not_counted():
    sim = Simulation(str(SCENE))
    evaluator = PlacementEvaluator(sim.model)

    middle_height = (
        evaluator._bin_bottom_height
        + evaluator._bin_top_height
    ) / 2
    allowed_radius = evaluator._allowed_radius(
        np.array([middle_height])
    )[0]

    _move_bottle_center(
        sim,
        evaluator,
        "bottle_1",
        np.array(
            [
                allowed_radius + 0.001,
                middle_height,
                0.0,
            ]
        ),
    )

    result = evaluator.evaluate(sim.data)

    assert result.num_inside == 0
    assert not result.success