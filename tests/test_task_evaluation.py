from pathlib import Path
import mujoco
import numpy as np

from abc_sim import make_task_evaluator

from robot.environment.simulation import Simulation 

SCENE = Path(__file__).resolve().parents[1] / "assets/put_bottles/put_bottle.xml"

def test_scene_has_no_bottles_in_bin():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    result = evaluator.evaluate_qpos_batch(
        sim.data.qpos[None, :]
    )

    assert result.metrics["num_active_bottles"][0] == 6
    assert result.metrics["num_bottles_in_bin"][0] == 0
    assert not result.scalar_success()

def test_one_bottle_in_bin_is_detected():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    qpos = sim.data.qpos.copy()

    bottle_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bottle_1_joint",
    )
    bin_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bin_joint",
    )

    bottle_qpos_adress = sim.model.jnt_qposadr[bottle_joint_id]
    bin_qpos_adress = sim.model.jnt_qposadr[bin_joint_id]

    qpos[bottle_qpos_adress : bottle_qpos_adress + 3] = qpos [
        bin_qpos_adress : bin_qpos_adress +3
    ]

    result = evaluator.evaluate_qpos_batch(qpos[None, :])

    assert result.metrics["num_active_bottles"][0] == 6
    assert result.metrics["num_bottles_in_bin"][0] == 1
    assert result.metrics["bottles_in_bin"][0] == ["bottle_1"]
    assert not result.scalar_success()

def test_all_bottles_in_bin_is_success():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    qpos = sim.data.qpos.copy()

    bin_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bin_joint",
    )
    bin_qpos_address = sim.model.jnt_qposadr[bin_joint_id]
    bin_position = qpos[
        bin_qpos_address : bin_qpos_address + 3
    ].copy()

    for bottle_number in range(1, 7):
        bottle_joint_id = mujoco.mj_name2id(
            sim.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            f"bottle_{bottle_number}_joint",
        )
        bottle_qpos_address = sim.model.jnt_qposadr[
            bottle_joint_id
        ]

        qpos[
            bottle_qpos_address : bottle_qpos_address + 3
        ] = bin_position

    result = evaluator.evaluate_qpos_batch(qpos[None, :])

    assert result.metrics["num_active_bottles"][0] == 6
    assert result.metrics["num_bottles_in_bin"][0] == 6
    assert result.scalar_success()

def test_bottle_just_outside_bin_is_not_counted():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    qpos = sim.data.qpos.copy()
    initial_result = evaluator.evaluate_qpos_batch(qpos[None, :])

    bin_bottom = initial_result.metrics["bin_bottom_y"]
    bin_top = initial_result.metrics["bin_top_y"]
    radius_bottom = initial_result.metrics["bin_radius_bottom"]
    radius_top = initial_result.metrics["bin_radius_top"]

    height_fraction = (0.0 - bin_bottom) / (bin_top - bin_bottom)
    allowed_radius = radius_bottom + height_fraction * (
        radius_top - radius_bottom
    )

    bin_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bin_joint",
    )
    bottle_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bottle_5_joint",
    )

    bin_address = sim.model.jnt_qposadr[bin_joint_id]
    bottle_address = sim.model.jnt_qposadr[bottle_joint_id]

    bin_rotation = np.empty(9)
    mujoco.mju_quat2Mat(
        bin_rotation,
        qpos[bin_address + 3 : bin_address + 7],
    )
    bin_rotation = bin_rotation.reshape(3, 3)

    position_just_outside = np.array(
        [allowed_radius + 0.001, 0.0, 0.0]
    )

    qpos[bottle_address : bottle_address + 3] = (
        qpos[bin_address : bin_address + 3]
        + bin_rotation @ position_just_outside
    )

    result = evaluator.evaluate_qpos_batch(qpos[None, :])

    assert result.metrics["num_bottles_in_bin"][0] == 0
    assert result.metrics["bottles_in_bin"][0] == []
    assert not result.scalar_success()

def test_evaluator_reset_clears_previous_success():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    successful_qpos = sim.data.qpos.copy()

    bin_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bin_joint",
    )
    bin_address = sim.model.jnt_qposadr[bin_joint_id]
    bin_position = successful_qpos[
        bin_address : bin_address + 3
    ].copy()

    for bottle_number in range(1, 7):
        bottle_joint_id = mujoco.mj_name2id(
            sim.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            f"bottle_{bottle_number}_joint",
        )
        bottle_address = sim.model.jnt_qposadr[
            bottle_joint_id
        ]
        successful_qpos[
            bottle_address : bottle_address + 3
        ] = bin_position

    successful_result = evaluator.evaluate_qpos_batch(
        successful_qpos[None, :]
    )

    assert successful_result.scalar_success()
    assert successful_result.metrics["ever_success"][0]
    assert (
        successful_result.metrics["max_bottles_in_bin_so_far"][0]
        == 6
    )

    evaluator.reset()

    reset_result = evaluator.evaluate_qpos_batch(
        sim.data.qpos[None, :]
    )

    assert not reset_result.scalar_success()
    assert not reset_result.metrics["ever_success"][0]
    assert reset_result.metrics["max_bottles_in_bin_so_far"][0] == 0

def test_simulation_reset_restores_initial_evaluation():
    sim = Simulation(str(SCENE))
    evaluator = make_task_evaluator(
        sim.model,
        "put_plastic_bottles_in_bin",
    )
    assert evaluator is not None

    initial_qpos = sim.data.qpos.copy()

    bottle_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bottle_1_joint",
    )
    bin_joint_id = mujoco.mj_name2id(
        sim.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "bin_joint",
    )

    bottle_address = sim.model.jnt_qposadr[bottle_joint_id]
    bin_address = sim.model.jnt_qposadr[bin_joint_id]

    sim.data.qpos[
        bottle_address : bottle_address + 3
    ] = sim.data.qpos[
        bin_address : bin_address + 3
    ]

    mujoco.mj_forward(sim.model, sim.data)

    changed_result = evaluator.evaluate_qpos_batch(
        sim.data.qpos[None, :]
    )

    assert changed_result.metrics["num_bottles_in_bin"][0] == 1

    sim.reset()
    evaluator.reset()

    reset_result = evaluator.evaluate_qpos_batch(
        sim.data.qpos[None, :]
    )

    assert np.allclose(sim.data.qpos, initial_qpos)
    assert reset_result.metrics["num_bottles_in_bin"][0] == 0
    assert reset_result.metrics["max_bottles_in_bin_so_far"][0] == 0
    assert not reset_result.scalar_success()