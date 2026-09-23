"""Closed-loop tests: IK on the measured joints -> position actuators -> physics.

These check the simulated tool site, not the kinematic model's FK, so they cover
the whole chain the SpaceMouse bridge relies on.
"""

from pathlib import Path
from typing import Literal

import numpy as np
import pytest

from robot import CONTROL_HZ
from robot.environment.simulation import (
    INIT_POS_LEFT,
    INIT_POS_RIGHT,
    LEFT,
    RIGHT,
    Simulation,
)
from robot.kinematics.cartesian_kinematics import ARM_JOINTS, CartesianKinematics

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")
SITE = "tcp"
JOINTS, GRIPPER = slice(0, ARM_JOINTS), ARM_JOINTS  # within one arm's 7 values
Side = Literal["left", "right"]

# The actuators are plain PD controllers, so under gravity the closed loop settles
# about a millimetre and a fraction of a degree short of the target.
POSITION_TOLERANCE = 0.002  # m
ROTATION_TOLERANCE = 1.0  # deg


@pytest.fixture(scope="module")
def loaded_sim() -> Simulation:
    return Simulation(SCENE)  # parsing the scene takes about a second


@pytest.fixture
def sim(loaded_sim: Simulation) -> Simulation:
    loaded_sim.reset()
    return loaded_sim


@pytest.fixture
def kin(sim: Simulation, side: Side) -> CartesianKinematics:
    return CartesianKinematics(sim.model, site_name=SITE, side=side)


@pytest.fixture(params=["left", "right"])
def side(request: pytest.FixtureRequest) -> Side:
    return request.param


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees between two rotation matrices"""
    cos = (np.trace(a.T @ b) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def rotated_about_tool_z(rotation: np.ndarray, degrees: float) -> np.ndarray:
    """Apply a rotation about the tool's local z axis.

    Parameters
    ----------
    rotation : ndarray, shape (3, 3)
        Initial tool orientation.
    degrees : float
        Rotation angle in degrees.

    Returns
    -------
    Updated tool orientation.
    ```
    ndarray, shape (3, 3)
    ```
    """
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return rotation @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tcp_pose(sim: Simulation, side: Side) -> tuple[np.ndarray, np.ndarray]:
    """Position and rotation of the simulated tool site"""
    site = sim.data.site(f"{side}_{SITE}_site")
    return site.xpos.copy(), site.xmat.reshape(3, 3).copy()


def track(
    sim: Simulation,
    kin: CartesianKinematics,
    side: Side,
    position: np.ndarray,
    rotation: np.ndarray,
    seconds: float = 1.0,
) -> None:
    """Run one arm's measured-joint IK loop while leaving the other untouched."""
    arm = LEFT if side == "left" else RIGHT
    gripper = (INIT_POS_LEFT if side == "left" else INIT_POS_RIGHT)[GRIPPER]
    for _ in range(round(seconds * CONTROL_HZ)):
        joints = kin.inverse(sim.state[arm][JOINTS], position, rotation)
        command = np.append(joints, gripper)
        if side == "left":
            sim.step(left=command)
        else:
            sim.step(right=command)


@pytest.mark.parametrize(
    "offset",
    [[0.03, 0.0, 0.0], [0.0, -0.03, 0.0], [0.0, 0.0, 0.03]],
    ids=["x", "y", "z"],
)
def test_the_tcp_reaches_a_shifted_target(
    sim: Simulation, kin: CartesianKinematics, side: Side, offset: list[float]
):
    position, rotation = tcp_pose(sim, side)
    target = position + offset

    track(sim, kin, side, target, rotation)

    reached_position, reached_rotation = tcp_pose(sim, side)
    assert np.linalg.norm(reached_position - target) < POSITION_TOLERANCE
    assert rotation_angle(reached_rotation, rotation) < ROTATION_TOLERANCE


def test_the_tcp_reaches_a_rotated_target(
    sim: Simulation, kin: CartesianKinematics, side: Side
):
    position, rotation = tcp_pose(sim, side)
    target = rotated_about_tool_z(rotation, 20)

    track(sim, kin, side, position, target)

    reached_position, reached_rotation = tcp_pose(sim, side)
    assert rotation_angle(reached_rotation, target) < ROTATION_TOLERANCE
    assert np.linalg.norm(reached_position - position) < POSITION_TOLERANCE


def test_only_the_selected_arm_moves(
    sim: Simulation, kin: CartesianKinematics, side: Side
):
    initial = sim.state.copy()
    position, rotation = tcp_pose(sim, side)

    track(sim, kin, side, position + [0.03, 0.0, 0.0], rotation)

    final = sim.state
    selected, other = (LEFT, RIGHT) if side == "left" else (RIGHT, LEFT)
    assert np.abs(final[selected][JOINTS] - initial[selected][JOINTS]).max() > 0.05
    assert final[selected][GRIPPER] == pytest.approx(
        initial[selected][GRIPPER], abs=1e-3
    )
    assert np.allclose(final[other], initial[other], atol=1e-3)


def test_both_arms_reach_independent_targets(sim: Simulation):
    kin_left = CartesianKinematics(sim.model, side="left")
    kin_right = CartesianKinematics(sim.model, side="right")
    position_left, rotation_left = tcp_pose(sim, "left")
    position_right, rotation_right = tcp_pose(sim, "right")
    target_left = position_left + [0.02, 0.0, 0.01]
    target_right = position_right + [0.0, 0.02, 0.01]

    for _ in range(CONTROL_HZ):
        joints_left = kin_left.inverse(
            sim.state[LEFT][JOINTS], target_left, rotation_left
        )
        joints_right = kin_right.inverse(
            sim.state[RIGHT][JOINTS], target_right, rotation_right
        )
        sim.step(
            left=np.append(joints_left, INIT_POS_LEFT[GRIPPER]),
            right=np.append(joints_right, INIT_POS_RIGHT[GRIPPER]),
        )

    reached_left, _ = tcp_pose(sim, "left")
    reached_right, _ = tcp_pose(sim, "right")
    assert np.linalg.norm(reached_left - target_left) < POSITION_TOLERANCE
    assert np.linalg.norm(reached_right - target_right) < POSITION_TOLERANCE
