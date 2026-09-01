"""Closed-loop tests: IK on the measured joints -> position actuators -> physics.

These check the simulated tool site, not the kinematic model's FK, so they cover
the whole chain the SpaceMouse bridge relies on.
"""

from pathlib import Path

import numpy as np
import pytest

from robot import CONTROL_HZ
from robot.environment.simulation import INIT_POS_LEFT, LEFT, RIGHT, Simulation
from robot.kinematics.cartesian_kinematics import ARM_JOINTS, CartesianKinematics

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")
SITE = "left_tcp_site"
JOINTS, GRIPPER = slice(0, ARM_JOINTS), ARM_JOINTS  # within one arm's 7 values

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
def kin(sim: Simulation) -> CartesianKinematics:
    return CartesianKinematics(sim.model, site_name=SITE)


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees between two rotation matrices"""
    cos = (np.trace(a.T @ b) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def rotated_about_tool_z(rotation: np.ndarray, degrees: float) -> np.ndarray:
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return rotation @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tcp_pose(sim: Simulation) -> tuple[np.ndarray, np.ndarray]:
    """Position and rotation of the simulated tool site"""
    site = sim.data.site(SITE)
    return site.xpos.copy(), site.xmat.reshape(3, 3).copy()


def track(
    sim: Simulation,
    kin: CartesianKinematics,
    position: np.ndarray,
    rotation: np.ndarray,
    seconds: float = 1.0,
) -> None:
    """Run the control loop like the bridge does: IK on the measured left joints,
    gripper held, right arm left at its reset command."""
    for _ in range(round(seconds * CONTROL_HZ)):
        joints = kin.inverse(sim.state[LEFT][JOINTS], position, rotation)
        sim.step(left=np.append(joints, INIT_POS_LEFT[GRIPPER]))


@pytest.mark.parametrize(
    "offset",
    [[0.03, 0.0, 0.0], [0.0, -0.03, 0.0], [0.0, 0.0, 0.03]],
    ids=["x", "y", "z"],
)
def test_the_tcp_reaches_a_shifted_target(
    sim: Simulation, kin: CartesianKinematics, offset: list[float]
):
    position, rotation = tcp_pose(sim)
    target = position + offset

    track(sim, kin, target, rotation)

    reached_position, reached_rotation = tcp_pose(sim)
    assert np.linalg.norm(reached_position - target) < POSITION_TOLERANCE
    assert rotation_angle(reached_rotation, rotation) < ROTATION_TOLERANCE


def test_the_tcp_reaches_a_rotated_target(sim: Simulation, kin: CartesianKinematics):
    position, rotation = tcp_pose(sim)
    target = rotated_about_tool_z(rotation, 20)

    track(sim, kin, position, target)

    reached_position, reached_rotation = tcp_pose(sim)
    assert rotation_angle(reached_rotation, target) < ROTATION_TOLERANCE
    assert np.linalg.norm(reached_position - position) < POSITION_TOLERANCE


def test_only_the_left_arm_moves(sim: Simulation, kin: CartesianKinematics):
    initial = sim.state.copy()
    position, rotation = tcp_pose(sim)

    track(sim, kin, position + [0.03, 0.0, 0.0], rotation)

    final = sim.state
    assert np.abs(final[LEFT][JOINTS] - initial[LEFT][JOINTS]).max() > 0.05
    assert final[LEFT][GRIPPER] == pytest.approx(initial[LEFT][GRIPPER], abs=1e-3)
    assert np.allclose(final[RIGHT], initial[RIGHT], atol=1e-3)
