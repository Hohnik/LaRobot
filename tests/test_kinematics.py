"""Unit tests for CartesianKinematics on the bimanual put_bottle scene."""

from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
import pytest

from robot import CONTROL_HZ
from robot.kinematics.cartesian_kinematics import (
    ARM_JOINTS,
    RADS_PER_SECOND,
    CartesianKinematics,
)

SCENE = str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml")
HOME = np.array([0.0, np.pi / 3, np.pi / 3, 0.0, 0.0, 0.0])
BENT = np.array([0.3, 1.2, 0.8, -0.4, 0.5, 0.9])  # every joint off zero
MAX_STEP = RADS_PER_SECOND / CONTROL_HZ  # rad a joint may move per control tick


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(SCENE)


@pytest.fixture(params=["left", "right"])
def kin(model: mujoco.MjModel, request: pytest.FixtureRequest) -> CartesianKinematics:
    return CartesianKinematics(model, request.param)


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


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("site", ["tcp", "grasp"])
@pytest.mark.parametrize("joints", [HOME, BENT], ids=["home", "bent"])
def test_forward_matches_the_mujoco_site_pose(
    model: mujoco.MjModel,
    site: Literal["tcp", "grasp"],
    side: Literal["left", "right"],
    joints: np.ndarray,
):
    """FK must agree with MuJoCo's own forward pass on the same model"""
    kin = CartesianKinematics(model, site_name=site, side=side)
    expected_indices = np.array(
        [
            model.jnt_qposadr[model.joint(f"{side}_joint{i}").id]
            for i in range(1, ARM_JOINTS + 1)
        ]
    )
    data = mujoco.MjData(model)
    data.qpos[expected_indices] = joints
    mujoco.mj_forward(model, data)

    pose = kin.forward(joints)

    assert np.array_equal(kin.qpos_indices, expected_indices)
    assert pose.shape == (4, 4)
    assert np.allclose(pose[:3, 3], data.site(f"{side}_{site}_site").xpos, atol=1e-9)
    assert np.allclose(
        pose[:3, :3], data.site(f"{side}_{site}_site").xmat.reshape(3, 3), atol=1e-9
    )
    assert np.array_equal(pose[3], [0, 0, 0, 1])


def test_forward_does_not_depend_on_earlier_calls(kin: CartesianKinematics):
    """The solver keeps an internal configuration; it must not leak between calls"""
    first = kin.forward(HOME)
    kin.forward(BENT)
    assert np.array_equal(kin.forward(HOME), first)


def test_forward_rejects_the_wrong_number_of_joints(kin: CartesianKinematics):
    with pytest.raises(ValueError):
        kin.forward(np.zeros(ARM_JOINTS + 1))


def test_inverse_holds_still_when_already_at_the_target(kin: CartesianKinematics):
    pose = kin.forward(HOME)
    joints = kin.inverse(HOME, pose[:3, 3], pose[:3, :3])
    assert np.allclose(joints, HOME, atol=1e-9)


def test_inverse_converges_on_a_reachable_pose(kin: CartesianKinematics):
    """Feeding IK its own output must settle on a shifted and rotated target"""
    pose = kin.forward(HOME)
    position = pose[:3, 3] + [0.03, -0.02, 0.03]
    rotation = rotated_about_tool_z(pose[:3, :3], 20)

    joints = HOME
    for _ in range(CONTROL_HZ // 2):  # half a second of control ticks
        joints = kin.inverse(joints, position, rotation)

    reached = kin.forward(joints)
    assert joints.shape == (ARM_JOINTS,)
    assert np.linalg.norm(reached[:3, 3] - position) < 1e-6
    assert rotation_angle(reached[:3, :3], rotation) < 1e-3


def test_inverse_moves_no_joint_faster_than_the_velocity_limit(
    kin: CartesianKinematics,
):
    pose = kin.forward(HOME)
    far_away = pose[:3, 3] + [0.5, 0.0, 0.0]

    joints = kin.inverse(HOME, far_away, pose[:3, :3])

    step = np.abs(joints - HOME)
    assert step.max() == pytest.approx(MAX_STEP)  # the limit is what holds it back
    assert np.all(step <= MAX_STEP + 1e-9)


def test_inverse_stays_within_the_joint_limits(
    model: mujoco.MjModel, kin: CartesianKinematics
):
    """An unreachable target must saturate at the joint range, not push past it"""
    pose = kin.forward(HOME)
    far_below = pose[:3, 3] + [0.0, 0.0, -0.8]

    joints = HOME
    for _ in range(3 * CONTROL_HZ):  # three seconds, plenty to reach the range
        joints = kin.inverse(joints, far_below, pose[:3, :3])

    joint_ids = [
        model.joint(f"{kin.side}_joint{i}").id for i in range(1, ARM_JOINTS + 1)
    ]
    lower, upper = model.jnt_range[joint_ids].T
    at_a_limit = np.isclose(joints, lower, atol=1e-6) | np.isclose(
        joints, upper, atol=1e-6
    )
    assert np.all(joints >= lower - 1e-6)
    assert np.all(joints <= upper + 1e-6)
    assert np.any(at_a_limit)  # a limit, not the singularity, is what stopped it


@pytest.mark.parametrize(
    ("joints", "position", "rotation"),
    [
        (np.zeros(ARM_JOINTS + 1), np.zeros(3), np.eye(3)),
        (HOME, np.zeros(2), np.eye(3)),
        (HOME, np.zeros(3), np.eye(4)),
    ],
    ids=["joints", "position", "rotation"],
)
def test_inverse_rejects_wrong_shapes(
    kin: CartesianKinematics,
    joints: np.ndarray,
    position: np.ndarray,
    rotation: np.ndarray,
):
    with pytest.raises(ValueError):
        kin.inverse(joints, position, rotation)
