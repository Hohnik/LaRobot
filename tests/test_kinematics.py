from pathlib import Path 
import mujoco
import numpy as np

from robot.kinematics.cartesian import ARM_JOINTS, CartesianKinematics
ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = (
    ROOT
    / "assets"
    / "put_bottles"
    / "assets"
    / "i2rt_yam"
    / "yam.xml"
)

MODEL = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

def test_forward_returns_valid_pose():
    kinematics = CartesianKinematics(MODEL, site_name="grasp_site")

    home_joints = np.array(
        [0.0, np.pi / 3, np.pi / 3, 0.0, 0.0, 0.0]
    )

    pose = kinematics.forward(home_joints)

    assert pose.shape == (4, 4)
    assert np.all(np.isfinite(pose))
    
def test_inverse_target_moves_grasp_site_towards_target():
    kinematics = CartesianKinematics(MODEL, site_name="grasp_site")

    current_joints = np.array(
        [0.0, np.pi / 3, np.pi / 3, 0.0, 0.0, 0.0]
    )

    current_pose = kinematics.forward(current_joints)

    target_position = current_pose[:3, 3].copy()
    target_position[0] += 0.01
    target_rotation = current_pose[:3, :3].copy()

    new_joints = kinematics.inverse(
        current_joints,
        target_position,
        target_rotation,
    )

    new_pose = kinematics.forward(new_joints)
    old_error = np.linalg.norm(
        target_position - current_pose[:3, 3]
    )
    new_error = np.linalg.norm(
        target_position - new_pose[:3, 3]
    )

    assert new_joints.shape == (ARM_JOINTS,)
    assert np.all(np.isfinite(new_joints))
    assert new_error < old_error
    