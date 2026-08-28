from pathlib import Path 
from robot.inverse_kinematics.cartesian import ARM_JOINTS, CartesianIK

import numpy as np 

ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = (
    ROOT
    / "assets"
    / "put_bottles"
    / "assets"
    / "i2rt_yam"
    / "yam.xml"
)

def test_forward_kinematics_returns_valid_pose():
    ik = CartesianIK(MODEL_PATH, site_name="grasp_site")

    home_joints = np.array(
        [0.0, np.pi / 3, np.pi / 3, 0.0, 0.0, 0.0]
    )

    pose = ik.forward_kinematics(home_joints)

    assert pose.shape == (4, 4)
    assert np.all(np.isfinite(pose))
    
def test_solve_target_moves_grasp_site_towards_target():
    ik = CartesianIK(MODEL_PATH, site_name="grasp_site")

    current_joints = np.array(
        [0.0, np.pi / 3, np.pi / 3, 0.0, 0.0, 0.0]
    )

    current_pose = ik.forward_kinematics(current_joints)

    target_position = current_pose[:3, 3].copy()
    target_position[0] += 0.01
    target_rotation = current_pose[:3, :3].copy()

    new_joints = ik.solve_target(
        current_joints,
        target_position,
        target_rotation,
    )

    new_pose = ik.forward_kinematics(new_joints)

    old_error = np.linalg.norm(
        target_position - current_pose[:3, 3]
    )
    new_error = np.linalg.norm(
        target_position - new_pose[:3, 3]
    )

    assert new_joints.shape == (ARM_JOINTS,)
    assert np.all(np.isfinite(new_joints))
    assert new_error < old_error
    