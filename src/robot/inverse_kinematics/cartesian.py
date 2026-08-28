from pathlib import Path

import mink
import mujoco
import numpy as np 

from robot import CONTROL_HZ

ARM_JOINTS = 6 

class CartesianIK:
    def __init__(
            self,
            model_path: str | Path,
            site_name: str = "grasp_site",
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.configuration = mink.Configuration(self.model)
        self.site_name = site_name
        self.frame_task = mink.FrameTask(
            frame_name=self.site_name,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1e-3,
        )

        self.limits = [
            mink.ConfigurationLimit(self.model),
        ]

    def forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        joint_positions = np.asarray(joint_positions, dtype=float) 

        if joint_positions.shape != (ARM_JOINTS,):
            raise ValueError(
                f"expected {ARM_JOINTS} arm joints, got {joint_positions.shape}"
            )

        qpos = self.configuration.q.copy()
        qpos[:ARM_JOINTS] = joint_positions
        self.configuration.update(qpos)

        pose = self.configuration.get_transform_frame_to_world(
            self.site_name,
            "site",
        )
        return pose.as_matrix()

    def solve_step(
            self,
            current_joint_positions: np.ndarray,
            target_pose: np.ndarray,
            dt: float = 1 / CONTROL_HZ,
    ) -> np.ndarray:
        current_joint_positions = np.asarray(
            current_joint_positions,
            dtype=float,
        )
        target_pose = np.asarray(target_pose, dtype=float)

        if current_joint_positions.shape != (ARM_JOINTS,):
            raise ValueError(
                f"expected {ARM_JOINTS} arm joints, "
                f"got {current_joint_positions.shape}"
            )

        if target_pose.shape != (4, 4):
            raise ValueError(
                f"expected target pose shape (4, 4), "
                f"got {target_pose.shape}"
            )
        qpos = self.configuration.q.copy()
        qpos[:ARM_JOINTS] = current_joint_positions
        self.configuration.update(qpos)

        self.frame_task.set_target(
            mink.SE3.from_matrix(target_pose)
        )

        velocity = mink.solve_ik(
            self.configuration,
            tasks=[self.frame_task],
            dt=dt,
            solver="daqp",
            damping=1e-3,
            limits=self.limits,
        )

        self.configuration.integrate_inplace(velocity, dt)

        return self.configuration.q[:ARM_JOINTS].copy()

    def solve_target(
        self,
        current_joint_positions: np.ndarray,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        dt: float = 1 / CONTROL_HZ,
    ) -> np.ndarray:

        target_position = np.asarray(target_position, dtype=float)
        target_rotation = np.asarray(target_rotation, dtype=float)

        if target_position.shape != (3,):
            raise ValueError(
                f"expected target position shape (3,), "
                f"got {target_position.shape}"
            )

        if target_rotation.shape != (3, 3):
            raise ValueError(
                f"expected target rotation shape (3, 3), "
                f"got {target_rotation.shape}"
            )
        # SpaceMouse uses 2 steps, mink uses homogenous transforms (4, 4)
        target_pose = np.eye(4)
        target_pose[:3, :3] = target_rotation
        target_pose[:3, 3] = target_position
        
        # IK calculation
        return self.solve_step(
            current_joint_positions,
            target_pose,
            dt=dt,
        )   