from typing import Annotated, Self

import numpy as np
import numpy.typing as npt

from robot import CONTROL_HZ

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Vector3 = Annotated[npt.NDArray[np.float64], "3D vector"]
Rotation = Annotated[npt.NDArray[np.float64], "3x3 rotation matrix"]
Pose = Annotated[npt.NDArray[np.float64], "4x4 homogeneous transformation matrix"]


class CartesianTarget:
    def __init__(
        self,
        position: Vector3 | None = None,
        rotation: Rotation | None = None,
    ) -> None:
        position = np.zeros(3) if position is None else np.array(position)
        rotation = np.eye(3) if rotation is None else np.array(rotation)
        assert position.shape == (3,), (
            f"position must be a 3D vector, got {position.shape}"
        )
        assert rotation.shape == (3, 3), (
            f"rotation must be a 3x3 matrix, got {rotation.shape}"
        )

        self.position: Vector3 = position
        self.rotation: Rotation = rotation
        self._steps = 0

    @classmethod
    def from_pose(cls, pose: Pose) -> Self:
        return cls(position=pose[:3, 3], rotation=pose[:3, :3])

    @property
    def pose(self) -> Pose:
        pose = np.eye(4)
        pose[:3, 3] = self.position
        pose[:3, :3] = self.rotation
        return pose

    @staticmethod
    def rotation_matrix(axis_angle: Vector3) -> Rotation:
        """Rotation matrix from an axis-angle vector (Rodrigues' formula).

        Direction of `axis_angle` is the rotation axis, its length is the angle in radians.
        """
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-12:
            return np.eye(3)
        x, y, z = axis_angle / angle
        K = np.array(
            [[0, -z, y], [z, 0, -x], [-y, x, 0]]
        )  # cross-product matrix: K @ v == axis × v
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def integrate(self, velocities: Velocities, dt: float = 1 / CONTROL_HZ) -> Pose:
        """Advance the pose by one step. velocity = (linear xyz, angular xyz)."""
        linear, angular = velocities[:3], velocities[3:]
        self.position = self.position + linear * dt
        self.rotation = self.rotation_matrix(angular * dt) @ self.rotation

        self._steps += 1
        if self._steps % 200 == 0:
            self._reorthogonalize()
        return self.pose

    def _reorthogonalize(self) -> None:
        """Snap self.rotation back to the nearest proper rotation matrix.

        Floating-point drift slowly makes it non-orthogonal. Setting all singular
        values to 1 fixes that; the det sign prevents flipping into a reflection.
        """
        u, _, vt = np.linalg.svd(self.rotation)
        sign = np.sign(np.linalg.det(u @ vt))
        self.rotation = u @ np.diag([1.0, 1.0, sign]) @ vt
