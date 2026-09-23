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
    """Track a target pose by integrating Cartesian velocities.

    Parameters
    ----------
    position : array_like, shape (3,), optional
        Initial position; defaults to the origin.
    rotation : array_like, shape (3, 3), optional
        Initial orientation; defaults to the identity.
    """

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
        """Create a target from a homogeneous pose matrix.

        Parameters
        ----------
        pose : ndarray, shape (4, 4)
            Homogeneous transform containing the initial position and rotation.

        Returns
        -------
        Target with copies of the supplied position and rotation.
        ```
        return cls(position=pose[:3, 3], rotation=pose[:3, :3])
        ```
        """
        return cls(position=pose[:3, 3], rotation=pose[:3, :3])

    @property
    def pose(self) -> Pose:
        """Get the current target pose.

        Returns
        -------
        New homogeneous transform containing the current position and rotation.
        ```
        ndarray, shape (4, 4)
        ```
        """
        pose = np.eye(4)
        pose[:3, 3] = self.position
        pose[:3, :3] = self.rotation
        return pose

    @staticmethod
    def rotation_matrix(axis_angle: Vector3) -> Rotation:
        """Rotation matrix from an axis-angle vector (Rodrigues' formula).

        Parameters
        ----------
        axis_angle : ndarray, shape (3,)
            Rotation axis scaled by the angle in radians.

        Returns
        -------
        Rotation matrix; identity for angles below 1e-12 radians.
        ```
        ndarray, shape (3, 3)
        ```
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
        """Advance the pose by one step.

        Parameters
        ----------
        velocities : ndarray, shape (6,)
            Linear xyz, then angular xyz, in the pose's reference frame.
            Rates use position units and radians per unit of `dt`.
        dt : float, optional
            Time step matching the velocity units; defaults to 1 / CONTROL_HZ.

        Returns
        -------
        Homogeneous transform containing the updated position and rotation.
        ```
        ndarray, shape (4, 4)
        ```
        """
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
