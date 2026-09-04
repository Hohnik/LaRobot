from typing import Annotated, Self, override

import numpy as np
import numpy.typing as npt
import pyspacemouse
from pyspacemouse import AxisConvention

from robot.inputs.input import Input

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Vector3 = Annotated[npt.NDArray[np.float64], "3D vector"]
Rotation = Annotated[npt.NDArray[np.float64], "3x3 rotation matrix"]
Pose = Annotated[npt.NDArray[np.float64], "4x4 homogeneous transformation matrix"]


class SpaceMouseInput(Input):
    def __init__(
        self,
        expo: float = 0.6,
        lin_scale: float = 0.12,
        ang_scale: float = 0.8,
    ) -> None:
        self.expo = expo
        self.lin_scale = lin_scale
        self.ang_scale = ang_scale
        self._spacemouse: pyspacemouse.SpaceMouseDevice | None = None

    @classmethod
    @override
    def is_available(cls) -> bool:
        return pyspacemouse.get_connected_devices() != []

    def __enter__(self) -> Self:
        assert self.is_available(), ConnectionError("SpaceMouse is not available")
        self._spacemouse = pyspacemouse.open(axis_convention=AxisConvention.ROS)
        return self

    def __exit__(self, *_: object) -> None:
        assert self._spacemouse is not None
        self._spacemouse.close()

    def read(self) -> tuple[Velocities, list[int]]:
        """Get the current velocity and button states from the SpaceMouse."""

        assert self._spacemouse is not None, (
            "SpaceMouse is not initialized. Use 'with SpaceMouseInput() as sm:'"
        )

        # NOTE: Each read() takes one queued report. Keep going until the
        # timestamp stops changing, meaning the queue is empty.
        state = self._spacemouse.read()
        for _ in range(64):
            t = state.t
            state = self._spacemouse.read()
            if state.t == t:
                break

        if not state.has_motion(0.01):
            return np.zeros(6, dtype=np.float64), state.buttons

        velocities: Velocities = np.array(
            [state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
            dtype=np.float64,
        )

        velocities[:3] = (
            (1 - self.expo) * velocities[:3] + self.expo * velocities[:3] ** 3
        ) * self.lin_scale
        velocities[3:] = (
            (1 - self.expo) * velocities[3:] + self.expo * velocities[3:] ** 3
        ) * self.ang_scale

        return velocities, state.buttons


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

    def integrate(self, velocities: Velocities, dt: float) -> Pose:
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


if __name__ == "__main__":
    import time

    assert SpaceMouseInput.is_available(), "SpaceMouse is not available"

    with SpaceMouseInput() as sm:
        target = CartesianTarget()

        while True:
            velocities, buttons = sm.read()
            pose = target.integrate(velocities, 0.001)
            print(buttons)
            # print(pose.round(4))

            time.sleep(0.001)
