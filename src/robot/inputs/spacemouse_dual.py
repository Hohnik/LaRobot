from typing import Annotated, Self, override

import numpy as np
import numpy.typing as npt
import pyspacemouse
from pyspacemouse import AxisConvention

from robot.inputs.input_dual import Input

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Buttons = Annotated[list[int], "Length 2 list with close=0, open=1"]


class SpaceMouseDual(Input):
    def __init__(
        self,
        expo: float = 0.6,
        lin_scale: float = 0.12,
        ang_scale: float = 0.8,
    ) -> None:
        self.expo = expo
        self.lin_scale = lin_scale
        self.ang_scale = ang_scale
        self._spacemouse_left: pyspacemouse.SpaceMouseDevice | None = None
        self._spacemouse_right: pyspacemouse.SpaceMouseDevice | None = None

    @classmethod
    @override
    def is_available(cls) -> bool:
        return len(pyspacemouse.get_connected_devices()) == 2

    def __enter__(self) -> Self:
        assert self.is_available(), ConnectionError("SpaceMouse is not available")
        self._spacemouse_left = pyspacemouse.open(
            device_index=0, axis_convention=AxisConvention.ROS
        )
        self._spacemouse_left.set_led(True)
        self._spacemouse_right = pyspacemouse.open(
            device_index=1, axis_convention=AxisConvention.ROS
        )
        return self

    def __exit__(self, *_: object) -> None:
        assert self._spacemouse_left is not None
        self._spacemouse_left.set_led(False)
        self._spacemouse_left.close()
        assert self._spacemouse_right is not None
        self._spacemouse_right.close()

    def read(self) -> tuple[Velocities, Velocities, Buttons, Buttons]:
        """Get the current velocity and button states from the SpaceMouse."""

        assert (
            self._spacemouse_left is not None and self._spacemouse_right is not None
        ), "SpaceMouse is not initialized. Use 'with SpaceMouse() as sm:'"

        # NOTE: Each read() takes one queued report. Keep going until the
        # timestamp stops changing, meaning the queue is empty.
        state_left = self._spacemouse_left.read()
        state_right = self._spacemouse_right.read()
        for _ in range(64):
            t_l = state_left.t
            t_r = state_right.t
            state_left = self._spacemouse_left.read()
            state_right = self._spacemouse_right.read()
            if state_left.t == t_l and state_right.t == t_r:
                break

        # if not state_left.has_motion(0.01):
        #     return np.zeros(6, dtype=np.float64), state_left.buttons
        #
        # if not state_right.has_motion(0.01):
        #     return np.zeros(6, dtype=np.float64), state_right.buttons

        velocities_left: Velocities = np.array(
            [
                state_left.x,
                state_left.y,
                state_left.z,
                state_left.roll,
                state_left.pitch,
                state_left.yaw,
            ],
            dtype=np.float64,
        )

        velocities_right: Velocities = np.array(
            [
                state_right.x,
                state_right.y,
                state_right.z,
                state_right.roll,
                state_right.pitch,
                state_right.yaw,
            ],
            dtype=np.float64,
        )

        velocities_left[:3] = (
            (1 - self.expo) * velocities_left[:3] + self.expo * velocities_left[:3] ** 3
        ) * self.lin_scale
        velocities_left[3:] = (
            (1 - self.expo) * velocities_left[3:] + self.expo * velocities_left[3:] ** 3
        ) * self.ang_scale

        velocities_right[:3] = (
            (1 - self.expo) * velocities_right[:3]
            + self.expo * velocities_right[:3] ** 3
        ) * self.lin_scale
        velocities_right[3:] = (
            (1 - self.expo) * velocities_right[3:]
            + self.expo * velocities_right[3:] ** 3
        ) * self.ang_scale

        return (
            velocities_left,
            velocities_right,
            state_left.buttons,
            state_right.buttons,
        )


if __name__ == "__main__":
    import time

    assert SpaceMouse.is_available(), "SpaceMouse is not available"

    with SpaceMouse() as sm:
        while True:
            velocities_left, velocities_right, buttons_left, buttons_right = sm.read()
            print(velocities_left, velocities_right, buttons_left, buttons_right)
            time.sleep(0.1)
