from typing import Annotated, Self, override

import numpy as np
import numpy.typing as npt
import pyspacemouse
from pyspacemouse import AxisConvention

from robot.inputs.input import Input

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Buttons = Annotated[list[int], "Length 2 list with close=0, open=1"]


class SpaceMouse(Input):
    def __init__(
        self,
        device_index: int,
        expo: float = 0.6,
        lin_scale: float = 0.12,
        ang_scale: float = 0.8,
    ) -> None:
        self.device_index = device_index
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
        self._spacemouse = pyspacemouse.open_by_path(
            device_index=self.device_index, axis_convention=AxisConvention.ROS
        )

        if self.device_index == 0:
            self._spacemouse.set_led(True)
        else:
            self._spacemouse.set_led(False)
        return self

    def __exit__(self, *_: object) -> None:
        assert self._spacemouse is not None
        if self.device_index == 0:
            self._spacemouse.set_led(False)
        self._spacemouse.close()

    def read(self) -> tuple[Velocities, Buttons]:
        """Get the current velocity and button states from the SpaceMouse."""

        assert self._spacemouse is not None, (
            "SpaceMouse is not initialized. Use 'with SpaceMouse() as sm:'"
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


if __name__ == "__main__":
    import time

    assert SpaceMouse.is_available(), "SpaceMouse is not available"

    with SpaceMouse(device_index=0) as sm:
        while True:
            velocities, buttons = sm.read()
            print(velocities, buttons)
            time.sleep(0.1)
