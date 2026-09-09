from typing import Annotated, Self

import numpy as np
import numpy.typing as npt

from robot.inputs.input import Input

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Buttons = Annotated[list[int], "Length 2 list with close=0, open=1"]


class Keyboard(Input):
    """# TODO: implement pynput version"""

    def __init__(self):
        raise NotImplementedError

    @classmethod
    def is_available(cls) -> bool:
        raise NotImplementedError

    def __enter__(self) -> Self:
        raise NotImplementedError

    def __exit__(
        self,
        *exc: object,  # *exc is for conventions in threading
    ) -> None:
        raise NotImplementedError

    def read(self) -> tuple[Velocities, Buttons]:
        raise NotImplementedError
