from abc import ABC, abstractmethod
from typing import Annotated, Self

import numpy as np
import numpy.typing as npt

Velocities = Annotated[
    npt.NDArray[np.float64], "3D linear velocity + 3D angular velocity"
]
Buttons = Annotated[list[int], "Length 2 list with close=0, open=1"]


class Input(ABC):
    """Abstract base class for input devices

    To implement:
        is_available: class method to check if the input device is available
        __enter__: context manager entry method
        __exit__: context manager exit method
    """

    @classmethod
    @abstractmethod
    def is_available(cls) -> bool: ...

    @abstractmethod
    def __enter__(self) -> Self: ...

    @abstractmethod
    def __exit__(
        self,
        *exc: object,  # *exc is for conventions in threading
    ) -> None: ...

    @abstractmethod
    def read(self) -> tuple[Velocities, Buttons]: ...
