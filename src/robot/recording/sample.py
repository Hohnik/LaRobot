from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robot.cameras.frame import Frame


@dataclass(frozen=True, slots=True)
class Sample:
    """One observation and the action selected from it."""

    timestamp_ns: int
    frames: tuple[Frame, ...]
    state: NDArray[np.float32]
    action: NDArray[np.float32]
