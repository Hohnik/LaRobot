from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robot.cameras.frame import Frame


@dataclass(frozen=True,slots=True)
class Sample:
    timestamp_s: float
    frames: tuple[Frame, ...]
    state: NDArray[np.float32]
    action: NDArray[np.float32]

    