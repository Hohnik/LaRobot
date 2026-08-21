from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class Frame:
    """A single frame from one named camera."""

    camera_name: str

    # Depth can be none because of the C920
    rgb: NDArray[np.uint8]
    depth: NDArray[np.uint16] | None = None
