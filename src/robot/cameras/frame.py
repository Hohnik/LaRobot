# Frame class for handling camera frames

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

@dataclass(frozen=True, slots=True)
class Frame:
    '''A single camer frame that combined with a timestemp'''

    camera_name: str
    sequence: int

    # Timestemps
    camera_timestemp_ns: int | None
    host_timestamp_ns: int 

    # Depth can be non because of the C920
    rgb: NDArray[np.uint8]
    depth: NDArray[np.uint8] | None = None