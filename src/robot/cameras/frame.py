from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class Frame:
    """ A single frame from one named camera.

    Parameters
    ----------
    camera_name : str
        The name of the camera that captured this frame.
    timestamp_ns_capture : int
        The timestamp in nanoseconds when the frame was captured.
    timestamp_ns_host : int
        The timestamp in nanoseconds when the frame was received.
    rgb : NDArray[np.uint8]
        The RGB image data of the frame.
    depth : NDArray[np.uint16] | None, optional
        The depth data of the frame, if available.
    """

    camera_name: str

    timestamp_ns_capture: int
    timestamp_ns_host: int

    rgb: NDArray[np.uint8]
    depth: NDArray[np.uint16] | None = None
