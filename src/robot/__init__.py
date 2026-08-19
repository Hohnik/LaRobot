from dataclasses import dataclass

import numpy as np

CONTROL_HZ = 30  # steps per second


@dataclass(frozen=True, slots=True)
class Observation:
    """
    joints  (14,) float32. Left arm, then right arm, 7 values each: 6 joint
            angles in radians, then 1 gripper opening in metres. The units are
            mixed because the hardware is. The action array has the same layout.
    images  camera name -> (H, W, 3) uint8 RGB. The newest frame at this tick.
            Empty when the environment runs without cameras.
    time    seconds since reset(). Starts at 0.0 in both worlds.
    """

    joints: np.ndarray
    images: dict[str, np.ndarray]
    time: float
