from pathlib import Path

import numpy as np

from robot.cameras.frame import Frame


class Recorder:
    """Writes frames and states of every recorded tick into write_dir"""

    def __init__(self, write_dir: str | Path):
        self.write_dir = Path(write_dir)
        self.step = 0

    def __enter__(self):
        self.write_dir.mkdir(parents=True, exist_ok=True)
        return self

    def record(self, frames: list[Frame], states: np.ndarray):
        # TODO: Implement writing to file
        self.step += 1

    def __exit__(self, type, value, traceback):
        return False
