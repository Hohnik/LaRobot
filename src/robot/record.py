from pathlib import Path
import numpy as np

class Recorder():
    def __init__(self, write_dir: str | Path):
        self.write_dir = Path(write_dir)
        self.step = 0

    def __enter__(self):
        self.write_dir.mkdir(parents=True, exist_ok=True)
        # TODO: Implement writing to file
        return self

    def record(self, images: np.ndarray, states: np.ndarray):
        self.step += 1


    def __exit__(self, type, value, traceback):
        return False