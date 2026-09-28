from contextlib import ExitStack
from pathlib import Path

import numpy as np

from robot.cameras.sim_camera import SimCamera
from robot.environment.simulation import Simulation
from robot.recording.recorder import Recorder
from robot.recording.sample import Sample

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"


def main() -> Path:
    """Record a short simulated episode without input hardware."""
    sim = Simulation(str(SCENE))

    with ExitStack() as stack:
        cameras = [
            stack.enter_context(SimCamera(sim, name, 224, 224, 10))
            for _, name in sim.list_cameras()
        ]
        write_dir = ROOT / "data" / "episodes"
        recorder = stack.enter_context(Recorder(write_dir))

        for _ in range(10):
            action = sim.data.ctrl.astype(np.float32)
            recorder.record(
                Sample(
                    timestamp_ns=int(sim.data.time * 1_000_000_000),
                    frames=tuple(camera.read() for camera in cameras),
                    state=sim.state,
                    action=action,
                )
            )
            sim.step(action[:7], action[7:])

    return recorder.file_path


if __name__ == "__main__":
    main()
