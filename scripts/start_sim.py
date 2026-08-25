"""Teleop demo: drive the simulated arms with GUI sliders and record."""

from contextlib import ExitStack
from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.cameras.sim_camera import SimCamera
from robot.environment.simulation import Simulation
from robot.recording.recorder import Recorder

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
CAMERA_NAMES = ("left", "right", "overhead")
CAMERA_FPS = 6  # one render costs ~30 ms; three at 30 Hz would not fit a tick


def main() -> None:
    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    view = ViserMujocoScene(server, sim.model, num_envs=1)
    low, high = sim.model.actuator_ctrlrange.T
    sliders = [
        server.gui.add_slider(
            label=f"a{i}",
            min=low[i],
            max=high[i],
            step=1e-3,
            initial_value=float(sim.state[i]),
        )
        for i in range(sim.model.nu)
    ]

    with ExitStack() as stack:
        cameras = [
            stack.enter_context(
                SimCamera(
                    sim,
                    name=name,
                    fps=CAMERA_FPS,
                    offset=i / CAMERA_FPS / len(CAMERA_NAMES),
                )
            )
            for i, name in enumerate(CAMERA_NAMES)
        ]
        with Recorder(ROOT / "recordings" / "last_session") as recorder:
            while True:
                recorder.record([camera.read() for camera in cameras], sim.state)

                action = np.array([slider.value for slider in sliders], np.float32)
                sim.step(action)
                view.update_from_mjdata(sim.data)


if __name__ == "__main__":
    main()
