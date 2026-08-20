from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation
from robot.cameras.SimCamera import SimCamera
from robot.cameras.camera import Camera
from robot.record import Recorder

sim = Simulation(
    str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml"),
    realtime=True,
)

left_camera: Camera = SimCamera(camera_name="left_camera", sim=sim)
top_camera: Camera = SimCamera(camera_name="top_camera", sim=sim)
right_camera: Camera = SimCamera(camera_name="right_camera", sim=sim)

left_camera.connect()
top_camera.connect()
right_camera.connect()

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

with Recorder("../recordings/last_session.txt") as recorder:
    while True:
        left_frame =left_camera.read()
        top_frame=top_camera.read()
        right_frame =right_camera.read()

        np.stack([left_frame.rgb, top_frame.rgb, right_frame.rgb] )
        recorder.record()



        sim.step(np.array([slider.value for slider in sliders], np.float32))
        view.update_from_mjdata(sim.data)
        sim.wait()