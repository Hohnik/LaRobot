from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene

from robot.environment.simulation import Simulation

sim = Simulation(
    str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml"),
    cams=(),
    realtime=True,
)
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

while True:
    sim.step(np.array([slider.value for slider in sliders], np.float32))
    view.update_from_mjdata(sim.data)
