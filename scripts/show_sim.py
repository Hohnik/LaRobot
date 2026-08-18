import time
from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene
from robot.sim.world import World

world = World(str(Path(__file__).parents[1] / "assets/put_bottles/put_bottle.xml"))
server = viser.ViserServer(port=8080)
view = ViserMujocoScene(server, world.model, num_envs=1)
low, high = world.model.actuator_ctrlrange.T
sliders = [
    server.gui.add_slider(
        label=f"a{i}", min=low[i], max=high[i], step=1e-3, initial_value=world.state[i]
    )
    for i in range(world.model.nu)
]

while True:
    world.step(np.array([slider.value for slider in sliders], np.float32))
    view.update_from_mjdata(world.data)
    time.sleep(world.dt)
