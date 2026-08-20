import itertools
import time

import mujoco
import numpy as np

from robot.cameras.camera import Camera
from robot.cameras.frame import Frame
from robot.environment.simulation import Simulation

class SimCamera(Camera):
    def __init__(self, sim: Simulation, name: str) -> None:
        self.sim = sim
        self.name = name
        self._id = int | None = None
        self._sequence = itertools.count(1)

    @classmethod
    def available(cls) -> bool:
        return True

    def connect(self) -> None:
        cameras = [self.sim.model.cam(i) for i in range(self.sim.model.ncam)]
        self._id = self.world.camera_id(self.name)

        # TODO: stream camera frames to somewere in a camera specific frequency to mimic real cam
        # This should just happen in the SimCamera's connect the real one does this automatically(???)

    def read(self) -> Frame:
        # TODO: Return the last frame written to the stream
        return 