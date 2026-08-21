import itertools
from typing import override

import mujoco

from robot.cameras.camera import Camera
from robot.cameras.frame import Frame
from robot.environment.simulation import Simulation


class SimCamera(Camera):
    def __init__(self, sim: Simulation, name: str, width: int, height: int) -> None:
        self.sim: Simulation = sim
        self.name: str = name

        self.width: int = width
        self.height: int = height
        self._renderer: mujoco.Renderer | None = None

        self._sequence: itertools.count[int] = itertools.count(1)

    @override
    @classmethod
    def is_available(cls) -> bool:
        return True

    @override
    def connect(self) -> None:
        cameras = [self.sim.model.cam(i) for i in range(self.sim.model.ncam)]
        # id of the camera were the name matches
        self._id: int = next(
            i for i, cam in enumerate(cameras) if cam.name == self.name
        )

        # TODO: stream camera frames to somewere in a camera specific frequency to mimic real cam
        # This should just happen in the SimCamera's connect the real one does this automatically(???)

    @override
    def read(self) -> Frame:
        # TODO: Return the last frame written to the stream
        raise NotImplementedError()

    @override
    def close(self) -> None:
        pass

    def frames(self) -> list[Frame]:
        """Renders every 30Hz a frame from each camera"""
        frames = []
        for camera in self.sim.model:
            self.renderer.update_scene(self.data, camera=camera)
            frame = Frame(camera.name, self.renderer.render())

            frames.append()
        return frames

    @property
    def renderer(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.sim.model, self.height, self.width)
        return self._renderer
