from functools import cache
from time import monotonic_ns
from typing import override

import mujoco

from robot.cameras.camera import Camera
from robot.cameras.frame import Frame
from robot.environment.simulation import Simulation


@cache  # Switching between multiple renderers is expensive so we reuse them where possible.
def _shared_renderer(sim: Simulation, width: int, height: int) -> mujoco.Renderer:
    """Reuse a renderer for each simulation and image size.

    Parameters
    ----------
    sim : Simulation
        Simulation whose model is rendered.
    width : int
        Image width in pixels.
    height : int
        Image height in pixels.

    Returns
    -------
    Renderer shared by calls with the same arguments.
    ```
    mujoco.Renderer
    ```
    """
    return mujoco.Renderer(sim.model, height, width)


class SimCamera(Camera):
    """Renders its view of the scene at its own rate, like a real camera stream"""

    def __init__(
        self,
        sim: Simulation,
        name: str,
        width: int = 224,
        height: int = 224,
        fps: int = 10,
        offset: float = 0.0,
    ) -> None:
        """Configure the camera's image size and simulation-time cadence.

        Parameters
        ----------
        sim : Simulation
            Simulation to render.
        name : str
            Camera name in the model.
        width : int, optional
            Image width in pixels; defaults to 224.
        height : int, optional
            Image height in pixels; defaults to 224.
        fps : int, optional
            Capture rate per simulation time unit; defaults to 10.
        offset : float, optional
            Negated initial capture timestamp; defaults to 0.0.
        """
        self.sim: Simulation = sim
        self.name: str = name
        self.width: int = width
        self.height: int = height
        self._min_interval: float = 1 / fps
        self._offset: float = offset
        self._camera_id: int | None = None
        self._last_frame: Frame | None = None
        self._latest_time: float = -offset

    @property
    def _renderer(self) -> mujoco.Renderer:
        return _shared_renderer(self.sim, self.width, self.height)

    @override
    @classmethod
    def is_available(cls) -> bool:
        return True

    @override
    def connect(self) -> None:
        """Resolve the camera name and warm up rendering."""
        names = [self.sim.model.cam(i).name for i in range(self.sim.model.ncam)]  # pyright: ignore[reportAny]
        try:
            self._camera_id = names.index(self.name)
        except ValueError:
            raise ValueError(
                f"Unknown camera {self.name!r}; Available cameras: {names}"
            ) from None

        # Render once to pre compile graphics shaders
        # Reset the time to the camera's offset
        _ = self.read()
        self._latest_time = -self._offset

    @property
    def connected(self) -> bool:
        return self._camera_id is not None

    def _render_due(self, now: float) -> bool:
        """True when the cached frame is missing, older than the interval, or pre-reset"""
        return (
            self._last_frame is None
            or now - self._latest_time >= self._min_interval
            or now < self._latest_time
        )

    @override
    def read(self) -> Frame:
        """Read the latest frame from the camera"""
        if self._camera_id is None:
            raise RuntimeError(f"{self.name}: call connect() before read()")

        now = self.sim.data.time
        if self._render_due(now):
            renderer = self._renderer
            renderer.update_scene(self.sim.data, camera=self._camera_id)
            self._last_frame = Frame(
                camera_name=self.name,
                timestamp_ns_capture=int(now * 1e9),
                timestamp_ns_host=monotonic_ns(),
                rgb=renderer.render(),
            )
            self._latest_time = now
        assert self._last_frame is not None  # _render_due() covers the None case
        return self._last_frame

    @override
    def close(self) -> None:
        self._camera_id = None
        self._last_frame = None
