import time
import warnings

import mujoco
import numpy as np

from robot import CONTROL_HZ

# 0-5 => Joints in radians, 6 => Gripper in m. Twice over: left arm, then right arm.
INIT_POS = (0, np.pi / 3, np.pi / 3, 0, 0, 0, 0) * 2

SUBSTEPS = 17  # physics substeps per control tick
TIMESTEP = 1 / (CONTROL_HZ * SUBSTEPS)  # one tick is then exactly 1 / CONTROL_HZ s


class Simulation:
    """One control tick is one step()"""

    def __init__(
        self,
        scene: str,
        realtime: bool = False,
    ):
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(scene)  # fixed data
        self.realtime: bool = realtime  # if True, step() will sleep to stay at ~30 Hz

        self.model.opt.timestep = TIMESTEP
        self.data: mujoco.MjData = mujoco.MjData(self.model)  # changing data
        self.qadr: np.ndarray = self.model.jnt_qposadr[
            self.model.actuator_trnid[:, 0]
        ]  # 14 actuated joints
        self._next_timestep: float | None = None  # initialized by the first step()
        self.reset()

    @property
    def state(self):
        return self.data.qpos[self.qadr].astype(np.float32)

    def step(self, action: np.ndarray):
        """A step() moves the state towards the action in 17 substeps per step"""
        self.data.ctrl[:] = action
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        self._wait_for_next_action_timestep()

    def _wait_for_next_action_timestep(self):
        """Sleep until the next action should be applied, to keep the control rate at CONTROL_HZ"""
        if not self.realtime:
            return

        if self._next_timestep is None:  # first step() after reset can be late
            self._next_timestep = time.perf_counter() - 1 / CONTROL_HZ

        self._next_timestep += 1 / CONTROL_HZ
        late = time.perf_counter() - self._next_timestep
        if late > 0.5 / CONTROL_HZ:
            warnings.warn(f"tick overrun by {late * 1e3:.0f} ms", stacklevel=2)
            self._next_timestep = time.perf_counter()  # reset if we are too late
        time.sleep(max(0.0, -late))

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = INIT_POS
        self.data.ctrl[:] = INIT_POS
        mujoco.mj_forward(self.model, self.data)
        self._next_timestep = None

    def list_cameras(self) -> list[tuple[int, str]]:
        return [
            (cam_id, self.model.cam(cam_id).name)  # pyright: ignore[reportAny]
            for cam_id in range(self.model.ncam)
        ]
