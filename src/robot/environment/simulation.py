"""The put-bottles MuJoCo scene: step it, read joints, grab camera frames."""

import time
import warnings

import mujoco
import numpy as np

from robot import CONTROL_HZ, Observation

# 0-5 => Joints in radians, 6 => Gripper in m. Twice over: left arm, then right arm.
INIT_POS = (0, np.pi / 3, np.pi / 3, 0, 0, 0, 0) * 2

SUBSTEPS = 17  # physics substeps per control tick
TIMESTEP = 1 / (CONTROL_HZ * SUBSTEPS)  # one tick is then exactly 1 / CONTROL_HZ s


class Simulation:
    """One control tick is one step(). Cameras cost time, so pass cams=() to go fast."""

    def __init__(
        self,
        scene: str,
        cams=("top", "left", "right"),
        height=168,
        width=224,
        realtime=False,
    ):
        self.height = height
        self.width = width
        self.realtime = realtime  # hold step() to the wall clock, for a live viewer
        self.model = mujoco.MjModel.from_xml_path(scene)  # All fixed data
        self.model.opt.timestep = TIMESTEP
        self.data = mujoco.MjData(self.model)  # All variable data
        self.qadr = self.model.jnt_qposadr[
            self.model.actuator_trnid[:, 0]
        ]  # 14 actuated joints
        self.cam_names = tuple(cams)
        self.cams = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c) for c in cams
        ]
        self._renderer = None
        self._next = 0.0  # wall-clock deadline of the next tick
        self.reset()

    @property
    def state(self):
        return self.data.qpos[self.qadr].astype(np.float32)

    def observation(self) -> Observation:
        return Observation(
            self.state,
            dict(zip(self.cam_names, self.frames(), strict=True)),
            self.data.time,
        )

    def reset(self) -> Observation:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = INIT_POS
        self.data.ctrl[:] = INIT_POS
        mujoco.mj_forward(self.model, self.data)
        self._next = time.perf_counter()
        return self.observation()

    def step(self, action) -> Observation:
        """Apply the action, let one control tick of time pass, then read the result.

        The real arm runs these same three lines. Only the clock is different:
        there, _wait_for_tick is how the time passes; here, mj_step already
        passed it. Read at the same point in both, or a policy learns a one-tick
        offset that the hardware does not have.
        """
        self.data.ctrl[:] = action
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        self._wait_for_tick()
        return self.observation()

    def _wait_for_tick(self):
        """Sleep until the deadline of the next tick.

        The deadline is absolute, not a duration. A caller that thinks for 20 ms
        therefore spends that time out of this sleep, and does not add it to the
        tick. The control rate stays at CONTROL_HZ.
        """
        if not self.realtime:
            return  # simulated time already passed; run as fast as the CPU allows
        self._next += 1 / CONTROL_HZ
        late = time.perf_counter() - self._next
        if late > 0.5 / CONTROL_HZ:
            warnings.warn(f"tick overrun by {late * 1e3:.0f} ms", stacklevel=2)
            self._next = time.perf_counter()  # drop the debt, do not sprint to catch up
        time.sleep(max(0.0, -late))

    def frames(self):
        return [self._shot(c) for c in self.cams]

    def _shot(self, cam):
        self.renderer.update_scene(self.data, camera=cam)
        return np.asarray(self.renderer.render())

    @property
    def renderer(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        return self._renderer
