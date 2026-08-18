"""The put-bottles MuJoCo scene: step it, read joints, grab camera frames."""

import mujoco
import numpy as np

# 0-6 => Joints in radians, 7 => Gripper in m
INIT_POS = (0, np.pi / 3, np.pi / 3, 0, 0, 0, 0) * 2
TIMESTEP, DECIMATION = 0.002, 17  # 29.4 Hz control


class World:
    def __init__(
        self, scene: str, cams=("top", "left", "right"), height=168, width=224
    ):
        self.height = height
        self.width = width
        self.model = mujoco.MjModel.from_xml_path(scene)  # All fixed data
        self.model.opt.timestep = TIMESTEP
        self.data = mujoco.MjData(self.model)  # All variable data
        self.qadr = self.model.jnt_qposadr[
            self.model.actuator_trnid[:, 0]
        ]  # 14 actuated joints
        self.cams = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, c) for c in cams
        ]
        self._renderer = None
        self.reset()

    @property
    def dt(self):
        return TIMESTEP * DECIMATION

    @property
    def state(self):
        return self.data.qpos[self.qadr].copy()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = INIT_POS
        self.data.ctrl[:] = INIT_POS
        mujoco.mj_forward(self.model, self.data)

    def step(self, action):
        self.data.ctrl[:] = action
        for _ in range(DECIMATION):
            mujoco.mj_step(self.model, self.data)

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
