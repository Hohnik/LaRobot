"""Teleop the ABC put-bottles sim with sliders, and record states + camera frames.

    uv run sim_teleop.py                    # viser on :8080, Ctrl-C to save
    MUJOCO_GL=egl uv run sim_teleop.py      # headless box

Tick the "record" box to log. Ctrl-C writes recording.npz.
To use a different input, replace the one `action = ...` line.
"""

import time

import mujoco
import numpy as np
import viser
from mjviser import ViserMujocoScene

from abc_minimal import eval_policy as ep
from abc_minimal.config import PutBottlesSimConfig

CAMS, H, W, SEED = ("top", "left", "right"), 168, 224, 0
GRIPPERS = (6, 13)  # action layout: 6 left joints, left gripper, 6 right, right gripper


class Sim:
    """Stands in for MJWarpSim: plain mujoco, so no mjwarp and no CUDA."""

    def __init__(self, model, data, *, height, width, gpu_id=None):
        self.model, self.data, self.r = (
            model,
            data,
            mujoco.Renderer(model, height, width),
        )

    def close(self):
        self.r.close()

    def load_state(self):
        pass  # env.data is already the state of record

    def forward(self):
        mujoco.mj_forward(self.model, self.data)

    def qpos(self):
        return np.asarray(self.data.qpos, np.float32)

    def set_ctrl(self, ctrl):
        self.data.ctrl[:] = ctrl

    def step(self, n):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def render(self):
        return np.array([_render(self.r, self.data, c) for c in range(self.model.ncam)])


def _render(r, data, cam):
    r.update_scene(data, camera=cam)
    return r.render()


def main():
    ep.MJWarpSim = Sim
    scene = PutBottlesSimConfig()
    env = ep.PutBottlesEnv(
        height=H,
        width=W,
        camera_keys=CAMS,
        scene=scene,
        prompt="sim put the plastic bottles in the bin",
    )
    env.reset(seed=SEED)

    server = viser.ViserServer(port=8080)
    view = ViserMujocoScene(server, env.model, num_envs=1)
    record = server.gui.add_checkbox("record", False)
    lo, hi = env.model.actuator_ctrlrange.T
    sliders = [
        server.gui.add_slider(
            mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i),
            0.0 if i in GRIPPERS else float(lo[i]),
            1.0 if i in GRIPPERS else float(hi[i]),
            0.001,
            float(q),
        )
        for i, q in enumerate(env.get_state_vanilla())
    ]

    dt, log = scene.timestep * scene.control_decimation, []
    print(f"viser on :8080, {1 / dt:.1f} Hz, Ctrl-C to save", flush=True)
    try:
        while True:
            t = time.perf_counter()
            action = np.array([s.value for s in sliders], np.float32)
            if record.value:
                log.append(
                    (env.get_state_vanilla(), action, env.obs_vanilla_state()["images"])
                )
            env.step_one_vanilla(action)
            view.update_from_mjdata(env.data)
            time.sleep(max(0.0, dt - (time.perf_counter() - t)))
    except KeyboardInterrupt:
        np.savez_compressed(
            "recording.npz",
            states=np.array([s for s, _, _ in log]),
            actions=np.array([a for _, a, _ in log]),
            **{c: np.array([i[c] for _, _, i in log]) for c in CAMS},
        )
        print(f"\nsaved recording.npz, {len(log)} steps", flush=True)
        env.close()


if __name__ == "__main__":
    main()
