from contextlib import ExitStack
from pathlib import Path

import numpy as np
import viser
from mjviser import ViserMujocoScene
from abc_minimal.config import SimEvalConfig
from abc_minimal.eval_policy import resolve_prompt
from abc_minimal.policy import DiTInferencePolicy

from robot.arm.teleoperation import ArmState, update_arm
from robot.environment.simulation import Simulation
from robot.cameras.sim_camera import SimCamera

ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
GRIPPER_OPEN, GRIPPER_SHUT = 0.0475, 0.0
GRIPPER_STEP = 0.005
LAG_LIMIT = 0.03
EXPO, LIN_SCALE, ANG_SCALE = 0.6, 0.4, 1.5
# NOTE: Values are not perfectly aligned with camera position.
POSITION, LOOK_AT, FOV = (0.086, 0.0, 1.6), (1.086, 0.0, 0), np.radians(60)


def main() -> None:
    sim = Simulation(str(SCENE), realtime=True)

    server = viser.ViserServer(port=8080)
    server.initial_camera.position = POSITION
    server.initial_camera.look_at = LOOK_AT
    server.initial_camera.fov = FOV

    view = ViserMujocoScene(server, sim.model, num_envs=1)
    view.camera_tracking_enabled = False

    checkpoint = Path('scripts/policies/abc_dit_xl_200k_model.pt')
    config = SimEvalConfig(
        checkpoint=str(checkpoint),
        task="put_plastic_bottles_in_bin",
        fast_inference=False,
        rtc=False,
    )
    config.prompt = resolve_prompt(config)

    policy = DiTInferencePolicy(
        checkpoint=checkpoint,
        config=config,
        device="cuda",  # Use an available device.
        model_config=config.model,
    )

    with ExitStack() as stack:
        cameras = {
            name: stack.enter_context(
                SimCamera(sim=sim, name=name, width=224, height=168, fps=30)
            )
            for name in ("top", "left", "right")
        }

        while True:
            obs = {
                "state":sim.state.copy(),
                "images": {
                    name: cameras[name].read().rgb
                    for name in ("top", "left", "right")
                },
                "prompt": config.prompt,
            }
            actions = policy.infer(obs)
            for action in actions[:15]:
                target = action.copy()
                target[[6, 13]] = np.clip(target[[6, 13]],0 ,1) * 0.0475
                sim.step(left=target[:7], right=target[7:])
                view.update_from_mjdata(sim.data)

if __name__ == "__main__":
    main()
