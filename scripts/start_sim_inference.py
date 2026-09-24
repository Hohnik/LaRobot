import argparse
from contextlib import ExitStack
from pathlib import Path
from time import perf_counter

import numpy as np
import viser
from abc_sim import make_task_evaluator
from abc_minimal.config import SimEvalConfig
from abc_minimal.eval_policy import RTCManager, resolve_prompt, build_summary
from abc_minimal.policy import DiTInferencePolicy
from mjviser import ViserMujocoScene

from robot.cameras.sim_camera import SimCamera
from robot.environment.simulation import Simulation

ACTION_DIM = 15
PREFIX_LENGTH = 4
LEAD_LENGTH = 4
RTC_START = ACTION_DIM - LEAD_LENGTH
FAST_INFERENCE = False
ROOT = Path(__file__).parents[1]
SCENE = ROOT / "assets/put_bottles/put_bottle.xml"
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

    checkpoint = Path("scripts/policies/abc_dit_xl_200k_model.pt")
    config = SimEvalConfig(
        checkpoint=str(checkpoint),
        task="put_plastic_bottles_in_bin",
        fast_inference=FAST_INFERENCE,
        fast_compile_mode="max-autotune",
        rtc=True,
        execute_chunk_dim=ACTION_DIM,
        rtc_inference_lead_steps=LEAD_LENGTH,
        rtc_prefix_length=PREFIX_LENGTH,
    )

    config.prompt = resolve_prompt(config)
    evaluator = make_task_evaluator(sim.model, config.task)

    if evaluator is None:
        raise RuntimeError(f"No evaluator available for task {config.task!r}")

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

        def read_observation():
            # Keep model-specific units and image layout at the policy boundary.
            state = sim.state.copy()
            state[[6, 13]] = np.clip(state[[6, 13]] / 0.0475, 0.0, 1.0)
            return {
                "state": state,
                "images": {
                    name: cameras[name].read().rgb.transpose(2, 0, 1)
                    for name in ("top", "left", "right")
                },
                "prompt": config.prompt,
            }

        if config.fast_inference:
            print("Compiling and warming up policy...", flush=True)
            started = perf_counter()
            warmup_noise = np.random.default_rng(0).standard_normal(
                (policy.chunk_length, policy.action_dim), dtype=np.float32
            )
            policy.enable_fast_inference(
                compile_mode=config.fast_compile_mode,
                warmup_obs=read_observation(),
                warmup_noise=warmup_noise,
                rtc_prefix_length=PREFIX_LENGTH,
            )
            print(
                f"Fast inference ready in {perf_counter() - started:.1f}s", flush=True
            )

        rtc_manager = RTCManager(
            policy=policy,
            prefix_length=PREFIX_LENGTH,
            inference_lead_steps=LEAD_LENGTH,
            execute_chunk_dim=ACTION_DIM,
        )
        stack.callback(rtc_manager.close)

        episode_started = perf_counter()
        obs = read_observation()
        actions = policy.infer(obs)
        
        evaluation = evaluator.evaluate_qpos_batch(
            sim.data.qpos[None,:]
        )
        max_reward = evaluation.scalar_reward()

        steps = 0
        success = False

        for chunk_index in range(config.num_chunks):
            for idx, action in enumerate(actions[:ACTION_DIM]):
                if(
                    idx == RTC_START
                    and chunk_index + 1 < config.num_chunks
                ):
                    rtc_manager.start(
                        obs=read_observation(),
                        current_actions=actions,
                        noise=None,
                    )

                target = action.copy()
                target[[6,13]] = (
                    np.clip(target[[6,13]],0,1) * 0.0475
                )
                sim.step(left=target[:7], right=target[7:])
                steps += 1

                evaluation = evaluator.evaluate_qpos_batch(
                    sim.data.qpos[None, :]
                )
                max_reward = max(
                    max_reward,
                    evaluation.scalar_reward(),
                )
                if evaluation.scalar_success():
                    success = True
                    break
                view.update_from_mjdata(sim.data)

            if success:
                break
            if chunk_index + 1 == config.num_chunks:
                break

            pred = rtc_manager.get()
            if not pred[2]:
                print("prediction lag!")
            actions = pred[0]

        output_dir = ROOT / "outputs" / "simulation_evaluation"
        output_dir.mkdir(parents=True, exist_ok=True)

        episode_result = {
            "world_index": 0,
            "world_seed": config.seed,
            "success": success,
            "final_success": evaluation.scalar_success(),
            "reward": evaluation.scalar_reward(),
            "max_reward": max_reward,
            "steps": steps,
            "wall_s": perf_counter() - episode_started,
            "termination_reason": "success" if success else "timeout",
            "chunk_metrics": [],
            "final_task_eval": evaluation.to_info(squeeze=True),
        }

        build_summary(
            config=config,
            ckpt_path=checkpoint.resolve(),
            device="cuda",
            worlds=[episode_result],
            out_dir=output_dir,
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fast_inference", type=bool, default=False)
    args = parser.parse_args()

    FAST_INFERENCE = args.fast_inference
    main()
