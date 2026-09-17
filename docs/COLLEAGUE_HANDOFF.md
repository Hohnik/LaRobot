# Teleop reference: colleague handoff

## Ready for review

The cleanup branch is ready for a source review. Both arms, shared-puck selection,
parking, D405 recording and replay have passed the attended checks described in
[STATION_VALIDATION](STATION_VALIDATION.md). Camera framing still needs physical
setup before useful demonstrations can be collected.

This is a reference for building the team's own implementation. Start with the
small owners and examples below, then read their integration in the operator.
[BRIDGE](BRIDGE.md) compares all six current LaRobot branch tips, including the
September 16 physical and ABC pushes. It records concrete compatibility problems
and a proposed implementation order.

The public review branch is
[`codex/teleop-reference-cleanup`](https://github.com/Hohnik/LaRobot/tree/codex/teleop-reference-cleanup),
based on Fable's `499d0b7` and the locally verified cleanup at `ee8421a`.
Julien authorized this separate branch on September 17. Existing team branches
and station working copies remain unchanged. The package manifest identifies
the exact archived revision and its checksums.

## Two small examples to run first

Use the environment setup in [README](../README.md). From the repository root:

```sh
.venv-teleop/bin/python examples/command_limits.py
.venv-teleop/bin/python examples/record_take.py
```

| Example | What to read and observe |
| --- | --- |
| [command_limits.py](../examples/command_limits.py) | Real SafeRobot and SessionResources around a fake arm. A 1 rad request sends 0.020 rad first, then stops advancing at a 0.040 rad measured-lag limit. All seven fake motor-off calls complete. |
| [record_take.py](../examples/record_take.py) | Real recording owner, JPEG writers and publication. Six synthetic samples and six images save and reload. Sampling after freeze leaves the take at 0.5 s. |

Neither example opens hardware. The recording example prints a new temporary
output directory and retains it for inspection. It never writes user recording
slots. Its wait loop is for a console demonstration; the live operator polls
completion while continuing its control loop.

The examples demonstrate existing components. They do not supply a complete
physical controller, parking policy or recovery guarantee.

## Read by feature

Each row names the implementation and tests that explain its contract.
The tests also document failure behavior that a short happy-path example omits.

| Question | Implementation | Checks to study |
| --- | --- | --- |
| Who owns devices when startup fails? | [session_resources.py](../src/yam/session_resources.py), [robot.py](../src/yam/robot.py) | `test_session_resources.py`, `test_robot_startup.py`, `test_session_lifecycle.py` |
| Where do every mode's command limits apply? | `robot.py:SafeRobot`, [session_health.py](../src/yam/session_health.py) | `test_fake_arm.py`, `test_vel_ff.py`, `test_session_health.py` |
| How does a puck become a joint target? | [session_input.py](../src/yam/session_input.py), [axis_map.py](../src/yam/inputs/axis_map.py), [teleop.py](../src/yam/teleop.py) | `test_session_input.py`, `test_axis_map.py`, `test_teleop_ik.py` |
| What changes when entering HOLD, GUIDE or TELEOP? | [session.py](../src/yam/session.py) | `test_arm_session.py`, `test_jaw_block.py` |
| How do waypoints, settling and jaw pauses work? | `session.py`, [motion.py](../src/yam/motion.py) | `test_motion.py`, `test_jaw_pause.py`, `test_park_target.py` |
| How do recording, labels and failed saves work? | [recording_session.py](../src/yam/recording_session.py), [recording_store.py](../src/yam/recording_store.py) | `test_recording_session.py`, `test_recording_completion.py`, `test_recording_store.py` |
| How do cameras stay independent of the control loop? | [cameras/session.py](../src/yam/cameras/session.py), [writer.py](../src/yam/cameras/writer.py) | `test_camera_startup.py`, `test_camera_boundaries.py`, `test_frame_writer.py` |
| How do replay, scrubbing and composites share time? | [playback_session.py](../src/yam/playback_session.py), [composite.py](../src/yam/composite.py), [recording.py](../src/yam/recording.py) | `test_playback_session.py`, `test_scrub.py`, `test_composite.py` |
| How does mirror mode coordinate two arms? | [mirror_session.py](../src/yam/mirror_session.py), [mirror.py](../src/yam/mirror.py) | `test_mirror_session.py`, `test_mirror.py` |
| What happens on quit, interruption or a fault? | [lifecycle.py](../src/yam/lifecycle.py), `session_resources.py` | `test_controlled_stop.py`, `test_session_lifecycle.py` |
| How are episodes exported and checked? | [episode.py](../src/yam/episode.py), [dataset.py](../src/yam/dataset.py), [recording_recovery.py](../src/yam/recording_recovery.py) | `test_episode.py`, `test_dataset.py`, `test_recording_recovery.py` |

Tests are under [tests](../tests/). For example:

```sh
.venv-teleop/bin/python tests/test_session_resources.py
.venv-teleop/bin/python checks/run_tests.py
```

## How to use the structure in LaRobot

Keep the team's typed inputs, separate Cartesian target/IK objects and MuJoCo
simulation. Adopt one feature at a time with an explicit owner and small tests.
The first useful changes are physical startup/cleanup and input-call compatibility.
Then agree data contracts and prove one recorder-to-loader example.

The current recording branch's MCAP does not load through the current training
converter. Two recorded synthetic samples produced four valid messages and zero
converted frames in the isolated probe. [BRIDGE section 3](BRIDGE.md#3-confirmed-issues-to-resolve-before-integration)
explains the topic, encoding and path differences. ABC dependency installation
and policy visualization do not resolve that mismatch.

Separate reusable mechanisms from rig policy. Queue ownership, timeline handling
and save rollback are useful implementation patterns. Serial mappings, jaw
calibration, waypoint coordinates, speed/thermal thresholds and stop behavior
require explicit agreement for the target rig. Input, gripper, image and timestamp
units also differ across the projects.

The reference's terminal prompts and historical documents need not become part
of the team's application. Use the tests and current contracts to understand a
component before adapting it to the team's style.

## Why the operator remains large

[teleop_session.py](../apps/teleop_session.py) is 1,973 lines. It still assembles
the owners and coordinates startup, key precedence, per-arm motion commands,
shared arrival gates and stop handling. Read it after the corresponding owner.
[RESTRUCTURING](RESTRUCTURING.md) explains the retained ordering requirements.

This is a substantial integration program, and its long main function remains a
maintenance cost. Moving that function into another large class would leave the
same coupling. A further runtime redesign should first define a cycle's inputs,
measurements, commands and failure results, then preserve those semantics in tests.
That redesign is unnecessary for studying or adapting individual features now.

This pass reduced the solver from 529 to 308 lines and ArmSession from 755 to 692.
Historical narratives were preserved in an explicitly labeled
[source-note archive](archive/ik-and-mode-source-notes.md). Current comments now
separate model estimates from encoder measurements and describe actual frame
contracts. The existing executable statements remain unchanged.

## Verification and installation boundary

The motor-control application physically tested on Linux is `025fab1`.
Later changes repair a dataset falsifier, clarify help/checker wording, correct
source explanations and add documentation/examples. They preserve its control
algorithms, calibration, command limits and stop policy.

The complete validation record is in [CLEANUP](CLEANUP.md) and
[STATION_VALIDATION](STATION_VALIDATION.md). The attended take contains 1,166 joint
samples and 357 readable D405 images. Its 1.00x replay completed into HOLD.
The images verify capture/save behavior but show an unsuitable upward view.
The full take and logs are retained outside `/tmp` on the station.

The test operator is separate from `/home/lavita/LaRobot` and the original
`/home/lavita/yam-robotics`. The latest team snapshot is clean `feature/physical`
at `fd2c64b`; its remote tip matched when checked. See
[STATION_COMMANDS](STATION_COMMANDS.md) for the folder and connection map.

Physical checks used a copy of the station's configuration, including its newer
uncommitted park poses. The source archive contains committed configuration.
Preserve and compare the station's waypoints before installation.

The archive excludes recordings, Python environments and the I2RT checkout.
Follow README for pinned dependencies. The incremental Git bundle requires
Fable's `499d0b7` history. Public source review is separate from installation and
integration into the teammate's working checkout. Preserve its configuration and
coordinate any later physical-device use.
