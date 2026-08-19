# Architecture — how this system is shaped, and why each shape won

> **Who this is for:** anyone who needs the level between [PLAN.md](PLAN.md) (what to build, in which order) and the code itself. That covers Julien reading on demand, a teammate lifting a module, and an agent orienting before an edit. **How to use it:** read §1 to §3 once, about ten minutes. §4 and §5 are lookup tables. Every claim links to its evidence in [FINDINGS.md](FINDINGS.md) instead of restating it, the same rule the plan follows.
>
> Note that this describes the walkthrough: the macOS prototype that proved everything once. The rebuild keeps the shapes and the seams ([PLAN.md](PLAN.md) §2 and §3). It does not keep the macOS workarounds.

## 1. The system in one picture

```
  SpaceMouse ─┐                                              ┌─ cameras (C920 + 2× D405)
  keyboard  ──┤  command sources                             │   one reader thread each
  (policy)  ──┘  (one seam: ROADMAP §10.6)                   │   newest frame wins
        │                                                    ▼
        ▼                                              CaptureSet.sample()
  ┌──────────────────── the 100 Hz session loop ────────────────────┐
  │  teleop_session.py — the SCRIPT: reads inputs, narrates, routes │
  │                                                                 │
  │   ArmSession (one per arm) — the CLASS: decides everything      │
  │   modes · park paths · jaw pause · settle gates · thermal guard │
  └──────────┬──────────────────────────────────────┬───────────────┘
             ▼ joint targets                        ▼ fresh frames, while recording
      SafeRobot (rate + lag clamp,            FrameSink → one writer thread
      BELOW all logic, FINDINGS §37.0)          per camera → JPEGs + index
             ▼                                       │
      gs_usb CAN (adapters by serial)                ▼
             ▼                              recordings/frames/<slot>/
      14 motors, 2 arms
                                                     │
      recordings/<slot>.json  ──────────────►  yam/episode.py  → <slot>.mcap   (C3 log)
      (one flat 14-joint timeline)     │
                                       └────►  yam/dataset.py → episode_<id>/  (C4 training set)
                                                 states_actions.bin · stacked mp4 · metadata
```

⭐ **Every device name in that picture is resolved through one module, `yam/platform.py`** — the only place in the repo that branches on the operating system. On macOS the CAN adapter is a gs_usb index verified by serial and the cameras need the uniqueID chain; on Linux the adapter is a SocketCAN interface named from its serial in sysfs and the cameras come from `/dev/v4l/by-id`. Nothing else in the diagram knows which machine it is on.

## 2. The five layers, and the one rule each is built around

**Devices: nothing is selected by position, ever.** CAN adapters are selected by USB serial, because the enumeration order changed twice in one session ([FINDINGS §0](FINDINGS.md) #5). Cameras are selected by measurement, or by serial through the uniqueID chain ([FINDINGS §70.15](FINDINGS.md)), and every remembered camera index is model-checked before it is trusted ([FINDINGS §71.5](FINDINGS.md)). SpaceMice are assigned by a wiggle gesture, because they report empty serials ([README](../README.md) §3). The rule exists because every index-based selection this project ever had eventually moved the wrong device.

**The robot stack: the clamp sits below all logic.** `build_robot` wraps the vendor chain in `SafeRobot`. Its per-cycle rate clamp (`max_speed`) and its command-ahead bound (`max_lag`) apply to every command from every mode, so no feature can reach around them. [FINDINGS §37.0](FINDINGS.md) is what happened when nobody knew this layer existed. `max_lag` matters more than it looks: the command may never run more than 0.25 rad ahead of the measured pose. That turns any far target into a ratchet, and it bounds how hard a blocked joint gets pushed. Four speed limits sit in series and the smallest one binds ([FINDINGS §65.0](FINDINGS.md)).

**ArmSession: the class decides, the script narrates.** Every decision that could be wrong lives in `yam/session.py`, where it is testable without an arm: mode transitions, the blended park paths, the jaw pause with its settle gate and grasp check, the thermal guard, the gripper clamp and its stall latch. The script, `apps/teleop_session.py`, reads keys, calls class methods and prints. This rule was violated once. The tested park code existed while the script ran its own untested copy, and the audit that merged them found two real defects ([FINDINGS §52.1](FINDINGS.md), [§68.9](FINDINGS.md)).

**The loop: state advances only in arrival and completion branches.** A park hands over to a playback in the branch where the park arrives. A composite leg advances where the previous leg completes. Never in a key branch, never on a timer ([FINDINGS §57.1](FINDINGS.md)). Two additions to that rule paid for themselves. Every park carries its purpose, so an arrival can only be credited to the thing it was actually for — [FINDINGS §72.1](FINDINGS.md) is the day a waypoint leg's arrival got credited to a playback armed in the same event. And the playback gate measures every arm against the recording's start pose before anything plays, because the bookkeeping has been wrong once and measuring is cheaper than trusting.

**The data lane: one flat timeline, provenance stamped twice, frames beside the file.** A recording is one 14-wide joint timeline in `--arms` order, which is ABC's own shape ([ROADMAP §9.2](ROADMAP.md)). Labels ride inside the file ([FINDINGS §70.10](FINDINGS.md)). A simulated recording is stamped as simulated in two independent fields ([FINDINGS §60.2](FINDINGS.md)). Camera frames live on disk beside the file, with dropped frames counted rather than silently missing ([FINDINGS §71.2](FINDINGS.md)). The episode exporter writes the C3 contract, and every mapping the file cannot derive on its own — which arm stands left, which camera looks from where — is required input, refused when absent ([FINDINGS §70.13](FINDINGS.md)). **Two exports leave from the same rows**: `yam/episode.py` writes the C3 log and `yam/dataset.py` writes the C4 training directory, both fed by one `state_action_rows()` so they can never describe a demonstration differently. The C4 video's encoding is strict on purpose, because the trainer computes where frame k is rather than decoding to find it, and `checks/check_dataset.py` re-reads the finished file to prove each property landed ([FINDINGS §74.1](FINDINGS.md)).

## 3. The base ideas — the five that generated everything above

1. **This stack fails by lying, never by crashing** ([FINDINGS §0](FINDINGS.md)). Every defect that mattered produced a confident, plausible, wrong answer and raised nothing. So: check values for plausibility, prefer the test that could falsify a claim, and measure the consequence rather than the mechanism.
2. **Refuse, never warn-and-continue** ([HANDOFF §4](HANDOFF.md) rule 4). The one warn-and-continue this project shipped burned a motor. A refusal costs a retry. A warning costs whatever the hazard was.
3. **A written number is a cache with no invalidation** ([FINDINGS §33.3](FINDINGS.md)). Live facts live in commands (`check_rig`, `check_recordings`, `check_arms_match`). Documents point at the command. Every document that broke this rule went stale within days, measured seven times.
4. **Every checker gets a falsifier** ([FINDINGS §70.8](FINDINGS.md)). A green run plus a stable catch count is evidence. A green run alone went blind three times without anyone noticing. The suite total is the catch counter for the suite itself: a total that drops while staying green means a check was disarmed.
5. **The operator's hardware time is the critical path** ([HANDOFF §4](HANDOFF.md) rule 11). Everything that can be proven without an arm is proven that way first: unit tests, then the lagging simulator driving the whole loop, which caught what 616 unit tests could not ([FINDINGS §62.1](FINDINGS.md)). The bench gets only the questions the bench alone can answer.

## 4. One demonstration, end to end — what actually happens

1. `teleop_session --yes --arms B,G --cameras c920,d405:2603,d405:2553` opens three cameras (identity chain plus model check), assigns the puck by wiggle, and builds both arms. Nothing has moved yet.
2. `w` starts a recording. Every cycle appends 14 measured joint values, and each camera's fresh frames go to its own writer thread (JPEG plus an index file, drops counted). The operator drives (`t`), hand-guides (`g`), or lets a waypoint run execute (`p 1 2 3` Enter). The recording rides through all of it, and `k` marks a bad stretch into the file.
3. `w` stops the recording on the same line the sampler stops ([FINDINGS §30.1](FINDINGS.md)). A digit saves `recordings/<slot>.json` plus `frames/<slot>/`, with the commit, the timestamp, the modes and the camera counts stamped in.
4. `checks/check_recordings.py` re-counts everything from disk — padding, labels, frames, provenance — and flags whatever disagrees.
5. `export_episode --slot 3 --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773` writes the C3 MCAP file: eight vector topics plus three camera topics, every stream on the 33,333,333 ns tick, actions as next-tick states, and every mapping auditable in the episode's own `/episode-meta`.
6. `export_dataset` with the same flags writes the C4 training directory instead, and `checks/check_dataset.py` re-reads it: the table's shape and finiteness, the action policy in the bytes, and every property of the video the loader depends on. The C4 gate (ABC's own loader) is the one verification this bench cannot run.

## 5. Where everything lives

| layer | module(s) | its tests |
|---|---|---|
| CAN + robot build | `yam/robot.py` · `yam/can.py` | `test_motor_faults` · `checks/check_rig.py` |
| safety envelope | `SafeRobot` (in `yam/robot.py`) · `yam/session.py` guards · `yam/collision.py` | `test_thermal_guard` · `test_jaw_block` · `test_collision` |
| arm behaviour | `yam/session.py` (`ArmSession`, park, jaw pause) · `yam/motion.py` | `test_arm_session` · `test_jaw_pause` · `test_motion` |
| inputs | `yam/inputs/` (spacemouse, keyboard, axis maps) | `test_puck_assignment` · `test_axis_map` · `test_keyboard` |
| teleop math | `yam/teleop.py` (IK, frames, workspace) | `test_teleop_ik` |
| recording + playback | `yam/recording.py` (`Trajectory`, labels, replay, scrub) | `test_recording` · `test_scrub` · `test_save_slot` |
| cameras | `yam/cameras/` (frame · grabber · capture · identity · specs · writer) | `test_capture` · `test_camera_identity` · `test_frame_writer` |
| episodes (C3 log) | `yam/episode.py` · `apps/export_episode.py` | `test_episode` |
| training sets (C4) | `yam/dataset.py` · `apps/export_dataset.py` | `test_dataset` · `checks/check_dataset.py` + `falsify_check_dataset.py` |
| what machine is this | `yam/platform.py` (CAN links, v4l cameras, clocks, permissions) | `test_platform` · `checks/check_platform.py` |
| the loop itself | `apps/teleop_session.py` (script only — decisions live above) | `checks/drive_sim_session.py` (the whole loop, simulated) |
| the simulator | `yam/fake/arm.py` (lags like the measured law) | `test_fake_arm` · `checks/falsify_fake_arm.py` |
| truth maintenance | `checks/check_*.py` and their `falsify_*.py` | `checks/run_tests.py` — one command, one total |

*Written 2026-08-19. When a shape here changes, change this file in the same commit. It is a map, and [FINDINGS §33.3](FINDINGS.md) says what happens to stale maps.*
