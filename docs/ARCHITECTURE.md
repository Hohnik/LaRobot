# Architecture — how this system is shaped, and why each shape won

> **Who this is for:** anyone who needs the level between [PLAN.md](PLAN.md) (what to build and in which order) and the code itself — Julien reading on demand, a teammate lifting a module, an agent orienting before an edit. **How to use it:** read §1-§3 once (~10 minutes); §4 and §5 are lookup tables. Every claim points at its evidence in [FINDINGS.md](FINDINGS.md) rather than restating it, the same rule as the plan.
>
> ⚠️ This describes the WALKTHROUGH's architecture — the macOS prototype that proved everything once. The rebuild keeps the shapes and the seams ([PLAN.md](PLAN.md) §2-§3); it does not keep the macOS workarounds.

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
      14 motors, 2 arms                              │
                                                     ▼
      recordings/<slot>.json  ──────────────►  yam/episode.py
      (one flat 14-joint timeline)             → <slot>.mcap (C3 contract)
```

## 2. The five layers, and the one rule each is built around

**Devices — nothing is selected by position, ever.** CAN adapters by USB serial ([FINDINGS §0](FINDINGS.md) #5: enumeration order changed twice in one session), cameras by measurement or by serial through the uniqueID chain ([FINDINGS §70.15](FINDINGS.md)) with a model check on every remembered index ([FINDINGS §71.5](FINDINGS.md)), SpaceMice by a wiggle gesture because they report empty serials ([README](../README.md) §3). The rule exists because every index-based selection this project ever had eventually moved the wrong device.

**The robot stack — the clamp sits BELOW all logic.** `build_robot` wraps the vendor chain in `SafeRobot`, whose per-cycle rate clamp (`max_speed`) and command-ahead bound (`max_lag`) apply to every command from every mode, so no feature can reach around them ([FINDINGS §37.0](FINDINGS.md) is what happened when nobody knew this layer existed). `max_lag` is the quietly load-bearing one: the command may never run more than 0.25 rad ahead of the measured pose, which turns any far target into a ratchet and bounds the push on a blocked joint. Four speed limits sit in series and the smallest binds ([FINDINGS §65.0](FINDINGS.md)).

**ArmSession — the class decides, the script narrates.** Every decision that could be wrong lives in `yam/session.py` where it is testable without an arm: mode transitions, the blended park paths, the jaw pause with its settle gate and grasp check, the thermal guard, the gripper clamp and stall latch. The script (`apps/teleop_session.py`) reads keys, calls class methods, and prints. The rule was violated once — the tested park code existed while the script ran its own copy — and the audit that merged them found two real defects ([FINDINGS §52.1](FINDINGS.md), [§68.9](FINDINGS.md)).

**The loop — state advances only in arrival and completion branches.** A park hands over to a playback in the branch where the park ARRIVES; a composite leg advances where the previous leg COMPLETES; never in a key branch, never on a timer ([FINDINGS §57.1](FINDINGS.md)). Two corollaries paid for themselves: every park carries its PURPOSE so an arrival can only be credited to the thing it was actually for ([FINDINGS §72.1](FINDINGS.md) — a pose-leg arrival was once credited to a playback armed in the same event), and the playback gate MEASURES every arm against the recording's start pose before anything plays, because bookkeeping has been wrong once and measurement is cheaper than trust.

**The data lane — one flat timeline, provenance stamped twice, frames beside the file.** A recording is one 14-wide joint timeline in `--arms` order (ABC's own shape, [ROADMAP §9.2](ROADMAP.md)), labels ride inside it ([FINDINGS §70.10](FINDINGS.md)), a simulated take is stamped `sim` in two independent fields ([FINDINGS §60.2](FINDINGS.md)), and camera frames live on disk beside the file with counted drops — never silently missing ([FINDINGS §71.2](FINDINGS.md)). The episode exporter writes the C3 contract with every mapping the file cannot derive — arm sides, camera roles — as REQUIRED input, refused when absent ([FINDINGS §70.13](FINDINGS.md)).

## 3. The base ideas — the five that generated everything above

1. **This stack fails by lying, never by crashing** ([FINDINGS §0](FINDINGS.md)). Every defect that mattered produced a confident, plausible, wrong answer and raised nothing. So: check values for plausibility, prefer the test that could falsify the claim, and measure the consequence rather than the mechanism.
2. **Refuse, never warn-and-continue** ([HANDOFF §4](HANDOFF.md) rule 4). The one warn-and-continue this project shipped burned a motor. A refusal costs a retry; a warning costs whatever the hazard was.
3. **A written number is a cache with no invalidation** ([FINDINGS §33.3](FINDINGS.md)). Live facts live in commands (`check_rig`, `check_recordings`, `check_arms_match`), documents point at the command, and every document that broke this rule went stale within days — measured seven times.
4. **Every checker gets a falsifier** ([FINDINGS §70.8](FINDINGS.md)). A green run plus a stable catch-count is evidence; a green run alone went blind three times without anyone noticing. The suite total itself is the catch-counter (746 today; a DROP while green means a check was disarmed).
5. **The operator's hardware time is the critical path** ([HANDOFF §4](HANDOFF.md) rule 11). Everything headless is proven headless first (unit tests, then the lagging simulator driving the whole loop — it caught what 616 unit tests could not, [FINDINGS §62.1](FINDINGS.md)); the bench gets only what the bench alone can answer.

## 4. One demonstration, end to end — what actually happens

1. `teleop_session --yes --arms B,G --cameras c920,d405:2603,d405:2553` opens three cameras (identity chain + model check), wiggle-assigns the puck, builds both arms — nothing has moved.
2. `w` starts a take: every cycle appends 14 measured joints, and each camera's fresh frames go to its writer thread (JPEG + index, drops counted). The operator drives (`t`), hand-guides (`g`), or lets a waypoint run execute (`p 1 2 3` Enter) — recording rides through all of it, and `k` marks a bad stretch into the file.
3. `w` stops it on the same line the sampler stops ([FINDINGS §30.1](FINDINGS.md)); a digit saves `recordings/<slot>.json` + `frames/<slot>/`, with commit, timestamp, modes, and camera counts stamped in.
4. `checks/check_recordings.py` re-counts everything from disk — padding, labels, frames, provenance — and flags what disagrees.
5. `export_episode --slot 3 --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773` writes the C3 MCAP: eight vector topics plus three camera topics, every stream on the 33,333,333 ns tick, actions as next-tick states, all mappings auditable in `/episode-meta`. The C4 gate (ABC's loader) is the one verification this bench cannot run.

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
| episodes | `yam/episode.py` · `apps/export_episode.py` | `test_episode` |
| the loop itself | `apps/teleop_session.py` (script only — decisions live above) | `checks/drive_sim_session.py` (the whole loop, simulated) |
| the simulator | `yam/fake/arm.py` (lags like the measured law) | `test_fake_arm` · `checks/falsify_fake_arm.py` |
| truth maintenance | `checks/check_*.py` + their `falsify_*.py` | `checks/run_tests.py` — one command, one total |

*Written 2026-08-19. When a shape here changes, change this file in the same commit — it is a map, and [FINDINGS §33.3](FINDINGS.md) says what happens to stale maps.*
