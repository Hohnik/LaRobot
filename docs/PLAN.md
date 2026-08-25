# The rebuild plan — what to build, what we proved, what will bite you

> **Who this is for:** the team rebuilding the bimanual YAM station from scratch. Julien's ruling ([FINDINGS §67.0](FINDINGS.md)): this repo is the finished walkthrough, you build the real one. "The walkthrough" means exactly that — every feature was built once here, proven on the real arms, and written up, so that your build starts from answers instead of from the same surprises.
>
> **How to use it:** read this file once, top to bottom. It takes about twenty minutes. It follows the phases A to E from your own setup guide, [Setup-Anleitung.md](Setup-Anleitung.md). For each phase it says three things: what the walkthrough proved, where reality disagreed with the guide, and the mistake you are most likely to make first. Work packages you can assign are in §3. Every claim links to its evidence in [FINDINGS.md](FINDINGS.md) instead of copying it, so this file stays short and the evidence stays in one place. One level below this plan is [ARCHITECTURE.md](ARCHITECTURE.md): how the walkthrough's code is shaped, the five rules behind the shapes, and where every module and its tests live.
>
> **Status:** ratified by Julien on 2026-08-19. This is the team's deliverable. The inputs that are still open (the task, the model choice, the C4 check) are listed in §4 as decisions, not blockers. Everything this plan describes exists and ran on the physical arms, including episodes with camera images ([FINDINGS §72.0](FINDINGS.md)) and the automated collection loop ([FINDINGS §73.0](FINDINGS.md)). Since 2026-08-19 the walkthrough also carries a **Linux device layer** and a **C4 training-set writer with its own verifier**, both described where they belong below.

## 0. The one-paragraph summary

The walkthrough proved the whole loop on macOS with two YAM arms. That covers nine things:

- **SpaceMouse teleop at about 85 passes a second on the Mac.** The puck steers the gripper through space, and inverse kinematics works out the joint angles that put it there. The loop asks for 100. ⭐ On your Linux station the same loop reaches about 97, because the shortfall is macOS waking a sleep late rather than any work in the loop ([PERFORMANCE.md](PERFORMANCE.md) section 2).
- **Hand-guiding.** The arm goes weightless under gravity compensation and you move it with your hands.
- **Waypoint runs that can grab.** The run pauses at a leg where only the jaws move, and it reports whether anything was actually gripped.
- **Recording and replaying hand-taught movements.**
- **Composite runs.** Waypoints and recorded movements chained in one sequence.
- **Mirror mode.** One arm follows the other, joint for joint.
- **Marking good and bad stretches while driving.**
- **A simulator that lags like the real hardware** and runs the whole session with nothing attached.
- **An exporter** that turns a recording into an MCAP episode, in exactly the topic shape of [Setup-Anleitung.md](Setup-Anleitung.md) C3. MCAP is the log-file format your guide specifies. The collection loop has since run end to end on the real arms with all three cameras: teach waypoints, run them, and the run records itself with images, which then export as a training episode ([FINDINGS §73.0](FINDINGS.md)). **One thing does not exist anywhere yet: an episode verified against ABC's own loader** (ABC is the training stack the episodes feed). Both halves of the format are written and self-checked, C3 as the MCAP log and C4 as the training directory ([FINDINGS §74.1](FINDINGS.md)), so what remains at that gate is the one question only your loader can answer. The most important thing we learned is not a feature. This stack fails by lying, never by crashing ([FINDINGS §0](FINDINGS.md)). §5 explains the defences that worked, because you will need the same ones.

## 1. What "done" means for the rebuild

1. Both arms driven from one process. Demonstrations collected as MCAP episodes that ABC's loader accepts unchanged (Gate C in your guide). A policy deployed through one adapter interface (Phase E).
2. Every safety property this repo established still holds: dry-run by default, park-then-disable on every exit, thermal guards that refuse instead of warning and carrying on, and one rate-and-lag clamp below all control logic.
3. The verification discipline of §5 is in place from day one. Adopting it is cheaper than rediscovering it. The first hardware day alone produced nine defects that were confident, wrong and silent ([FINDINGS §0](FINDINGS.md)).

## 2. The phase map — your guide's plan against what actually happened

### Phase A (OS, drivers, ABC repo)

This phase went differently here, on purpose: everything ran on macOS, which the vendor SDK does not support. The fix was one argument, not a redesign: `bustype="gs_usb"` instead of libusb ([HISTORY §2.1](HISTORY.md)).

For your rebuild on Ubuntu, Phase A applies as your guide writes it, and none of the macOS workarounds apply to you. One thing does apply: the [README](../README.md) bring-up checklist. It contains four things, all of them true of the hardware whatever operating system you run:
>
> - the motor LED table
> - the order for reading fault codes
> - which recovery needs a USB replug, and which needs a mains power cycle
> - the gripper's per-session encoder shift

### Phase B (hardware and physical setup) — mostly proven, three corrections

**CAN and arms (B2): proven beyond the guide.** Two arms, two buses, one process at about 85 passes a second on the Mac and about 97 on Linux, with roughly three times the timing headroom needed. Two rules from hard experience:

- Select adapters by serial number, never by index. The enumeration order changed twice in one session, and selecting by index would have moved the wrong arm ([FINDINGS §0](FINDINGS.md) #5).
- Put the CAN adapters on a powered USB 3 hub, not behind a dock. The one hard crash this rig had was the whole USB bus sagging away mid-session: seven motors latched fault code `0xD`, and both adapters fell into their bootloaders ([FINDINGS §46.0](FINDINGS.md)). Budget for that hub.

**Cameras (B3): two corrections to the guide.**

- The D405 over plain UVC (the ordinary webcam protocol) gives you colour, and on Linux it also publishes a real 16-bit depth node that OpenCV cannot open. Measured both ways: from the pixels on macOS ([FINDINGS §63.0](FINDINGS.md)) and from the V4L2 formats on Ubuntu ([FINDINGS §75.7](FINDINGS.md)). So install librealsense from the start if you want depth, and expect nothing from OpenCV on that node.
- ⛔ **A D405 publishes three capture nodes on Linux and the FIRST one is depth.** `/dev/video2` is `Z16` depth, `/dev/video4` is infrared, `/dev/video6` is colour. Pick the node by reading its pixel formats, never by taking the first: opening depth and storing it as colour is a dataset that trains wrongly and raises nothing. Watch the ordering too, because the infrared node also offers `UYVY` ([FINDINGS §75.7](FINDINGS.md)).
- ⛔ **Ask for MJPG, and ask for it BEFORE you ask for the size.** A C920 at 1280x720 in uncompressed YUYV is capped at **10.01 fps by its own firmware**; in MJPG the same camera does **29.92 fps**. YUYV is the format it advertises first, so it wins by default. Measured on Ubuntu 2026-08-19 ([FINDINGS §76.1](FINDINGS.md)). ⚠️ macOS ignores the request and chooses well by itself, which is exactly why this went unnoticed until Linux: the code that skipped the codec request looked fine on the machine it was written on.
- ⛔ **A camera can accept a size and then deliver nothing at it, with no error anywhere.** The D405's colour node advertises 1280x720 at 30 fps, accepts it, reports it back, and on Ubuntu it delivered **zero frames**. At 848x480 and below the same node streams at up to 89.88 fps. `v4l2-ctl --stream-mmap` got nothing either, so this sits below OpenCV; the USB link is 5 Gbit/s, so it is not bandwidth. ⚠️ **And it is INTERMITTENT rather than dead**: the same mode was measured dead twice and alive twice within four hours ([FINDINGS §76.2](FINDINGS.md)). ⭐ **On macOS the pattern is inverse**: 1280x720 works there and 848x480 is broken. **A camera's usable modes belong to the camera plus the platform, and they can come and go.** So: read a real frame at open, take the size from that frame, and step the size down until one delivers. Never trust `cap.get` after `cap.set`, because it reports the request. ⚠️ Nothing needs 1280x720 from a wrist camera anyway, because the C4 export shrinks every view to 224x224.
- ⛔⛔ THE ROOM CAN HALVE YOUR FRAME RATE, and the driver will keep reporting 30. V4L2 has a control called `exposure_dynamic_framerate`. When it is on, the camera may lengthen its exposure by slowing itself down. Measured on one C920, same size and format, hours apart: 29.92 fps in daylight and 14.98 fps in a dim room. Its kernel default is 0 and the station's camera reads 1 ([FINDINGS §76.16](FINDINGS.md)). ⛔ This is worse for a dataset than a consistently slow camera. Two demonstrations of one task, recorded morning and evening, contain twice as many images as each other, and nothing in the file records why. That is a hidden variable in your training set. ⭐ Read the control at camera open and say so. Whether to turn it off is a real trade: a steady rate costs you darker pictures. `v4l2-ctl -d /dev/videoN --set-ctrl=exposure_dynamic_framerate=0`.
- ⭐⭐ librealsense is a pip wheel rather than an apt install, and it needs no root at all. `uv run --with pyrealsense2` sees the camera in seconds with nothing compiled and nothing installed system-wide. Measured on Ubuntu 24.04: colour at 1280x720 at a steady 30.0 fps where plain V4L2 is intermittent, plus depth (z16) up to 1280x720 streaming alongside it ([FINDINGS §76.10](FINDINGS.md)). ⛔ If you use it, key camera identity on `asic_serial_number` and never on `serial_number`. The same D405 reports `260522273162` for one and `260323072846` for the other, and the USB descriptor, `/dev/v4l/by-id` and this walkthrough's recordings all use the ASIC one. Getting that wrong renames every camera in your dataset and reads on screen as "camera not attached" ([FINDINGS §76.11](FINDINGS.md)).
- The C920 reports an empty USB serial ([FINDINGS §70.6](FINDINGS.md)), so a config that keys every camera by serial fails for it. Key the D405s by serial and the C920 by model name. Two identical D405s cannot be told apart by anything they capture. The identification chain that solves this without touching a lens: a USB camera's AVFoundation uniqueID is its location ID, vendor ID and product ID packed into one number, so serial to uniqueID resolves with no root access ([FINDINGS §70.15](FINDINGS.md), `yam/cameras/identity.py`). Only uniqueID to OpenCV index still needs one confirmation per port arrangement. On Ubuntu the whole question dissolves, because librealsense reads serials directly.

**SpaceMice (B4): one correction.** They also report empty serials. Identity comes from a wiggle gesture at session start: the session asks you to move the puck you want to assign ([README](../README.md)). One puck can drive a whole multi-arm session, because the arm selection also aims the puck ([FINDINGS §68.8](FINDINGS.md)).

**There is no e-stop on this hardware.** Wall power is the only hard cut. That fact shaped every motion feature here — slow, bounded, interruptible, dry-run by default — and it must shape yours.

### Phase C (teleop and recording) — the heart of the walkthrough, and where the traps live

**IK chain (C1).** Proven with mink and MuJoCo inside that loop. Two IK solves cost 0.1 ms against a 10 ms budget. The guide's advice stands: use ABC's `yam.xml` as the robot model. One lesson worth reading even if nothing is broken: a pure rotation once dragged the tool point 44 cm sideways, and the fix explains how the solver thinks ([FINDINGS §18](FINDINGS.md)).

**Robot loop (C2).** The shape that worked: the class decides, the script narrates. One `ArmSession` object per arm holds every decision; the loop reads inputs, calls methods and prints ([ROADMAP §9.5](ROADMAP.md)). On top of that, adopt your own policy-as-an-input idea: everything that produces commands (puck, keyboard, policy) is reached through one interface.

⛔ Here is the state of that interface on 2026-08-20. This plan used to claim it was finished on both sides, and it was finished on neither. Your `inputs/` directory names `policy.py` and `mcap_recording.py` beside `spacemouse.py`, which is the design decision that matters: a trained policy and a replayed recording are both command sources. Your `Input` base class then declares one method, `is_available()`, and says nothing about how a command is read. This repo had the opposite problem: a working `TwistReader.read()` returning six numbers, and no declared interface at all until 2026-08-20.

⭐ So the two halves fit together, and neither side had both. The reading half is now declared here as `src/yam/seams.py::CommandSource`, one method, and `tests/test_seams.py` checks that the working class still satisfies it. What is left to agree is whether that method signature is the one you want. Once it is agreed, a policy driving the arms is a small piece of work rather than a design question. [BRIDGE.md](BRIDGE.md) section 5 is the whole comparison.

**The speed model, measured once so nobody re-measures it wrong.**

- The arm follows a moving target with a fixed delay: lag ≈ 0.04-0.10 rad plus 0.033 s times the speed, the same on all six joints ([ROADMAP §7.5.1](ROADMAP.md)).
- Four speed limits sit in series and the smallest one binds ([FINDINGS §65.0](FINDINGS.md)). If raising a limit changes nothing, you raised the wrong one.
- Velocity feedforward works and is capped at gain 1. Above 1 the velocity setpoint contradicts the position command by construction ([FINDINGS §68.6](FINDINGS.md)).
- The 1-2 cm repeatability floor is static friction. The arm settles short of its target by 0.02-0.04 rad in a direction that does not repeat. Teleop only feels exact because your eyes close the loop ([FINDINGS §69.2](FINDINGS.md)). If a task needs sub-centimetre repeatability, the one real lever is kp, the position gain, and it has caveats ([ROADMAP §8.2](ROADMAP.md) item 17).

**Recorder (C3): built and confirmed.** Every arm records into one timeline (ABC's 14-value shape). Provenance is stamped in two independent fields, so a simulated recording can never pass as real ([FINDINGS §60.2](FINDINGS.md)). Labels are marked while driving ([FINDINGS §70.10](FINDINGS.md)), grabs run through the jaw pause ([FINDINGS §70.5](FINDINGS.md)), and composite runs chain waypoints and recordings ([FINDINGS §70.12](FINDINGS.md)). The collection method is Julien's ruling after driving both: most demonstrations by hand-guiding and replay, with the SpaceMouse kept for corrections. Executing a task with the puck is hard; resetting the scene is trivial ([ROADMAP §6.6](ROADMAP.md)).

**Export (C3/C4): the exporter writes the contract as written.** Exact topic names and dimensions, the 33,333,333 ns tick, joint-space actions, and the arm sides never defaulted ([FINDINGS §70.13](FINDINGS.md)). Camera frames use the same ticks. The session saves JPEGs beside every recording, and dropped frames are counted rather than silent. The exporter joins them to the ticks by nearest timestamp. The camera-to-position mapping is as mandatory as the sides ([FINDINGS §71.2](FINDINGS.md)). This ran on the arms on 2026-08-19: camera-carrying recordings at 30 fps with zero drops, one exported as an episode with images, and all three cameras held 30 fps together on one USB tree ([FINDINGS §71.4](FINDINGS.md)). Two things stand between that and Gate C. First, a three-camera run inside a live session (the capture side is proven; the cost of three JPEG encoders inside the loop is the one open number). Second, C4 itself: convert a mini-sample and verify it against ABC's loader before collecting anything you intend to keep. The guide's warning is right — encoding mistakes are only repairable by recollecting. One trap we hit so you do not have to ([FINDINGS §71.5](FINDINGS.md)): a stale camera-index cache pointed both wrist cameras at the webcam, silently. Verify the model of a cache-resolved camera at open, every time.

### Phase D (training) and Phase E (deployment)

The walkthrough did not touch them, on purpose ([ROADMAP: deliberately-not-doing](ROADMAP.md)). Two things from here matter for Phase E. The policy adapter should be the same input interface the teleop uses — one seam, and your repo already sketches it. And the deploy loop inherits the full safety envelope of §1 point 2, unchanged. A policy is just another command source. It gets the same clamps a human gets.

## 3. The work packages

Each package names its deliverable, what you can copy from this repo, what you rebuild, and the mistake you are most likely to make first. "Lift" means the logic and its tests translate nearly verbatim: plain Python on numpy, no macOS dependence unless the row says so.

| WP | Deliverable | Lift from here | The trap |
|---|---|---|---|
| 1 | CAN/robot layer: `build_robot`, `SafeRobot` (rate and lag clamps below all logic), teardown order | `yam/robot.py`, `yam/can.py` concepts; on Ubuntu the SDK's SocketCAN path replaces the gs_usb workaround | the vendor's own `close()` gets the teardown order wrong, and a raised arm sags if you trust it ([HISTORY §5](HISTORY.md)) |
| 2 | Safety envelope: thermal guard, gripper clamp and stall latch, workspace sphere, floor, incident recorder | `ThermalGuard`, `hold_jaw`, `check_grasp`, the incident writer — all pure functions, all tested | the jaws cook motor 7 if any path bypasses the clamp; ask of every guard, "what path reaches the hazard without passing through you?" ([HANDOFF §4](HANDOFF.md) rule 7) |
| 3 | Inputs: SpaceMouse reader, axis maps per arm and frame, keyboard, policy adapter — all behind one interface | `yam/inputs/*` plus your own `inputs/` skeleton | empty USB serials; the wiggle assignment; a dead puck must read as centred and park the arm, never drop it ([FINDINGS §68.2](FINDINGS.md)) |
| 4 | Session and modes: one `ArmSession` per arm, park machinery (blended paths, jaw pause, settle gate), the mode keys | `yam/session.py` and its tests; the lesson of [FINDINGS §52.1](FINDINGS.md): the tested code must be the code that runs | a park started by anything else must cancel a pending playback, and handovers happen in arrival branches only ([FINDINGS §57.1](FINDINGS.md)) |
| 5 | Recording, composite runs, labels | `yam/recording.py`, `yam/motion.py`, the composite queue design ([ROADMAP §6.6.1a](ROADMAP.md)) | stop must freeze the recording instantly (the padding bug, [FINDINGS §30](FINDINGS.md)); labels are data, never control |
| 6 | Cameras: capture threads, the shared `Frame` record, identification, bandwidth measurement | `yam/cameras/*` plus `apps/capture_probe.py`; on Ubuntu add librealsense depth | bandwidth exhaustion is dropped frames with no error ([FINDINGS §34.5](FINDINGS.md)), and enumeration order is not index order ([FINDINGS §22](FINDINGS.md)) |
| 7 | Episode export and the C4 gate: recordings to MCAP **and to the C4 training directory**, then loader verification | `yam/episode.py` (C3) **and `yam/dataset.py` (C4, with `checks/check_dataset.py` + its falsifier)** — both read their own output back; swap in ABC's `export_mcap.py` encoding once verified | the sides are physical and never derivable; a wrong default mirrors the whole dataset silently ([FINDINGS §70.13](FINDINGS.md)) |
| 8 | Simulator and checkers: an arm that lags like the measured law, one test runner, a falsifier for every checker | `yam/fake/arm.py`, `checks/run_tests.py`, the falsify pattern | the simulator caught a crash that 616 unit tests missed ([FINDINGS §62.1](FINDINGS.md)); §5 below is this row's reasoning |
| 9 | Deploy loop (Phase E): the policy adapter, 30 Hz inference, dry-run first | the input interface from WP3; everything else is new | a policy is a command source; if it does not pass through WP1's clamps and WP2's guards, nothing else here matters |

## 4. Decisions

**Made, with evidence. Do not re-litigate without new evidence.**

- Joint-space actions. ABC's own choice, and what we command anyway.
- Demonstrations mainly by guide-and-replay, with per-waypoint variation tolerance ([ROADMAP §6.6](ROADMAP.md)).
- Collision avoidance stays manual at this bench spacing. Julien's ruling; bounding spheres report a collision that is not one at 0.70 m ([FINDINGS §60.3](FINDINGS.md)).
- Velocity feedforward capped at gain 1 ([FINDINGS §68.6](FINDINGS.md)).
- The gripper's ±2π encoder shift is per-session state, never a config value ([FINDINGS §40](FINDINGS.md)).
- No ZMQ for one station. Your guide's own advice, confirmed by one process driving both arms.
- The bench sides: left = G, right = B (Julien, 2026-08-19 — [FINDINGS §72.6](FINDINGS.md)). The working definition of "left" is left as the top camera's image sees it, because that image is the frame the policy trains in. The first properly aimed top frame verifies the sides in one glance.

**Open, each with its owner.**

- The task. Julien and the team. Deliberately open — everything here was built task-agnostic.
- The model. Probably diffusion/DiT; papers with the team.
- The noise bound for varied replays. Julien, about two minutes ([ROADMAP §8.2](ROADMAP.md) item 9 has the recommendation).
- Raising kp for sub-centimetre grabs. Julien, a bench decision (item 17).
- Camera mounts, aim and calibration. Deferred by ruling until the layout is final ([ROADMAP §8.4](ROADMAP.md)). Note that the walkthrough's C920 currently films a laptop lid, so its episodes are pipeline proofs, never data ([FINDINGS §72.6](FINDINGS.md)).
- The gripper unit in the dataset. Team plus C4: your `Observation` stores the gripper in metres, this walkthrough's episodes store it normalised from 0 to 1, and C3 gives the end-effector dimension without a unit. ABC's `export_mcap.py` adjudicates, and whoever loses converts ([ROADMAP §10.6](ROADMAP.md)).

## 5. The method chapter — how this stack fails, and the defences that worked

Every defect that mattered produced a confident, plausible, wrong answer, and none of them raised an exception ([FINDINGS §0](FINDINGS.md)). Transmit echoes decoded as motor replies. A gravity model 39% short while holding 4.3 kg, with a calm screen. A checker that stayed green while validating nothing, caught only because its falsifier counts catches ([FINDINGS §70.8](FINDINGS.md)). The defences are cheap, and they compound:

1. Check values for plausibility, never just for the absence of an exception. Prefer a test that could falsify the claim over one that agrees with it.
2. A written number is a cache with no invalidation. Replace claims with the command that recomputes them (`check_rig`, `check_recordings`, `check_arms_match`, and so on). This plan follows its own rule: it points, it does not copy.
3. Every checker gets a falsifier that feeds it known-broken input and counts the catches. A green run plus a stable catch count is evidence. A green run alone is not. Three times a checker had stopped detecting anything, and this is how each was found. One of them while a refactor was in progress.
4. One suite, one command, one total (`checks/run_tests.py`). Two test files sat red for days because nothing ran everything ([FINDINGS §67.5](FINDINGS.md)). The total is the catch counter for the suite itself: a total that drops while staying green means a check was disarmed.
5. Simulate the whole loop, not just the units. The lagging fake arm plus a scripted session caught what 616 unit tests could not.
6. A verifier bounds errors of commission, never omission ([FINDINGS §50.2](FINDINGS.md)). Reading the code against its own claims found the defects no checker could. Budget reading time; it is where the expensive bugs died.
7. Write findings down where the next person looks, at the moment they happen. This repo's FINDINGS file is why this plan can point instead of guess. Keep one.

## 6. What not to rebuild — measured dead ends

- Velocity feedforward above gain 1. A contradiction by construction ([FINDINGS §68.6](FINDINGS.md)).
- Integral catch-up for mirror following. It integrates noise and wanders ([FINDINGS §67.1](FINDINGS.md)).
- Depth over UVC on macOS. Colour only, measured from the pixels ([FINDINGS §63.0](FINDINGS.md)).
- Draining camera queues with `grab()` on a blocking backend. It produced 6 fps ([FINDINGS §21](FINDINGS.md)).
- Configurable dwell times for grabs. The jaws-only leg is the pause ([ROADMAP §6.6.2](ROADMAP.md)).
- Index-based device selection of any kind.
- Trusting `system_profiler` for USB facts on macOS ([FINDINGS §23](FINDINGS.md)).

---

**Where to go next**

- [ARCHITECTURE.md](ARCHITECTURE.md) for how the walkthrough's code is shaped, and section 2 for every word used here
- [BRIDGE.md](BRIDGE.md) for how this maps onto your own repo, branch by branch, and what still has to meet
- [PERFORMANCE.md](PERFORMANCE.md) before you decide a capture size or a compression quality
- [COMMANDS.md](COMMANDS.md) for every command and key in the walkthrough
- [FINDINGS.md](FINDINGS.md) for the evidence behind any claim above, by section number
