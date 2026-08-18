# Roadmap — from "the arm twitches" to "I drive it with the SpaceMouse"

> **Purpose of this file.** The README says what is *true now*. This says what we are going to do, **in what order, and why that order** — because the ordering is the part that carries the reasoning, and it is the part that gets lost between sessions.
>
> Julien's stated near-term goal (2026-08-10): *"be able to control the arm with the space mouse — a single arm with a single space mouse — and then we can go from there."* Everything below is ordered to reach that as directly as safety allows, and no more.

> ## ⚠️ Steps 1-4 below are DONE. Read [HANDOFF.md §5.5](HANDOFF.md) for the live task list.
>
> This file is kept because the **ordering arguments** are still the valuable part — why simulation came before hardware, why gravity compensation came before teleop, why the gripper was the safe first mover. But as a to-do list it is spent: teleop works on the real arm, and step 5 (the MCAP recorder) is the next unbuilt thing. **Step 4's caution about rotation signs is now refined by measurement** — see [FINDINGS §10](FINDINGS.md): rotation happens about the tool point, so a wrong rotation sign twists the wrist in place rather than swinging the gripper through space.
>
> **Axis remapping is also built** (2026-08-10): `scripts/map_axes.py` decides which puck axis drives which motion with no hardware at all, and `teleop_sim.py` now applies the same map — so the entire "step 1 in simulation first" argument below finally holds for axis conventions too, which was the one thing it could not previously test.

---

## ⭐ STATUS, 2026-08-10 — steps 1-4 are DONE. Read this before the step list below.

The numbered steps were written before any of them had been attempted, and most are now history. What is still live is **step 6**, which is where Julien wants to go next.

| step | state |
|---|---|
| 1b gripper teleop · 1 sim teleop · 2 whole-arm over gs_usb · 3 gravity comp + hand-guiding · 4 SpaceMouse → real arm | ✅ **done**, all on hardware |
| axis mapping (not in the original list) | ✅ **done** — any puck direction can drive any motion, tuned on the arm, per-arm maps available |
| **6 two arms, two SpaceMice** | ⭐ **NEXT — designed below, not built** |
| 5 recorder → MCAP | open. Deliberately after teleop feels right, but **before** collecting demos in anger |
| 7 cameras | open, nothing depends on it yet |

⚠️ The step *ordering* below is therefore stale — step 5 now follows step 6. The **reasoning** in each step is not stale, which is why they are kept rather than deleted.

## The target, stated precisely

**One SpaceMouse produces a 6-DoF cartesian twist. One YAM arm accepts joint positions. Teleop is the function between them, run at 100 Hz, safely.**

```
SpaceMouse twist  →  integrate to a target EE pose  →  IK  →  joint targets  →  arm
   (6 numbers)          (a pose that persists)      (mink)     (7 numbers)
```

The middle two boxes are the whole problem. The outer two are done: the SpaceMouse is decoded and verified on all six axes (README §4), and the arm accepts joint commands and moves (README §5).

⚠️ **Why the arm cannot simply be driven by the SpaceMouse directly.** A SpaceMouse gives *cartesian velocity* of the end effector. The arm takes *joint angles*. There is no fixed mapping between them — it depends on the arm's current configuration — which is exactly what inverse kinematics computes. `docs/Setup-Plan.md` §4.2 names this as the single largest deviation from the papers, which all teleoperate with GELLO leader arms that hand over joint angles directly and need no IK at all.

---

## ⛔ The binding constraint is the desk, not the software — 2026-08-10

Julien: *"it's not really safe right now. The only thing that should be moved is the gripper opening and closing and the gripper twisting… as soon as the SpaceMouse is connected I can move everything from the desk and we can control the whole thing."*

**So the ordering below changed, and this is why.** The cartesian IK loop is *already working* in simulation — software is ahead of the workspace. Steps that move the whole arm through space (gravity comp, cartesian teleop on hardware) are **blocked on clearing the desk**, not on code. Meanwhile motors 6 and 7 — `gripper_twist` and `gripper_jaws` — can move freely, because neither changes the arm's reach.

⭐ **That makes a real SpaceMouse-driven robot possible today**, which is step 1b.

---

## Step 1b — SpaceMouse → gripper twist + jaws, on the real arm ⭐ **do this now**

`scripts/teleop_gripper.py`. Two motors, no IK.

    puck YAW (twist)     →  motor 6, gripper_twist
    puck Z (push / lift) →  motor 7, gripper_jaws

**Why this is not a throwaway detour.** It proves the exact half of the teleop stack that IK cannot: reading the device and driving real motors together in one 100 Hz loop, with a deadman, bounds and a clean shutdown. When the desk is clear, IK drops in *above* this — the loop, the safety envelope and the shutdown path all survive unchanged. It is the same code shape, minus the coordinate transform.

**And it answers the question Julien actually cares about right now:** is the SpaceMouse connected and does moving it move the robot.

⚠️ `gripper_jaws` has no trustworthy limits (`gripper_limits: null`, `needs_calibration: true`), so it is clamped to a window around wherever it starts, never to an absolute target, and `--max-torque` is what stops it closing hard on itself. `--no-jaws` runs twist only.

**Done when:** Julien twists the puck and the gripper twists.

---

## Step 1 — Teleop in simulation, end to end

**Do this first, and do all of it, before the real arm is involved.**

**Why first.** `get_yam_robot(sim=True)` returns a `SimRobot` exposing the *same* API as the hardware object — `get_joint_pos()`, `command_joint_pos()`, `get_observations()`, `enable_gravity_comp()`. Verified working on macOS 2026-08-10. So the entire teleop chain can be written, run and debugged against simulation and then moved to hardware **by changing one flag**, with no rewrite and no second code path.

**Why that matters more than it sounds.** The first version of any IK loop is wrong — wrong axis conventions, wrong frame, wrong sign, wrong integration order, singularities near the workspace edge. Each of those, on a physical arm, is a joint slamming toward a limit. Debugging them in simulation costs nothing and risks nothing. **This is not a detour on the way to the real arm; it is the cheapest possible way to get there.**

**Also:** it needs no hardware at all, so it can proceed while the arms are unplugged, at LaVita, anywhere.

Sub-steps:
1. `mink` IK against the vendored YAM MJCF (`yam_linear_4310_d405.xml`, which already carries a `tcp_site` end-effector frame). Drive it with a *scripted* target first — a slow circle — so IK is validated with no input device in the loop.
2. Swap the scripted target for the real SpaceMouse. Still simulation, so a wrong sign is a shrug.
3. Optional MuJoCo viewer. **Optional on purpose:** Julien asked to skip visualisation if it slows things down, and he is right that it is not on the critical path. It is one line when wanted, and it is the fastest way to see *why* an IK bug is a bug.

**Done when:** moving the SpaceMouse moves the simulated arm sensibly in all six axes, joint limits hold, and nothing diverges near a singularity.

---

## Step 2 — Make the full-arm chain work over gs_usb

**Why this is a step at all.** Everything working today drives *one motor at a time* through `DMSingleMotorCanInterface`. Teleop needs all seven at once, and the layer that does that — `DMChainCanInterface`, used by `get_yam_robot()` — **hardcodes SocketCAN** (`dm_driver.py:409`, `if "can" in channel:`), with no argument to override it. That is the same wall §2.1 of the README describes, one layer up, and it has to come down the same way.

**What it unlocks — none of which is optional for teleop:**

| | why it is required |
|---|---|
| All 7 motors in one loop | teleop commands a whole configuration each cycle, not a joint at a time |
| **Gravity compensation** | see step 3 — without it a 6-joint arm cannot be commanded gently at all |
| Gripper force limiter | `linear_4310.yml`'s clog-force thresholds; the safe way to close on an object |
| `motor_offsets` / ±2π wrap fix | `get_yam_robot()` does this at init; hand-rolled control silently does not |

**Approach.** Same shape as `patch_gs_usb_for_macos()`: a small, documented, verified monkeypatch in `src/yam_can.py` that makes `DMSingleMotorCanInterface` resolve to the gs_usb backend when handed an adapter index, so `DMChainCanInterface` and `get_yam_robot()` work unmodified. ⛔ Not a fork of the vendor tree — `third_party/i2rt` stays a clean upstream checkout that can be re-pulled.

**Done when:** `get_yam_robot(channel=<B>, sim=False)` returns a working robot and `get_joint_pos()` returns the same seven numbers `ping_motors.py` reports.

---

## Step 3 — Gravity compensation, and hand-guiding

**Why before teleop, not after.** A YAM arm weighs ~4.3 kg and holds itself up with motor torque alone. At the gentle gains used so far, commanding all six joints *without* gravity compensation means the arm sags under its own weight, the controller fights it, and everything reads as "the IK is wrong" when it is not. **Gravity compensation is what makes joint position commands mean what they say.**

It is also the **safest possible whole-arm test**: the arm holds its current pose and follows no trajectory, so there is nothing to overshoot. And it is the first genuinely impressive moment — the arm becomes back-drivable and you can push it around by hand.

⚠️ This is the first time the arm holds real torque against gravity. It gets its own gated step, its own command, and the 400 ms firmware timeout intact (README §5).

**Done when:** the arm holds position without sagging, and can be pushed by hand and stays where put.

---

## Step 4 — SpaceMouse → the real arm ⭐ **the goal**

With steps 1-3 done this is a flag change plus a safety envelope, not new logic.

Additional guards that only make sense on hardware:
- a **workspace box** — reject any IK target outside a conservative volume around the start pose
- **velocity clamping** on the joint targets, independent of what IK asks for
- a **deadman**: releasing the SpaceMouse (all axes zero) freezes the target rather than drifting
- start with **translation only**, then enable rotation, so a wrong rotation sign cannot swing the wrist

**Done when:** Julien moves the puck and the arm follows. That is the milestone.

---

## Step 5 — Recorder → MCAP in ABC's exact schema

**Why it comes straight after teleop and not later.** The moment teleop works, every session is potentially training data. `docs/Setup-Plan.md` §6.1 is unambiguous: write MCAP with ABC's exact topic names and the whole data → training → eval half of the stack works **unmodified**. Get it wrong and every demo has to be re-collected, which is hours of a human's time rather than minutes of a computer's.

Log everything from the start — SpaceMouse input, resulting EE pose, **and** the IK-produced joint angles — so the action space can be chosen per experiment without re-collecting (Setup-Plan §4.3).

### ⭐ No, this does not mean running ROS2 — the question, answered once

**There is no middleware anywhere in this system, and none is needed.** Julien asked on 2026-08-12; the answer is worth writing down because it will be asked again, especially by anyone coming from `Hohnik/LaRobot`, which targets Ubuntu.

- **The transport is direct USB CAN.** `python-can` with the **`gs_usb`** (candleLight) backend over libusb, in **one process** that talks straight to the motors at 100 Hz. No ROS2, no DDS, no ZeroMQ, no gRPC, no sockets. Verified by search: no `rclpy`, `rospy`, `zmq` or equivalent in `src/`, `scripts/` or the vendored I2RT SDK.
- **The only ROS2 that appears anywhere is a set of schema NAMES**, inside `third_party/i2rt/i2rt/utils/ recording.py`: `sensor_msgs/msg/JointState` and `sensor_msgs/msg/Temperature`. ⭐ **MCAP is a file format, and ROS2 message definitions are being used as a serialisation schema inside it.** Writing a file that *describes* its records with ROS2 message definitions requires no ROS2 installation, no node, no bus and no running graph — which is why `mcap-ros2-support` is a dependency and `rclpy` is not.
- ⚠️ **So the interop point with ABC is a FILE, not a bus.** That is a good thing and it should stay that way: a middleware would add latency and failure modes to a 100 Hz loop that currently has ~3.7 ms of spare budget, in exchange for nothing this rig needs. Everything runs on one machine.

⚠️ Declared in `pyproject.toml` but **not imported by our code yet**: `mcap`, `mcap-ros2-support`, `dm-env`, `tyro`, `pydantic`. They arrived with the I2RT SDK and with this step's plan. That is fine, but it means their presence is *not* evidence that anything uses them — check before assuming.

---

## Step 6 — Two arms, two SpaceMice ⭐ **what Julien asked for next, 2026-08-10**

*"I would like to get to the point where we can control both arms with both mice."*

### The blocker is not the hardware, and it is not the compute

Both were checked rather than assumed:

| | measured | verdict |
|---|---|---|
| two arms, two CAN buses, one 100 Hz loop | `move_both_grippers.py`, genuinely independent trajectories | **proven** |
| CAN budget, 14 motors | ~6.2 ms/cycle against a 10 ms deadline | **fits** |
| **two IK solves per cycle** — never previously measured | **0.100 ms** mean, p99 0.110 ms | **negligible** |

So ~6.3 ms of a 10 ms budget, ~3.7 ms spare. ⚠️ The 6.2 ms figure came from register reads and is a lower bound; it says nothing about the loop once cameras and inference compete for CPU.

**The actual blocker is that `teleop_session.py` is single-arm all the way through.** `robot`, `teleop`, `mode`, `gripper_value`, `prev_q`, `home_ee`, `park_target`, `last_active_axis`, `guide_ref`, `stall_since` and `max_temp_seen` are all one arm's state, held in one function's locals.

### The design: extract `ArmSession`, then run N of them

One object owns **one arm's** robot, `CartesianTeleop`, axis map, mode and cached state, exposing roughly `enter_mode()` / `step(dt)` / `shutdown()`. The script holds a list and the loop iterates. Single-arm and bimanual then become the same code with N=1 or N=2.

⛔ **Why extraction and not a second `teleop_bimanual.py`.** Duplication has bitten this repo three times: `src/spacemouse.py` exists because device logic was copy-pasted and a fix landed in only one copy; the simulator's own `twist_from_axes()` ignored the axis map for the same reason; and PARK went around the gripper clamp because the clamp lived only in the teleop branch. A second control loop would be the fourth — and it would be the one driving two arms at once.

### ⭐ The de-risking that matters: `--arms B` must run the N-arm code with N=1

Then the **refactor** is verifiable against a single arm — behaviour Julien already knows the feel of — **independently of** the bimanual hardware risk. If N=1 feels identical, the restructure is sound, and going to N=2 introduces exactly one new variable.

Without that, the first bimanual run tests a ~400-line restructure *and* two-arm coordination at once, and any failure is unattributable. Session 4 is the argument: three changes that passed 34 tests, three dry runs and a simulated IK loop produced three failures on first hardware contact, one of which dropped 4.3 kg. **Stage the variables.**

### Decisions this needs, with the recommendation

| question | recommendation | why |
|---|---|---|
| Do mode keys apply to one arm or both? | **The selected arm.** `a` cycles B → G → BOTH; the status line always shows which | A global `g` puts **8.6 kg** weightless at once, and GUIDE is the mode where a dynamics-model error becomes a *falling* arm (FINDINGS §11.1) |
| Does *driving* apply to one arm or both? | **Always both** — each arm follows its own puck, continuously | That is the actual goal. Only *edits and mode changes* need a selector |
| Start mode | **HOLD**, and refuse `--start-mode guide` when N>1 | Two arms going weightless on a first run is the worst possible first run |
| Per-arm axis maps | ✅ **built** — shared by default, `--fork-map` to diverge | Julien: *"probably the same, actually. But maybe that should be options to map them separately"* |
| Puck assignment | ✅ **built** — `pick_device_by_wiggle(exclude=…)` | Without it the same puck can be assigned to both arms silently: two arms following one hand, which reads as a control bug |
| A fault on one arm | **stops both**, then the existing consent flow for each | A chain death on B must not leave G uncommanded and sagging |

### ⛔⭐⭐ 6.1 AUDIT BEFORE STARTING, 2026-08-13 — `ArmSession` has fallen a day behind, and the plan below was not executable as written

**Read this before touching the restructure.** Two things were checked rather than assumed, and both change the order of work.

⛔⭐ **FINDING 1: `ArmSession` implements a park model that was explicitly REPLACED the day it was written.** The class was committed at `0f965bc`, **2026-08-12 14:16**. Commit `b120300` landed at **15:15 the same day**, and its own message opens *"Blend THROUGH waypoints — the smoothing I built before was the wrong thing."* That commit replaced a per-leg trapezoidal speed ramp, which stops dead at every waypoint, with a single blended `JointPath` through all of them. **Julien's words are the specification it was written to:** *"instead of moving and then jittering ninety degrees to the next side, in a smooth curve it would go to the next point."*

**What the class actually contains today, checked by search:** `park_speed`, `park_start`, `park_speed_factor`, a queue with `next_leg()` and `abandon_queue()`. ⛔ **It contains no `JointPath`, no `park_s` cursor, no `park_marks`, no blend radius and no easing profile.** So the handoff's instruction to *"replace `park_path`, `park_s` … with a list of `ArmSession`"* **cannot be carried out**: the class has nowhere to put them.

⛔ **`teleop_session.py` has had TEN commits since the class was written**, and they include the entire recorder (`e89b745`), its four hardware fixes (`0e268ed`), the live status rows and adjustable ramp (`690afd9`), independent easing profiles (`e712e2f`), and everything from 2026-08-13. **The class knows about none of it.** Its 17 tests still pass, so it is correct about the design it models; that design is a day out of date.

⛔⭐⭐ **FINDING 2: THE RECORDER MUST NOT MOVE INTO `ArmSession`, and a comment in the code currently says it should.** `teleop_session.py` carries the note *"this whole block moves into ArmSession when main() is restructured"* above the recording state. **That is wrong, and §9.2 is the reason:** ABC's `states_actions.bin` is **14 states and 14 actions per timestep, two arms in ONE timeline**. A recorder that belongs to an arm produces one file per arm and **cannot** produce that format. **The recorder is session-level and spans both arms.** *(The comment has been corrected.)*

### ⭐ The corrected migration map — what goes where

| state in `main()` today | destination | why |
|---|---|---|
| `robot` · `teleop` · `mode` · `gripper_value` · `prev_q` · `home_ee` · `guide_ref` · `park_cmd` · `park_target` | **into `ArmSession`** | one arm's own state, already the class's job |
| `park_path` · `park_s` · `park_marks` · `park_leg_t` · `park_start_t` · `blend_idx` · `ease_idx` · `park_ramp` | **into `ArmSession`, but the class must LEARN them first** | each arm follows its own blended path at its own speed; the class currently models the superseded park |
| `take` · `take_to_save` · `take_t0` · `take_modes` | ⛔ **session-level, spanning BOTH arms** | ABC needs both arms in one timeline (§9.2). One recorder samples every arm each cycle |
| `replay` · `replay_s` · `replay_speed` · `replay_held_s` · `replay_worst_lag` · `replay_prev_target` · `tracking` · `replay_pending` · `replay_slot` | ⛔ **session-level** | a playback of a two-arm recording drives both arms from one cursor. Splitting the cursor per arm would let the arms drift apart in time, which is the one thing a bimanual demonstration must not do |
| `pending` · `park_sequence` · `slots` · key handling | **stays in the script** | which arm a key applies to is a session question, and the `a` selector is the answer |
| building the robot · reading the SpaceMouse · IK stepping | **stays in the script** | building energises motors and must stay visible; `CartesianTeleop` owns IK |

### Order of work — REVISED 2026-08-13

⛔ **Step 0 is new, and skipping it is how the restructure produces a diff nobody can review.**

0. ⏳ **MOSTLY DONE 2026-08-13 18:15, and it was declared done once too early.** ✅ The blended `JointPath` park is in: one path, an arc-length cursor, waypoint marks reported rather than stopped at, the cursor waiting past 0.15 rad of lag, arrival gated on the cursor, the two park clocks, and the easing profile and ramp as live knobs. `step_path()` returns a `ParkStep` carrying every number the script prints ([FINDINGS §36.2](FINDINGS.md)). ✅ **The gripper stall guard is in too, and it had been missing entirely** while the class carried a dead `stall_since` variable that made it look present — a safety guard that exists because motor 7 was cooked three times ([FINDINGS §36.5](FINDINGS.md)). ✅ The docstring's false claim to a fifth `map` mode is corrected. **Tests 17 → 27.** No `main()` code has been touched, so there is nothing here to test on the arm. ✅⛔ **STEP 0 IS FULLY CLOSED AS OF 2026-08-13 EVENING, and its last item was WITHDRAWN rather than completed** ([FINDINGS §37.3](FINDINGS.md)). It used to read: *move the per-cycle `MAX_JOINT_STEP` clamp and the joint-limit margin out of the teleop branch and into the class's single command path, on working-contract rule 7.* **I retracted that myself and both halves are wrong:** `SafeRobot` already rate-limits and lag-limits **every** command from **every** mode one layer below all control logic, so rule 7's question was already answered; and applying `JOINT_LIMIT_MARGIN` to PARK would **refuse to return to a pose the arm has already physically held**, because park targets come from `s <digit>`, which saves a *measured* pose. ⚠️ **The lesson kept with it: rule 7 is a question, not a verdict** — I pattern-matched "a guard lives in one branch" onto "a guard is being bypassed", and one search would have shown the lower layer already covered it. ⛔ **Nothing in step 0 is waiting on Julien.** *(This bullet still asked for his approval until 2026-08-14; so did [HANDOFF](HANDOFF.md)'s copy. Both are corrected.)*
1. ✅✅ **DONE 2026-08-14 AND CONFIRMED ON THE ARM.** One arm's state moved onto `ArmSession` — **247 of 247 references**, five commits, each leaving the script runnable ([FINDINGS §50](FINDINGS.md)). Julien drove every mode on it: *"Everything feels great. And as before, QQ works. Uh, all of the modes work."* ([§51](FINDINGS.md)). ⚠️ **The published size figures in the earlier version of this bullet were wrong and are corrected in [§36.3](FINDINGS.md):** it read "338 references, `mode` alone 93" from a text search that counted comments; parsed properly it was **247** and `mode` was **48**.
   - ⚠️ **Not in the class on purpose:** building the robot, reading the SpaceMouse, key handling, and IK stepping.
   - ⚠️ **Not in the class either, decided in Finding 2:** recording and playback stay session-level and span every arm.
   - ⛔⭐ **And what step 1 did NOT do, discovered on 2026-08-14 by checking rather than assuming ([FINDINGS §52.1](FINDINGS.md)): it moved the STATE, not the BEHAVIOUR.** The script calls exactly one method on the class (`arm.alive()`). Its own closures still do the entering, the clamping and the whole park, so `ArmSession.step_path()` and its tests describe a park **that never runs**. Collapsing the two is item 23 of [§8.2](ROADMAP.md) and needs its own bench session.
2. ✅✅ **COMPLETE, 2026-08-14.** Sixteen commits. `--arms` and `--arm`, the `a` selector, one status row per arm, and then **everything below `ArmSession` made per arm**: the robot, the puck, the axis map, the control frame, the base pose and slots, the CONTROLS memory, the temperatures, the last chain read, this cycle's puck deflection. **34 fields, proven by `check_restructure.py`, which now makes eight checks.** Details, including every defect the series introduced and how each was caught: [FINDINGS §52](FINDINGS.md), [§53](FINDINGS.md), [§54](FINDINGS.md).
   - ✅ **Confirmed on the arm at N=1, twice**, the second time with CONTROLS, a `p 0` park and `q q` ([FINDINGS §53.0](FINDINGS.md)).
   - ⭐ **Which keys aim at the selection and which stay session-wide** is a table in [FINDINGS §54.3](FINDINGS.md). ⛔ Map edits go to ONE arm because a SHARED map is one object ([§53.5](FINDINGS.md)).
   - ⭐ **The shutdown parks every arm together** and has 12 tests where it had none ([FINDINGS §54.4](FINDINGS.md)).
3. ✅✅⭐⭐⭐ **DONE ON THE HARDWARE, 2026-08-14.** Julien: *"Wow. Everything seems to work, and it seems quite good."* Two pucks, two robots, both arms in TELEOP from their own mice, `q q` parking both together. Every line of his output checked against what it proves: [FINDINGS §55.0](FINDINGS.md). ⭐ **The loop ran at 91 Hz with two arms**, against 83-88 for one. *(The original wording of this step, kept because it is what was asked for: `--arms B,G`, starting in HOLD, gripper enabled, desk clear.)*
   - ⚠️ **The old wording of this bullet said it needed asking for arm G first.** He freed it himself before it was needed. `--arms B,G --start-mode hold`, desk clear, gripper enabled. ✅ **Everything it needs is checked and true:** both CAN adapters on the bus, **both** arms' grippers calibrated in `config/gripper_limits.json`, two SpaceMice attached, and arm G free (his own words, 2026-08-14). ⭐ **The procedure, with the five things to try in order and what cannot work yet, is [FINDINGS §54.7](FINDINGS.md).** ⛔ Nothing agent-side blocks it: working-contract rule 1 means the run itself is his.
4. ✅ **DONE, 2026-08-14.** He entered GUIDE on each arm separately AND on both at once (`⭐ MODE: GUIDE on B+G`), which is 8.6 kg weightless. ⬜ CONTROLS on two arms is still untested, and `m` refuses while BOTH is selected by design. *(The original caution, kept because it is why the step existed: GUIDE last, because that is where a dynamics-model error becomes a falling arm.)* ⏳ **The earlier version of this bullet said GUIDE on BOTH was untested.** In his 2026-08-14 run he entered GUIDE on **each arm separately** (`a` to B, `g`, then `a` to G, `g`), which is exactly the cautious version this step asks for. ⬜ **GUIDE on BOTH at once is untested**, and `g` with BOTH selected is 8.6 kg going weightless in one keypress. ⬜ CONTROLS on two arms is untested and `m` refuses while BOTH is selected, by design.
5. ✅⭐⭐ **DONE AND RUN ON THE HARDWARE, 2026-08-14.** Julien: *"works really well in general… G followed."* He drove the leader with the puck and hand-guided it, and the follower copied both ([FINDINGS §56.0](FINDINGS.md)). ⛔ Its stop message named three causes and the real one was a fourth; it measures now ([§56.1](FINDINGS.md)). ⭐ At the prompt `i` switches copy ↔ mirror, so a wrong guess costs no restart. `i` engages it: the selected arm leads, the other follows joint for joint. `pick_pair()` decides who leads (the selected arm) and refuses when BOTH are selected. It asks twice, like `l`, because engaging moves the follower while the operator's hands are on the leader. **18 tests in `scripts/test_mirror.py`.** ⬜ **The ten-minute procedure is [FINDINGS §55.7](FINDINGS.md)** and it is the next thing that needs Julien. ⚠️ `MIRROR_SIGNS` is still a geometric prediction; it only matters for `--mirror mirror`, and the default `copy` is right for arms side by side.
6. ⏳⭐⭐ **BUILT 2026-08-14, NEVER RUN.** A recording is every arm's joints concatenated in `--arms` order, which is ABC's own 14-wide shape ([§9.2](ROADMAP.md)), with the arm list in the metadata so a `B,G` file cannot be replayed onto `G,B` with the arms swapped. Old one-arm recordings still play. Every arm parks to its own slice of the start pose and **none plays until all have arrived**. `w` and `l` no longer refuse ([FINDINGS §56.3](FINDINGS.md)). ⬜ **The procedure is [FINDINGS §56.8](FINDINGS.md).** ⚠️ **This is NOT the MCAP export**, which is still deferred pending his friend's schema ([§8.2](ROADMAP.md) item 7): this is our own JSON, in the same shape, so the export becomes a serialisation rather than a re-collection.

## Step 6.5 — ⭐ Saved positions, sequences, and smooth motion between them

**Julien's idea, 2026-08-12**, and it is a better one than it first looks: *"it would make sense to have more options to save more positions … hit `s` and then a number every time we wanna save a position, and then hitting `p` and then the number would park to that position. And then if we would hit `p` and multiple numbers following each other, then the robot arm could go from each position to each next position … we also wanted to include the smoother motions, so we would have to have an option to increase the speed between the positions."*

⭐ **Why this is on the critical path rather than a nicety.** A named list of poses the arm can be driven through, repeatably, is the first half of **demo collection** — step 5, which is the professor's SFT milestone. It is also the first thing in this repo that moves the arm through a plan rather than under a hand. Build it as if the recorder will be attached to it, because it will be.

### What is already done (2026-08-12)

✅ **Storage**, pure and tested in `src/yam_robot.py`: `park_slots()` and `with_park_slot()`, 6 tests. ⚠️ **The legacy file shape is read, not replaced** — `config/park_pose.json` is `{"B": [q…]}` on the rig right now, it is *measured calibration*, and `q p d` depends on it. A bare list is read as the `default` slot, so nothing that works today stops working.

### The interaction — decided, with the reasoning

| question | decision | why |
|---|---|---|
| Do bare digits conflict? | **No, and this is why two-key sequences are right.** `1 2 3` already flip rotation axes in the drive modes and `x y z` flip translation; `4 5 6` are free. A digit *after* `s` or `p` is an argument, not a command | Julien proposed exactly this shape unprompted, and it is the only one that does not fight the existing map |
| `p` with no digit | **Still parks to the default**, unchanged | ⛔ `q p d` is the hands-free shutdown and Ctrl-C now depends on the same pose. Muscle memory here is a safety property, not a preference |
| How does a sequence end? | Digits accumulate, **Enter or space runs it**, `q`/Esc cancels, and the queue is echoed as it builds (`sequence: 1 → 3 → 2 … Enter to run`) | A modal state in a loop that drives 4.3 kg must be *visible* at every keystroke |
| Which park loop runs a sequence? | ⭐ **The interleaved one** (`mode == "park"`), extended with a queue — on arrival, pop the next target instead of dropping to HOLD | ⛔ Load-bearing. `park_and_wait()` **blocks**, so it does not run the thermal guard or read keys. A multi-leg sequence is minutes of motion; running it blind on temperature would re-open the hole [FINDINGS §24.1](FINDINGS.md) just closed |
| Speed between poses | `+` / `-` adjust **park speed while in PARK mode** (they mean linear teleop scale elsewhere, which is meaningless there), shown in the status line | Context-dependent keys are already the pattern — `1-6` mean different things in CONTROLS mode — and the status line makes it visible rather than surprising |
| Abort | **Any key** stops a sequence, as it already stops a park | Unchanged from the behaviour he knows |

### The smoothing — the part that needs care

Today `advance_park_command()` moves every joint at a **constant** rate until it arrives: a trapezoid with no ramps, so it starts and stops abruptly. That is fine for one short move to a park pose and it is *not* fine for a sequence, where every waypoint becomes a jerk in the middle of a motion someone is watching.

**The recommendation: a trapezoidal velocity profile** — ease in over a fixed distance, cruise, ease out into the target — implemented as an optional `ease` argument so the default stays bit-for-bit what is on hardware today:

```
speed_factor = min(1, travelled / ramp, remaining / ramp)     # clamped to [floor, 1]
```

⚠️ **Three reasons to keep it opt-in at first.** `advance_park_command()` is pure with 15 tests and its behaviour is *confirmed on the arm*; the ramp distance is a feel question only Julien can answer; and a deceleration bug shows up as **overshoot**, which in park is the arm arriving somewhere it was not aimed. Ship it behind a flag, tune the ramp on hardware, then make it the default.

⭐ **Deliberately NOT doing: spline/blended waypoints** — smoothing *through* a waypoint rather than stopping at each. It is the obviously nicer motion and it is a much larger change: it needs a real trajectory representation, it makes "which pose is the arm at" ambiguous, and it removes the per-leg stall check that currently catches an obstruction. Stop-at-each-waypoint first, blended later, and only if the motion genuinely needs it.

### Order of work

1. ✅ Storage + tests *(done 2026-08-12)*.
2. ✅ `s`+digit and `p`+digit, on the interleaved park.
3. ✅ Sequences: the queue, the echo, the abort — and ⛔ **leaving PARK for any reason abandons the rest**, said out loud when it happens.
4. ✅ Park speed on `+`/`-` while in PARK mode.
5. ✅ **Easing** — on by default, `--no-smooth` disables it. The opt-in caution was withdrawn after reading `advance_park_command`: scaling an already-clamped step *down* cannot overshoot, so the risk that justified the flag did not exist.
6. ✅ **Corner blending** (`src/motion.py`, 12 tests) — ⛔ **and this was the feature actually being asked for.** Item 5 shapes *speed*; this shapes the *path*. Building only the first left the arm stopping dead at every waypoint, which is the jitter Julien described. Both now exist and are independent.
7. ✅ **The interaction**: `p Enter` base · `p 1 Enter` one pose · `p 1 2 3 Enter` shows the plan and waits for a second Enter · `-/+` speed and `,/.` corners work **while typing as well as while moving**.

8. ✅ **Easing as its own axis** (2026-08-12) — five profiles cycled with `e`: `none` · `in` · `out` · `both` · `s-curve`. ⭐ **Corner blending and easing are independent and both are needed**: blending decides the *shape*, easing decides the *speed along it*. Ctrl-C uses `out` — full speed from the first step, soft landing — because a shutdown move should leave at once.
9. ✅ **The end-of-park wait** — `settled` had shared the `blocked` timer, so every park finishing outside the 0.02 rad tolerance idled for four seconds before admitting it had arrived. Two questions, two patiences: 0.5 s and 4 s.

⬜ **What is left in this area:** nothing structural — only Julien's judgement on the arm about the default corner radius (`smooth`, 0.15 rad), the default ease (`both`) and the default speed. All three are live knobs, so tuning them needs no code.

### ⭐ What the five ease profiles actually mean, in plain terms

Julien asked, and "I don't know what s-curve is" is a fair thing not to know.

Every profile answers one question: **how does the speed change at the two ends of a move?** The middle is always full speed.

| profile | start | stop | when you want it |
|---|---|---|---|
| `none` | instant | instant | shortest possible move; a small jolt at each end |
| `in` | gentle | instant | leaving a delicate position, arriving somewhere it does not matter |
| `out` | instant | gentle | ⭐ **what Ctrl-C uses** — go now, land softly |
| `both` | gentle | gentle | the general-purpose one |
| `s-curve` | *very* gentle | *very* gentle | the smoothest, and the slowest off the mark |

**The difference between `both` and `s-curve`** is what is being smoothed. `both` ramps the **speed** — but the *acceleration* still jumps from nothing to a constant value the instant it starts, which the arm feels as a small shove. `s-curve` ramps the acceleration too, so the force builds up gradually. In an editor this is the difference between dragging a linear keyframe handle and a Bézier one.

⚠️ **Both ends use `√` now, not a straight line, and that is why the tail stopped crawling.** With a straight ramp the speed is proportional to the distance left, which is exponential decay — it halves in equal time steps and never quite arrives. Constant deceleration is `v ∝ √s` and it *does* arrive. Measured: the last 0.1 rad of a 1 rad move went from 1.08 s to 0.59 s while the rest was unchanged.

### ⭐ The two "patiences", and whether they are smart

Also a fair question. **They exist because two different things look identical from outside: an arm that has finished, and an arm that is stuck.** Both stop making progress. The only way to tell them apart is *how far from the target it stopped* and *how long you are willing to wait before deciding*.

- **0.5 s — "has the controller finished settling?"** The arm never lands exactly on the commanded pose; it settles a fraction of a degree short, where its stiffness balances gravity. Half a second of no improvement while already *close* means it has arrived as well as it ever will.
- **4 s — "is something in the way?"** Stopping *far* from the target is a different claim, and a much more serious one. Four seconds before saying so avoids crying wolf over a slow patch.

**Is it necessary?** Yes, and the history says so: they were one timer, and every park that finished outside the 0.02 rad tolerance — most of them — sat apparently idle for four seconds before admitting it had arrived. **Is it smart?** It is the minimum honest answer. The alternative is one number, which either declares arrival too eagerly (and hides an obstruction) or waits too long (which is what he saw). ⚠️ If it ever needs revisiting, the number to change is the 0.5 s, and the symptom would be a park declaring success while the arm is visibly still moving.

### ⭐⭐ Is waypoint playback good training data? — thought through, 2026-08-12

Julien's idea: *"define the waypoints in guide mode and then just play it without the hands in the way, so the robot can see all of the positions and the camera input as it should be — but at the same time it's predefined and not done with a mouse. Is that good data for the robot to learn?"*

It is a genuinely good question and the answer is **no for policy learning, yes for almost everything around it** — and the reason is precise enough to be worth keeping.

⛔ **Why it fails as training data.** An imitation policy learns a mapping from *what it sees* to *what to do*. A replayed trajectory is **the same every time regardless of what the camera sees**. So in the training set the action is statistically independent of the observation — and a model fitting that data has **no reason to look at the image at all**. It can score perfectly by memorising the trajectory and ignoring the camera, which is exactly the model you do not want. Move the object 5 cm and it does the identical thing.

⛔ **The second failure is subtler and worse: no corrective behaviour.** A human teleoperating drifts slightly off, notices, and pulls back — so the demos naturally contain thousands of tiny examples of *"you are a bit off, here is the way back."* A replay is perfect every time, so the policy never sees a single recovery. The first time it makes a small error at run time it is in a state the training data never contained, and errors compound. This is the standard covariate-shift argument that motivates DAgger, and it is the reason scripted demos underperform human ones even when the scripted ones look cleaner.

✅ **What it IS excellent for, and these are real:**

1. **Validating the recording pipeline** — a repeatable trajectory is the perfect test signal for MCAP schema, timestamp alignment, camera-to-joint sync and dropped frames. You cannot debug a recorder against data that is different every time.
2. **Measuring the rig** — tracking error, latency, repeatability, thermal drift over a hundred identical cycles. All of that needs a motion that does not vary.
3. ~~⭐ **AUTOMATED SCENE RESET, which is the one with real leverage.** The genuine bottleneck in demo collection is not performing the task, it is *putting everything back* between takes. A waypoint run that picks the object up and returns it to a randomised start position means Julien can record demo after demo without touching the scene — the playback creates the conditions, the human still provides the demonstration.~~ ⛔ **REFUTED BY JULIEN ON 2026-08-12, on both of its claims. Struck rather than deleted so nobody inherits it.** The bottleneck is *executing* the task with the SpaceMouse, not resetting the scene — and an automated reset is close to circular, because placing an object at a chosen position **is** the task being learned. Full account, and what replaces it, in [§6.6 below](#66-where-the-training-data-comes-from).

⚠️ **The one way playback could become training data** is if the waypoints were *generated per episode from the observed scene* — object detected here, so approach there — because then the action genuinely depends on the observation. That is a different and much larger project (a scripted policy with perception), and it is worth knowing it exists rather than assuming replay is simply unusable.

⚠️ **If a future session wants more interpolation options** — he compared this to Premiere Pro — the honest ranking is: (a) per-waypoint speed, so one leg can be slow and the next quick, which needs the slot file to carry more than a pose; (b) *dwell* at a waypoint, i.e. pause N seconds before continuing, which is what a pick-and-place demo actually needs; (c) true spline interpolation through the waypoints rather than blended corners, which is prettier and much harder to bound. **(b) is the one with real downstream value** — it is the difference between a motion and a *task*, and it is what the MCAP recorder will want to replay.

⭐ **Julien's ruling on all three, 2026-08-12:** *"all three sound really good, but they don't have to be done right now and they don't have to be main options. They could just be extra options that we can press a button for."* So they are **deferred, and they are extras** — additional knobs on the run that already exists, not a redesign.

The place they attach is already built: the run plan line and the `pending`/`confirm` key handler in `teleop_session.py`, where speed, corners, ease profile and ramp length already live. **Per-waypoint dwell is the one to do first**, because it needs the slot file to carry `{pose, dwell}` instead of a bare pose — and `park_slots()` / `with_park_slot()` already tolerate that shape change, since a slot's value only has to be a non-empty list. ⚠️ Adding dwell also turns a saved sequence into a *task description* rather than a path, which is exactly what the recorder wants to replay — see the data-collection analysis above for why that matters more than a prettier curve.

⚠️ **Deliberately still NOT doing: Cartesian-space blending.** These are *joint* poses, so a joint-space path needs no IK, cannot hit a singularity, and provably stays inside the joint range the waypoints span. A Cartesian blend would look smoother in the world at the cost of an IK solve per sample and a singularity risk on every corner, for poses that were never Cartesian to begin with. Revisit only if a task genuinely needs a straight line *in the world*, and note that recorded demos would then need Cartesian waypoints too.

⭐ **One decision arrived during implementation and is worth keeping:** Julien ruled that Ctrl-C must *always* return to the base pose regardless of what has been saved since — which turned "the park pose" from one variable into **two different things**. Slot `0` is the base: the pose the arm is released in, changed only by a deliberate `s 0`. Slots 1-9 are waypoints, and Ctrl-C ignores them entirely. **A pose that is safe to be let go in is not the same as a pose you want to return to mid-task**, and before this the two shared a variable that `s` silently overwrote.

## 6.6 Where the training data comes from

> **Read this section if you read nothing else in this file.** It decides what the recorder (step 5) has to do, and the recorder is the next big piece of engineering.
>
> Written 2026-08-12, **in plain language on purpose**, because Julien reads this section himself. ⛔ **Almost nothing in it is built yet** — see the last table for exactly what exists.

### First: what a demo is, and what the robot learns from one

A **demo** is one recording of the robot doing the task once. Several times a second it stores two things: what the cameras saw, and what the joints were told to move to.

Training turns a pile of those recordings into a single rule: *when you see this, do that.*

⭐ **That rule can only work if what the robot sees actually predicts what it should do.** Keep that sentence in mind. Everything below follows from it.

### Why a plain replay does not work

Suppose we teach the arm a path by hand, then play that path back a hundred times with the object always in the same place.

Every recording now has the same picture and the same movement. Nothing in the training rewards looking at the picture, because the picture never changes. The model can score perfectly by memorising the movement and ignoring the camera completely. Move the object 5 cm and it does the same thing as before, into empty air.

That is the whole objection to replaying a fixed scene, and it is real.

### Why Julien's version does work

His version differs in one way, and that one way changes everything:

> *"If we slightly move the object, placing a new waypoint for the new object isn't difficult. If we just use `g` mode for that it's very easy, compared to trying to find it with the mouse."*

Move the object by hand. Then hand-guide the arm to the new grab position and save it. Now the movement is different in every recording **and so is the picture**, and the two change together. The picture predicts the movement. That is real training signal.

⭐ **So the rule is: the variety has to come from the object moving, not from the path moving on its own.** That single line settles most of the design questions below.

### The one thing a replay is missing, and how to put it back

A person driving the robot wobbles. They go a bit wrong, notice, and pull back. So a human demo is full of small examples of *"you have drifted, here is the way back."*

A replay never wobbles, so the model never sees a single recovery. The first time it goes slightly wrong in real use, it is in a situation that was not in the training data, and from there things get worse rather than better.

⭐ **We can create those recoveries on purpose, and more evenly than a human produces them by accident.** Two ways, both cheap on this rig:

1. **Start the replay slightly off the path.** The arm drives back onto it. Every frame of that is a recording of *"I am off, here is the way back"*, paired with a picture that shows the arm off the path.
2. **Add a small wobble during the replay**, while still aiming at the correct path, so the arm drifts and the recorded movement is the correction.

This is a known technique with a paper behind it: *DART: Noise Injection for Robust Imitation Learning* (Laskey et al., 2017). It exists to buy back exactly what clean recordings lack.

⛔ **One requirement, and getting it wrong ruins the dataset silently.** The recording must store what the arm was **actually told to do**, not the tidy path we planned. Store the plan instead and every recording claims "I was perfectly on track" while the picture shows the arm off to one side. The model then learns nothing useful, and nothing about the files looks wrong. Write this into the recorder's design.

### Noise per waypoint: Julien's refinement, and it is right

His idea, 2026-08-12:

> *"Maybe not put noise on the waypoints like when the object is being picked up, those waypoints might need to be exact, and maybe the jittering needs to be very, very slight on those. But the others, how to get there, the waypoints for those could be very different, maybe the noise could be higher."*

This is better than one noise setting for the whole path, and the reason is that the two kinds of waypoint have completely different tolerances.

| waypoint | how much error it survives | so what noise is right |
|---|---|---|
| where the gripper closes on the object | millimetres, set by the size of the object | none, or very little. Its position should come from the object, re-taught by hand |
| on the way there: above, around, retreating | centimetres, set by "do not hit anything" | plenty. Different approach routes are useful variety |

⭐ Noise on the approach teaches the model that many routes to the same grab are fine, which is real robustness. Noise on the grab teaches it to be sloppy at the one moment when precision decides success or failure.

⛔ **And a failed grab is worse than a wasted minute: it is a bad demo labelled as a good one.** So we have to know whether each episode actually worked.

⭐ **Which we can detect for free, using hardware we already read every cycle.** The gripper is position controlled and we read its position. If the jaws close on a 3 cm object they stop at 3 cm; if nothing is there they close further, to the empty-hand position. **So "did the grab work" is a number we already have.** That gives an automatic success-or-failure label per episode, which is exactly what is needed to throw out the poisoned ones. ⚠️ It needs one calibration step per object (measure where the jaws stop when holding it), and it cannot tell a good grab from an awkward one. It can tell "holding something" from "holding nothing", which is the case that matters.

### Recording your voice while driving: his other idea

> *"Maybe later in the process we want a possibility to record data with a microphone where as a user I can also say, like, 'I like this. No, this is bad. This way is bad. I'm currently doing something bad. This is good again.' And then we could match the text to the specific time code of the waypoints and then maybe custom score the waypoints, so that it's clear later during reinforcement learning into the model which of the pathways are good and which are bad."*

The mechanism is easy and everything needed runs locally. Record the microphone with timestamps, transcribe it with Whisper on the laptop, and store the words with their times alongside the joint and camera data.

⭐ **The valuable part is the label, not the audio.** So there are two versions, and the cheap one is worth having first:

- **A keypress.** One key for good, another for bad, pressed while the arm moves. Instant, exactly timed, no transcription, and it fits the key handling that already exists.
- **Voice.** Better when both hands are busy, which they are during teleop. That is the argument for doing it his way, and it is a good one. Both can coexist.

⚠️ **The timing needs care, and this is the part that would quietly go wrong.** A person comments *after* the event, roughly half a second to two seconds later. So a comment at time `t` marks the stretch from about `t − 2s` to `t`, not the instant of `t`. Attach the label to the moment of the words and it lands on the recovery rather than on the mistake, teaching the opposite of what was meant.

**What the labels are actually good for, cheapest first:**

1. ⭐ **Throwing away the bad stretches before training.** Reliable, simple, and it works with ordinary imitation learning. This alone justifies building it.
2. **Training more heavily on the good stretches** (weighting rather than deleting).
3. **Proper reinforcement learning from the comments.** Real, and a much larger project: human comments are sparse and loosely timed, which is its own research area (learning a reward model from human feedback). Worth knowing it exists; not the thing to build first.

⭐ **A second, unrelated use for the microphone, possibly worth more.** Modern policies are often *told* what to do in words as well as shown pictures. Saying *"pick up the red block"* at the start of each episode gives us the text half of that for free, at a cost of one sentence per recording. ⛔ **If there is any chance of using a model that takes a language instruction, record it from the very first episode. It cannot be added afterwards.**

### Recording in guide mode: he likes this, and it may beat waypoints

> *"One good idea is definitely recording everything in the guide mode and then replaying it. That's a smart idea, definitely."*

Instead of saving a handful of poses and letting the code interpolate between them, stream the joint positions continuously while he hand-guides the arm through the whole task. Then replay that recording with the scene reset, and record the cameras on the replay pass.

**Why this is likely better than waypoints:**

- The movement carries human timing and hesitation, so it is much closer to a real demo.
- No inverse kinematics anywhere, so no singularities and nothing to solve. Hand-guiding is already a demonstration in joint angles, which is the form the arm wants.
- The path is physically achievable by definition, because the arm actually went there.
- His hands are in the frame only during teaching. The recorded pass contains nothing but the robot and the object. Ordinary hand-guided teaching puts the human's arms in every frame, and a model then either learns to look for a hand (which is absent when it runs on its own) or the frames have to be masked.

⭐ **And there is a reason this is the natural thing to do.** The papers this project follows all use **GELLO leader arms**: a small copy of the arm that a person moves, which hands over joint angles directly with no IK at all. `docs/Setup-Plan.md` §4.2 names using a SpaceMouse instead as the single biggest difference between this rig and the papers. **A YAM arm in guide mode is very close to being a leader arm already.** So the difficulty with the SpaceMouse is the expected cost of that difference, not a personal failing.

⚠️ **Two things to watch:**

- The movements come from the teaching pass and the pictures from the replay pass, so the replay has to follow the taught path closely enough that picture and movement still line up. Checkable: the park loop already reports how far the arm is running behind the command.
- Hand-guiding smoothly is its own skill. A wobbly taught path replayed at speed could be jerky. Light smoothing, or replaying at the speed it was taught, both help.

### 6.6.2 A pause where the jaws move — built as a decision, 2026-08-13

⛔ **The problem blocks every grab.** Blending means the arm curves *through* a waypoint without stopping, which is the smooth motion Julien asked for and confirmed on the arm. The gripper is simply another number in the joint vector, so a blended corner between "above the object, jaws open" and "at the object, jaws closed" **closes the jaws during the descent.** The object is grabbed early or shoved away.

⭐⭐ **The obvious fix was a dwell time saved against each waypoint, and it turned out to be the wrong shape.** A dwell needs a storage change, a way to type it, and a number the operator has to guess. None of that is necessary, because **a leg where the gripper command changes and the arm does not IS the pause**, and how long it should last is not a preference. It is however long the jaws take, which can be measured while it happens.

⭐ **So a run saved the natural way needs no thought at all:**

```
w0  where the arm is now
w1  above the object, jaws open
w2  at the object, jaws open
w3  at the object, jaws closed     <- only the gripper changed
w4  lifted, jaws closed
```

`plan_gripper_stops()` returns `segments = [[0,1,2], [3,4]]` and `gripper_legs = [(2,3)]`. The run then blends through `w0 w1 w2`, stops, commands the jaws, waits for them, and blends through `w3 w4`. ⭐ **The arm arrives at the object exactly, because the corner it would otherwise have rounded is now the end of a segment.**

⚠️ **A leg that moves the arm and the gripper at once is reported rather than split.** Both readings are defensible, closing while approaching or stopping and closing, and only the operator knows which he meant. Guessing wrong on 4.3 kg is worse than saying so, so today's behaviour is kept and a warning names the leg.

**What is built:** ✅ `src/motion.py::plan_gripper_stops` and `RunPlan`, with **9 tests**. They pin the grab case, the no-gripper case, two grabs in one run, a noise-sized gripper difference not counting, a `--no-gripper` 6-value pose, and the invariant that every waypoint appears in exactly one segment.

**What is missing, and it is the part that touches confirmed-working code:**

1. `begin_path()` builds **one** `JointPath`. It has to build one per segment and keep a queue.
2. On reaching the end of a segment, command the jaws and wait. ⭐ **Waiting is measured, not timed**: proceed when the jaws stop moving, which also covers a jaw that stalls on the object. ⚠️ It needs its own timeout, or a jammed gripper stops the run for ever.
3. ⛔ **The gripper must be commanded through `clamp_gripper`**, as everywhere else. A pose saved with the jaws on a mechanical stop would otherwise hold them there, which cooked motor 7 three times ([FINDINGS §4](FINDINGS.md)).
4. The run plan line should say how many stops there are, so a grab is visible before Enter.

⚠️ **Deliberately left for its own session.** The park is confirmed-working code that `q p d` and Ctrl-C both depend on, and segmenting it is a real change to its shape. The decision is proven; the wiring is not.

### ⭐⭐ 6.6.1 Mixing waypoints and recordings in one run — his idea, 2026-08-13

> *"The smoothing and stuff to vary the runs would be great, or to connect the waypoint idea with the recording idea. So for example, when I saved positions one, two, three and four, I can choose a mode where one and two gets played back, and then from two to three I can move the robot arm in whatever way. And whenever it stops, and I stop recording that position, then from there it moves to position three, and then two to three is my recording plus the late movement, and then it continues to four. Or something else maybe that's even smarter than that, where we can then have intermediate recordings that we change up, and then we can change up the starting and the end parts through different noise things that we were thinking about or smoothing."*

⭐⭐ **This is the best idea in this section, and the reason is that it puts the precision exactly where the task needs it and the variation exactly where the task tolerates it.** A run becomes a list of legs, and each leg is one of two kinds:

| leg kind | what it is | tolerance | so what varies |
|---|---|---|---|
| **planned** | the existing blended move between two saved poses | centimetres, bounded by "do not hit anything" | ⭐ speed, corner radius, easing, and **noise on the waypoints** |
| **taught** | a hand-guided recording played back | millimetres, set by the object | ⛔ nothing, or almost nothing. Re-taught when the object moves |

⭐ **Compare that against the noise rule two sections above.** The rule says variation must live in the (observation, action) pair jointly, never in the action alone, and that the grab waypoint survives millimetres while the approach survives centimetres. **A composite run is that rule expressed as a data structure.** The approach legs are planned, so they take noise. The grab leg is taught, so it stays exact. Nothing has to remember which waypoint is special: the leg kind says so.

⚠️ **And it answers a problem the recording feature has on its own.** A single hand-taught recording covers the whole task, so re-teaching after the object moves means re-teaching all of it. With a composite run, only the taught leg near the object needs re-teaching. **That is the difference between a minute per episode and ten seconds.**

**What it needs, and none of it exists yet:**

1. **A run is a list, and each entry names a kind.** Today `park_sequence` is a list of digits meaning saved poses. It would become a list of `("pose", "3")` and `("take", "1")` entries. The typing UI needs a way to say which, and the existing two-key idiom fits: digits stay poses, and something like `w` then a digit inside a sequence means a recording.
2. **Joining a taught leg to a planned one.** ⛔ **The seam is the real work.** A planned leg ends at a saved pose. A taught leg starts wherever the hand started it. If those differ, the join is a jump, which is the same hazard `start_pose()` already guards for a lone playback. ⭐ **The clean answer: a taught leg's own first and last poses become waypoints in their own right**, so the planned legs either side aim at them rather than at whatever was saved by hand. Then no seam can be discontinuous by construction.
3. **A pause at each waypoint**, already a prerequisite for grabbing anything at all (see above). A seam between two legs is exactly where a pause belongs.
4. **Recording a leg in place, mid-run.** His description has the arm stop at pose 2, him hand-guide it, and the run continue from there. That means GUIDE has to be enterable *inside* a run and the run resumable, which the current mode machine cannot do: leaving PARK abandons the rest, deliberately. ⚠️ **That rule exists because an arm resuming a planned trajectory after someone pressed HOLD is doing something nobody asked for.** So resuming needs to be an explicit, visible act rather than a relaxation of that rule.

⭐ **The order I would build it in:** the pause at each waypoint first, because grabbing needs it anyway. Then composite runs assembled from *already saved* poses and recordings, which needs no change to the mode machine. Teaching a leg in the middle of a run last, because it is the only part that touches the rule about abandoning a run.

### The three ways to collect demos, side by side

| | how it works | good | bad |
|---|---|---|---|
| **SpaceMouse teleop** | he drives with the puck, we record | natural corrections; matches what the papers assume | he reports it is very hard; slow; failed attempts still look like demos |
| **Waypoint teach and replay** | save poses by hand, replay them | easy and fast; clean frames | interpolated motion with no human timing; needs a pause at each grab |
| **Guide-mode record and replay** | hand-guide the whole task, replay it | human timing; no IK; clean frames; closest to the papers' leader arms | not built; needs the two passes to line up |

⭐ **Best guess at the right answer, which is not one of the three on its own:**

1. Most of the dataset from **guide-mode teaching and replay**, with the object moved by hand every episode and the grab re-taught to match it.
2. **Recoveries added** by starting some replays slightly off the path and by adding wobble.
3. **The SpaceMouse kept, with a different job.** Not for whole demos, but for nudging a trained model in the moments where it fails. That needs a few seconds of input rather than a whole task, which is the one situation where a hard six-axis device is fine. ⭐ **None of the teleop work is wasted by this.**

### The measurement that settles it, and it takes 20 minutes

Everything above is argument. The comparison is cheap and it is real:

⭐ **Do five demos each way** and write down, per demo: the wall-clock seconds, whether the task actually worked, and where the time went.

⚠️ Rough guesses only, so we can judge whether the experiment is worth running. Teach-and-replay looks like 40 to 70 seconds an episode (move the object, about 5 s; re-teach the grab in guide mode, 20 to 40 s; replay and record, 10 to 20 s), so 100 episodes in under two hours. Teleop at 60 to 90 seconds an attempt with a high failure rate is several times that, and its failures cost more than time. Published work on this kind of task usually uses somewhere between 50 and 200 demos, so 100 is the right number to plan for. ⚠️ **That last figure comes from the literature, not from anything measured here.**

### His answers to the three questions this section used to ask

**Which task?** Not decided, and deliberately not yet: *"I have no idea about the task yet. That's irrelevant right now because we first have to get the setup right."* ⭐ **So build the collection machinery to be task-agnostic.** Do not bake in a number of waypoints, a grab shape, or an object size anywhere.

**Which kind of model?** ⭐ **Most likely diffusion**, because *"we want to have the newest research"*, plus interest in vision-language-action models and world models, and specifically the ABC research. He has papers to share and will send them.

⭐ **This closes a question that was open, and it closes it in his favour.** A **diffusion policy** is a model that can hold *several* valid answers for the same picture instead of being forced to pick one. A simpler model trained to minimise squared error averages the answers instead, so two different good routes come out as one bad route between them. **So deliberately collecting several different ways to do the same task is safe with diffusion, and would have been harmful with the simpler kind.** His *"many different ways to do the actual task"* is fine.

⚠️ **What to check against the papers when he sends them.** He asked specifically where our code might not match the research and where connecting pieces are missing. These are the candidates, to be confirmed against real papers rather than guessed at:

- **What counts as an "action".** We command joint positions. Much of the literature commands end-effector movements instead. Both can be recorded, and step 5 already says to record both. ⛔ Deciding late is only possible if both are recorded from the start.
- **A fixed recording rate.** Diffusion policies usually predict a short block of future moves at once, which assumes evenly spaced samples. Our loop runs at 100 Hz; recordings are normally reduced to 10 to 30 Hz. That has to be a deliberate, timestamped resampling, not "whatever the loop happened to manage".
- **A fixed image size and frame rate**, per camera, decided before collecting rather than after.
- **A text field for the instruction**, if a language-conditioned model is possible at all.
- **A success or failure flag per episode**, which the gripper trick above can fill in.
- **Which camera produced which image**, since B and G currently have different cameras.

**Does the provenance matter?** ⭐ **Yes, a lot**, and he defined what he means by it:

> *"As far as I understand, like, having the history and everything we found and problems we had and being able to reproduce everything and connect it to other research papers, because we're doing a student project and would want to connect everything to research and maybe build on this research."*

⭐ **That is a concrete requirement on the recorder, not a vague wish.** Every episode has to carry enough information to be reproduced and cited later:

- which arm, and the git commit of the code that recorded it
- which cameras, by serial, at what resolution and frame rate
- the calibration in force at the time (gripper limits, axis map, control frame)
- how it was collected: taught by hand or driven; which waypoints; what noise settings; what speed and easing
- a timestamp, and the success-or-failure flag

⚠️ **All of that is cheap to write at collection time and impossible to reconstruct afterwards.** A dataset without it is a folder of videos nobody can cite. ⭐ This is also the one place where this repo's existing habit already pays off: `FINDINGS.md`, the long commit messages and this file are the *"everything we found and the problems we had"* half of what he described, and they already exist.

### What exists today, and what does not

He asked directly whether the features under discussion are built. Mostly they are not.

| feature | state |
|---|---|
| single-arm driving: guide, teleop, hold, park, waypoint runs, blended corners, easing | ✅ **built, and confirmed on the arm** |
| both arms from one script | ⬜ **not built.** `--arms` does not exist. `src/arm_session.py` is the class it needs, with 17 tests, but nothing uses it yet. ROADMAP step 6 |
| mirroring one arm onto the other | ⬜ **half built.** `src/mirror.py` and its 14 tests exist. The script that opens both arms does not. Needs the two-arm work first |
| recording a movement in guide mode and replaying it | ✅ **BUILT 2026-08-13, and unconfirmed on the arm.** ✅ `src/recording.py` holds the movement itself: `Trajectory`, with a guard against time going backwards, interpolation between samples, resampling onto an even clock, time scaling, a measured `max_joint_speed()` to check a playback speed against, and JSON that stays diffable. **23 tests, no hardware.** ✅ Wired in: `w` records (every cycle, every mode), `w` again then a digit saves to `recordings/<n>.json` with the git commit and a timestamp, and `l` then a digit plays it back after parking to the start pose. ⛔ **Never run on hardware yet** |
| noise on waypoints, chosen per waypoint | ⬜ **nothing exists** |
| a pause at a waypoint, so a grab has time to happen | ⬜ **nothing exists**, and it is needed before any grab can be replayed at all. See below |
| microphone or keypress labels | ⬜ **nothing exists** |
| writing recordings to a file at all | ⬜ **nothing exists.** This is step 5 |
| both D405 depth cameras working together | ⬜ **one works, the second is unplugged.** ROADMAP step 7 |

⛔ **One thing that IS already true and matters: a saved pose carries all seven joints, including the gripper, and a park drives the gripper to it.** Checked in the code (`src/yam_robot.py`, `park_target_from`). So a saved sequence can already open and close the jaws, which means a pick-and-place is in principle replayable today.

⛔ **But blended corners and grabbing fight each other, and this is the missing piece.** "Blending" means the arm curves *through* a waypoint without stopping. A grab needs the opposite: stop, let the jaws travel, wait. Worse, the gripper is just another number in the joint list, so a smooth corner between "above the object, jaws open" and "at the object, jaws closed" closes the jaws **during the descent**, which either grabs early or shoves the object away.

⭐ **So a pause at each waypoint is required, not an extra.** §6.5 above already ranked it first among the deferred options; this promotes it to a prerequisite. A grab needs roughly: approach with the jaws open, arrive, pause, close the jaws, pause, lift. A pause also implies a stop, which switches blending off exactly where blending is wrong, so one setting covers both needs and no separate per-waypoint blending switch is required. The storage (`park_slots()` / `with_park_slot()`) already tolerates the shape change.

## Step 7 — Cameras

C920 plus the wrist D405s. Needed for data collection, not for teleop. Deliberately last of the near-term set because nothing else depends on it.

**Where it actually stands, 2026-08-11.** The C920 works in a window and in the terminal, cameras are selectable **by name** — identified by measurement, not by any list order, after a positional guess turned out to be wrong about two of four ([FINDINGS §22](FINDINGS.md)) — and one D405 is mounted on arm B, measured, and **delivering a colour picture over plain UVC with no SDK at all**. So this step is further along than "needed for data collection" suggests: the wrist view is drivable today, and `librealsense` is an upgrade for depth, intrinsics and camera controls rather than a prerequisite.

### 7.1 What the D405 can and cannot do right now — asked by Julien, 2026-08-12

He asked: *"I don't even know what's capable with them and how they should be used."* Written in plain language, because he reads this. **Current wiring: arm B has a D405, arm G has the Logitech C920, and he is about to put the second D405 on arm G.** Photographs of both arms were provided on 2026-08-12.

**What a D405 is.** A small stereo depth camera. Two lenses, a few centimetres apart, and a chip on board that compares the two views to work out how far away things are. It is built for close-up work, roughly a hand's reach, which is exactly the distance from a wrist mount to whatever the gripper is holding. ⚠️ **The exact usable range is from the product datasheet and has not been confirmed on this unit** — the manual is at `intelrealsense.com/get-started`, and `rs-enumerate-devices` reports it once librealsense is installed.

| | works today | needs `librealsense` |
|---|---|---|
| a colour picture | ✅ **measured** — it appears as an ordinary webcam, so OpenCV opens it with no driver at all | — |
| choosing resolution and frame rate | ✅ works | — |
| **depth** (how far away each pixel is) | ⛔ no | ✅ yes |
| **intrinsics** (the lens numbers needed to turn a pixel into a direction in 3D) | ⛔ no | ✅ yes |
| exposure, gain and other camera controls | ⛔ no | ✅ yes |
| **telling two D405s apart by serial number** | ⛔ **no, and this is the one that bites** | ✅ yes |

⛔ **The second D405 breaks how cameras are currently identified, and this is the single most important thing in this section.** Cameras are identified by asking each one for a picture size that only one of them supports ([FINDINGS §22](FINDINGS.md)). **Two identical D405s support exactly the same sizes, so that method cannot separate them.** They do have real serial numbers, unlike the two SpaceMice — but the plain-webcam path cannot read a serial. So there are two options and no third:

1. **Install `librealsense`** (`brew install librealsense`, a prebuilt package, no compiling) and select each camera by serial. ⭐ **This is the recommendation** — it also unlocks depth, intrinsics and exposure, so it pays for itself.
2. **Identify by hand each session**: cover one lens, see which index goes dark, note it. ⚠️ Works, but it is a manual step before every session and it is exactly the kind of thing that gets skipped and then produces a dataset where the two arms' images are swapped.

### ⭐ 7.1.1 What Julien has to run to check the cameras, 2026-08-13

He asked: *"the camera is plugged in, and we don't need to plug in the g camera yet. We can use the b camera. I need to make sure that all of the camera setup stuff works. So let me know what I need to test for that or if you can test everything yourself."*

⛔ **The agent cannot do any of it.** Opening a camera needs macOS camera permission, which is granted per application, and the agent's shell is not the application that has it ([FINDINGS §21.1](FINDINGS.md)). Enumerating over `ioreg` works and opening does not. So these are his to run, in this order, and each answers one question.

| # | command | what it proves, and what a wrong answer looks like |
|---|---|---|
| 1 | `uv run scripts/camera_view.py --list` | ⭐ **The most useful one.** Every camera, its name, and whether it delivers colour. ⚠️ Expect the D405 to be listed under a name containing `Depth` and still report **colour**, which is correct and was once misread as a fault ([FINDINGS §8](FINDINGS.md)). ⛔ Also answers an open question: **does the D405 appear ONCE or more than once?** Its USB tree has four interfaces (`Depth`, `Depth`, `Y`, `RGB`), so if a second entry appears, some depth or infrared data may be reachable with no SDK at all ([FINDINGS §28.2](FINDINGS.md)) |
| 2 | `uv run scripts/camera_view.py --camera d405 --big` | Selecting by name works, and the window shows a live picture. ⭐ **Use the window, not `--term`, whenever the point is to look at the picture.** The terminal path re-encodes every frame as a PNG and is permanently softer |
| 3 | `uv run scripts/camera_view.py --camera d405 --term` | The terminal view. ⭐ **This is the one to check after 2026-08-13**: the picture used to shrink over time and never recover, and the controller was rewritten ([FINDINGS §29](FINDINGS.md)). Watch `sent WxH` in the status line for a minute or two. It should settle and stay, not creep downwards |
| 4 | press `1` to `6` in either view | Capture size changes, and the picture visibly changes with it. These keys once appeared dead because the sent image was pinned ([FINDINGS §21.4](FINDINGS.md)) |
| 5 | `sudo rs-enumerate-devices -s` | ⛔ **Needs sudo, and that is expected** ([FINDINGS §28.4](FINDINGS.md)). Reports firmware, and the serial. ⚠️ **It reports a DIFFERENT serial from the USB descriptor** (`260322274021` against `255323071773`), which is unexplained. Run it beside `ioreg` and settle it before any serial is written into a config file |

⭐ **What cannot be answered without the second D405**, and therefore waits: whether two identical cameras can be told apart at all, and whether two of them fit on one USB controller. Both are in §7.2 below.

⚠️ **One measurement worth taking while he is in there:** the D405's latency. The C920 is about 200 ms and that is the camera itself rather than software ([FINDINGS §21.3](FINDINGS.md)). A machine-vision camera may be far quicker, and the whole reason for wanting it is a wrist view you can drive from. **Do not assume the 200 ms carries over.**

### 7.2 What actually needs doing, in order

1. ⭐ **Install `librealsense` and select by serial.** Unblocks the identification problem above before it can corrupt a recording. Also answers what the camera can do, via `realsense-viewer`.
2. ⭐ **Measure the mount: where each camera sits and which way it points, relative to the wrist.** ⛔ There is already a `camera` control frame in the code, and it assumes a **modelled** mount with a 25° tilt. The photographs show a bracket on the side of the wrist that is very unlikely to match that model. **Until it is measured, drive in the `tool` frame (`v` cycles them), not `camera`** — the existing warning in [COMMANDS.md](COMMANDS.md) is correct and this confirms why.
3. **Plug in the second D405 and measure USB bandwidth.** Two USB-3 cameras streaming at once can exceed what one laptop controller carries. ⚠️ It fails as dropped frames, not as an error message, which is the worst failure mode for training data. [FINDINGS §8](FINDINGS.md) already flags this as needing a re-check.
4. **Measure the D405's latency.** The C920 is about 200 ms and that is the camera's own doing, not software ([FINDINGS §21.3](FINDINGS.md)). A machine-vision camera may be far quicker. ⚠️ **Re-measure rather than assuming the number carries over.**
5. **Decide whether depth is wanted at all.** Most image-based policies use colour only. So depth is a "know what we have" item, not a blocker on anything.
6. ✅ **Put the same camera on both arms** — he is already doing this. Two different cameras means two different fields of view, colours and latencies, which is an avoidable inconsistency in a bimanual dataset.

### 7.3 ⭐ The cable — asked by Julien, 2026-08-12, with photographs

> *"Currently the depth cameras are connected via a cable directly to the computer, but the cable's very short, and it's kind of in the way of the robots. So is that the best possible way to do it? Or is it usually connected to the robot in some way?"*

**No, the current arrangement is not good, and the photographs show two separate problems.**

1. ⛔ **The plug takes the strain.** The cable leaves the camera sideways and hangs in a free loop. When the wrist twists (motor 6), that loop winds up and pulls on the connector. A USB-C socket is not built for repeated pulling and twisting, and it is glued to a camera that costs real money.
2. ⛔ **The cable is loose inside the workspace**, where the arm can catch, drag or pinch it. On a rig with no emergency stop ([HANDOFF §4.5](HANDOFF.md)) that is a snag risk during a motion nobody is holding a hand over.

**What is normally done, and it is worth copying.** Route the camera cable **along the arm**: clip or tie it to each link, and leave a small loop of slack at each joint so the joint moves the loop instead of pulling the plug. Then run one longer cable from the base of the arm to the computer. ⭐ **The photographs show a black braided sleeve already running along the arm** (the motor and CAN wiring), so there is an existing path to follow and clip to.

⚠️ **Do not simply buy a long USB cable.** A passive USB-3 cable at 5 Gbps is reliable to about 2 m and sometimes 3 m; beyond that it needs either an **active** cable (with a chip in it) or a **powered USB-3 hub** acting as a repeater partway along. Over-long passive cable shows up as dropped frames and random disconnects, never as a clear error.

⭐ **Recommended shape**, cheapest first:

1. A **short** USB-C cable from the camera down the arm to a **powered USB-3 hub** clamped near the arm's base, then one cable from the hub to the laptop. Keeps the flexing part short and replaceable, and the hub is where the second D405 plugs in too. ⚠️ The hub must be **USB 3** and **externally powered** — a bus powered USB-2 hub will silently throttle both cameras. (A "Selore" hub is visible on the desk in the arm G photograph; check which kind it is.)
2. If a hub is not wanted, a single **active** USB-C cable of the length actually needed, clipped along the arm.

⚠️ **There is no wireless option worth considering.** The D405 has no computer of its own; it must be cabled to a host.

⭐ **And one thing that must happen after any re-cabling: re-verify the camera identification.** Moving a camera to a different port changes its index, and index is not identity ([FINDINGS §22](FINDINGS.md)) — this is the mistake that got two of four cameras wrong once already.

---

## ⭐⭐ 7.5 How fast can the arms actually move? — asked 2026-08-13, and MEASURED the same day

> ⭐ **Read §7.5.1 first.** It carries the measured answer. Everything before it is the reading that came first, and **two of its conclusions were refuted by that measurement** — they are struck through and left in place rather than deleted, because a prediction recorded in advance and then tested is the most useful thing this file can hold.

> Julien: *"why is there a 1.5 rad/s max speed? Is that based on the arms or our code? Could this be increased? Seems to be the same issue when we control the arms in teleop, and it lags behind, or moves longer than the mouse controls, right? Can we build our code in a way where the arms can move faster? What is the fastest theoretical movement for the arms?"*

### The short answers

1. ⭐ **1.5 rad/s is OUR number, not the arm's.** It is `MAX_JOINT_STEP` (0.015 rad per cycle) times `CONTROL_HZ` (100). Somebody chose 0.015 as a safety clamp on how far a joint may be told to move in one cycle.
2. ⭐ **Yes, it is the same limit that makes teleop feel laggy.** The clamp is applied to the joint targets the IK produces, so a quick push on the puck is trimmed and the arm falls behind the goal. `CartesianTeleop.lead()` already measures that gap and the status line warns `⚠️ STUCK lead` when it pins.
3. ⛔ **The model files do not state a real limit.** `yam.urdf` carries `velocity="1"` and `effort="1"` on every joint. Those are placeholders: 1 Nm cannot hold a 4.3 kg arm up, so the file was never filled in. **Nothing in the vendored SDK gives a joint speed limit for the arm.** (`flow_base_client.py` has velocity caps, and those belong to a mobile base, not to this.)
4. ⚠️ **The motors are very unlikely to be the limit at 1.5 rad/s.** Joints 1-3 are DM4340, joints 4-7 are DM4310, with gear ratios read off the bus as 40/40/40/10/10/10/10. A geared quasi-direct-drive motor of this class runs far above 1.5 rad/s at the output. ⛔ **Marked unverified**: no datasheet is vendored here, so this is not a measurement.
5. ⛔ **~~What actually limits tracking is the position gain~~ — REFUTED BY MEASUREMENT, 2026-08-13 16:35.** The gains are real, and they are not what sets the tracking error. Full account: [FINDINGS §34.1](FINDINGS.md), and the corrected answer is §7.5.1 below. From `third_party/i2rt/i2rt/robots/config/yam_v1.yml`:

   ```
   kp: [80.0, 80.0, 80.0, 10.0, 10.0, 10.0]
   kd: [ 5.0,  5.0,  5.0,  1.5,  1.5,  1.5]
   ```

   **The three wrist joints do have a position gain of 10, eight times softer than the shoulder and elbow.** ⛔ **But the measured difference in tracking error is 1.13x, not 8x.** Two things were wrong with the reasoning: `kd` drops alongside `kp` (5 → 1.5), so the quantity that governs following error is `kd/kp` and its ratio is only 2.4; and even 2.4 overpredicts, because the error is dominated by a **delay of about 0.033 s that is nearly identical on all six joints**. ⭐ Where the gains *do* show up is the standing position error while holding still: 0.039 rad on the stiff joints against 0.066 rad on the soft ones.

### ⭐⭐ BUILT 2026-08-13: the cheap half of the measurement, with no new motion

⭐ **Every playback now reports how well each joint kept up.** `src/recording.py::TrackingLog` records, per joint, the worst lag and the commanded speed at that moment, plus the top commanded speed and the lag at that moment. The table prints at the end of any playback longer than 20 cycles.

⭐⭐ **AND IT IS SAVED, since 2026-08-13 16:52.** Each playback writes `recordings/tracking/<slot>_<timestamp>.json` with the rows plus the arm, slot, commit, playback speed, taught speed, elapsed and waiting time, worst lag and the measured `loop_hz`. ⛔ **Added because the measurement behind §7.5.1 existed only as a paste into a chat window**, hand-copied out of a terminal, and could not have been repeated a week later. Timestamped rather than slot-named, so nothing overwrites anything — unlike the recordings themselves ([FINDINGS §34.7](FINDINGS.md)).

⛔ **WHY THIS RATHER THAN A SPEED SWEEP, and the reasoning is the point.** The obvious tool is a script that drives one joint faster and faster until it cannot keep up. That script would deliberately command the arm faster than any existing code allows, and **the agent cannot test it.** Session 4 is the standing warning: three changes passed their tests and produced three failures on first hardware contact, one of which dropped 4.3 kg ([FINDINGS §11](FINDINGS.md)). ⭐ A playback already commands a hand-taught path and already measures the lag, so **the same question can be answered with hardware time Julien is already spending and no new risk at all.**

⚠️ **How to read the table, because it is not a clean experiment:**

- The playback **holds its clock** once the arm falls behind, so the commanded speeds are uneven rather than a sweep. Every pair is real; the coverage is patchy.
- **Load depends on the arm's pose.** The same joint at the same speed lags differently with the arm extended and folded.
- It only reports speeds a recording happened to contain. His reach 2.9 rad/s at the 99th percentile, which covers the range in question.

⭐ **What to look for.** A joint whose worst lag reaches 0.15 rad at a low commanded speed is soft and needs a stiffer gain rather than a higher limit. A joint that reaches 2 rad/s with 0.05 rad of lag has room, and the clamp is what is holding it back. ⛔ **~~Expect the three wrist joints to look much worse than the shoulder and elbow~~ — this prediction was written here, tested on 2026-08-13, and REFUTED.** They look about 13% worse, not eight times worse. The reasoning error and the corrected model are in §7.5.1 below and [FINDINGS §34.1](FINDINGS.md). *The prediction is left visible rather than deleted, because a refuted prediction that was recorded in advance is the most useful kind of entry in this file.*

### ⭐⭐ 7.5.1 ANSWERED 2026-08-13, 16:35 — the arm follows a path with a fixed DELAY, not a gain-shaped error

> ⛔⭐⭐ **STOP — READ [FINDINGS §37.0](FINDINGS.md) BEFORE ANY NUMBER BELOW.** Everything in this section measures a *requested* speed. **`SafeRobot` rate-limits every command from every mode to 1.0 rad/s per joint**, below all control logic, and no analysis here accounted for it. **The arm has never been commanded above 1.0 rad/s in any measurement this repo holds**, so "the arm tracks up to about 1.9 rad/s" is wrong and the model built on it describes a clamp rather than the arm. ⭐ **What survives:** below 1.0 rad/s the arm follows with under 0.09 rad of error, and the gains barely shape that. **What to do about going faster is [FINDINGS §37.2](FINDINGS.md).**
>
> ⭐ **Refit 2026-08-13 17:21 with a third run.** The third run was held out of the model first, which measured how good the model actually is, and then folded in. Working, raw tables and every caveat: [FINDINGS §35.2](FINDINGS.md), with the two-run original at [FINDINGS §34.1](FINDINGS.md).

⭐ **The answer, in one line: the commanded position runs ahead of the real one by roughly `0.04 to 0.10 rad + 0.033 s × speed`, and the 0.033 s is the same on every joint.** Three playbacks on arm B produced 33 speed-and-lag pairs.

| what the model says | number |
|---|---|
| delay, mean over the three **kp 80** joints | 0.0335 s |
| delay, mean over the three **kp 10** joints | 0.0324 s |
| **measured delay ratio between them** | **0.97x — no gain dependence at all** |
| ratio predicted from `kp` alone (the answer §7.5 point 5 gave) | 8.00x |
| ratio predicted from `kd/kp` (a better theory, still wrong) | 2.40x |
| standing offset, **kp 80** joints | 0.037 rad |
| standing offset, **kp 10** joints | **0.080 rad** |
| **measured offset ratio** | **2.16x** |

⭐⭐ **So the error splits in two, and only one half cares about the gains.** The part that grows with speed is a **delay of 0.033 s, identical across both gain groups**. The part that does not grow with speed is a **constant, and it is 2.16x bigger on the soft joints**. ⚠️ **Do not read 2.16 as confirming the 2.40 from `kd/kp`** — `kd/kp` is the coefficient of the *speed* term, and the speed term is the one showing no gain dependence. Matching it to the constant would be treating a numerical coincidence as a mechanism.

⭐ **Why this changes what to do.** A gain-shaped error would mean *stiffen the wrist joints to go faster*, which is the more dangerous change and the one §7.5's closing paragraph rightly wanted evidence for. **The evidence says stiffening would not buy speed.** A delay shared by all six joints points at the **command pipeline**: the loop period is 11-12 ms at the 83-88 Hz measured, so 0.033 s is about three cycles, and the CAN request/response round trip and the SDK's own queueing are the other candidates. ⛔ **None of those three has been separated from the others**, and that is the only remaining reason to build the active sweep.

⭐⭐ **HOW FAST, IN PRACTICE — and the conclusion is to leave the 1.5 rad/s clamp exactly where it is.**

| commanded speed | joints the fit puts past the 0.15 rad hold threshold |
|---|---|
| 1.0 · 1.2 · **1.5 (the clamp)** | none |
| 2.0 and above | `forearm_pitch` (crosses at 1.89), `gripper_twist` (1.91) |

⛔ **But the scatter is what a person experiences, and it is large.** The held-out test measured this model as **±25% wrong on any single point**, and run C recorded `forearm_pitch` at **0.156 rad of lag at only 1.16 rad/s** where the fit says 0.131. At these slopes ±0.020 rad of fit error is worth about **±0.7 rad/s** of crossing speed. **So individual moments cross 0.15 rad anywhere between roughly 1.2 and 2.5 rad/s, depending on the pose.**

⛔⭐ **~~Decision: the clamp stays at 1.5, and this question is closed.~~ REOPENED the same evening.** The 1.5 rad/s clamp was never what bound anything: `SafeRobot` caps every command at **1.0 rad/s**, one layer below ([FINDINGS §37.0](FINDINGS.md)). ⭐ **The question is open again and it now has a concrete answer:** raise `SafeRobot(max_speed=…)` first, because nothing else changes until it moves, then `MAX_PLANNED_JOINT_SPEED`, and watch `SafeRobot.max_lag` (0.25 rad) rather than heat. The ordered plan is [FINDINGS §37.2](FINDINGS.md). ⚠️ **All three are safety limits and each is Julien's to raise.**

⚠️⚠️ **ONE CONSEQUENCE FOR THE RECORDER THAT IS NOT YET RESOLVED — see [FINDINGS §35.2](FINDINGS.md).** When a playback waits, the replay is **slower than the demonstration was**, unevenly, concentrated wherever the fast joints moved. Run C was 6.1% stretched. §6.6 already requires recording the pose actually commanded rather than the nominal plan, which fixes positions. **It says nothing about timing**, so a dataset built from replays carries human paths at slightly non-human timing. **Open question for step 5.**

### ⭐ The active sweep, if the range above is still too wide

Everything in §7.5 above the answer is reading. The number nobody has is **how fast a joint can be commanded before the arm falls further behind than we accept**, which is currently 0.15 rad for a playback and 0.25 rad inside `SafeRobot`. §7.5.1 narrows it to a range from hardware time already spent; a sweep would pin it.

**The design of the measurement, so it can be built straight away:**

- One joint at a time, starting with a **shoulder** (kp 80) and then a **wrist** (kp 10), because the answer will differ between them by a lot.
- Command a slow triangle wave of a fixed amplitude, well inside the joint limits with the existing margin.
- Step the speed up in small increments, holding each for a couple of seconds.
- Record commanded position, measured position and the resulting lag, and stop the moment the lag exceeds a bound well under `SafeRobot`'s 0.25 rad.
- ⛔ It sends setpoints, so it is Julien's to run under working contract rule 1, and it needs an abort on any key.
- Output: a table of speed against lag, per joint. **That single table sets `MAX_PLANNED_JOINT_SPEED` honestly and tells us whether raising `kp` is worth trying.**

⚠️ **Raising `kp` is a separate and more serious change** than raising a speed clamp. Stiffer joints hold position better and hit harder, and gravity compensation was already 39% short at the elbow once ([FINDINGS §11](FINDINGS.md)). It should follow the measurement, not precede it, and it should be one joint at a time.

## ⭐ 7.6 The second SpaceMouse as a continuous speed dial — his idea, 2026-08-13

> *"Could we add a mode where the second space mouse could be activated to control the speed of the robot movement in a continuous fashion?"*

⭐ **This is a good idea and it is better than it first sounds, because a SpaceMouse is spring-centred.** Let go and it returns to zero. **So a puck used as a speed dial is a deadman by construction**: release it and the motion stops. On a rig with no emergency stop ([HANDOFF §4.5](HANDOFF.md)) that is a real safety property rather than a convenience.

**Where it applies, most useful first:**

1. ⭐⭐ **Scrubbing a recorded movement.** Push forward to advance through the recording, release to freeze, pull back to run it backwards. That is a video editor's scrub wheel, which is the mental model he already uses for this project (he compared the easing to Premiere Pro). It also makes the playback speed something felt rather than typed, which is the right way to find a speed that tracks.
2. **Park and sequence runs.** The same dial on `p` runs, replacing `-`/`+`.
3. **Teleop.** One puck drives, the other scales how fast. ⚠️ Less obviously useful, because the puck's own deflection is already a speed.

⛔ **The conflict: the second puck is currently arm G's.** `pick_device_by_wiggle(exclude=…)` assigns one puck per arm. A speed dial therefore needs either a single-arm session or a third device.

> ### ⛔⭐ CORRECTED 2026-08-14 — the sentence that used to follow overstated the blocker
>
> It said: *"So this belongs after the two-arm work, not before it, or it will fight the assignment logic that already exists."* ⭐ **That is weaker than it sounds, and Julien asked about this feature again on 2026-08-14, which is what prompted re-reading it.**
>
> **Today only ONE arm is driven.** Arm G is shared with a colleague and usually unplugged ([FINDINGS §35.6](FINDINGS.md)), and every session so far has been arm B alone. **So the second puck is genuinely free right now**, and the conflict is with a *future* two-arm session rather than with anything that exists.
>
> ⭐⭐ **The real argument for waiting is narrower: the puck would need a third role in the assignment logic ("this one is a dial, not an arm"), and designing that before the two-arm work means designing it twice.** That is a cost, and it is a much smaller one than "it will fight the existing logic".
>
> ⭐ **Recommendation, and it is his call.** The most useful version by his own ranking is **scrubbing a recorded movement**, and that one is the least entangled: it is about *playback*, not about driving an arm, so it touches the assignment logic once and the control loop hardly at all. **It could be built before step 2 if he wants it.** ⚠️ Rough size: opening and reading the second puck is already solved code, so the work is the scrub loop and its mode, plus tests. **Comparable to one feature-sized session.**
>
> ⚠️ **What has NOT changed:** the design caution below still holds, and so does the reason the dial must be a mode you enter rather than a change to how playback always works.

⚠️ **One design caution.** Mapping a spring-centred axis to speed means the neutral position is *stopped*, so a run needs continuous input to proceed. That is right for scrubbing and wrong for a long unattended playback. **Both behaviours are wanted, so the dial should be a mode you enter rather than a change to how playback always works.**

## 8. ⭐⭐ What is still missing for the whole system to work — 2026-08-12

> Julien asked for this directly: *"everything for the whole setup needs a full list of what is still missing for the full system to work."* Built from a photograph of the desk he sent on 2026-08-12, plus `ioreg` run the same day. Written in plain language because he reads it.

### 8.0 What the rig is, right now

From the photograph and from `ioreg`, both on 2026-08-12:

- Two arms on separate base plates, on the **same wooden desk as the laptop**, facing inward. Base serials `JG260704018` (B) and `JG260704022` (G).
- One MacBook drives everything. Both CAN adapters are on the bus (`2081337C594E5018` = B, `20593383594E5018` = G).
- ⭐ **Both SpaceMice are connected**, one either side of the laptop.
- One USB hub, to the left of the laptop.
- **Arm B carries the D405** (serial `255323071773`, USB SuperSpeed). **Arm G carries the Logitech C920.**
- ⛔ **Only one D405 is on the bus.** The second is still unplugged.
- Cables run loose across the desk and down to the floor.
- ⛔ **No camera looks at the workspace from outside.** Both cameras are on wrists.
- ⛔ **No emergency stop.** The wall plug is the only cut-off ([HANDOFF §4.5](HANDOFF.md)).

### 8.1 Hardware still missing

| what | why it matters | how urgent |
|---|---|---|
| ⭐⭐ **A camera that sees the whole workspace**, and something to mount it on | ⛔ **The biggest gap, and it is on no other list.** Nearly all published manipulation work uses a wrist camera **and** a fixed camera watching the scene. A wrist camera alone cannot see where the object is until the arm is already near it, so a model trained on wrist views only has to search blindly at the start of every episode | **high**, and before collecting any dataset |
| The second D405, on arm G | Two different cameras on two arms means two fields of view, two colour responses and two latencies in one dataset. He is already planning this | high |
| A powered **USB 3** hub with enough ports | Two D405s streaming at once may exceed what one laptop controller carries. ⚠️ It fails as dropped frames, never as an error | measure before assuming |
| Objects to manipulate, and somewhere repeatable to put them | The task is undecided, but a dataset needs the same object and a marked start area. The "RobCo building blocks" bag in the photograph may already be this | with the task |
| Cable clips or sleeving | Loose cable inside the arms' reach is a snag risk on a rig with no emergency stop | cheap, do it whenever |
| A longer USB cable per wrist camera | Only needed if the cable is routed along the arm. See §8.4 | deferred, his call |
| ⚠️ **A switched mains power strip within arm's reach** | There is no emergency stop and buying a real one may not be worth it. A switched strip is a few euros and gives one thing to hit. Not a substitute for a proper stop, and better than reaching for a plug | worth one decision |
| Something holding the base plates down | The plates sit on the desk with the laptop. ⚠️ **Unknown from the photograph whether they are clamped.** Worth checking rather than assuming | check |

### 8.2 ⭐⭐ EVERY open piece of work, in the order I would build it

> ⛔ **This table is the single complete list.** Julien asked on 2026-08-13 to *"deep check if everything was sensibly noted down"*, so anything discussed anywhere appears here with a pointer, whether it is his idea, mine, or a defect. If something is not in this table it is not tracked.
>
> ⭐⭐⭐ **SCOPE RULING 2026-08-18 ([FINDINGS §67.0](FINDINGS.md)): this repo is a finished WALKTHROUGH for a from-scratch rebuild by his team.** Finish the features and the setup, write the consolidation plan ([§8.5](#85--the-consolidation-task-he-asked-for-and-when-it-should-happen)), and drop anything that measures or hardens THIS bench as if it were final. Rows are marked rather than deleted, because "decided against, and why" belongs in the plan. The filter, applied:
>
> - ✅ **Closed by his 2026-08-18 report:** 29 (jaw block confirmed) · 39 (catchup ran, weak, superseded by 44) · 42 (ceilings stay).
> - ⬜ **In scope — finish the features:** 3 + 10 (the jaw pause, and wiring `check_grasp` into it) · 6 (timestamped multi-camera capture) · 7 (the dataset recorder/export) · 8 (labels while driving) · 9 + 12 (noise per waypoint, mixed runs) · 5 (telling two D405s apart — waits for the second D405 on the bus) · 26 (record-while-mirroring, one free labelled try).
> - ⬜ **In scope — make the walkthrough code truthful:** 23 (collapse the two parks) · 21 (throttle message) · 28 (control frame on the status row) · 31 (`chain_alive`) · 43 (settings screen hides the status row) · 45 (exit summary frames).
> - ✅ **All four one-liners answered 2026-08-18 ([FINDINGS §67.7](FINDINGS.md)):** 32 closed (Marius's file arrived) · 38 is the agent's to settle (newest prompt cancels the older) · 13 build it · 44 build it. ⭐ **And the standing principle those answers carry: when a feature hesitates between "build here" and "plan-note", the default is BUILD** — the prototype exists so the plan can explain things that were actually done once.
> - ⏸ **Out of scope by the ruling — carried into the plan as notes, not built here:** 14 (loop rate, now trended) · 15 (park timing stays as-is) · 18 (scaling-limit verification) · 20 (powered USB 3 hub — a hardware fact the rebuild must know) · 22 (desk height) · 25 (collision stays manual, his standing ruling) · 27 (max-lag measurement) · 35 (finer collision geometry — only needed for a warning nobody ruled in) · 37 (per-joint speed ceilings).


| # | what | state |
|---|---|---|
| 1 | ✅✅⭐⭐⭐ **Both arms from one script — STEPS 1-5 DONE, THE TWO-ARM RECORDER BUILT AND UNRUN, 2026-08-14** | ✅ Steps 1-4 confirmed on the hardware: `ArmSession`, `--arms B,G`, the first two-arm run, GUIDE on each arm and on both. ✅ **Step 5, MIRROR, ran** — *"works really well in general… G followed."* ⏳ **Step 6, the two-arm recorder, is built and has never run**: a recording is every arm's joints in one timeline, ABC's own shape, old files still play, and `w`/`l` no longer refuse. **Procedure: [FINDINGS §56.8](FINDINGS.md).** ⭐ **512 headless tests, eight mechanical checks.** ⛔ New from mirror: **nothing knows where the other arm is** (item 25). [ROADMAP §6.1](#61-audit-before-starting-2026-08-13--armsession-has-fallen-a-day-behind-and-the-plan-below-was-not-executable-as-written) |
| 2 | ⭐ **Record a movement in GUIDE, then replay it** | ✅ **built 2026-08-13** (`src/recording.py` + 37 tests, wired into the session as `w` and `l`). ⛔ Unconfirmed on the arm. [ROADMAP §6.6](#66-where-the-training-data-comes-from) |
| 3 | ⏳ **A pause where the jaws move** | **half built 2026-08-13, and the design turned out better than a dwell time.** ✅ `src/motion.py::plan_gripper_stops` decides where a run must split, with 9 tests. ⬜ The park machinery has to run the segments and wait for the jaws. ⭐ **No configuration is needed**, which is why this is now smaller than it looked. [ROADMAP §6.6.2](#662-a-pause-where-the-jaws-move--built-as-a-decision-2026-08-13) |
| 4 | ⏳⭐⭐ **Mirroring — BUILT 2026-08-14, never run** | ✅ `src/mirror.py` + 18 tests, and `i` in the session engages it: the selected arm leads, the other follows joint for joint. It asks twice like `l`, because engaging moves the follower while the operator's hands are on the leader. ⬜ **Ten-minute procedure: [FINDINGS §55.7](FINDINGS.md).** ⚠️ `MIRROR_SIGNS` is a geometric prediction and only matters for `--mirror mirror` |
| 5 | **Telling two identical D405s apart** | nothing exists. Use the wiggle approach, [FINDINGS §28.5](FINDINGS.md). ⛔ **NEEDS BOTH D405s ATTACHED — only one was on 2026-08-17.** The whole problem is distinguishing two identical devices. ⛔⭐ **And step zero for EVERY camera item: macOS camera access is NOT granted**, so OpenCV opens nothing at all ([FINDINGS §60.4](FINDINGS.md)) |
| 6 | **Capturing several cameras with timestamps** | nothing exists. Images must line up with joint data or the dataset is unusable |
| 7 | **The recorder** (step 5) | nothing exists. Metadata list in [ROADMAP §6.6](#66-where-the-training-data-comes-from) |
| 8 | **Good/bad labels while driving** | nothing exists. Keypress first, microphone second |
| 9 | **Noise per waypoint** | nothing exists |
| 10 | ✅ **Detecting whether a grab worked**, from the gripper position | **built 2026-08-13.** `src/yam_robot.py::check_grasp`, 7 tests. ⭐ It needs no new hardware and no per-object calibration: calibration closes the jaws onto themselves, so a normalised 0 means empty and anything above it is an object's width. ⛔ It refuses to answer unless the jaws were told to close AND have stopped moving, because mid-close looks identical to holding a wide object. ⬜ Nothing calls it yet; it needs the pause in item 3 first, since that is where a run waits for the jaws |
| 11 | ✅✅ **Measure how fast a joint can actually be commanded — CLOSED 2026-08-13 17:21** | ✅ **Answered from three playbacks and no new motion, then tested on held-out data.** Lag ≈ `0.04 to 0.10 rad + 0.033 s × speed`, and the 0.033 s is the **same on all six joints** (ratio 0.97), so the limit is a shared delay rather than the per-joint gains. The gains show up only in the constant, 2.16x bigger on the soft joints. ✅ **Decision: the 1.5 rad/s clamp STAYS.** Individual moments cross the 0.15 rad hold threshold anywhere from about 1.2 to 2.5 rad/s depending on pose, so the clamp sits inside the scatter band, which is the right place for it. ⭐ **No further speed test is needed**, and the threshold playback this item used to ask for happened by accident at 0.607x. ⬜ **The active sweep stays designed and unbuilt, and now has exactly one remaining purpose:** separating the delay's three candidate causes (loop period, CAN round trip, SDK queueing). That is a different question from "how fast", so build it only if that question becomes worth answering. [ROADMAP §7.5.1](#751-answered-2026-08-13-1635--the-arm-follows-a-path-with-a-fixed-delay-not-a-gain-shaped-error) |
| 12 | ⭐ **Mixed runs: planned legs and hand-taught legs in one sequence** | nothing exists. His idea, and it turns the noise rule into a data structure. Needs 3 first. [ROADMAP §6.6.1](#661-mixing-waypoints-and-recordings-in-one-run--his-idea-2026-08-13) |
| 13 | ⬜⭐⭐ **The second SpaceMouse as a continuous speed dial — IN SCOPE, his ruling 2026-08-18** | ⬜ **Build it in the prototype.** His ruling, and it carries the general principle: *"I would build everything in this prototype so that we know how to do it, and then we can explain how to do it in the rebuild plan."* His note: *"should be quite easy… should be made intelligently in some way"* ([FINDINGS §67.7](FINDINGS.md)). His own ranking of the roles still stands: scrubbing a recorded movement first | nothing exists. **His idea, first raised 2026-08-13 and raised again 2026-08-14** (*"didn't I already tell you about this?"* — yes, and it was written up the same day). ⛔ **The old note said this needs the two-arm work first, and that overstated it**: only one arm is driven today, so the second puck is free, and the real cost is designing a third puck role twice. ⭐ **The most useful version by his own ranking, scrubbing a recorded movement, is the least entangled and could be done before step 2.** His call. [ROADMAP §7.6](#76-the-second-spacemouse-as-a-continuous-speed-dial--his-idea-2026-08-13) |
| 14 | ⚠️ **Find out why the control loop runs below 100 Hz — and it got SLOWER** | ⛔ **83 and 84 Hz in the two runs of 2026-08-13 16:34, against ~87 Hz measured earlier the same day.** The warning fires correctly (its threshold is 92 Hz). Unexplained candidates: the **second D405 that appeared on the bus** ([FINDINGS §34.5](FINDINGS.md)), general machine load, the per-cycle tracking log. ⭐ **Every playback now writes `loop_hz` into its saved tracking file**, so this becomes a trend rather than an anecdote. ⚠️ **It also makes a constant wrong:** `MAX_PLANNED_JOINT_SPEED` is `0.015 × 100`, so the real teleop ceiling is ~1.25 rad/s not 1.5 ([FINDINGS §34.2](FINDINGS.md)). [FINDINGS §31.1](FINDINGS.md) |
| 15 | ⚠️ **Decide whether PARK should use measured time** | ⛔ **Deliberately NOT changed.** A park at "0.40 rad/s" actually moves at about 0.35, because the cursor advances in nominal time. It is slower than stated, which is the safe direction, and Julien has tuned his speed preferences against the current behaviour. Changing it would silently speed every park up by 15%, so it is his call. [FINDINGS §31.1](FINDINGS.md) |
| 16 | ✅⛔⭐⭐⭐ **Read the D405's 848x480 mode as 16-bit depth — CLOSED 2026-08-17, ANSWERED NO** | ⛔⭐⭐ **The D405 is a plain COLOUR camera over UVC on macOS.** All four modes, **848x480 included** (it does exist, the old probe simply never asked for it), read as ordinary photographs: **0.00% exactly-zero pixels**, three channels that differ, no rough-beside-smooth split ([FINDINGS §63.0](FINDINGS.md)). ⭐ **What the NO buys: nobody needs to spend a day on `librealsense` and `sudo`** hoping for free depth. Depth needs the SDK or it needs nothing | ⛔ **`--probe` returns `ÿÿÿÿ` for the codec in EVERY row**, which is `CAP_PROP_FOURCC` returning -1: the pixel format is **unreadable** through macOS's AVFoundation backend, and re-running cannot change that. ⚠️⭐ **Two corrections to this item's own premise**: 848x480 was never swept (the probe's list is fixed at 1920x1080/1280x720/960x540/640x480/424x240), and every larger request collapses to 1280x720. ⭐ **The answer must come from the PIXEL DATA, not a label** → item 36 ([FINDINGS §62.5](FINDINGS.md)) | ⭐ **Nobody had noticed that `camera_view.py --probe` already sweeps resolutions and codecs and reports the real FOURCC pixel format.** So this needs ONE command from Julien, not new code: `uv run scripts/camera_view.py --camera d405 --probe`. ⛔ **The agent CANNOT run it** — macOS grants camera access per parent app and an agent shell has a different identity ([FINDINGS §61.3](FINDINGS.md)). ⚠️⭐ **And there is a contradiction to settle**: the device calls itself a depth camera and the code warns to expect depth, but his 2026-08-17 measurement says **colour at 1280x720**. ⛔ Do not force the mode into 8-bit before the FOURCC is read | ⛔ **Do not touch until the pixel format is checked.** If 848x480 is the depth stream then depth is available with no SDK, which has been an open goal since [FINDINGS §8](FINDINGS.md). Forcing it into an 8-bit photograph would throw that away. [FINDINGS §31.2](FINDINGS.md) |
| 20 | ⚠️⭐⭐ **A powered USB 3 hub for the CAN adapters, NOT shared with a dock** | ⛔ **New 2026-08-14, and it now has the strongest evidence of any hardware item.** Both CANables sit behind two cascaded hubs and a *"USB BillBoard"* dock on bus 0. On 2026-08-14 the arm fell mid-session, **all seven motors latched `0xD loss of communication`**, both adapters came up in their bootloader, and one camera stopped reporting its serial ([FINDINGS §44.3](FINDINGS.md), [§46.0](FINDINGS.md)). **Seven motors reporting a lost bus means the adapter went away, not a motor.** ⛔ **The sag is not fixable in software** — once the link is gone nothing can command a hold ([FINDINGS §44.4](FINDINGS.md)). **This is the fix.** Fourth DFU occurrence |
| 21 | ⚠️⭐ **The throttle message names a cause it never measured — the LAST of four** | ⛔ New 2026-08-14. `⚠️ SLOWED to 19% (near the reach limit)` printed while the arm's manipulability was **0.1713**, which that throttle's own docstring calls *"middle of the workspace, comfortable"* ([FINDINGS §41.2](FINDINGS.md)). **The line shows the throttle percentage and not the speed setting it is throttling**, so the number cannot be read either way. ⭐ **Fix: show the linear speed and the effective speed, and say what was measured rather than what is assumed.** Small, and it belongs with any other status-line work |
| 22 | ⚠️ **Measure the desk height once** | ⛔ New 2026-08-14. The floor limit is `0.0`, the model's base plane, after Julien rejected `+0.05` (*"then I can't really pick anything up from the table"*) and `−0.10` (*"it's still gonna crash into the table"*) ([FINDINGS §47.1](FINDINGS.md)). **Both objections were right and together they bracket the answer.** ⚠️ It is a sanity bound rather than desk protection **until the desk height relative to the model origin is measured**, which [ROADMAP §8.4](#84-deliberately-deferred-and-by-whose-decision) defers by his own ruling. `--floor -0.005` if a flat object needs a few mm |
| 18 | ⚠️⭐ **The motor scaling limits cannot be verified, and a wrong one is silent** | ⛔ **New 2026-08-14, and it is a known blind spot rather than a task with an obvious next step** ([FINDINGS §38.1](FINDINGS.md)). `PMAX`/`VMAX`/`TMAX` are **not** motor registers this stack can read — the SDK decodes every feedback frame with hardcoded Python constants (`POSITION_MAX 12.5`, `VELOCITY_MAX 45`, `TORQUE_MAX 54` in `i2rt/motor_drivers/utils.py`). If a motor's stored limits were ever changed, every position, velocity and temperature reading would be wrong by a constant factor **and nothing would raise**. ⭐ What bounds it today, for free: the gripper limit reconciliation rules out a gross change, because a doubled scale would push arm G's jaw outside its saved band. ⚠️ A ~10% change is not ruled out. ⛔ And "both arms read zero at rest" is **not** evidence, because zero is scale-invariant. **Next step if it ever matters: measure one joint's true angle physically and compare it to the decoded value.** Not worth doing until something depends on absolute accuracy |
| 19 | ⭐ **Document what the two motor LEDs on each arm mean — ✅ DONE 2026-08-14** | ✅ From the DAMIAO DM-J4340-2EC manual, cross-checked against the SDK's own error table ([FINDINGS §39.0](FINDINGS.md)). **Green steady = enabled · red STEADY = disabled, the normal idle state · red FLASHING = a latched fault**, codes `8`-`E`. ⛔ **It refuted [FINDINGS §36.0](FINDINGS.md)'s hypothesis that blinking was the normal idle indication**, so arm G's lights on 2026-08-13 were a real fault. ✅ Also fixed the reason its type is unknown: `ping_motors.py`'s `--attempt-error-clear` flag was wired to nothing, so the vendor's clear-loop erased every fault it was sent to read ([FINDINGS §39.1](FINDINGS.md)) |
| 29 | ✅⭐⭐⭐ **The jaw block — CONFIRMED ON HARDWARE 2026-08-18: the stall fires ONCE** | ✅ **His report, 2026-08-18: mirror with an object in the follower's grip *"worked out fine"*, and *"the block only shows once."*** That confirms the 3% clearing margin AND the message rationing in one run ([FINDINGS §67.2](FINDINGS.md)). ⚠️ One unparsed phrase from the same report (*"the camp would like to go slower"*) is recorded there and asked in chat | ✅ `ArmSession.hold_jaw` latches at the stalled jaw position and holds further-closed commands there, keeping the grip while the motor stops pushing. ⛔ **His first run showed it firing three times** (0.117, 0.098, 0.104) because the clearing rule released on ANY value above the block, and a measured jaw jitters by thousandths — so noise disarmed it. ⭐ A **3% margin** now separates jitter from a deliberate open. ⛔⭐ **I also had a test asserting the wrong behaviour and defending it in its docstring** ([FINDINGS §63.1](FINDINGS.md)). ⬜ **Needs one hardware run**: mirror with an object in the follower's grip, and the stall should fire ONCE | ✅ `ArmSession.hold_jaw` latches at the stalled jaw position. Further-closed commands are held there, so the motor stops pushing **but keeps the grip**; any OPENING command is obeyed and clears the latch. Wired into MIRROR **and** TELEOP, since both re-command the jaws every cycle. ⛔ **The release always worked and was undone next cycle** — his log shows 0.152, 0.151, 0.150, 0.147 and *"14 times now"*, because a one-cycle correction against a 90 Hz source can only nibble. 11 tests including his full scenario ([FINDINGS §62.3](FINDINGS.md)) | ⛔ **New 2026-08-15, from Julien squeezing the leader's jaws while the follower already held something.** In MIRROR the follower's jaw command is the leader's MEASURED jaw position, re-sent every cycle, so the stall guard releases and the next cycle pushes back on, every 0.4 s for as long as he squeezes ([FINDINGS §58.2](FINDINGS.md)). ✅ **The message is rationed now** (once, then every five seconds with a count). ⬜ **The real fix: remember the release point and refuse to command tighter until the leader opens past it.** ⛔ **Do NOT guess the sign.** "Tighter" in RAW joint terms is not obvious, the raw jaw frame flips by ±2π between sessions ([§40](FINDINGS.md)), and a wrong sign would command the jaws HARDER onto the object — the exact way motor 7 was cooked three times. Establish the direction from `calibrate_gripper.py`'s output or a two-line probe first |
| 30 | ✅⭐⭐⭐ **A simulation harness that runs the whole session loop with no hardware — DONE 2026-08-17** | ✅ `src/fake_arm.py` + `--sim`. An arm that **lags**, with constants from the measured law in item 11, wrapping the **real** `SafeRobot`. A simulated two-arm session records, saves, parks with *"waiting for B"*, replays a REAL recording, prints the tracking table and disables all 14 motors — **18/18 checks, no hardware** ([FINDINGS §60.2](FINDINGS.md)). ⭐ Cross-checked against hardware on the same recording: real 0.029-0.076 rad of lag, simulated 0.056-0.069. ⭐ Driving it found **four** defects, including a simulated recording written to `recordings/` labelled `live:`. ⛔ Cannot speak to feel, gravity or thermals | ✅ `src/fake_arm.py` is built and tested: an arm that **lags**, with `tau` and the friction deadband taken from the measured law in item 11 rather than invented, wrapping the **real** `SafeRobot`. It can block a joint, kill the chain and blind the thermal read. 16 tests, and `scripts/falsify_fake_arm.py` breaks it five ways to prove they bite ([FINDINGS §59.0](FINDINGS.md)). ⛔ **STILL MISSING and it is the half that pays off:** `scripts/teleop_session.py` has no `--sim` flag, so the loop still cannot run without an arm. Deliberately deferred — that edit touches the session script and Julien was driving the real arms with it | ⭐ **New 2026-08-15, and it is the item that unblocks the most.** Three of this week's defects would have been caught by it: a cursor advanced twice per cycle, a playback that cancelled itself, and a stale variable feeding the tracking log ([FINDINGS §58.4](FINDINGS.md) item 2). ⭐ **Most pieces exist**: `scripts/teleop_sim.py` drives the IK with no arm, MuJoCo and `mink` are already dependencies, and `test_park_arms.py` plus `test_status_row.py` show the fake-robot shape. ⛔ **What is missing is a fake that behaves like the real thing over TIME** — a first-order lag per joint so a command is followed rather than teleported, plus `SafeRobot`'s rate and following-error limits — so `main()`'s loop can run end to end. ⚠️ **It cannot replace hardware for feel, gravity compensation or thermal behaviour**, and saying so is part of building it |
| 32 | ✅ **The `Setup-Anleitung.md` link — CLOSED 2026-08-18: the file existed all along, on Marius's side** | ✅ **Julien sent Marius's original via WhatsApp; copied in byte-identical as [docs/Setup-Anleitung.md](Setup-Anleitung.md), and the repo resolves 991/991 links for the first time.** It is the team's original plan (gates, the MCAP contract, the two-branch training strategy, the 12-mistakes table) — the consolidation plan should be written against it ([FINDINGS §67.7](FINDINGS.md)) | ⚠️ **Found 2026-08-15.** `check_links.py` has been reporting `787/788` all along and every session read that as "fine". It is the ONLY unresolved link, it is **not** from this session's work, and it sits at the end of a German paragraph about operative steps after Gate D1. ⛔ **Do NOT guess the fix.** Either that file was planned and never written, or the link should point at [COMMANDS.md](COMMANDS.md) or back into [Setup-Plan.md](Setup-Plan.md) itself — only Julien knows which, and inventing a `Setup-Anleitung.md` to satisfy a checker would be worse than the broken link. ⭐ **Ask him.** No hardware needed. ⚠️⭐ **AND I BROKE A SECOND LINK IN THIS VERY ROW while writing it** — it pointed at a `SETUP.md` that does not exist, so the count went 787/788 → 788/790 and I nearly shipped it. **`check_links.py` must run before the commit, not after** |
| 33 | ✅⭐⭐⭐ **The flags become SAVED DEFAULTS — DONE 2026-08-17** | ✅ `src/settings.py` + `--save-defaults`. Three layers: built-in constant → `config/session_defaults.json` → command-line flag, and a typed flag always wins. ⛔ `--yes`, `--arms` and `--sim` can never be saved, each for a stated reason. ⭐ Four of the settings are safety limits, so the plan names any saved value **looser than the built-in** — a flag is visible in shell history, a saved default is not. ⛔ **Gitignored on purpose**: a `git pull` must never change how fast the arm may move. 26 tests ([FINDINGS §61.1](FINDINGS.md)) |
| 34 | ✅⭐⭐ **Change the settings LIVE — DONE 2026-08-17, the `n` key** | ✅ A SETTINGS screen listing the six limits, with the built-in value shown beside any that differ. `1-6` picks · `-`/`+` changes · `0` reverts to the session start · `s` saves · `t`/`g`/`h` leaves. ⛔ `max_speed` and `max_lag` are pushed onto the live `SafeRobot` objects, so a change binds on the next cycle, and the screen says so. ⭐ **I asked whether this was wise and he was right that my worry was confused**: a keypress he makes IS him typing the value, and the axis map is the precedent ([FINDINGS §62.4](FINDINGS.md)) | ⚠️ **New 2026-08-17.** His words were *"default options that can be changed in some controls mode and then should be saved"*. ✅ The saving half is done (item 33) and covers the complaint he actually had, which was retyping six flags. ⬜ **Not done: editing them mid-session.** ⭐ Some already have live keys (`-`/`+` linear, `,`/`.` rotation, `ö`/`ä` ease), so the shape exists; what is missing is the speed and limit settings and a key to write the file. ⚠️ **Ask him whether he wants this at all** before building it — a live editor for a SAFETY limit is a different proposition from a flag, because the value that gets saved may then be one nobody typed |
| 35 | ⬜⭐⭐ **Finer collision geometry: `mj_geomDistance` against the real meshes** | ⛔ **New 2026-08-17, and it is the PREREQUISITE for any collision warning.** At his ~0.70 m spacing the bounding-sphere estimate says **2.5 cm at rest**, because `link3`'s declared sphere is 0.197 m around a much thinner link. **So a warning built on it would fire constantly while the arms sit still.** ⭐ MuJoCo can measure mesh-to-mesh distance directly. Hardware-free ([FINDINGS §60.3](FINDINGS.md)) |
| 36 | ✅⭐⭐ **Tell depth from a photograph by looking at the PIXEL DATA — DONE 2026-08-17, and it answered item 16** | ✅ `scripts/probe_camera_pixels.py`. Julien ran it and every mode came back an ordinary photograph ([FINDINGS §63.0](FINDINGS.md)). ⭐ Its 9 synthetic tests **found a blind spot in it first**: a divide-by-zero guard silently skipped the 16-bit check in exactly the case it exists for. ⛔ The agent can never run it | ✅ `scripts/probe_camera_pixels.py --index 0`. Captures one frame at **848x480** (which nothing had ever requested), 1280x720, 640x480 and 424x240, and reports four signatures: a spike of exactly-zero pixels (failed stereo matching), three identical channels, one channel smooth beside one rough (a 16-bit value split across bytes), or ordinary colour statistics. Saves every frame as `.npy`. ⭐ 9 synthetic tests, and they **found a blind spot**: a divide-by-zero guard silently skipped the 16-bit check in exactly the case it exists for. ⛔ **The agent can never run it** ([FINDINGS §62.5](FINDINGS.md), [§61.3](FINDINGS.md)) | ⛔ **New 2026-08-17, and it replaces item 16's approach.** OpenCV cannot report the D405's pixel format on macOS at all, so asking for a label is a dead end. ⭐ **A 16-bit depth frame packed into 8-bit channels has statistics no photograph has**: one channel varying smoothly while another varies wildly, or all three channels identical. So capture ONE frame at each of the three real sizes (1280x720, 640x480, 424x240) and report shape, dtype, per-channel histograms and channel-to-channel correlation. ⛔ **The agent can write this and can NEVER run it** ([FINDINGS §61.3](FINDINGS.md)) |
| 37 | ⬜⭐ **Joint 6 (`gripper_twist`) tops out near 2.6 rad/s, and nothing else has been measured this way** | ⭐ **New 2026-08-17, from a MIRROR run that measured it.** The follower was allowed 10 rad/s and managed **2.64** on joint 6, which is what made the diagnosis switch to *"the arm, not the software"* ([FINDINGS §62.0](FINDINGS.md)). ⬜ **Worth doing: the same measurement for all six joints**, so the per-joint ceiling is known rather than discovered one mirror stop at a time. ⚠️ MIRROR already produces the data; it just is not being collected |
| 38 | ⚠️⭐ **Two pending prompts can be open at once, and nothing says so** | ⚠️ **New 2026-08-17.** His log has the gripper-button learning (`b`) and the SETTINGS screen (`n`) both armed together. They coexist because `b` waits on a PUCK button while SETTINGS reads the KEYBOARD, so neither blocks the other, and **a puck press he made for one was consumed by the other**. ⛔ Harmless today: nothing moves and no setting is corrupted. ⚠️ But two invisible modal states is the shape that produces a *"why did that do nothing?"* session later. ⭐ **Needs Julien's decision on which should win** before it is built ([FINDINGS §63.3](FINDINGS.md)) |
| 39 | ✅⏸⭐⭐ **The mirrored arm sits ~2 cm short — CATCHUP RAN 2026-08-18, HIS VERDICT: WEAK. Closed; the real path is item 44** | ✅⏸ **Ran on hardware, reported 2026-08-18, and the feature is weak by his own verdict**: jittery and shaking at high values (20), slow aimless drift even at low ones (*"it doesn't really work that well"*). ⭐ Both behaviours are what an integrator aimed at a moving noisy target must do, so the design is the limit rather than a bug ([FINDINGS §67.1](FINDINGS.md)). ⛔ Default stays OFF, the 20 ceiling stays (his ruling, same report). **The real fix for the droop is velocity feedforward, item 44** | ⭐⭐ **Julien's question, and the answer is both halves.** The follower is position-controlled, so it settles where its motor force balances gravity and friction, always SHORT of the command — the 0.04-0.10 rad constant in item 11's measured law. ⛔ **And nothing in software ever read that back**: `follower_target()` copies the leader's angles and the command converges to exactly them, so the follower sits at `leader − droop` forever. ⭐ Measured: 0.024 rad of joint error is **11.3 mm at the tip** in his reaching pose, so his 2 cm sphere is arithmetic rather than an impression. ✅ `--mirror-catchup` (setting 7 on the `n` screen) accumulates the error into a clamped bias and aims past the leader. ⛔ **DEFAULT OFF** — it changes what 4.3 kg does. ⬜ **Needs one hardware run: `--mirror-catchup 3` and see whether the sphere closes** ([FINDINGS §64.0](FINDINGS.md)) |
| 40 | ✅⭐⭐⭐ **The mirror gap is PER JOINT now, 2026-08-17** | ⛔ Every mirror stop in his logs was on joint 5 or 6, the two that barely move the tip, and one threshold for all six is why he had to raise `--mirror-gap` to **1.335** — which at the elbow's 0.418 m/rad allows **56 cm of tip error**. ⭐ Measured tip metres per radian across four of his own poses: 0.333, 0.390, **0.418**, 0.169, 0.100, **0.051**. So the multipliers are `(1.26, 1.07, 1.00, 2.48, 4.00, 4.00)`, capped at 4x. ⭐ His 0.364 rad wrist stop passes at the DEFAULT gap now; the same error on the elbow still stops it. ⚠️ **The cap is deliberate**: 1.4 rad on the gripper twist is 80° of gripper rotation, which ruins a grasp while barely moving the tip ([FINDINGS §64.1](FINDINGS.md)) |
| 41 | ✅⭐⭐ **The settings keys walk a LADDER of round numbers, 2026-08-17 — and its fallback carried a regression, FIXED 2026-08-18** | ⛔⭐ **The rework's fallback moved a value the WRONG WAY when it sat outside the ladder's ends** (legal via flag or saved default): `+` on `floor` 0.5 dropped it to 0.1, `-` on `max_speed` 0.1 raised it to 0.25. The tests caught it and sat red, unseen — no runner runs the suite as a whole ([FINDINGS §67.5](FINDINGS.md)). ✅ A press with no rung left in its direction now leaves the value untouched | ⛔ The 1.25 ratio gave him `1.000 → 1.250 → 1.562 → 1.953 → 2.441 → 3.052 → 3.815 → 4.768 → 5.960 → 7.451`, so **2, 4 and 10 were unreachable by key** and the values leaked into messages as `limit 1.33514404296875`. ✅ Ladders per setting, a between-rungs value moves to the nearest rung in the direction pressed, and every limit printed to a person is formatted ([FINDINGS §64.2](FINDINGS.md)) |
| 42 | ✅⭐ **Every live ceiling was REACHED in one session — ANSWERED 2026-08-18: they stay** | ✅ **His ruling, 2026-08-18, given while reporting the catchup test at 20:** *"It could stay that way"* — the ceilings are backstops, not working values, and none is lowered ([FINDINGS §67.1](FINDINGS.md)) | ⚠️ **New 2026-08-17.** He ended a session at `max_speed 20`, `teleop_speed 20`, `max_lag 3.0` and `linear_scale 2.0` — all four at their ceilings, from pressing `+`. ⭐ They are doing their job as a backstop against a held key. ⛔ **They are NOT out of reach**, so nothing should be treated as a margin just because a ceiling exists |
| 43 | ⬜⭐ **The `n` screen hides the status row, so a setting cannot be judged while it is tuned** | ⚠️ **New 2026-08-17.** He pressed `-`/`+` on `mirror_catchup` **33 times** — 3, 5, 8, 12, 8, 5, 3, 2, 1, 0, 1, up to the ceiling and back down twice — because the row showing the correction was covered by the screen he was tuning it on. ⭐ **A live editor whose effect is invisible while editing is half a feature.** ⬜ Options: keep one status line visible under the screen, or make the settings block shorter than the terminal so both fit. ⚠️ Also reconsider `i` inside the screen, which he pressed wanting to re-engage mirror ([FINDINGS §65.4](FINDINGS.md)) |
| 44 | ⏳⭐⭐⭐ **Velocity feedforward — BUILT 2026-08-18, unrun on hardware** | ✅ **Built the same day as his go** ([FINDINGS §67.9](FINDINGS.md)): `--vel-ff` / setting 9, OFF by default, the setpoint is the rate-limited command's own derivative (bounded by construction), the jaw excluded, 8 new tests, the sim loop 25/25 with it on. ⬜ **Owes one gentle hardware run at `--vel-ff 0.25`** — watch for buzz/overshoot, back off live with `n` `9` `-` | ✅ **His go: *"try to build everything else relevant, especially regarding the motor speed."*** Build off by default; he tests slowly ([FINDINGS §67.7](FINDINGS.md)). It is also the real answer to the weak catchup (item 39) | ⭐ **New 2026-08-17, from his latency question.** The DM motors' MIT-mode command carries `(kp, kd, position, velocity, torque)` and we send **velocity = 0**, so all torque must come from position error — which is exactly the measured `0.033 s × speed` lag ([FINDINGS §66.1](FINDINGS.md)). Sending the target's velocity lets torque flow before error builds. ⛔ Needs work in the I2RT interface layer and a careful bring-up (a wrong feedforward oscillates); **his ratification before it touches the arm** |
| 45 | ✅ **The exit summary compared a map across FRAMES — FIXED 2026-08-18** | ✅ Both lines now print in the same frame and the frame is named on the line (`axis map G (camera frame): …`), so a frame difference can never again read as a scrambled map. Verified through the sim drive's full exit path | ⚠️ **New 2026-08-17.** G ended the session in the camera frame, so the exit printed G's camera-frame row against a `was:` line in world labels — reading as a scrambled, saved map. **Disk was verified untouched, twice.** Print both lines in the same frame, or say which frame each is in ([FINDINGS §66.2](FINDINGS.md)) |
| 31 | ✅⭐ **`chain_alive` in every incident file was always `false` — FIXED 2026-08-18** | ✅ **Liveness is captured into a local BEFORE the `shutdown_robot()` loop and recorded as `chain_alive_at_teardown`** — renamed so old files' meaningless field cannot be confused with the real measurement. A source-ordering test pins capture-before-shutdown ([FINDINGS §67.6](FINDINGS.md)) | ⛔ **New 2026-08-15, found by opening an incident file for the first time** ([FINDINGS §58.45](FINDINGS.md)). The incident block runs AFTER `shutdown_robot()` on purpose, so the motors are off before anything is attempted — and the chain is therefore already stopped when the field is read. **A field with the same value in every file is not a measurement**, and this one reads as *"the chain was dead when it stopped"*, which is the exact distinction that decided whether a park was possible on 2026-08-14 ([§46.0](FINDINGS.md)). ⭐ **Fix: capture each arm's liveness into a local BEFORE the shutdown call and record that.** Two lines, no hardware needed. ⚠️ The rest of the file is verified good: modes, joints, torques, per-joint temperatures, EE, limits, loop rate, commit and the whole USB bus, for both arms |
| 27 | ✅⚠️ **`SafeRobot`'s following-error clip — RAISED AND TESTED 2026-08-17, and it is NOT what limits MIRROR** | ✅ Julien ran `--max-lag` at **0.4** and **1.0**. His verdict: *"max lag seems to work when I have a high lag. It seems to be able to work for longer, and then it can just catch up."* ⛔⭐⭐ **But it made no difference to MIRROR, because MIRROR stops on `--mirror-gap` (built-in 0.35), not on `max_lag`.** Fourfold on `max_lag` moved the stopping point by 0.012 rad. The stop message was naming the wrong flag and has been fixed ([FINDINGS §61.0](FINDINGS.md)). ⬜ Still open: whether a high `max_lag` helps **playback and teleop** tracking, which is a separate measurement from mirror | ⛔ **New 2026-08-15.** `SafeRobot` applies TWO limits and only the rate one was ever discussed: the command may also never be more than **`max_lag` = 0.25 rad** from the measured position ([FINDINGS §57.3](FINDINGS.md)). ⭐ **That is what stops a MIRROR follower from being pulled closer by more speed**: the gap is `(leader − command) + (command − measured)`, and the second term is clipped, so the leader only has to get 0.10 rad ahead of the command to trip a 0.35 rad limit. ⚠️ **Raising it is the one lever nobody has pulled**, and it is a genuine safety limit: the clip is also a torque limit, because the PD term is `kp · (command − measured)`. Loosening it makes the motor push harder, which is how you get faster tracking AND how you get a hard hit. ⛔ **His decision, and it wants a measurement first:** watch `limited_cycles` during a mirror run at `--max-speed 2` and see how often it actually bites |
| 28 | ⭐ **Show each arm's control frame on its status row** | ⚠️ New 2026-08-15, small. `v` aims at ONE arm, so two arms can be driven in different frames — one in `world` while the other follows its own wrist in `tool`. **Nothing on screen says which is which.** The recording metadata records both since tonight; the live row does not. Cheap, and it only matters once he actually uses two frames at once |
| 25 | ⏳⭐⭐ **Nothing knows where the other arm is — MEASURED, AND HIS RULING IS MANUAL AVOIDANCE, 2026-08-17** | ⭐ **Bases ~0.70 m apart, so the arms CAN reach each other, and he intends to move them CLOSER.** So the 1.20 m escape in [FINDINGS §59.3](FINDINGS.md) does not apply and never will at this spacing. ⭐ **His decision: avoid collisions manually by choosing what the features do**, so ⛔ **no automatic refusal is added.** ✅ `src/collision.py` + `scripts/check_collision.py` measure it. ⛔⭐ **At 0.70 m the WORST case is both arms AT REST: 2.5 cm of conservative clearance**, confirming his *"one arm can easily touch the other when standing still"*. ⚠️ But the closest pair is a gripper tip against `link3`, whose declared bounding sphere is 0.197 m around a much thinner link — **so bounding spheres are too coarse to give a usable margin at this spacing**, and a warning built on them would cry wolf at rest. ⬜⭐ **PREREQUISITE for any status-row warning: finer geometry, i.e. MuJoCo's `mj_geomDistance` against the real meshes rather than one sphere per body.** Hardware-free ([FINDINGS §60.3](FINDINGS.md)) | ✅ `src/collision.py` + `scripts/check_collision.py` compute the conservative minimum distance between the two arms from the shipped MuJoCo model, and **refuse nothing** by design. ⭐⭐ **The cheap answer may close it entirely: beyond 1.20 m of base separation the existing 0.60 m reach limit already makes a collision impossible** — except in GUIDE, where nothing can stop a hand. ⛔ **BLOCKED ON ONE TAPE-MEASURE READING from Julien:** the distance between the two bases, which nothing in the repo records and no software can derive. ⬜ Then his decision on a margin ([FINDINGS §59.3](FINDINGS.md)) | ⛔ **New 2026-08-14, and MIRROR mode is what made it matter.** Every limit in this project is per arm and relative to that arm's own base: the reach sphere, the floor, the joint margins. **No code anywhere knows that a second arm exists in the same space.** ⚠️ Until 2026-08-14 that was almost harmless, because every motion had a hand on the arm or a puck under it. **MIRROR is the first mode where an arm moves with nobody's hand on it**, and the two arms stand side by side, so a leader reaching across can drive the follower into it. ⭐ **Today the only guard is the operator**, and the mirror plan line says so in as many words. ⭐ **What a real fix looks like:** both arms already exist as MuJoCo models (that is how `mink` solves IK), so putting both in ONE scene and asking for the minimum distance between their bodies is a known quantity per cycle. That is the same shape as the workspace clamp: measure, then refuse. ⚠️ **Not cheap**, and it wants Julien's decision about how close is too close |
| 26 | ⭐ **Record while mirroring, and get a two-arm demonstration from one hand** | ⭐ **New 2026-08-14, and it is free rather than a feature.** The recorder samples EVERY arm each cycle, and MIRROR moves the follower, so pressing `w` while mirroring captures both arms in one timeline — a two-arm demonstration produced by hand-guiding ONE arm. ⚠️ Whether that is *good training data* is a different question and belongs with [ROADMAP §6.6](#66-where-the-training-data-comes-from): the follower's motion is a rate-limited copy, so it lags the leader slightly and never shows independent two-hand coordination. ⭐ **Worth trying precisely because it is free**, and worth labelling: the `method` field records `B:guide+G:mirror`, so the dataset says how it was made |
| 23 | ⛔⭐⭐ **Collapse the TWO park implementations — the tested one is the one that does not run** | ⛔ **New 2026-08-14, and it is the finding of step 2** ([FINDINGS §52.1](FINDINGS.md)). Checked rather than assumed: `teleop_session.py` calls exactly ONE method on `ArmSession` (`arm.alive()`). Its own closures still do the entering, the clamping and the whole park, so **`ArmSession.step_path()` and the 45 tests around it describe a park that never executes.** ⚠️ A reader who sees those tests pass would be wrong about the code that moves 4.3 kg, and nothing said so. ⭐ **Fix: delete the script's closures and call the class's methods**, one group per commit. ⛔ **NOT mechanical** — the script does `arm.mode = "park"` and then calls its own `enter_hold()`, which leaves the mode alone, while the class's `enter_hold()` sets `mode = "hold"`. A naive substitution at that one site leaves a park running in HOLD. Every site needs the same check, and the result needs one bench pass over all modes |
| 24 | ✅⭐⭐ **Check `COMMANDS.md`'s flag list against the parsers, mechanically — DONE 2026-08-15** | ✅ `scripts/check_flags.py` reads all 79 documented `uv run` lines and validates each against the target script's real `argparse` declarations; `scripts/falsify_check_flags.py` proves it can see a break. ⭐ **It found three defects in itself, and the third is the lesson**: a rule added to remove a false positive silently created a false negative, visible only because the falsification count dropped 7 → 6 ([FINDINGS §59.1](FINDINGS.md)). ⬜ Remaining: nothing required. The ⚠️ list of never-mentioned flags is advisory | ⛔ New 2026-08-14, after the eighth and ninth instances of the staleness pattern were found in that file by reading it to add one row ([FINDINGS §52.3](FINDINGS.md)): it advertised `--box`, deleted on 2026-08-14, and described `ö`/`ä` as the gripper step, wrong since 2026-08-13. ⭐ **A document listing flags goes stale every time a flag changes and nothing checks it.** The check is small: every flag named in `COMMANDS.md` must exist in the script's `argparse`, and every flag in `argparse` must be named in `COMMANDS.md`. Same shape as the four `check_*` scripts — replace the claim with the command that recomputes it ([FINDINGS §33.3](FINDINGS.md)) |
| 17 | ⬇️ **Raise the wrist position gains — DEMOTED 2026-08-13, the reason for it was refuted** | ⛔ **The case for this rested on the wrist joints tracking much worse than the shoulder because `kp` is 10 against 80. Measured over three runs, the speed-dependent error is the SAME on both groups (ratio 0.97x, not 8x)** ([ROADMAP §7.5.1](#751-answered-2026-08-13-1635--the-arm-follows-a-path-with-a-fixed-delay-not-a-gain-shaped-error)). **So stiffening them would buy no speed at all**, while a stiffer joint still hits harder and gravity compensation was already 39% short at the elbow once ([FINDINGS §11](FINDINGS.md)). ⭐ **Where the soft gains DO cost something is the constant part of the error**, 0.080 rad against 0.037, a factor of 2.16 — that shows up as how accurately a joint holds and settles. **So if this is ever done, do it for holding accuracy, never for speed, and one joint at a time.** |

### 8.3 Decisions still missing

- **The task.** Undecided on purpose: *"we first have to get the setup right."*
- **Which model.** Probably diffusion. Papers to come from him.
- **Recording rate, image size, and what counts as an action.** ⛔ All three have to be fixed *before* collecting, because changing them later means re-collecting.
- **A start-of-session checklist.** Gripper calibration after a power cycle already exists; camera identification and a bandwidth check will need to join it.
- ❓⭐ **Should the quit menu be able to RESUME the session?** New 2026-08-14 from his *"q p doing the base position and then going back to teleoperate and continuing"*. ⛔ It is a structural change, since the quit menu sits *after* the control loop has exited, so resuming means an outer loop around `main()`'s body. **It collides with the half-finished restructure and should wait for it** ([FINDINGS §49.1](FINDINGS.md)). ⚠️ **It may also be unnecessary: plain `p` already parks without quitting, and `t` carries on.** That is now printed in the menu and the help.
- ❓⭐ **Should a park target be allowed that is NOT a measured pose?** New 2026-08-14, from his *"park should allow for a normal park mode to zero or to, like, the standard position"*. ⛔ **Every park slot today is a pose the arm physically held**, saved with `s <digit>`, and [FINDINGS §37.3](FINDINGS.md) turned on exactly that property. A computed all-zero target puts the tip 0.206 m out and 0.174 m up, so it is very likely fine, **and "very likely fine" is the wording that precedes the failures in [FINDINGS §0](FINDINGS.md).** ⭐ **The alternative needs no code: he saves the standard pose once with `s 0`.** It then becomes a measured base slot and `q q` returns to it forever.

### 8.4 Deliberately deferred, and by whose decision

⭐ **Measuring exactly where each camera sits and points: deferred, his ruling.** *"The exact measuring of what the setup looks like we haven't done yet. That's still a task that's open, but it's not for this current setup, because we will still move everything around. Currently we just have to make sure that everything is connected and works, and that we have some type of guiding system that we can then work towards when we reimplement the whole thing from scratch."*

⚠️ **So the goal right now is "connected and working", not "measured and final".** One consequence to keep: until the mount is measured, drive in the `tool` frame and not `camera`, because the `camera` frame assumes a modelled 25° tilt that the photographs do not match.

⭐ **The cable: he has it under control.** *"Don't worry about any problems with the cable. I have everything under control. It's just that it would be good so that we can do that better in a different way later."* Two facts of his worth keeping, because they change what "route it along the arm" would cost: it needs **a much longer cable than they own**, and **there is nothing on the arm to plug into**, so it would be one continuous run from the wrist to the base rather than a socket at the base.

### 8.5 ⭐ The consolidation task he asked for, and when it should happen

His words: *"Later, one task would be to consolidate everything in the whole code and think about deeply and plan the full structure of what is necessary to build this architecture, and then create, like, an assignment plan or something like that, maybe a bit more creative fitting to the actual situation, which then will help us to work through everything."*

⭐⭐ **PROMOTED 2026-08-18: this consolidation plan is now THE endgame deliverable** ([FINDINGS §67.0](FINDINGS.md)). His ruling makes the sequence explicit: finish the features and the setup, then consolidate everything into a full plan — what is necessary, all the problems a rebuild will run into — and hand it to his team, who rebuild from scratch. The still-open decisions below (the task, the model, the papers) stop being *blockers* and become **open-decision sections inside the plan**, each with the evidence this repo already holds.

⛔ **This should happen after the setup works, not before, and the reason is specific.** The architecture depends on three things that are all still open: the task, the model, and the papers being matched. A structure planned against a rig that does not yet run end to end is a guess dressed as a plan, and this repo has a name for that ([FINDINGS §0](FINDINGS.md)).

**What it needs as input, so it can be started the moment those arrive:** the papers he will send · the task · the model choice · and one complete run of the pipeline, even a bad one, so the plan is written against measured behaviour.

---

## 9. ⭐⭐ The target stack, and what the research actually demands of this rig — 2026-08-12

> ⛔ **Read this before any large restructuring, and before the recorder.** Julien sent the project's own stack plan and its three papers on 2026-08-12, saying *"we have no idea what we're doing, so I just wanted to let you know what we're thinking of"*, and asking what is smart, what to watch for, and **how to reconnect into the systems that already exist** if this repo becomes the template for a rebuild.
>
> ⭐ **Everything below was read from the actual sources on 2026-08-12, not recalled.** Two of the three papers are newer than any model's training data, so anything stated from memory would have been invented.

### 9.1 The plan, in his own words

> *"Stack: Wir nehmen so viel es geht aus `github.com/amazon-far/abc` und bauen einen stabilen, lokalen Stack fuer unsere Roboter. Teleoperation (via 3d Space Mouse) zum Datensammeln fuer einen oder zwei machbare Aufgaben, SFT fuer diese Tasks (vmtl. VLA basierte Policy) und eine funktionierende Sim sind unsere minimalen Meilensteine."*
>
> *"Algorithmische Arbeit: einen bis jetzt noch nicht ganz so im Fokus stehenden Ansatz verfolgen: ENPIRE, und `arxiv.org/abs/2607.00272`. Um nicht komplett den Mainstream aussen vor zu lassen, moechte ich auch auf `arxiv.org/abs/2607.15275` schauen. Alternative Themen (RL z.B.) waeren natuerlich auch super, falls der Stack steht."*

⚠️ **He was explicit that none of this is fixed.** Treat it as direction, not specification.

### 9.2 ⭐⭐ What ABC actually is, and the part that changes our plans

`github.com/amazon-far/abc` is an open behaviour-cloning stack. **Behaviour cloning** means training a model to copy recorded demonstrations. It ships a diffusion policy called **ABC-DiT**, a large public dataset, pretrained checkpoints, training code and evaluation tools.

⭐ **It vendors the YAM MuJoCo model from i2rt-robotics, the same source this repo vendors.** So it was built for our arms.

⛔⭐ **THE FINDING THAT MATTERS MOST: its training data format tells us the shape of everything upstream of it.**

```
episode_<uuid>/
  states_actions.bin               # 28 columns, float64  (14 states + 14 actions)
  combined_camera-images-rgb.mp4   # 30 fps, 224x224, several camera views stacked
  episode_metadata.json            # task name, cameras, resolutions, timing
```

Read that carefully, because four decisions fall straight out of it:

1. ⛔⭐ **14 states and 14 actions means TWO ARMS IN ONE TIMELINE.** Fourteen is 2 x 7 joints. **So both arms must be recorded together, at the same rate, in one file.** Two separate terminal sessions, which is how Julien has driven both arms so far, physically cannot produce this. **Two arms from one script stops being the next nice feature and becomes a prerequisite of the data format.** See [ROADMAP step 6](#step-6--two-arms-two-spacemice--what-julien-asked-for-next-2026-08-10) and `src/arm_session.py`.
2. ✅ **Actions are joint positions, and that is what we already command.** A worry recorded earlier in [ROADMAP §6.6](#66-where-the-training-data-comes-from), that much of the literature commands end-effector movements instead, does not apply to ABC. **We match it today.** Recording both anyway stays cheap insurance.
3. ⭐ **Images are 224x224 at 30 fps, several cameras stacked into one video.** So the resolution argument that has run through [FINDINGS §21](FINDINGS.md) is irrelevant to training: 224 pixels is tiny. What matters instead is **which camera is in which slot of the stack**, every single time. ⛔ That is exactly the two-identical-D405s problem ([FINDINGS §28.6](FINDINGS.md)), and getting it wrong swaps the arms' views in the dataset while every file still looks fine.
4. ⭐ **`episode_metadata.json` already exists as the place for provenance.** The list in [ROADMAP §6.6](#66-where-the-training-data-comes-from) should be written into **their** field names rather than ours, wherever they overlap.

⭐⭐ **And the thing to check first, because it could cut the work by a factor of five.** ABC publishes a **large dataset (`XDOF/ABC-130k` on Hugging Face) and pretrained checkpoints**. Fine-tuning an existing checkpoint needs far fewer demonstrations than training from nothing. The professor's milestone is **SFT** (supervised fine-tuning) on one or two tasks, which is exactly that. ⚠️ **So the "50 to 200 demos" figure in [ROADMAP §6.6](#66-where-the-training-data-comes-from) may be several times too high for our case.** ⛔ Unverified: check what the ABC README and the checkpoints actually support before planning a collection session around either number.

⚠️ **Two practical facts from the same README.** Training runs on **8 GPUs** (`torchrun --nproc-per-node 8`), so it cannot happen on his MacBook and compute is a gap on no list yet. And the vision backbone is **DINOv3**, whose licence forbids military and weapons use, which is worth knowing in a university project even though it changes nothing here.

⭐ **One milestone may be largely free.** ABC ships `viz_policy.py` and `eval_policy.py` running against the YAM MuJoCo model. *"Eine funktionierende Sim"* is a stated milestone, and much of it may already exist rather than needing to be built.

### 9.3 The three papers, and what each one demands of the hardware

| paper | what it is | what it needs from us |
|---|---|---|
| **ENPIRE** ([2606.19980](https://arxiv.org/abs/2606.19980), NVIDIA / CMU / Berkeley, June 2026) | A harness that lets a coding agent improve a policy in the real world, on a loop: **reset the scene, run the policy, check whether it worked, improve.** Four modules: Environment (automatic reset and checking), Policy Improvement, Rollout (evaluate on one or several robots at once), Evolution. Reached 99% on real dexterous tasks | ⛔ **automatic scene reset** · ⛔ **automatic success checking** · parallel rollouts on both arms · a **scriptable API** an agent can call · **unattended running**, which on a rig with no emergency stop is a real problem |
| **ASPIRE** ([2607.00272](https://arxiv.org/abs/2607.00272), June 2026) | Writes and refines robot control **programs** (code as the policy) and builds a reusable **skill library**, using evolutionary search over task sequences | **"fine-grained multimodal traces"** for diagnosing its own failures · named, reusable motion primitives · the same scriptable API |
| **RoboTTT** ([2607.15275](https://arxiv.org/abs/2607.15275), July 2026) | A **vision-language-action** policy whose context stretches to **8000 timesteps**, about 1000x the usual. Enables one-shot imitation from **human video** and long multi-stage tasks | ⛔ **long, unbroken recordings** (8000 steps at 30 fps is ~4.5 minutes) · a **language instruction** per episode · ⭐ **human video demonstrations** |

### 9.4 ⭐⭐ Where this changes decisions we have already made

**Automatic scene reset: Julien's objection was right for now and stops being right later.** He argued it is close to circular, because a robot that can place an object at a chosen spot can already do the task. ⭐ **ENPIRE's whole method is built on automatic reset, so the tension is real and worth resolving rather than picking a side.**

The resolution, and it holds both positions:

- **At the demo-collection stage he is right.** No policy exists, so a reset would have to be a hand-taught open-loop path, and teaching one per start position costs more than moving the object by hand.
- **At the self-improvement stage he stops being right, for two reasons.** First, **a reset does not need to be precise.** Tipping pins out of a box, sweeping objects into a rough area or pushing something back is far easier than grasping it, and ENPIRE's own tasks are like this. Second, **by then a policy exists** that can partly do the task, so the circularity is gone.
- ⭐ **So automatic reset moves from "not worth it" to "required later, and it can be crude."** Do not build it now. Do not design it out either: the recorder and the session API should not assume a human is present between episodes.

**Automatic success checking stops being a nicety.** Both ENPIRE and ASPIRE need the robot to know whether it succeeded, without a human watching. ⭐ The gripper trick in [ROADMAP §6.6](#66-where-the-training-data-comes-from) (the jaws stop at the object's width when holding something, and close further when holding nothing) is exactly that primitive, and it now has a research reason rather than only a data-hygiene one.

⭐⭐ **His hands in the frame during guide-mode teaching may be an asset, not a problem.** [ROADMAP §6.6](#66-where-the-training-data-comes-from) treats the teaching pass as something to keep out of the recording. **RoboTTT learns from human video demonstrations.** So the teaching pass is potentially a second, differently-useful recording. ⛔ **Record it rather than discarding it.** It costs a file and cannot be recovered later.

**A language instruction per episode is now near-certain rather than speculative.** The plan says *"vmtl. VLA basierte Policy"*, RoboTTT is a vision-language-action model, and ABC's own text encoder is CLIP. ⭐ So the spoken-instruction half of the microphone idea in [ROADMAP §6.6](#66-where-the-training-data-comes-from) is the part with the clearest payoff, and it is one sentence per recording.

⚠️ **A tension worth watching, not resolving yet.** ABC's episodes are short clips at a fixed size. RoboTTT wants ~4.5 minutes of unbroken context. **Do not build a recorder that can only produce short clips.** Record continuously with real timestamps and cut clips out afterwards, because that direction is reversible and the other is not.

### 9.5 ⭐⭐ The architecture conclusion, and it answers his rebuild question

He asked how this becomes a template to rebuild from, and how to reconnect into what already exists. **All three papers want the same thing from us, and our code is currently the opposite of it.**

What they want:

- **The robot as a callable library**, with a clean API a script or a coding agent can drive. ENPIRE and ASPIRE both have an agent writing policy code against it.
- **Structured traces**, one record per cycle, filterable. ASPIRE diagnoses its own failures from them.
- **An automatic success signal.**
- **Long continuous recordings**, with a language instruction attached.
- **Both arms in one process**, because the data format is 14 columns wide.

What we have: `scripts/teleop_session.py`, an interactive terminal program with roughly 2000 lines in one `main()`, one arm per process, printing for a human.

⭐ **So the rebuild has a clear shape, and it is not a rewrite from zero.** Build a control library with a clean API, structured logging and a success signal. Make the interactive session **one client** of that library rather than the place the logic lives.

⭐ **And `src/arm_session.py` is already the first step in exactly that direction.** Its rule is *the class decides, the script narrates*, with no printing inside the class, which is why 17 tests can prove it without hardware. ⛔ **That design choice was made for testability and it turns out to be the shape the research needs.** Wiring it in is the single highest-value piece of work available, and it now has three independent reasons: bimanual driving, the 14-column data format, and every paper above.

⚠️ **What is still needed before the consolidation task in [ROADMAP §8.5](#85--the-consolidation-task-he-asked-for-and-when-it-should-happen) can be done properly:** the chosen task · whether fine-tuning an ABC checkpoint is possible · a machine with GPUs · and one complete end-to-end run, even a bad one, so the plan is written against measured behaviour rather than against this document.

---

## 10. ⭐⭐⭐ Team handover: the cleanup and target architecture — PROPOSED 2026-08-17, awaiting Julien's ratification

⭐ **His ask:** *"fully clean up the repo and have a clear plan of what the repo should look like when my team wants to recreate it… structure it in a really well designed architectural way."* ⛔ **This section is the plan, deliberately not yet executed** — a restructure is consequential and hard to reverse, so it gets ratified first (working-contract rule 11).

⭐⭐ **UPDATE 2026-08-18 — his ruling rescoped the goal** ([FINDINGS §67.0](FINDINGS.md)): this repo is a **finished walkthrough**, the team rebuilds from scratch against a consolidation plan, and that plan plus §10.4 (the knowledge code cannot carry) are the core of the endgame. The agent then recommended skipping the *physical* restructure as no longer worth its cost. ⭐⭐⭐ **HE OVERRULED THAT THE SAME DAY, and the restructure is RATIFIED** ([FINDINGS §67.7](FINDINGS.md)): *"the file organization might be important because we want to have a really smartly built repo… we can actually use and look at it, because it's already well organized."* ⬜ **New input before executing:** explore the colleagues' branches in the public `Hohnik/LaRobot` repo — how they structure code, where their ideas come from, what their current plans are — and integrate that into this layout, deviating where a better idea exists (*"you don't have to do everything as we're doing it"*). §10.5's order of work stands, with that exploration inserted before step 3.

### 10.1 What a team member needs on day one

1. ⭐ **One entry document** that says what this is, what runs, and the three commands that prove the rig works (`check_rig` → `ping_motors` → a dry-run session). Today that is [HANDOFF.md](HANDOFF.md), which has grown into a session log; the entry role should split out.
2. **`uv sync` and it runs** — already true, and the strongest thing about this repo.
3. **The safety rules in one place**: no `--yes` without intent, the four-limit chain, arm G is shared, no e-stop so wall power is the cut-off.

### 10.2 The target layout

```
yam-robotics/
├── README.md            ← the day-one door: what, safety, three proof commands, map
├── pyproject.toml
├── src/yam/             ← ⭐ a real package, importable as `yam.*`
│   ├── robot.py         (yam_robot.py: SafeRobot, build_robot, thermal, grippers)
│   ├── can.py           (yam_can.py)  · teleop.py · session.py (arm_session.py)
│   ├── mirror.py · recording.py · motion.py · settings.py · collision.py
│   ├── fake/            (fake_arm.py — the simulator)
│   └── ui/              (screen.py, keyboard.py, spacemouse.py)
├── apps/                ← things you RUN: teleop_session.py, camera_view.py, map_axes.py …
├── checks/              ← check_*.py + falsify_*.py  (read-only diagnostics)
├── tests/               ← test_*.py, runnable as one suite
├── config/              ← measured calibration (tracked) + session_defaults.json (ignored)
├── docs/                ← HANDOFF (live state) · FINDINGS (evidence) · ROADMAP · COMMANDS
└── third_party/i2rt/    ← untouched vendor code
```

⭐ **The main real change is `src/ → src/yam/` as an installed package**, killing every `sys.path.insert` header, plus sorting `scripts/` (41 files today) into `apps/`, `checks/`, `tests/`. ⚠️ Each `git mv` batch must keep the suite green — the checkers (`check_restructure`, `check_flags`, `check_links`) exist exactly to make such a move safe.

### 10.3 Cruft to remove or relocate (verify, then delete — never blind)

- `scripts/temp.py`-style leftovers and `__pycache__` (gitignore covers it) · superseded probes if any duplicate a checker.
- `docs/Setup-Plan.md`'s broken link (item 32 — **his decision**) and its German/English split: decide ONE language for team docs.
- `recordings/` stays gitignored; `incidents/` policy: keep last N.

### 10.4 What "recreate it" must include that code cannot carry

⭐ The hardware bring-up facts live in [FINDINGS](FINDINGS.md) but a team needs them distilled into the README: the CANable DFU recovery ritual (USB vs mains, [HANDOFF](HANDOFF.md)), the gripper ±2π frame shift, the SpaceMouse empty-serial wiggle assignment, macOS camera permission being per-app, and the LED table. **A checklist, one page, written once the restructure lands.**

### 10.5 Order of work (each step green before the next)

1. His ratification of this section, and answers: one language? keep German docs? package name `yam`?
2. `tests/` + one runner (pytest already available) — no moves yet, just collection.
3. `src/yam/` package + import fixes, suite green.
4. `apps/` + `checks/` split, `check_flags` updated for new paths, docs updated the same commit.
5. README rewrite as the day-one door; HANDOFF slims back to live-state.
6. The one-page hardware bring-up checklist (§10.4).

### 10.6 ⭐⭐ What the colleagues' LaRobot branches show — explored 2026-08-18, his instruction

> ⭐ **His instruction:** explore the team's branches, understand how their stuff looks and what their plans are, integrate those plans into ours, and deviate where a better idea exists. Fetched from the public `Hohnik/LaRobot` (remote `larobot`): `main` + `feature/sim` + `feature/camera-framework` + `feature/spacemouse`, 4-6 commits each, early skeleton stage.

**What LaRobot is:** the team's greenfield rebuild starting point. A `src/robot/` package split by domain — `inputs/` · `cameras/` · `sim/` · `record/` · `train/` · `evaluate/` — plus `tests/` (pytest + conftest, per-domain folders), a `justfile` as command runner, and the ABC `put_bottles` sim assets. Working today: a viser slider-teleop of the ABC sim recording to `.npz` (`main.py`), and a plain-MuJoCo `World` class at 29.4 Hz (`TIMESTEP 0.002 × DECIMATION 17`, no CUDA needed). Targets Ubuntu 24.04, Python ≥3.14.

⭐⭐ **Their two ideas worth adopting in our §10.2 layout:**

1. **Policy-as-an-input.** `inputs/` holds `input.py` (an `Input` ABC), `keyboard.py`, `mouse.py`, `spacemouse.py` — **and `policy.py`**. Everything that produces commands implements one interface, so a trained policy plugs in exactly where the human does. That is ABC Phase E1's policy-adapter stated structurally, and our session loop should be described (and eventually restructured) the same way: command sources behind one interface.
2. **The `Frame` dataclass** (`cameras/frame.py`): `camera_name` · `sequence` · `camera_timestamp_ns | None` · `host_timestamp_ns` · `rgb` · `depth: None`-able *"because of the C920"*. Dual timestamps is exactly what item 6 (timestamped capture) needs — ⭐ **our capture tooling should align its field names with their `Frame` so the rebuild lifts it unchanged.**

⭐ **Consequences for §10.2:** `src/yam/` gains domain subpackages rather than staying flat — `inputs/` (spacemouse, keyboard) · `cameras/` · the existing `fake/` — and `tests/` gets per-domain folders matching theirs. A `justfile` is worth adding (they already use `just`; our COMMANDS.md lines map onto recipes). Package name `yam` stays (their package is `robot`; two names is fine, the rebuild picks one).

⚠️ **What NOT to copy (and say so in the plan):** their sim `World` and our `fake_arm` solve different problems — theirs renders the ABC task scene for policy work, ours *lags like the measured hardware* so the control loop can be falsified. Both belong in the rebuild, as different modules. And their empty stubs (`camera.py`, `spacemouse.py`, `policy.py`, `record.py` are placeholder files) mean the *proven* implementations still live here — which is exactly why this walkthrough exists.

## Deliberately NOT doing, and why

- **Joint-space jogging as a stepping stone** (SpaceMouse axis → one joint each, no IK). Genuinely simpler and it would prove the plumbing sooner. Rejected as the *main* path because it is throwaway — the plan needs cartesian control — and because simulation already provides a risk-free place to debug the real thing. **Kept as a fallback:** if IK fights us, joint jogging still gets a SpaceMouse driving the arm the same day.
- **Anything on the ABC training side.** No data exists yet. Training is downstream of step 5.
- **Forking `third_party/i2rt`.** Patch from outside; keep the upstream checkout re-pullable.
- **Chasing the 100 Hz question further.** Measured, answered, 3× headroom (README §2).

---

## Open questions for Julien

| # | Question | Why it matters |
|---|---|---|
| 1 | ~~How much clear space is around arm 1?~~ **Answered: not safe yet.** Desk to be cleared | Gates every whole-arm step. Reordered the roadmap — see the top |
| 2 | ~~Are the **D405 wrist cameras** mounted?~~ **Answered 2026-08-11: one is, on arm B, plugged in, measured** (serial `255323071773`, USB SuperSpeed) **and delivering a colour picture over plain UVC — no SDK needed.** The second is with arm G and unplugged. ⚠️ Two D405s share every capture mode, so they cannot be told apart by measurement; when the second is plugged in, select by `--index` and confirm by covering one — FINDINGS §22 | Scopes step 7 |
| 3 | ~~The second SpaceMouse~~ — **answered by measurement, see below** | — |
| 4 | ~~Is there an **e-stop**, or is wall power the only cut-off?~~ **Answered by Julien: wall sockets only, there is NO e-stop.** Hence every new motion path being slow, bounded and interruptible, and `h` HOLD / `q` being the real stops — HANDOFF §4.5 | Changes how aggressive the step-4 safety envelope needs to be |


---

## ~~⛔ Do NOT connect the second SpaceMouse yet~~ — RESOLVED, and both are now connected

> ⭐ **This section is spent, and is kept only because the reasoning below is still the reason the fix looks the way it does.** Both SpaceMice are connected as of 2026-08-12 (visible in his desk photograph, and both appear in `ioreg`). The problem it warned about was solved rather than avoided: `pick_device_by_wiggle(exclude=…)` in `src/spacemouse.py` asks the operator to move the puck they want, so identity comes from a gesture instead of from an index. ⭐ **That same idea is now the answer to a second problem** with the same shape, two identical D405 cameras that no list can tell apart: see [FINDINGS §28.5](FINDINGS.md).

Julien offered to free a USB port and connect the second SpaceMouse. **Recommendation: don't, yet.** Not because "one thing at a time", but because of a specific fact checked on 2026-08-10:

```
hid.enumerate() for VID 0x256f:
  usage=0x08  serial=''  path=b'DevSrvsID:4295192284'
  usage=0x30  serial=''  path=b'DevSrvsID:4295192284'
  usage=0x33  serial=''  path=b'DevSrvsID:4295192284'
```

**The SpaceMouse reports an empty serial number.** So the trick that saved us on the CAN adapters — select by serial, never by index — **does not transfer**. Two SpaceMice would be indistinguishable except by `path`, a macOS IOService registry ID that changes on replug and carries no meaning.

That is exactly the bug class that already bit twice today: `find_device()` returns `multi[0]`, so with two pucks attached, *which arm a given puck drives would be arbitrary and would silently change between runs.* Connecting the second one before fixing selection means debugging teleop and device-identity at once.

**What to do instead, in order:** get one puck driving one arm (step 1b) → then build selection by **USB topology** (which hub port a device is on, stable while nothing is re-cabled) → *then* connect the second.

**And keep the camera plugged in.** Nothing needs to be freed up: the camera is not competing for anything we need today, and unplugging it costs a re-verification later for no gain.
