# Roadmap — from "the arm twitches" to "I drive it with the SpaceMouse"

> **Purpose of this file.** The README says what is *true now*. This says what we are going to do,
> **in what order, and why that order** — because the ordering is the part that carries the reasoning,
> and it is the part that gets lost between sessions.
>
> Julien's stated near-term goal (2026-08-10): *"be able to control the arm with the space mouse — a single
> arm with a single space mouse — and then we can go from there."* Everything below is ordered to reach that
> as directly as safety allows, and no more.

> ## ⚠️ Steps 1-4 below are DONE. Read [HANDOFF.md §5.5](HANDOFF.md) for the live task list.
>
> This file is kept because the **ordering arguments** are still the valuable part — why simulation came
> before hardware, why gravity compensation came before teleop, why the gripper was the safe first mover.
> But as a to-do list it is spent: teleop works on the real arm, and step 5 (the MCAP recorder) is the next
> unbuilt thing. **Step 4's caution about rotation signs is now refined by measurement** — see
> [FINDINGS §10](FINDINGS.md): rotation happens about the tool point, so a wrong rotation sign twists the
> wrist in place rather than swinging the gripper through space.
>
> **Axis remapping is also built** (2026-08-10): `scripts/map_axes.py` decides which puck axis drives which
> motion with no hardware at all, and `teleop_sim.py` now applies the same map — so the entire "step 1 in
> simulation first" argument below finally holds for axis conventions too, which was the one thing it could
> not previously test.

---

## ⭐ STATUS, 2026-08-10 — steps 1-4 are DONE. Read this before the step list below.

The numbered steps were written before any of them had been attempted, and most are now history. What is
still live is **step 6**, which is where Julien wants to go next.

| step | state |
|---|---|
| 1b gripper teleop · 1 sim teleop · 2 whole-arm over gs_usb · 3 gravity comp + hand-guiding · 4 SpaceMouse → real arm | ✅ **done**, all on hardware |
| axis mapping (not in the original list) | ✅ **done** — any puck direction can drive any motion, tuned on the arm, per-arm maps available |
| **6 two arms, two SpaceMice** | ⭐ **NEXT — designed below, not built** |
| 5 recorder → MCAP | open. Deliberately after teleop feels right, but **before** collecting demos in anger |
| 7 cameras | open, nothing depends on it yet |

⚠️ The step *ordering* below is therefore stale — step 5 now follows step 6. The **reasoning** in each step
is not stale, which is why they are kept rather than deleted.

## The target, stated precisely

**One SpaceMouse produces a 6-DoF cartesian twist. One YAM arm accepts joint positions. Teleop is the
function between them, run at 100 Hz, safely.**

```
SpaceMouse twist  →  integrate to a target EE pose  →  IK  →  joint targets  →  arm
   (6 numbers)          (a pose that persists)      (mink)     (7 numbers)
```

The middle two boxes are the whole problem. The outer two are done: the SpaceMouse is decoded and verified on
all six axes (README §4), and the arm accepts joint commands and moves (README §5).

⚠️ **Why the arm cannot simply be driven by the SpaceMouse directly.** A SpaceMouse gives *cartesian velocity*
of the end effector. The arm takes *joint angles*. There is no fixed mapping between them — it depends on the
arm's current configuration — which is exactly what inverse kinematics computes. `docs/Setup-Plan.md` §4.2
names this as the single largest deviation from the papers, which all teleoperate with GELLO leader arms that
hand over joint angles directly and need no IK at all.

---

## ⛔ The binding constraint is the desk, not the software — 2026-08-10

Julien: *"it's not really safe right now. The only thing that should be moved is the gripper opening and
closing and the gripper twisting… as soon as the SpaceMouse is connected I can move everything from the desk
and we can control the whole thing."*

**So the ordering below changed, and this is why.** The cartesian IK loop is *already working* in simulation —
software is ahead of the workspace. Steps that move the whole arm through space (gravity comp, cartesian
teleop on hardware) are **blocked on clearing the desk**, not on code. Meanwhile motors 6 and 7 —
`gripper_twist` and `gripper_jaws` — can move freely, because neither changes the arm's reach.

⭐ **That makes a real SpaceMouse-driven robot possible today**, which is step 1b.

---

## Step 1b — SpaceMouse → gripper twist + jaws, on the real arm ⭐ **do this now**

`scripts/teleop_gripper.py`. Two motors, no IK.

    puck YAW (twist)     →  motor 6, gripper_twist
    puck Z (push / lift) →  motor 7, gripper_jaws

**Why this is not a throwaway detour.** It proves the exact half of the teleop stack that IK cannot: reading
the device and driving real motors together in one 100 Hz loop, with a deadman, bounds and a clean shutdown.
When the desk is clear, IK drops in *above* this — the loop, the safety envelope and the shutdown path all
survive unchanged. It is the same code shape, minus the coordinate transform.

**And it answers the question Julien actually cares about right now:** is the SpaceMouse connected and does
moving it move the robot.

⚠️ `gripper_jaws` has no trustworthy limits (`gripper_limits: null`, `needs_calibration: true`), so it is
clamped to a window around wherever it starts, never to an absolute target, and `--max-torque` is what stops
it closing hard on itself. `--no-jaws` runs twist only.

**Done when:** Julien twists the puck and the gripper twists.

---

## Step 1 — Teleop in simulation, end to end

**Do this first, and do all of it, before the real arm is involved.**

**Why first.** `get_yam_robot(sim=True)` returns a `SimRobot` exposing the *same* API as the hardware object —
`get_joint_pos()`, `command_joint_pos()`, `get_observations()`, `enable_gravity_comp()`. Verified working on
macOS 2026-08-10. So the entire teleop chain can be written, run and debugged against simulation and then
moved to hardware **by changing one flag**, with no rewrite and no second code path.

**Why that matters more than it sounds.** The first version of any IK loop is wrong — wrong axis conventions,
wrong frame, wrong sign, wrong integration order, singularities near the workspace edge. Each of those, on a
physical arm, is a joint slamming toward a limit. Debugging them in simulation costs nothing and risks
nothing. **This is not a detour on the way to the real arm; it is the cheapest possible way to get there.**

**Also:** it needs no hardware at all, so it can proceed while the arms are unplugged, at LaVita, anywhere.

Sub-steps:
1. `mink` IK against the vendored YAM MJCF (`yam_linear_4310_d405.xml`, which already carries a `tcp_site`
   end-effector frame). Drive it with a *scripted* target first — a slow circle — so IK is validated with no
   input device in the loop.
2. Swap the scripted target for the real SpaceMouse. Still simulation, so a wrong sign is a shrug.
3. Optional MuJoCo viewer. **Optional on purpose:** Julien asked to skip visualisation if it slows things
   down, and he is right that it is not on the critical path. It is one line when wanted, and it is the
   fastest way to see *why* an IK bug is a bug.

**Done when:** moving the SpaceMouse moves the simulated arm sensibly in all six axes, joint limits hold, and
nothing diverges near a singularity.

---

## Step 2 — Make the full-arm chain work over gs_usb

**Why this is a step at all.** Everything working today drives *one motor at a time* through
`DMSingleMotorCanInterface`. Teleop needs all seven at once, and the layer that does that —
`DMChainCanInterface`, used by `get_yam_robot()` — **hardcodes SocketCAN** (`dm_driver.py:409`,
`if "can" in channel:`), with no argument to override it. That is the same wall §2.1 of the README describes,
one layer up, and it has to come down the same way.

**What it unlocks — none of which is optional for teleop:**

| | why it is required |
|---|---|
| All 7 motors in one loop | teleop commands a whole configuration each cycle, not a joint at a time |
| **Gravity compensation** | see step 3 — without it a 6-joint arm cannot be commanded gently at all |
| Gripper force limiter | `linear_4310.yml`'s clog-force thresholds; the safe way to close on an object |
| `motor_offsets` / ±2π wrap fix | `get_yam_robot()` does this at init; hand-rolled control silently does not |

**Approach.** Same shape as `patch_gs_usb_for_macos()`: a small, documented, verified monkeypatch in
`src/yam_can.py` that makes `DMSingleMotorCanInterface` resolve to the gs_usb backend when handed an adapter
index, so `DMChainCanInterface` and `get_yam_robot()` work unmodified. ⛔ Not a fork of the vendor tree —
`third_party/i2rt` stays a clean upstream checkout that can be re-pulled.

**Done when:** `get_yam_robot(channel=<B>, sim=False)` returns a working robot and `get_joint_pos()`
returns the same seven numbers `ping_motors.py` reports.

---

## Step 3 — Gravity compensation, and hand-guiding

**Why before teleop, not after.** A YAM arm weighs ~4.3 kg and holds itself up with motor torque alone. At the
gentle gains used so far, commanding all six joints *without* gravity compensation means the arm sags under
its own weight, the controller fights it, and everything reads as "the IK is wrong" when it is not.
**Gravity compensation is what makes joint position commands mean what they say.**

It is also the **safest possible whole-arm test**: the arm holds its current pose and follows no trajectory,
so there is nothing to overshoot. And it is the first genuinely impressive moment — the arm becomes
back-drivable and you can push it around by hand.

⚠️ This is the first time the arm holds real torque against gravity. It gets its own gated step, its own
command, and the 400 ms firmware timeout intact (README §5).

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

**Why it comes straight after teleop and not later.** The moment teleop works, every session is potentially
training data. `docs/Setup-Plan.md` §6.1 is unambiguous: write MCAP with ABC's exact topic names and the whole
data → training → eval half of the stack works **unmodified**. Get it wrong and every demo has to be
re-collected, which is hours of a human's time rather than minutes of a computer's.

Log everything from the start — SpaceMouse input, resulting EE pose, **and** the IK-produced joint angles —
so the action space can be chosen per experiment without re-collecting (Setup-Plan §4.3).

### ⭐ No, this does not mean running ROS2 — the question, answered once

**There is no middleware anywhere in this system, and none is needed.** Julien asked on 2026-08-12; the answer
is worth writing down because it will be asked again, especially by anyone coming from `Hohnik/LaRobot`, which
targets Ubuntu.

- **The transport is direct USB CAN.** `python-can` with the **`gs_usb`** (candleLight) backend over libusb,
  in **one process** that talks straight to the motors at 100 Hz. No ROS2, no DDS, no ZeroMQ, no gRPC, no
  sockets. Verified by search: no `rclpy`, `rospy`, `zmq` or equivalent in `src/`, `scripts/` or the vendored
  I2RT SDK.
- **The only ROS2 that appears anywhere is a set of schema NAMES**, inside `third_party/i2rt/i2rt/utils/
  recording.py`: `sensor_msgs/msg/JointState` and `sensor_msgs/msg/Temperature`. ⭐ **MCAP is a file format,
  and ROS2 message definitions are being used as a serialisation schema inside it.** Writing a file that
  *describes* its records with ROS2 message definitions requires no ROS2 installation, no node, no bus and no
  running graph — which is why `mcap-ros2-support` is a dependency and `rclpy` is not.
- ⚠️ **So the interop point with ABC is a FILE, not a bus.** That is a good thing and it should stay that
  way: a middleware would add latency and failure modes to a 100 Hz loop that currently has ~3.7 ms of spare
  budget, in exchange for nothing this rig needs. Everything runs on one machine.

⚠️ Declared in `pyproject.toml` but **not imported by our code yet**: `mcap`, `mcap-ros2-support`, `dm-env`,
`tyro`, `pydantic`. They arrived with the I2RT SDK and with this step's plan. That is fine, but it means
their presence is *not* evidence that anything uses them — check before assuming.

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

So ~6.3 ms of a 10 ms budget, ~3.7 ms spare. ⚠️ The 6.2 ms figure came from register reads and is a lower
bound; it says nothing about the loop once cameras and inference compete for CPU.

**The actual blocker is that `teleop_session.py` is single-arm all the way through.** `robot`, `teleop`,
`mode`, `gripper_value`, `prev_q`, `home_ee`, `park_target`, `last_active_axis`, `guide_ref`, `stall_since`
and `max_temp_seen` are all one arm's state, held in one function's locals.

### The design: extract `ArmSession`, then run N of them

One object owns **one arm's** robot, `CartesianTeleop`, axis map, mode and cached state, exposing roughly
`enter_mode()` / `step(dt)` / `shutdown()`. The script holds a list and the loop iterates. Single-arm and
bimanual then become the same code with N=1 or N=2.

⛔ **Why extraction and not a second `teleop_bimanual.py`.** Duplication has bitten this repo three times:
`src/spacemouse.py` exists because device logic was copy-pasted and a fix landed in only one copy; the
simulator's own `twist_from_axes()` ignored the axis map for the same reason; and PARK went around the
gripper clamp because the clamp lived only in the teleop branch. A second control loop would be the fourth —
and it would be the one driving two arms at once.

### ⭐ The de-risking that matters: `--arms B` must run the N-arm code with N=1

Then the **refactor** is verifiable against a single arm — behaviour Julien already knows the feel of —
**independently of** the bimanual hardware risk. If N=1 feels identical, the restructure is sound, and going
to N=2 introduces exactly one new variable.

Without that, the first bimanual run tests a ~400-line restructure *and* two-arm coordination at once, and
any failure is unattributable. Session 4 is the argument: three changes that passed 34 tests, three dry runs
and a simulated IK loop produced three failures on first hardware contact, one of which dropped 4.3 kg.
**Stage the variables.**

### Decisions this needs, with the recommendation

| question | recommendation | why |
|---|---|---|
| Do mode keys apply to one arm or both? | **The selected arm.** `a` cycles B → G → BOTH; the status line always shows which | A global `g` puts **8.6 kg** weightless at once, and GUIDE is the mode where a dynamics-model error becomes a *falling* arm (FINDINGS §11.1) |
| Does *driving* apply to one arm or both? | **Always both** — each arm follows its own puck, continuously | That is the actual goal. Only *edits and mode changes* need a selector |
| Start mode | **HOLD**, and refuse `--start-mode guide` when N>1 | Two arms going weightless on a first run is the worst possible first run |
| Per-arm axis maps | ✅ **built** — shared by default, `--fork-map` to diverge | Julien: *"probably the same, actually. But maybe that should be options to map them separately"* |
| Puck assignment | ✅ **built** — `pick_device_by_wiggle(exclude=…)` | Without it the same puck can be assigned to both arms silently: two arms following one hand, which reads as a control bug |
| A fault on one arm | **stops both**, then the existing consent flow for each | A chain death on B must not leave G uncommanded and sagging |

### Order of work

1. ⏳ Extract `ArmSession` with **no behaviour change**; run `--arms B` and confirm it feels identical.
   - ✅ **The class exists and is tested** — `src/arm_session.py`, 17 tests against a fake robot
     (2026-08-12). State, mode transitions, park stepping with the ramp, the queue, and the thermal
     guard per arm. **The class decides, the script narrates:** no method prints, so every decision
     is testable without hardware.
   - ⬜ **Wiring it into `teleop_session.py` — the remaining half, and the risky one.** ~1000 lines of
     `main()` currently hold that state as locals. Deliberately left for a session of its own:
     mixing "write the class" and "restructure the loop" produces a diff nobody can review and that
     only Julien can test. Session 4 is the standing warning.
   - ⚠️ **Not in the class on purpose:** building the robot (it energises motors — stays visible in
     the script), reading the SpaceMouse, key handling (which arm a key applies to is a *session*
     question), and IK stepping (`CartesianTeleop` owns it).
2. Add the `a` selector and per-arm status lines. Still one arm.
3. `--arms B,G`, starting in HOLD, gripper enabled, desk clear.
4. Only then GUIDE and CONTROLS on two arms.
5. Mirror mode on top — `src/mirror.py` and its 14 tests already exist; it needs the two-arm process
   from step 3 and nothing else.

## Step 6.5 — ⭐ Saved positions, sequences, and smooth motion between them

**Julien's idea, 2026-08-12**, and it is a better one than it first looks: *"it would
make sense to have more options to save more positions … hit `s` and then a number every
time we wanna save a position, and then hitting `p` and then the number would park to
that position. And then if we would hit `p` and multiple numbers following each other,
then the robot arm could go from each position to each next position … we also wanted to
include the smoother motions, so we would have to have an option to increase the speed
between the positions."*

⭐ **Why this is on the critical path rather than a nicety.** A named list of poses the
arm can be driven through, repeatably, is the first half of **demo collection** — step 5,
which is the professor's SFT milestone. It is also the first thing in this repo that
moves the arm through a plan rather than under a hand. Build it as if the recorder will
be attached to it, because it will be.

### What is already done (2026-08-12)

✅ **Storage**, pure and tested in `src/yam_robot.py`: `park_slots()` and
`with_park_slot()`, 6 tests. ⚠️ **The legacy file shape is read, not replaced** —
`config/park_pose.json` is `{"B": [q…]}` on the rig right now, it is *measured
calibration*, and `q p d` depends on it. A bare list is read as the `default` slot, so
nothing that works today stops working.

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

Today `advance_park_command()` moves every joint at a **constant** rate until it arrives:
a trapezoid with no ramps, so it starts and stops abruptly. That is fine for one short
move to a park pose and it is *not* fine for a sequence, where every waypoint becomes a
jerk in the middle of a motion someone is watching.

**The recommendation: a trapezoidal velocity profile** — ease in over a fixed distance,
cruise, ease out into the target — implemented as an optional `ease` argument so the
default stays bit-for-bit what is on hardware today:

```
speed_factor = min(1, travelled / ramp, remaining / ramp)     # clamped to [floor, 1]
```

⚠️ **Three reasons to keep it opt-in at first.** `advance_park_command()` is pure with
15 tests and its behaviour is *confirmed on the arm*; the ramp distance is a feel
question only Julien can answer; and a deceleration bug shows up as **overshoot**, which
in park is the arm arriving somewhere it was not aimed. Ship it behind a flag, tune the
ramp on hardware, then make it the default.

⭐ **Deliberately NOT doing: spline/blended waypoints** — smoothing *through* a waypoint
rather than stopping at each. It is the obviously nicer motion and it is a much larger
change: it needs a real trajectory representation, it makes "which pose is the arm at"
ambiguous, and it removes the per-leg stall check that currently catches an obstruction.
Stop-at-each-waypoint first, blended later, and only if the motion genuinely needs it.

### Order of work

1. ✅ Storage + tests *(done 2026-08-12)*.
2. ✅ `s`+digit and `p`+digit, on the interleaved park.
3. ✅ Sequences: the queue, the echo, the abort — and ⛔ **leaving PARK for any reason
   abandons the rest**, said out loud when it happens.
4. ✅ Park speed on `+`/`-` while in PARK mode.
5. ✅ **Easing** — on by default, `--no-smooth` disables it. The opt-in caution was
   withdrawn after reading `advance_park_command`: scaling an already-clamped step
   *down* cannot overshoot, so the risk that justified the flag did not exist.
6. ✅ **Corner blending** (`src/motion.py`, 12 tests) — ⛔ **and this was the feature
   actually being asked for.** Item 5 shapes *speed*; this shapes the *path*. Building
   only the first left the arm stopping dead at every waypoint, which is the jitter
   Julien described. Both now exist and are independent.
7. ✅ **The interaction**: `p Enter` base · `p 1 Enter` one pose · `p 1 2 3 Enter` shows
   the plan and waits for a second Enter · `-/+` speed and `,/.` corners work **while
   typing as well as while moving**.

8. ✅ **Easing as its own axis** (2026-08-12) — five profiles cycled with `e`:
   `none` · `in` · `out` · `both` · `s-curve`. ⭐ **Corner blending and easing are
   independent and both are needed**: blending decides the *shape*, easing decides the
   *speed along it*. Ctrl-C uses `out` — full speed from the first step, soft landing —
   because a shutdown move should leave at once.
9. ✅ **The end-of-park wait** — `settled` had shared the `blocked` timer, so every park
   finishing outside the 0.02 rad tolerance idled for four seconds before admitting it
   had arrived. Two questions, two patiences: 0.5 s and 4 s.

⬜ **What is left in this area:** nothing structural — only Julien's judgement on the
arm about the default corner radius (`smooth`, 0.15 rad), the default ease (`both`) and
the default speed. All three are live knobs, so tuning them needs no code.

⚠️ **If a future session wants more interpolation options** — he compared this to
Premiere Pro — the honest ranking is: (a) per-waypoint speed, so one leg can be slow and
the next quick, which needs the slot file to carry more than a pose; (b) *dwell* at a
waypoint, i.e. pause N seconds before continuing, which is what a pick-and-place demo
actually needs; (c) true spline interpolation through the waypoints rather than
blended corners, which is prettier and much harder to bound. **(b) is the one with real
downstream value** — it is the difference between a motion and a *task*, and it is what
the MCAP recorder will want to replay.

⚠️ **Deliberately still NOT doing: Cartesian-space blending.** These are *joint* poses,
so a joint-space path needs no IK, cannot hit a singularity, and provably stays inside
the joint range the waypoints span. A Cartesian blend would look smoother in the world
at the cost of an IK solve per sample and a singularity risk on every corner, for poses
that were never Cartesian to begin with. Revisit only if a task genuinely needs a
straight line *in the world*, and note that recorded demos would then need Cartesian
waypoints too.

⭐ **One decision arrived during implementation and is worth keeping:** Julien ruled that
Ctrl-C must *always* return to the base pose regardless of what has been saved since —
which turned "the park pose" from one variable into **two different things**. Slot `0` is
the base: the pose the arm is released in, changed only by a deliberate `s 0`. Slots 1-9
are waypoints, and Ctrl-C ignores them entirely. **A pose that is safe to be let go in is
not the same as a pose you want to return to mid-task**, and before this the two shared a
variable that `s` silently overwrote.

## Step 7 — Cameras

C920 plus the wrist D405s. Needed for data collection, not for teleop. Deliberately last of the near-term set
because nothing else depends on it.

**Where it actually stands, 2026-08-11.** The C920 works in a window and in the terminal, cameras are
selectable **by name** — identified by measurement, not by any list order, after a positional guess turned
out to be wrong about two of four ([FINDINGS §22](FINDINGS.md)) — and one D405 is mounted on arm B, measured,
and **delivering a colour picture over plain UVC with no SDK at all**. So this step is further along than
"needed for data collection" suggests: the wrist view is drivable today, and `librealsense` is an upgrade for
depth, intrinsics and camera controls rather than a prerequisite.

---

## Deliberately NOT doing, and why

- **Joint-space jogging as a stepping stone** (SpaceMouse axis → one joint each, no IK). Genuinely simpler and
  it would prove the plumbing sooner. Rejected as the *main* path because it is throwaway — the plan needs
  cartesian control — and because simulation already provides a risk-free place to debug the real thing.
  **Kept as a fallback:** if IK fights us, joint jogging still gets a SpaceMouse driving the arm the same day.
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

## ⛔ Do NOT connect the second SpaceMouse yet — and this is measured, not a preference

Julien offered to free a USB port and connect the second SpaceMouse. **Recommendation: don't, yet.** Not
because "one thing at a time", but because of a specific fact checked on 2026-08-10:

```
hid.enumerate() for VID 0x256f:
  usage=0x08  serial=''  path=b'DevSrvsID:4295192284'
  usage=0x30  serial=''  path=b'DevSrvsID:4295192284'
  usage=0x33  serial=''  path=b'DevSrvsID:4295192284'
```

**The SpaceMouse reports an empty serial number.** So the trick that saved us on the CAN adapters — select by
serial, never by index — **does not transfer**. Two SpaceMice would be indistinguishable except by
`path`, a macOS IOService registry ID that changes on replug and carries no meaning.

That is exactly the bug class that already bit twice today: `find_device()` returns `multi[0]`, so with two
pucks attached, *which arm a given puck drives would be arbitrary and would silently change between runs.*
Connecting the second one before fixing selection means debugging teleop and device-identity at once.

**What to do instead, in order:** get one puck driving one arm (step 1b) → then build selection by **USB
topology** (which hub port a device is on, stable while nothing is re-cabled) → *then* connect the second.

**And keep the camera plugged in.** Nothing needs to be freed up: the camera is not competing for anything we
need today, and unplugging it costs a re-verification later for no gain.
