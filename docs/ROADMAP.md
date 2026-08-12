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

### ⭐ What the five ease profiles actually mean, in plain terms

Julien asked, and "I don't know what s-curve is" is a fair thing not to know.

Every profile answers one question: **how does the speed change at the two ends of a
move?** The middle is always full speed.

| profile | start | stop | when you want it |
|---|---|---|---|
| `none` | instant | instant | shortest possible move; a small jolt at each end |
| `in` | gentle | instant | leaving a delicate position, arriving somewhere it does not matter |
| `out` | instant | gentle | ⭐ **what Ctrl-C uses** — go now, land softly |
| `both` | gentle | gentle | the general-purpose one |
| `s-curve` | *very* gentle | *very* gentle | the smoothest, and the slowest off the mark |

**The difference between `both` and `s-curve`** is what is being smoothed. `both`
ramps the **speed** — but the *acceleration* still jumps from nothing to a constant
value the instant it starts, which the arm feels as a small shove. `s-curve` ramps the
acceleration too, so the force builds up gradually. In an editor this is the
difference between dragging a linear keyframe handle and a Bézier one.

⚠️ **Both ends use `√` now, not a straight line, and that is why the tail stopped
crawling.** With a straight ramp the speed is proportional to the distance left, which
is exponential decay — it halves in equal time steps and never quite arrives.
Constant deceleration is `v ∝ √s` and it *does* arrive. Measured: the last 0.1 rad of
a 1 rad move went from 1.08 s to 0.59 s while the rest was unchanged.

### ⭐ The two "patiences", and whether they are smart

Also a fair question. **They exist because two different things look identical from
outside: an arm that has finished, and an arm that is stuck.** Both stop making
progress. The only way to tell them apart is *how far from the target it stopped* and
*how long you are willing to wait before deciding*.

- **0.5 s — "has the controller finished settling?"** The arm never lands exactly on
  the commanded pose; it settles a fraction of a degree short, where its stiffness
  balances gravity. Half a second of no improvement while already *close* means it has
  arrived as well as it ever will.
- **4 s — "is something in the way?"** Stopping *far* from the target is a different
  claim, and a much more serious one. Four seconds before saying so avoids crying wolf
  over a slow patch.

**Is it necessary?** Yes, and the history says so: they were one timer, and every park
that finished outside the 0.02 rad tolerance — most of them — sat apparently idle for
four seconds before admitting it had arrived. **Is it smart?** It is the minimum
honest answer. The alternative is one number, which either declares arrival too
eagerly (and hides an obstruction) or waits too long (which is what he saw). ⚠️ If it
ever needs revisiting, the number to change is the 0.5 s, and the symptom would be a
park declaring success while the arm is visibly still moving.

### ⭐⭐ Is waypoint playback good training data? — thought through, 2026-08-12

Julien's idea: *"define the waypoints in guide mode and then just play it without the
hands in the way, so the robot can see all of the positions and the camera input as it
should be — but at the same time it's predefined and not done with a mouse. Is that
good data for the robot to learn?"*

It is a genuinely good question and the answer is **no for policy learning, yes for
almost everything around it** — and the reason is precise enough to be worth keeping.

⛔ **Why it fails as training data.** An imitation policy learns a mapping from *what
it sees* to *what to do*. A replayed trajectory is **the same every time regardless of
what the camera sees**. So in the training set the action is statistically independent
of the observation — and a model fitting that data has **no reason to look at the
image at all**. It can score perfectly by memorising the trajectory and ignoring the
camera, which is exactly the model you do not want. Move the object 5 cm and it does
the identical thing.

⛔ **The second failure is subtler and worse: no corrective behaviour.** A human
teleoperating drifts slightly off, notices, and pulls back — so the demos naturally
contain thousands of tiny examples of *"you are a bit off, here is the way back."* A
replay is perfect every time, so the policy never sees a single recovery. The first
time it makes a small error at run time it is in a state the training data never
contained, and errors compound. This is the standard covariate-shift argument that
motivates DAgger, and it is the reason scripted demos underperform human ones even
when the scripted ones look cleaner.

✅ **What it IS excellent for, and these are real:**

1. **Validating the recording pipeline** — a repeatable trajectory is the perfect test
   signal for MCAP schema, timestamp alignment, camera-to-joint sync and dropped
   frames. You cannot debug a recorder against data that is different every time.
2. **Measuring the rig** — tracking error, latency, repeatability, thermal drift over
   a hundred identical cycles. All of that needs a motion that does not vary.
3. ~~⭐ **AUTOMATED SCENE RESET, which is the one with real leverage.** The genuine
   bottleneck in demo collection is not performing the task, it is *putting everything
   back* between takes. A waypoint run that picks the object up and returns it to a
   randomised start position means Julien can record demo after demo without touching
   the scene — the playback creates the conditions, the human still provides the
   demonstration.~~
   ⛔ **REFUTED BY JULIEN ON 2026-08-12, on both of its claims. Struck rather than
   deleted so nobody inherits it.** The bottleneck is *executing* the task with the
   SpaceMouse, not resetting the scene — and an automated reset is close to circular,
   because placing an object at a chosen position **is** the task being learned. Full
   account, and what replaces it, in
   [§6.6 below](#66--how-demos-will-actually-be-collected--his-correction-2026-08-12).

⚠️ **The one way playback could become training data** is if the waypoints were
*generated per episode from the observed scene* — object detected here, so approach
there — because then the action genuinely depends on the observation. That is a
different and much larger project (a scripted policy with perception), and it is worth
knowing it exists rather than assuming replay is simply unusable.

⚠️ **If a future session wants more interpolation options** — he compared this to
Premiere Pro — the honest ranking is: (a) per-waypoint speed, so one leg can be slow and
the next quick, which needs the slot file to carry more than a pose; (b) *dwell* at a
waypoint, i.e. pause N seconds before continuing, which is what a pick-and-place demo
actually needs; (c) true spline interpolation through the waypoints rather than
blended corners, which is prettier and much harder to bound. **(b) is the one with real
downstream value** — it is the difference between a motion and a *task*, and it is what
the MCAP recorder will want to replay.

⭐ **Julien's ruling on all three, 2026-08-12:** *"all three sound really good, but they
don't have to be done right now and they don't have to be main options. They could just
be extra options that we can press a button for."* So they are **deferred, and they are
extras** — additional knobs on the run that already exists, not a redesign.

The place they attach is already built: the run plan line and the `pending`/`confirm`
key handler in `teleop_session.py`, where speed, corners, ease profile and ramp length
already live. **Per-waypoint dwell is the one to do first**, because it needs the slot
file to carry `{pose, dwell}` instead of a bare pose — and `park_slots()` /
`with_park_slot()` already tolerate that shape change, since a slot's value only has to
be a non-empty list. ⚠️ Adding dwell also turns a saved sequence into a *task
description* rather than a path, which is exactly what the recorder wants to replay —
see the data-collection analysis above for why that matters more than a prettier curve.

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

## 6.6 ⭐⭐ How demos will actually be collected — his correction, 2026-08-12

> ⛔ **This section supersedes the "automated scene reset" claim in §6.5 and part of the
> reasoning around it.** It is the most consequential design conversation this project has
> had, because it decides *where the training data comes from* — and the previous answer
> was wrong about which part of the job is expensive.
>
> ⚠️ **Nothing here is built yet.** It is a design and a research question, written down in
> full because it will be picked up in a different session, probably by an agent with no
> context. Read it before building any part of the recorder (step 5).

### What Julien reported, in his words

> *"It's definitely very hard — or it seems to be very hard — to control and execute the
> task with the space mouse. So the hard part is definitely not setting up the task. I can
> just take the object with my hand, place it somewhere, put the robot back in the zero
> gravity guide mode, or put it in a random position, whatever. And then I have to execute
> the task with a space mouse, which is very difficult, and there's no automated way to set
> up the scene — because if the robot can just place the object at a random location, then
> it can already control itself."*

And his proposal:

> *"Place the waypoints and then play the task, and then place the waypoints slightly
> differently and play the task slightly differently. Or place the waypoints and think of
> different ways in which the splines could connect, or the order of how we place the
> points. Or we could maybe even slightly add random jittering on the waypoints that we
> place. All of those features we could add, and then we could really easily create many
> different ways to do the actual task. Because if we slightly move the object, placing a
> new waypoint for the new object isn't difficult — if we just use `g` mode for that it's
> very easy, compared to trying to find it with the mouse — even though, as you said, we
> wouldn't have all of the pullback and drift and those types of teleoperate problems."*

### ⛔ Where the previous analysis was wrong, precisely

Three separate errors, and they are different kinds:

1. ⛔ **"The genuine bottleneck is putting everything back between takes" was a claim about
   *his* rig, imported from the general literature without checking.** It is often true —
   for a trained operator on a mature rig, reset dominates. It is **false here**: one
   person, an unfamiliar 6-DoF rate-control device, a task never yet performed once.
   Resetting means picking an object up with a hand. *Verify the consequence, not the
   mechanism* (working contract rule 5) — this reasoned from a pattern instead of from the
   rig, and he found it out by trying.

2. ⛔ **The circularity objection is correct and was missed entirely.** For the robot to
   place an object at a randomised start position it must pick it up *from wherever it
   currently is*. Open-loop playback can only do that from a **deterministic** pose. So an
   automated reset needs: the demo to always end with the object in a fixed, graspable
   pose (a cube dropped in a bin does not land in a known orientation), plus one
   hand-taught reset path per start position it is supposed to produce.
   ⭐ **And the sharpest form of the error is internal:** the same analysis said one page
   later that per-episode waypoints from the observed scene are *"a different and much
   larger project"* — which is exactly what a real reset policy is. **The recommendation
   quietly depended on the thing it had just called out of scope.**

3. ⚠️ **The choice was framed as "replay vs teleop", and it is not.** If SpaceMouse teleop
   is hard enough that a task cannot be demonstrated at all, the real comparison is
   **replay vs nothing**, and 100 imperfect demos beat 8 excellent ones. That reframing is
   his, and it is the decisive one.

### ✅ What the previous analysis got right, and it still holds — but conditionally

**The causal-confusion argument is sound, and the condition it depends on is exactly what
his proposal breaks.** Stated properly, so the distinction is usable:

- A policy trained by imitation fits a mapping from **what it sees** (`o`, the camera) to
  **what to do** (`a`, the commanded joints).
- **Pure replay with a fixed scene:** `a` is a function of time alone and `o` is
  ~identical every episode. The action carries no information about the observation, so
  nothing in the training signal rewards *looking at the image*. The policy can score
  perfectly by memorising the trajectory, and moving the object 5 cm changes nothing it
  does. ✅ Correct, and still correct.
- ⭐ **His proposal is not that.** *"If we slightly move the object, placing a new waypoint
  for the new object isn't difficult"* — when the object moves **and** the waypoints are
  re-taught to match, the action becomes a function of the object's position, which is
  what the camera sees. The dependence runs through his eye and his hand, but it is
  genuinely there. **That is real learning signal, and it is the thing behaviour cloning
  needs.**

⭐ **So the honest name for his scheme is not "replay". It is *kinesthetic teaching with a
clean-scene replay for recording*** — a standard and respected demonstration modality,
not a degenerate one.

⭐ **And it has an advantage over both alternatives that was under-sold before.** Ordinary
kinesthetic teaching puts the human's hands in every camera frame, so the policy either
learns to look for a hand — a feature that is absent at test time — or the frames have to
be masked. **Teaching and recording are separate passes here**, so the recorded episode
shows nothing but the robot and the object. That was in his original idea (*"just play it
without the hands in the way, so the robot can see the camera input as it should be"*) and
it is a real, non-obvious win.

### ⚠️ The one substantive objection that survives — and how to answer it

**A replay contains no corrective behaviour.** A human teleoperating drifts, notices, and
pulls back, so the demos are full of tiny *"you are off, here is the way back"* examples. A
replay is perfect every time. The first small error at run time therefore puts the policy in
a state the data never contained, and errors compound. (This is the standard covariate-shift
argument: behaviour-cloning error grows with the square of the episode length in the worst
case, against linear growth for a method that collects corrections on-policy — the argument
DAgger exists for.)

⭐⭐ **But the replay engine can MANUFACTURE that data, more systematically than a human
produces it by accident. This is the strongest new idea in this section:**

1. **Perturbed starts.** Begin the replay from a joint configuration slightly off the
   nominal path. The controller drives back onto it, and every recorded step is literally
   *"I am off-path, here is the way back"*, labelled with an image showing the arm off-path.
2. **Bounded noise injected during the replay**, with the nominal path still the target —
   so the arm wanders and the recorded action is the correction. This is DART-style noise
   injection (Laskey et al., 2017, *"DART: Noise Injection for Robust Imitation Learning"*),
   and it exists precisely to buy back the robustness that clean demos lack.

⭐ Both are nearly free here: the path is already a parameterised object with a cursor
(`src/motion.py::JointPath`), and the loop already measures how far the arm is running
behind (`MAX_CURSOR_LAG`).

⛔ **The load-bearing requirement:** the recorded action must be **what was actually
commanded** and the observation **what was actually seen**. Recording the nominal waypoint
instead of the perturbed command produces a dataset that looks fine and teaches nothing.
Write this into the recorder's design, not into a comment.

### ⛔⭐ THE ONE ITEM ON HIS LIST THAT CAN POISON THE DATASET

**Jittering the waypoints WITHOUT moving the object is not free data augmentation — it is
actively harmful**, and it is the item that sounds most obviously good.

If the object stays put and the waypoints move, then one observation is paired with many
different actions. Two consequences:

- A unimodal, zero-mean jitter centred on the correct grasp mostly averages out, so it
  behaves as label noise and is *nearly* harmless.
- ⛔ **But it teaches, with every sample, that the action does not depend on the image** —
  the image is constant while the action varies. That is the causal-confusion failure
  amplified, not fixed. And jitter large enough to make some episodes *fail* means training
  on failures labelled as demonstrations.

⭐ **THE RULE, and it is the single most important line in this section: variation must live
in the (observation, action) pair jointly, never in the action alone.** Jitter the waypoints
*and* move the object. A jitter knob that does not require the object to move should not
exist, or should be labelled as a *robustness* tool for the motion engine rather than a
data-collection one.

⚠️ **The same caution, weaker, applies to two more of his four variations.** Reordering
waypoints and changing the spline/corner parameters vary the *trajectory* while the
observation is unchanged. They buy a little robustness to path shape — the policy learns
that many paths reach the same grasp — but they do **not** buy the thing BC most needs,
which is coverage of *object positions*. Cheap and mildly useful; not a substitute for
moving the object. (Varying *speed* is the most defensible of the three: it changes the
dynamics the policy sees, which is real robustness if the action space is timed joint
targets.)

⚠️ **And one that bears directly on his phrasing.** *"Many different ways to do the actual
task"* is a benefit **only with a policy class that can represent more than one answer.**
Trained with a plain squared-error head, several genuinely different demonstrations from the
same object position get averaged into something that does neither — a well-known BC failure
and the reason diffusion policies and action-chunking architectures exist. So: deliberate
multimodality is a liability with an MSE policy and an asset with a diffusion / ACT-style
one. **Worth settling before collecting a dataset built around it.**

### ✅ What was checked in the code rather than assumed

- ⭐ **A saved waypoint carries all seven joints, gripper included, and PARK commands the
  jaws through the clamp** (`src/yam_robot.py::park_target_from`, `park pose` in the startup
  plan reads `[…, 0.03, 0.035]`). **So a pick-and-place is replayable today.** This was the
  gate: had a slot held only the six arm joints, no replayed sequence could grasp anything
  and the whole idea would have needed a storage change first.
- ⛔ **But corner blending and grasping fight each other, and this is the real missing
  piece.** Blending is *defined* as flowing through a waypoint without stopping — and a
  grasp needs the arm to stop, the jaws to travel, and time to pass. Worse, the gripper is
  just another dimension of the joint vector, so a corner between "above the object, open"
  and "at the object, closed" makes the jaws close **during the descent**, closing early or
  pushing the object away.
- ⭐ **So per-waypoint DWELL is a prerequisite, not an extra.** §6.5 already ranked it *"the
  one with real downstream value"* and deferred it as an optional knob on his ruling. That
  ranking was right and this promotes it: a sequence needs
  `approach(open) → at-object(open) → dwell → at-object(closed) → dwell → lift(closed)`.
  A dwell > 0 also implies a stop, which suppresses blending exactly where blending is
  wrong — so `{pose, dwell}` covers both needs and no separate per-waypoint blend flag is
  required. `park_slots()` / `with_park_slot()` already tolerate that shape.

### ⭐⭐ A third option neither of us named, and it may be the best one

**Record the hand-guided motion CONTINUOUSLY in GUIDE mode, instead of recording waypoints.**

The papers this project follows (ABC / ASPIRE / ENPIRE) all teleoperate with **GELLO leader
arms**, which hand over joint angles directly and need no IK at all — `docs/Setup-Plan.md`
§4.2 names using a SpaceMouse instead as *the single largest deviation from the papers*.
⭐ **A YAM arm in GUIDE mode is very nearly a leader arm already:** hand-guiding *is*
joint-space demonstration. So his instinct is rediscovering why the papers use leader arms,
and his difficulty with the SpaceMouse is the *expected* cost of that deviation, not a
failure on his part.

The extension: stream `q(t)` while he hand-guides the arm through the whole task, then
**replay that trajectory with the scene reset and record the camera on the replay pass.**

- ✅ Actions carry human timing, hesitation and correction — the very thing waypoint
  interpolation throws away, and the objection above becomes much smaller.
- ✅ No IK, no SpaceMouse, no singularities, and the trajectory is **dynamically feasible by
  construction** because the arm physically went there.
- ✅ Clean frames, because the recording pass has no hands in it.
- ✅ Needs almost nothing new: GUIDE exists, replay exists, the recorder needs `q(t)`.
- ⚠️ **Actions from the teach pass and observations from the replay pass must be aligned** —
  the replay has to reproduce the taught trajectory closely enough that image `t`
  corresponds to action `t`. Checkable: the park loop already reports tracking error and
  cursor lag.
- ⚠️ Hand-guiding smoothly is its own skill; a wobbly path replayed at speed could be
  jerky. Light temporal smoothing, or replay at the recorded speed.

⭐ **This is the top candidate to investigate**, and it is strictly more informative than
waypoints. Waypoints remain the right thing for *scripted* motion — approach poses, reset
paths, rig measurement.

### ⭐ The hybrid that is probably the actual answer

Not either/or:

1. **Bulk of the dataset** — teach and replay, with the object moved by hand every episode
   and the grasp re-taught to match. Continuous hand-guided teaching if the third option
   works; waypoints if it does not.
2. **Recovery data** — perturbed starts and injected noise, to manufacture the corrections
   a clean replay cannot contain.
3. ⭐ **The SpaceMouse changes role rather than being abandoned** — from *the demo tool* to
   *the correction tool*. Nudging a trained policy for a few seconds where it fails
   (a DAgger-style loop) is the one regime where a hard 6-DoF device is fine, because it
   needs seconds of input rather than a whole task. **None of the teleop work is wasted.**

### ⚠️ Before building around "the SpaceMouse is too hard", two cheap checks

His difficulty is expected (see the leader-arm point above), but part of it may be tuning
rather than the device, and finding out costs minutes:

- **Control frame.** Driving in `world` while watching a wrist camera is much harder than
  driving in `tool`, where "forward" means *forward as the gripper sees it*. `v` switches
  it, each frame keeps its own axis map, and this has never been tried on hardware for a
  task.
- **A fine-speed mode.** `-`/`+` already scale the linear speed; the hard part of a grasp is
  the last two centimetres, which wants a much lower scale than the approach. A momentary
  "slow" state, or simply turning the speed down for the approach, is worth one attempt
  before concluding the device cannot do it.

⛔ **Do not block the data-collection design on these.** They are worth one session's
curiosity; the design above stands either way.

### ⭐ The measurement that settles this, and it takes 20 minutes

Everything above is argument. ⛔ **The comparison is empirical and cheap:**

**Time five episodes each way** — five demos by SpaceMouse teleop, five by teach-and-replay
with the object moved between each — and record, per episode: wall-clock seconds, whether
the task actually succeeded, and how much of the time was spent on what.

⚠️ **Rough estimates only, explicitly not measurements, for deciding whether the experiment
is worth running:** teach-and-replay looks like ~40-70 s per episode (move the object ~5 s,
re-teach the grasp in GUIDE ~20-40 s — the approach and place waypoints often need no
change if only the pick position moved — replay and record ~10-20 s), so ~100 episodes in
1.5-2 hours. Teleop at 60-90 s per attempt with a substantial failure rate is several times
that, and its failures are not merely wasted time — they are plausible-looking bad
demonstrations. ⚠️ Comparable published real-robot BC work typically uses ~50-200 demos per
task, so ~100 is the right order to plan for; that figure is literature-typical, not
measured here.

**Run the measurement before committing to either.** It is exactly this repo's method: the
one thing that has consistently beaten a confident argument is twenty minutes of measuring.

### Open questions that are genuinely Julien's

1. **Which task?** Everything above assumes something pick-and-place-shaped. The number of
   waypoints, whether dwell is needed, and whether one grasp re-teach per episode is enough
   all follow from the task, and it has never been named.
2. **Does the provenance of the demos matter to the professor / to his friend's plan?** The
   MCAP file is identical either way — actions are actions — but *"collected by kinesthetic
   teaching and replay"* is a real methodological difference from *"collected by
   teleoperation"*, and if results are ever compared against the papers it should be
   declared rather than discovered. A social question, not a technical one.
3. **Which policy class?** It decides whether deliberate multimodality is an asset or a
   liability (above), and it is downstream of his friend's plan rather than of this repo.

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
