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

1. Extract `ArmSession` with **no behaviour change**; run `--arms B` and confirm it feels identical.
2. Add the `a` selector and per-arm status lines. Still one arm.
3. `--arms B,G`, starting in HOLD, gripper enabled, desk clear.
4. Only then GUIDE and CONTROLS on two arms.

## Step 7 — Cameras

C920 plus the wrist D405s. Needed for data collection, not for teleop. Deliberately last of the near-term set
because nothing else depends on it.

**Where it actually stands, 2026-08-11.** The C920 works in a window and in the terminal, cameras are now
selectable **by name** rather than by an index that moves on replug, and one D405 is mounted on arm B and
measured. ⛔ **The one thing that changed the plan:** OpenCV can open the D405 over plain UVC, but macOS
exposes only its **depth** stream — so the "no SDK needed" shortcut cannot produce a picture to drive by, and
`brew install librealsense` moves from optional to required for this step. [FINDINGS §22](FINDINGS.md).

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
| 2 | ~~Are the **D405 wrist cameras** mounted?~~ **Answered 2026-08-11: one is, on arm B, and it is plugged in and measured** (serial `255323071773`, USB SuperSpeed). The second is with arm G and unplugged. ⛔ **And the cheap software path is closed:** macOS exposes only its *depth* stream over UVC, so a colour picture needs `librealsense` — FINDINGS §22 | Scopes step 7 |
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
