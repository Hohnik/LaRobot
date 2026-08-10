# Roadmap — from "the arm twitches" to "I drive it with the SpaceMouse"

> **Purpose of this file.** The README says what is *true now*. This says what we are going to do,
> **in what order, and why that order** — because the ordering is the part that carries the reasoning,
> and it is the part that gets lost between sessions.
>
> Julien's stated near-term goal (2026-08-10): *"be able to control the arm with the space mouse — a single
> arm with a single space mouse — and then we can go from there."* Everything below is ordered to reach that
> as directly as safety allows, and no more.

---

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

**Done when:** `get_yam_robot(channel=<arm1>, sim=False)` returns a working robot and `get_joint_pos()`
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

## Step 6 — Second SpaceMouse → bimanual

The hard part is already done: `move_both_grippers.py` proved two arms on two independent CAN buses driven
from one 100 Hz loop, with genuinely independent trajectories. Bimanual teleop is that, with IK in the middle.

## Step 7 — Cameras

C920 plus the wrist D405s. Needed for data collection, not for teleop. Deliberately last of the near-term set
because nothing else depends on it.

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
| 1 | **How much clear space is around arm 1?** Table edges, walls, the other arm, anything fragile | Steps 3-4 move the *whole* arm through space for the first time. Gripper twist could not reach anything; a shoulder or elbow can |
| 2 | Are the **D405 wrist cameras** mounted? | Scopes step 7 |
| 3 | The **second SpaceMouse** — owned, or still to buy? | Scopes step 6 |
| 4 | Is there an **e-stop**, or is wall power the only cut-off? | Changes how aggressive the step-4 safety envelope needs to be |
