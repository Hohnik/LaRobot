# Commands — the whole inventory, by how much they can move

> **Every script that can transmit is a dry run by default.** Leaving off `--yes` prints the full plan and
> sends nothing. That is the single convention to remember.
>
> **Everything takes `--arm B` or `--arm G`**, resolved by adapter *serial*, never by index.
> Default is `B`. Run them from `~/Developer/Projects/yam-robotics`.

---

## The four you actually need

```bash
# 1. Dial in which puck direction drives which motion. NO HARDWARE — arms can be unplugged.
uv run scripts/map_axes.py

# 2. Everyday driving: guide by hand, teleop with the SpaceMouse, park. All in one session.
uv run scripts/teleop_session.py --yes --arm B

# 3. After ANY power cycle, AND once per arm before its very first run. ~10 s, jaws only.
#    ⛔ --arm is NOT optional here: without it this calibrates B whatever you meant,
#    driving the wrong arm's jaws into both stops. G needs its own run — as of
#    2026-08-10 config/gripper_limits.json holds B only, which is why G refuses
#    to start with the gripper enabled.
uv run scripts/calibrate_gripper.py --yes --arm B
uv run scripts/calibrate_gripper.py --yes --arm G

# 4. Is the arm alive? Enables all 7, reads, disables. Nothing moves.
uv run scripts/read_arm_state.py --yes --arm B
```

### `teleop_session.py` — the keys

| key | mode / action |
|---|---|
| `g` | **GUIDE** — zero gravity, the arm is weightless, push it by hand |
| `t` | **TELEOP** — the SpaceMouse drives the end effector |
| `h` | **HOLD** — the arm holds its pose. The safe idle |
| `p` | **PARK** — drive back to the saved pose at 0.4 rad/s. Press `h` or `t` to stop |
| **`m`** | **MAP** — remap which puck axis drives which motion. **The arm HOLDS; the puck moves nothing** |
| `s` | save the current pose as the park pose |
| `o` / `c` | open / close the gripper |
| `[` / `]` | gripper step slower / faster |
| `r` | wrist rotation on / off *(on by default)* |
| `,` / `.` | rotation speed slower / faster |
| **`x` `y` `z`** | flip **X / Y / UP** — **saved to `config/spacemouse_map.json`** |
| **`1` `2` `3`** | flip **ROLL / PITCH / YAW** — same file |
| `-` / `+` | linear speed slower / faster |
| `?` | reprint the key list |
| `b` | **assign the puck buttons** to gripper open/close — works in **any** mode. Then hold a button to move the jaws (in TELEOP/CONTROLS); `f` swaps them |
| `q` | **QUIT** — goes to HOLD and *asks*. Then **`p` to park**, `g` to go weightless and park by hand, `d` to disable |

⭐ **`q` `p` `d` is a hands-free shutdown.** The park pose defaults to **wherever the arm was when the
session started**, so unless you have saved one with `s`, pressing `p` at the quit prompt drives it back to
where it began and `d` then releases it. This also means the two arms no longer have to be placed the same
way before a session — each parks back to its own measured start.

Useful flags: `--start-mode hold|guide|teleop` · `--no-rotation` · `--no-gripper` · `--linear-scale 0.2` · `--box 0.4`

⚠️ **`x` `y` `z` `1` `2` `3` flip a ROBOT MOTION, not a puck axis.** Under the identity map that is
the same arithmetic, so the hand-dialled file still means what it meant. Under a permutation it is the
only reading that stays useful: pressing `x` means *"the gripper goes the wrong way"*, which is a
statement about the arm.

### ⭐ CONTROLS mode (`m`) — set up the mouse while watching the arm

**The arm DOES move** — that is the point. But only along the **one axis you push hardest**, at **half
speed**. So the motion is unambiguous: one gesture, one motion, and you can see what it does.

⛔ **Moving the puck never changes the map.** Only the keys below do. (This was the opposite in the first
version, and exploring destroyed a hand-dialled map — FINDINGS §11.2.)

| key | action |
|---|---|
| *push the puck* | the arm performs whatever that control is bound to. **Nothing is edited** |
| **`f`** | **reverse the direction of the control you just used** — the main one |
| `1`…`6` | **SWAP** that control with another motion's — **1**=X **2**=Y **3**=UP **4**=ROLL **5**=PITCH **6**=YAW. Both move, so nothing is orphaned, and **the same key again swaps back** |
| `u` | that control drives nothing |
| **`b`** | **assign the two puck buttons** to gripper OPEN / CLOSE — it asks you to press each one. Then `f` swaps them, exactly as it reverses an axis |
| `0` | revert the whole map to how it was when the session started |
| `-` / `+` | linear speed | 
| `,` / `.` | rotation speed |
| `r` | wrist rotation on / off |
| `t` / `g` / `h` / `m` | leave to TELEOP / GUIDE / HOLD / HOLD |

The status line shows which control you last used, which motion it drives, and the resulting speed:

```
[CONTROLS] puck y     -0.62  → Y -0.037 m/s   (f reverses, 1-6 reassigns)
```

"The control you just used" is remembered **with no timeout**, so `f` still works after you have let go of
the puck. Reassigning a control that another motion was using **unbinds that motion** and says so, so one
gesture can never drive two.

**Saving is no longer unconditional:** the file is written only if the map actually changed, and the previous
contents are copied to `config/spacemouse_map.prev.json` first.

### One map or two? — `--fork-map`

By default **both arms share one map**, and the plan line says so out loud:

```
  map scope   : SHARED — edits here affect BOTH arms
```

If G genuinely needs different directions — a mirrored arm may well want an inverted Y — give it its own,
seeded from whatever it uses today:

```bash
uv run scripts/teleop_session.py --yes --arm G --fork-map     # G gets its own map
uv run scripts/teleop_session.py --yes --arm G --share-map    # ...and back to the shared one
```

⛔ **Check the scope line before editing.** Tuning G while it is still on the shared map changes B too.

### The six motions — measured, not assumed

`teleop.step()` integrates the twist in the **world frame**, so these do not change when the wrist turns.

| # | motion | world | what it does |
|---|---|---|---|
| 1 | `X` | `+X` | horizontal, straight out from the base at `base_yaw = 0` |
| 2 | `Y` | `+Y` | horizontal, 90° left of +X seen from above |
| 3 | `UP` | `+Z` | straight up, away from the table |
| 4 | `ROLL` | about `+X` | twists the tool; **the tool point stays put** |
| 5 | `PITCH` | about `+Y` | tips the tool; **the tool point stays put** |
| 6 | `YAW` | about `+Z` | spins the tool about vertical; **the tool point stays put** |

Measured in simulation 2026-08-10: gravity is `(0,0,−9.81)` and joint 1 rotates about world Z, so +Z
is up; a unit twist on each component moved the tcp along exactly that axis (0.0499 m for a commanded
0.05); each rotation component rotated about exactly that world axis with **≤0.3 mm** of tool-point
drift over 17°. *A wrong rotation sign therefore twists the wrist in place rather than flinging the
gripper across the desk.*

⚠️ **"Forward" and "left" are deliberately not claimed.** +X and +Y are horizontal, but which one
points away from *you* depends on how the arm is turned on the desk, and no file here records that.
Bind them by watching the arm, or pick one and flip it if it feels wrong.

⚠️ **No shift keys anywhere, and unrecognised keys do nothing.** Both were bugs: rotation speed used `R`/`T`,
and any unknown key — *including Enter* — used to cancel PARK.

⚠️ With two SpaceMice attached it asks you to **move the puck you want to assign to this arm**. They both
report an empty serial number, so there is nothing else to key an assignment off.

---

## ⭐ Driving from the camera's point of view

Two terminals. Neither the camera process nor the frame setting can move a motor on its own.

```bash
uv run scripts/camera_view.py --list            # which index is the arm-mounted camera?
uv run scripts/camera_view.py --index 1 --big   # the live view
```

Then in the session, press **`v`** to cycle the control frame, or start with `--frame tool`:

| frame | what "forward" on the puck means | use it when |
|---|---|---|
| **world** *(default)* | a fixed direction on the desk; does **not** turn with the wrist | you are looking **at** the arm |
| **tool** | where the gripper points — **turns with the wrist** | you are looking **through** the wrist camera ⭐ |
| **camera** | the **modelled D405** optical frame | ⛔ only once the real wrist cameras are mounted |

⛔ **Use `tool`, not `camera`, for the hand-mounted C920.** `camera` uses the D405's mounting transform
from the MJCF — a 25° cant off the flange — which is simply not where a webcam cable-tied on by hand is
sitting. Nobody has measured that mount, and inventing the transform is the exact failure this repo keeps
cataloguing. Mount the webcam roughly looking the way the gripper points and use `tool`.

⚠️ **First run needs macOS camera permission.** You will see
`OpenCV: not authorized to capture video` until it is granted — macOS raises the dialog for the app running
the terminal. If no dialog appears, grant it in **System Settings → Privacy & Security → Camera**. This is a
system gate; no code change fixes it.

`--measure 10` captures headlessly and reports the real frame interval, so "no latency" is checked rather
than asserted. `--flip` and `--rotate` handle a camera mounted mirrored or sideways.

## Read-only — cannot move anything, no `--yes` needed

```bash
uv run scripts/probe_hardware.py        # enumerate HID, open the SpaceMouse, listen 5 s
uv run scripts/probe_can.py             # listen-only CAN watch; the transceiver cannot even ACK
uv run src/spacemouse_live.py --seconds 25 --until-complete   # live 6-axis readout
uv run scripts/map_axes.py              # ⭐ dial in the axis map. SpaceMouse only, no robot at all
uv run scripts/teleop_sim.py --demo     # the FULL teleop loop against MuJoCo. No hardware at all
uv run scripts/teleop_sim.py            # ...driven by the real SpaceMouse, still simulation only
```

`teleop_sim.py` now applies `config/spacemouse_map.json`, so **a mapping can be verified in
simulation before the arm is involved**. Until 2026-08-10 it ignored the map entirely — which made
the one place an axis convention is free to get wrong the one place it could not be tested.

## Tests — no hardware, no device, no simulation

```bash
uv run scripts/test_axis_map.py && uv run scripts/test_park_target.py
```

34 checks. The two that matter most: the hand-dialled `spacemouse_map.json` is compared against the
**old** formula over 500 random inputs, so this refactor cannot have silently thrown away the bench
time that produced it; and a 7-joint park pose against a 6-DoF `--no-gripper` robot no longer raises.

### The MuJoCo viewer — needs an env var on this machine

⛔ **Plain `uv run mjpython …` FAILS** with `Library not loaded: @rpath/libpython3.12.dylib`. `mjpython`
dlopens the venv interpreter and the uv-managed CPython does not put `libpython` on its rpath. The library
*does* exist, so one variable fixes it (verified: `mjpython OK, mujoco 3.11.0`):

```bash
DYLD_FALLBACK_LIBRARY_PATH="$HOME/.local/share/uv/python/cpython-3.12.12-macos-aarch64-none/lib" uv run mjpython scripts/teleop_sim.py --view
```

Without the viewer the run continues headless, so `--view` is optional in every sense.

---

## Transmits, but cannot move a motor

These send CAN frames yet never enable anything — register reads only.

```bash
uv run scripts/identify_arm.py --yes --arm B              # motor models, sw versions, safety timeout
uv run scripts/identify_arm.py --arm B --scan 1 8 --yes   # what is on this bus at all
uv run scripts/bench_can.py --yes --cycle --samples 8000     # control-rate measurement
```

**`--scan` is the first thing to run when something feels wrong.** If it reports 0 devices, the problem is
power or wiring, not software — see `FINDINGS.md` §8.

---

## Enables motors, sends no setpoint — the arm should not move

```bash
uv run scripts/ping_motors.py --yes --arm B      # per-motor position, torque, TEMPERATURE, error
uv run scripts/read_arm_state.py --yes --arm B   # all 7 through the whole-arm chain
```

`ping_motors.py` is also the temperature check. Idle is **31-36 °C**; **41-42 °C while holding a pose is
normal thermal equilibrium, not a fault.**

---

## ⛔ Moves the arm

```bash
uv run scripts/teleop_session.py --yes --arm B                 # ⭐ the one to use
uv run scripts/calibrate_gripper.py --yes --arm B              # jaws to both stops, gently, once
uv run scripts/hold_pose.py --yes --arm B                      # hold the pose; success = nothing moves
uv run scripts/hold_pose.py --yes --arm B --zero-gravity       # weightless, hand-guide only
uv run scripts/teleop_gripper.py --yes --arm B                 # gripper twist + jaws only, no IK
uv run scripts/move_one_motor.py --yes --arm B --delta 1.5 --cycles 3   # one motor, bounded
uv run scripts/move_both_grippers.py --yes                        # both arms' grippers, one loop
```

**Superseded but kept:** `teleop_arm.py` was the two-phase version. `teleop_session.py` replaces it — the
phases became live mode switches, and the snap bug it contained is fixed there, not here.

---

## Two arms

**Separately, today:** just change the flag. Each session assigns its own puck.

```bash
uv run scripts/teleop_session.py --yes --arm G
```

**Simultaneously:** not built yet. The hard half is proven — `move_both_grippers.py` already drives two arms
from one 100 Hz loop across two buses — but cartesian bimanual needs one process holding both robots, two
`CartesianTeleop` instances and two pucks assigned up front. Budget is fine: 14 motors ≈ 6.2 ms/cycle against
a 10 ms deadline.

---

## Recovery

| symptom | do this |
|---|---|
| `--scan` reports **0 devices** | **Check the arms' wall power.** Both arms silent at once = shared cause = mains |
| motor over-temperature | power-cycle the arms, then **re-run `calibrate_gripper.py`** — limits shift |
| jaws slam at startup | the saved limits are stale. Re-run `calibrate_gripper.py` |
| "fail to communicate with motor N", varying N | desync; already mitigated. Re-run — it is not a dead motor |
| arm drooped after a crash | nothing is lost. Restart the session, `g` to go weightless, reposition, `s` to save a park pose |
