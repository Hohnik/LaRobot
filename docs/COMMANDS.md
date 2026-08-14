# Commands — the whole inventory, by how much they can move

> **Every script that can transmit is a dry run by default.** Leaving off `--yes` prints the full plan and sends nothing. That is the single convention to remember.
>
> **Everything takes `--arm B` or `--arm G`**, resolved by adapter *serial*, never by index. Default is `B`. Run them from `~/Developer/Projects/yam-robotics`.
>
> ⭐⭐ **`teleop_session.py` takes `--arms B,G` and DRIVES BOTH ARMS**, confirmed on the hardware 2026-08-14 ([FINDINGS §55](FINDINGS.md)). `--arm` is unchanged and is still the one to type for one arm; the two spellings must agree or the session refuses. Each arm gets its own puck (assigned by wiggle), its own axis map, its own park pose and its own status row.
>
> ⛔ **Three things refuse with two arms, on purpose:** `--start-mode guide` (two arms weightless before anything is on screen), `w` and `l` (the recorder holds one arm's joints, so it would capture half a demonstration), and `m` while BOTH is selected (it edits one map from one wiggle). ⚠️ **This paragraph said "`--arms B,G` refuses today" until 2026-08-14 evening**, which was true for a few hours.

---

## The four you actually need

```bash
# 1. Dial in which puck direction drives which motion. NO HARDWARE — arms can be unplugged.
uv run scripts/map_axes.py

# 2. Everyday driving: guide by hand, teleop with the SpaceMouse, park. All in one session.
uv run scripts/teleop_session.py --yes --arm B

# 3. After ANY power cycle, AND once per arm before its very first run. ~10 s, jaws only.
#    ⛔ --arm is NOT optional here: without it this calibrates B whatever you meant,
#    driving the wrong arm's jaws into both stops.
#    ✅ BOTH arms are calibrated as of 2026-08-14, so --arms B,G runs with the gripper
#    enabled. (This comment said "B only" until then, dated 2026-08-10.)
#    ⚠️ The ±2π frame shift differs per session and per arm — B needed −2π and G +2π on
#    2026-08-14 — and build_robot() reconciles it automatically. Never write a direction
#    into the file; run ping_motors.py to see the current one.
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
| **`m`** | **CONTROLS** — remap which puck axis drives which motion. ⚠️ **The arm MOVES**, one isolated axis at half speed; only keys edit the map |
| `s` then `0`-`9` | save this pose — **`0` is the BASE pose Ctrl-C returns to**, `1`-`9` are waypoints Ctrl-C ignores |
| `p` then digits then `Enter` | drive to one pose, or **one blended motion through several** (`p 1 2 3 Enter` shows the plan, `Enter` again runs it) |
| `o` / `c` | open / close the gripper |
| `ö` / `ä` | **how long the ease lasts** — shorter / longer, in every mode. ⛔ **Corrected 2026-08-14: this row still said "gripper step" and that has been wrong since 2026-08-13.** The gripper step is `--gripper-step` now, because one key meaning two things is what pushed the step to its 0.200 ceiling by accident ([FINDINGS §30](FINDINGS.md)). ⭐ `[` / `]` are aliases and still work; `ö`/`ä` exist because the brackets are **AltGr+8 / AltGr+9** on a German layout ([FINDINGS §27.7](FINDINGS.md)) |
| **`w`** | ⭐ **RECORD a movement** taught by hand. Press again to stop, then `0`-`9` to save it. Works in any mode; GUIDE is the point of it. Recording moves nothing, so a mis-press is harmless |
| **`l`** then `0`-`9` | ⭐ **PLAY a recording back.** Shows the plan and waits for **Enter**, so a slip on `l` (which sits beside `ö`/`ä`) can never start the arm. It parks to the recording's start pose first, then follows it in real time. `-`/`+` set the speed, capped from the recording's own measured top speed |
| `e` | cycle the **ease profile** — `none` / `in` / `out` / `both` / `s-curve`. Works in **any** mode |
| `v` | control **frame** — world / tool / camera. ⚠️ Each frame keeps its **own** axis map |
| **`i`** | ⭐⭐ **MIRROR — the selected arm leads, the other follows it joint for joint.** Julien's idea: hand-guide one arm in GUIDE and the other reproduces the movement. It **asks twice**, like `l`: engaging starts a motion on the follower while your hands and eyes are on the leader. `i` again turns it off, and so does any mode key aimed at the follower. ⚠️ **Hold the leader still until the status row says FOLLOWING** — until then the follower is closing the initial gap at 0.30 rad/s, and a moving target may never let it close. ⛔ Needs two arms and exactly ONE selected. `--mirror mirror` negates the joints that reverse when arms FACE each other; the default `copy` is right for arms side by side, which is how they stand today |
| **`a`** | ⭐ **which arm the MODE keys aim at** — `B` → `G` → `BOTH`. **Driving always drives every arm**; only mode changes and edits are aimed. ⛔ Aimed rather than global because `g` on two arms is **8.6 kg** going weightless in one keypress ([FINDINGS §11.1](FINDINGS.md)). With one arm it says so and changes nothing. Refuses while CONTROLS (`m`) is open, since that wizard belongs to the arm it was entered on |
| `r` | wrist rotation on / off *(on by default)* |
| `,` / `.` | rotation speed slower / faster — **corner blending while a run is being typed or moving** |
| **`x` `y` `z`** | flip **X / Y / UP** — **saved to `config/spacemouse_map.json`** |
| **`1` `2` `3`** | flip **ROLL / PITCH / YAW** — same file |
| `-` / `+` | linear speed slower / faster — **park speed while in PARK or while a run is being typed** |
| `?` | reprint the key list |
| `b` | **assign the puck buttons** to gripper open/close — works in **any** mode. Then hold a button to move the jaws (in TELEOP/CONTROLS); `f` swaps them |
| `q` | **QUIT** — goes to HOLD and *asks*. Then **`p` to park**, `g` to go weightless and park by hand, `d` to disable |

⭐ **`q` `p` `d` is a hands-free shutdown.** The park pose defaults to **wherever the arm was when the session started**, so unless you have saved one with `s`, pressing `p` at the quit prompt drives it back to where it began and `d` then releases it. This also means the two arms no longer have to be placed the same way before a session — each parks back to its own measured start.

Useful flags: `--arms B,G` · `--start-mode hold|guide|teleop` · `--mirror copy|mirror` · `--no-rotation` · `--no-gripper` · `--linear-scale 0.2` · `--gripper-step 0.02` · `--reach 0.60` · `--floor 0.0` · `--fork-map` / `--share-map`

⛔ **`--box` is gone and this line advertised it until 2026-08-14.** It was replaced by `--reach` (how far the tip may go from the base) and `--floor` (how low it may go) when the workspace limit became a sphere plus a floor ([FINDINGS §43](FINDINGS.md)). Passing `--box` now errors, deliberately, rather than accepting a flag whose meaning had moved.

⚠️ **`x` `y` `z` `1` `2` `3` flip a ROBOT MOTION, not a puck axis.** Under the identity map that is the same arithmetic, so the hand-dialled file still means what it meant. Under a permutation it is the only reading that stays useful: pressing `x` means *"the gripper goes the wrong way"*, which is a statement about the arm.

### ⭐ CONTROLS mode (`m`) — set up the mouse while watching the arm

**The arm DOES move** — that is the point. But only along the **one axis you push hardest**, at **half speed**. So the motion is unambiguous: one gesture, one motion, and you can see what it does.

⛔ **Moving the puck never changes the map.** Only the keys below do. (This was the opposite in the first version, and exploring destroyed a hand-dialled map — FINDINGS §11.2.)

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

"The control you just used" is remembered **with no timeout**, so `f` still works after you have let go of the puck. Reassigning a control that another motion was using **unbinds that motion** and says so, so one gesture can never drive two.

**Saving is no longer unconditional:** the file is written only if the map actually changed, and the previous contents are copied to `config/spacemouse_map.prev.json` first.

### One map or two? — `--fork-map`

By default **both arms share one map**, and the plan line says so out loud:

```
  map scope   : SHARED — edits here affect BOTH arms
```

If G genuinely needs different directions — a mirrored arm may well want an inverted Y — give it its own, seeded from whatever it uses today:

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

Measured in simulation 2026-08-10: gravity is `(0,0,−9.81)` and joint 1 rotates about world Z, so +Z is up; a unit twist on each component moved the tcp along exactly that axis (0.0499 m for a commanded 0.05); each rotation component rotated about exactly that world axis with **≤0.3 mm** of tool-point drift over 17°. *A wrong rotation sign therefore twists the wrist in place rather than flinging the gripper across the desk.*

⚠️ **"Forward" and "left" are deliberately not claimed.** +X and +Y are horizontal, but which one points away from *you* depends on how the arm is turned on the desk, and no file here records that. Bind them by watching the arm, or pick one and flip it if it feels wrong.

⚠️ **No shift keys anywhere, and unrecognised keys do nothing.** Both were bugs: rotation speed used `R`/`T`, and any unknown key — *including Enter* — used to cancel PARK.

⚠️ With two SpaceMice attached it asks you to **move the puck you want to assign to this arm**. They both report an empty serial number, so there is nothing else to key an assignment off.

---

## ⭐ Driving from the camera's point of view

Two terminals. Neither the camera process nor the frame setting can move a motor on its own.

```bash
uv run scripts/camera_view.py --list                  # names, indices, and the checks
uv run scripts/camera_view.py --camera c920 --term    # ⭐ by NAME, drawn in this terminal
uv run scripts/camera_view.py --camera c920 --big     # by name, in a window
```

⭐ **Select by name, not by index.** The index is an AVFoundation artefact that moves on replug; the name does not. `--camera` takes any part of the name plus the aliases `d405`, `realsense`, `c920`, `iphone`, `builtin`, or a `vid:pid`. It **refuses** when a name matches nothing or matches more than one camera, and never falls back to index 0.

⛔ **The mapping is MEASURED, not read off macOS's list** — each index is asked for a resolution only one camera supports, and whoever answers exactly is that camera. That costs a few seconds at startup and is deliberately not cached. Assuming macOS's order was OpenCV's got two of four cameras wrong on 2026-08-11; [FINDINGS §22](FINDINGS.md) has the whole account. ⚠️ **Two cameras of the same model share every mode and cannot be told apart this way** — when the second D405 is plugged in, use `--index` and confirm by covering one.

**Keys in the terminal view** (`--term`):

```
q quit · f mirror · r rotate · b draw mode (blocks / iterm / kitty)
+ / -   how much of the pane the picture fills
1-6     CAPTURE size — what the camera sends the Mac. ⭐ These are THIS camera's own
        modes, ascending, so 6 is the best it can do (2560x1472 on the C920)
[ ]     DETAIL — what the Mac sends the terminal;  0 back to automatic
```

⚠️ **Capture size and detail are different knobs, and the second one is the one that bites.** The image sent to the terminal is automatically `min(pane, capture, protocol budget)`. In **kitty/Ghostty** that budget is tight because the kitty graphics protocol has **no JPEG** — PNG only, ~25x the encode time and ~30x the bytes — so detail there costs frame rate in a way it does not in iTerm2. Watch the `draw ms` readout: past half a frame it warns, and `[` is the fix. ⭐ `--term-test` sends one image in **each** protocol, so if your terminal happens to take iTerm2's escape as well, you can have a much sharper picture for free.

Then in the session, press **`v`** to cycle the control frame, or start with `--frame tool`:

| frame | what "forward" on the puck means | use it when |
|---|---|---|
| **world** *(default)* | a fixed direction on the desk; does **not** turn with the wrist | you are looking **at** the arm |
| **tool** | where the gripper points — **turns with the wrist** | you are looking **through** the wrist camera ⭐ |
| **camera** | the **modelled D405** optical frame | ⛔ only once the real wrist cameras are mounted |

⛔ **Use `tool`, not `camera`, for the hand-mounted C920.** `camera` uses the D405's mounting transform from the MJCF — a 25° cant off the flange — which is simply not where a webcam cable-tied on by hand is sitting. Nobody has measured that mount, and inventing the transform is the exact failure this repo keeps cataloguing. Mount the webcam roughly looking the way the gripper points and use `tool`.

⚠️ **First run needs macOS camera permission.** You will see `OpenCV: not authorized to capture video` until it is granted — macOS raises the dialog for the app running the terminal. If no dialog appears, grant it in **System Settings → Privacy & Security → Camera**. This is a system gate; no code change fixes it.

`--measure 10` captures headlessly and reports the real frame interval, so "no latency" is checked rather than asserted. `--flip` and `--rotate` handle a camera mounted mirrored or sideways.

## Read-only — cannot move anything, no `--yes` needed

```bash
uv run scripts/probe_hardware.py        # enumerate HID, open the SpaceMouse, listen 5 s
uv run scripts/probe_can.py             # listen-only CAN watch; the transceiver cannot even ACK
uv run src/spacemouse_live.py --seconds 25 --until-complete   # live 6-axis readout
uv run scripts/map_axes.py              # ⭐ dial in the axis map. SpaceMouse only, no robot at all
uv run scripts/teleop_sim.py --demo     # the FULL teleop loop against MuJoCo. No hardware at all
uv run scripts/teleop_sim.py            # ...driven by the real SpaceMouse, still simulation only
```

`teleop_sim.py` now applies `config/spacemouse_map.json`, so **a mapping can be verified in simulation before the arm is involved**. Until 2026-08-10 it ignored the map entirely — which made the one place an axis convention is free to get wrong the one place it could not be tested.

## Tests — no hardware, no device, no simulation

```bash
uv run scripts/test_axis_map.py && uv run scripts/test_park_target.py
```

34 checks. The two that matter most: the hand-dialled `spacemouse_map.json` is compared against the **old** formula over 500 random inputs, so this refactor cannot have silently thrown away the bench time that produced it; and a 7-joint park pose against a 6-DoF `--no-gripper` robot no longer raises.

### The MuJoCo viewer — needs an env var on this machine

⛔ **Plain `uv run mjpython …` FAILS** with `Library not loaded: @rpath/libpython3.12.dylib`. `mjpython` dlopens the venv interpreter and the uv-managed CPython does not put `libpython` on its rpath. The library *does* exist, so one variable fixes it (verified: `mjpython OK, mujoco 3.11.0`):

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

**`--scan` is the first thing to run when something feels wrong.** If it reports 0 devices, the problem is power or wiring, not software — see `FINDINGS.md` §8.

---

## Enables motors, sends no setpoint — the arm should not move

```bash
uv run scripts/ping_motors.py --yes --arm B      # per-motor position, torque, TEMPERATURE, error
uv run scripts/read_arm_state.py --yes --arm B   # all 7 through the whole-arm chain
```

`ping_motors.py` is also the temperature check. Idle is **31-36 °C**; **41-42 °C while holding a pose is normal thermal equilibrium, not a fault.**

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

**Superseded but kept:** `teleop_arm.py` was the two-phase version. `teleop_session.py` replaces it — the phases became live mode switches, and the snap bug it contained is fixed there, not here.

---

## Two arms

**Separately, today:** just change the flag. Each session assigns its own puck.

```bash
uv run scripts/teleop_session.py --yes --arm G
```

**Simultaneously:** not built yet. The hard half is proven — `move_both_grippers.py` already drives two arms from one 100 Hz loop across two buses — but cartesian bimanual needs one process holding both robots, two `CartesianTeleop` instances and two pucks assigned up front. Budget is fine: 14 motors ≈ 6.2 ms/cycle against a 10 ms deadline.

---

## Recovery

| symptom | do this |
|---|---|
| `--scan` reports **0 devices** | **Check the arms' wall power.** Both arms silent at once = shared cause = mains |
| motor over-temperature | power-cycle the arms, then **re-run `calibrate_gripper.py`** — limits shift |
| jaws slam at startup | the saved limits are stale. Re-run `calibrate_gripper.py` |
| "fail to communicate with motor N", varying N | desync; already mitigated. Re-run — it is not a dead motor |
| arm drooped after a crash | nothing is lost. Restart the session, `g` to go weightless, reposition, `s` to save a park pose |
