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
| **`i`** | ⭐⭐ **MIRROR — the selected arm leads, the other follows it joint for joint.** ⛔ **NOTHING CHECKS FOR THE ARMS COLLIDING**; no arm knows where the other one is ([ROADMAP §8.2](ROADMAP.md) item 25), and this is the first mode where an arm moves with no hand on it. ⭐ **At the prompt, `i` switches copy ↔ mirror**, so a wrong guess about how the arms stand costs no restart. ⭐ **Pressing `w` while mirroring records BOTH arms**, so one hand produces a two-arm demonstration. Julien's idea: hand-guide one arm in GUIDE and the other reproduces the movement. It **asks twice**, like `l`: engaging starts a motion on the follower while your hands and eyes are on the leader. `i` again turns it off, and so does any mode key aimed at the follower. ⚠️ **Hold the leader still until the status row says FOLLOWING** — until then the follower is closing the initial gap at 0.30 rad/s, and a moving target may never let it close. ⛔ Needs two arms and exactly ONE selected. `--mirror mirror` negates the joints that reverse when arms FACE each other; the default `copy` is right for arms side by side, which is how they stand today |
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

Useful flags: `--arms B,G` · `--start-mode hold|guide|teleop` · `--mirror copy|mirror` · **`--max-speed 2`** · **`--teleop-speed 2`** · `--mirror-gap 0.6` · `--no-rotation` · `--no-gripper` · `--linear-scale 0.2` · `--gripper-step 0.02` · `--reach 0.60` · `--floor 0.0` · `--fork-map` / `--share-map`

⛔⭐⭐ **THREE SPEED LIMITS SIT ON TOP OF EACH OTHER AND ONLY THE LOWEST ONE BINDS.** The plan line prints all of them and flags the ones you raised. `--max-speed` is `SafeRobot`'s rate cap. **`--teleop-speed` is the per-cycle clamp that actually binds TELEOP, a park and a playback** — raising `--max-speed` alone leaves teleop exactly as fast as it was, which is why it felt like nothing happened ([FINDINGS §57.3](FINDINGS.md)). `--mirror-gap` is a tolerance rather than a speed. ⛔ **And there is a fourth limit no flag touches:** `SafeRobot` also holds the command within **0.25 rad of the measured pose**, which is what stops a mirror follower from being pulled closer by more speed ([ROADMAP §8.2](ROADMAP.md) item 27).

⛔ **They are all safety limits.** `SafeRobot` clamps every command from every mode to it, **below all control logic**, so the park speed, any playback multiplier and the mirror follow speed are all above it and never bind ([FINDINGS §37.0](FINDINGS.md)). ⚠️ **It is a SOFTWARE limit, not the hardware's**: Julien's own hand-guided recordings reach **2.4 to 3.7 rad/s**, so the motors do those speeds and only this refuses to command them ([FINDINGS §37.2](FINDINGS.md)). ⭐ **Raise it one step at a time** — `--max-speed 1.5`, then 2.0 — and watch the `⚠️ STUCK lead` warning on the status row rather than temperature. What cooks these motors is holding still against a stop, not moving; the hottest reading in a 337-second session was 43 °C against a 55 °C warning. The plan line prints the value and flags it when it is above the default.

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
uv run scripts/probe_camera_pixels.py --index 0       # ⭐ depth or photograph? from the PIXELS
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

## ⭐⭐⭐ Stop typing the flags — save them once

```bash
uv run scripts/teleop_session.py --arms B,G --start-mode hold --max-speed 4 --teleop-speed 4 --mirror-gap 0.6 --max-lag 0.4 --save-defaults
```

⭐ **That is a DRY RUN (no `--yes`), so nothing is energised and the settings are still written.** Afterwards `uv run scripts/teleop_session.py --yes --arms B,G` runs with all of it.

⭐ **Three layers, and a flag always wins:** built-in constant → `config/session_defaults.json` → the flag you type. So a saved default replaces the constant, and a flag still overrides the file for one run.

⛔ **`--yes`, `--arms` and `--sim` are never saved.** Energising the motors must be a conscious act every time.

⚠️ **The plan names any saved value that is LOOSER than the built-in limit**, because a flag is visible in your shell history and a saved default is not. Delete `config/session_defaults.json` to go back to the built-in values, or edit it by hand. ⛔ It is gitignored, so it does not travel to another clone: a `git pull` must never change how fast the arm may move. [FINDINGS §61.1](FINDINGS.md).

### ⭐⭐⭐ THE FOUR SPEED LIMITS, and the smallest one always wins

| flag | units | what it bounds |
|---|---|---|
| `--linear-scale` | **m/s** | how fast a full puck push asks the **TIP** to move |
| `--teleop-speed` | **rad/s** | how far the IK answer may move **one joint** per cycle |
| `--max-speed` | **rad/s** | the same cap again, **below all control logic** |
| `--max-lag` | **rad** | ⭐ how far the **command** may run ahead of where the arm **IS** |

⛔⭐⭐ **`--max-lag` IS NOT A SPEED.** Every cycle the command is pulled back to `measured ± max-lag`. So reaching a far target is a **ratchet**: ask for 1.01 rad while the arm sits at 0.12 with a 0.25 lag, and the command sent is 0.37. The arm moves, the command advances, repeat. ⭐ **A blocked joint therefore never arrives, and that is the point** — it bounds how hard the motor pushes, expressed as a distance rather than a force.

⚠️⭐ **IF RAISING A SPEED CHANGES NOTHING, YOU ARE RAISING THE WRONG ONE.** At 2.0 m/s of tip demand with a 0.4 m lever the joints only need ~5 rad/s, so a `--teleop-speed` of 15 is never reached. ⭐ **Check `linear` first** — the session's plan now prints all four with what each one does. [FINDINGS §65.0](FINDINGS.md).

### ⭐⭐ `n` — the SETTINGS screen, live

Press `n` in any session. It lists the six limits with their current values:

`1-6` pick one · `-` / `+` change it · `0` back to the session's start · `s` SAVE for every later session · `t` / `g` / `h` leave

⛔ **`max_speed` and `max_lag` take effect on the arms immediately**, because they are the two limits that bound how fast 4.3 kg may move. ⭐ The screen shows the built-in value beside anything you have changed, so a pushed safety limit is always visible. [FINDINGS §62.4](FINDINGS.md).

## ⭐⭐ `--sim` — run the WHOLE session with nothing attached

```bash
uv run scripts/teleop_session.py --sim --yes --arms B,G --start-mode hold
```

⭐ **No arms, no CAN adapter, no SpaceMouse.** It builds simulated arms that **lag** the way the real ones were measured to, wraps them in the **real** `SafeRobot`, and runs the same loop. Drive it with the keys: `a` `t` `g` `h` `w` `l` `p` `i` `q`. ⚠️ `--yes` is still required, because the loop really runs; nothing can move because nothing is attached.

⭐ **The pucks report zero deflection**, so TELEOP holds still. That is the honest stand-in for nobody's hand on the mouse. ⭐⭐ **A `--sim` session CAN replay a real recording** (`l` then `7`), which is the best use of it: check a playback before committing it to 4.3 kg.

⛔⭐ **Simulated recordings go to `recordings/sim/` and are stamped `simulated: true` with a `sim:` method prefix.** They must never be confused with real demonstrations, which are destined to become training data.

⛔ **What `--sim` cannot tell you:** anything about feel, gravity compensation, thermal behaviour, or the axis map. It catches sequencing, state-machine, cursor and following-error bugs, which is where this week's defects lived. Full limits: [FINDINGS §59.0](FINDINGS.md), what it found: [FINDINGS §60.2](FINDINGS.md).

## ⭐ The checkers — no hardware, and they answer real questions

```bash
uv run scripts/check_rig.py                          # what is on the USB bus. Never transmits
uv run scripts/check_flags.py                        # do the commands in docs/ actually work?
uv run scripts/check_links.py                        # every relative link and § reference
uv run scripts/check_recordings.py                   # what is in each recording slot
uv run scripts/check_restructure.py                  # the N-arm restructure is still coherent
uv run scripts/check_collision.py --separation 0.9   # ⭐ how close can the two arms get?
uv run scripts/drive_sim_session.py                  # ⭐⭐ the WHOLE loop end to end, simulated
```

⭐ **`check_collision.py` needs ONE tape-measure reading** — the metres between the two arm bases — because nothing in the repo records it and no software can derive it. Add `--yaw-b 180` if they face each other. ⭐⭐ **It may close the whole collision question in one line:** each arm is already held inside a 0.60 m sphere around its own base, so **beyond 1.20 m of base separation a collision is geometrically impossible** while that limit is enforced. ⛔ Except in GUIDE mode, where nothing can stop a hand. [ROADMAP §8.2](ROADMAP.md) item 25, [FINDINGS §59.3](FINDINGS.md).

⭐ **`check_flags.py` reads every `uv run` line in `docs/` and validates it against the real parser** — a flag that does not exist, a value outside `choices`, a value that will not parse as its type. `COMMANDS.md` had gone stale four times in two days before it existed, and one stale line recommended a command that drives the jaws into both stops ([FINDINGS §59.1](FINDINGS.md)).

### ⭐ Proving the checkers are not just green

```bash
uv run scripts/falsify_fake_arm.py       # break the simulated arm 5 ways; each must be caught
uv run scripts/falsify_check_flags.py    # 7 broken commands must be reported, 3 good ones not
```

⭐⭐ **`drive_sim_session.py` is the only thing that runs the 3000-line loop end to end**, and it earned its place: it caught a crash that **616 unit tests missed**. The save handler read a local assigned only inside the overwrite-guard branch, so the first save of any session raised `UnboundLocalError`. The 12 tests of that exact decision all passed, because they call the pure function directly and the defect was in the CALL SITE. ⚠️ **Extracting a decision into a testable function does not test the code that calls it** ([FINDINGS §62.1](FINDINGS.md)).

⛔⭐⭐ **A checker that has never caught anything is indistinguishable from one that cannot.** Both of these were green on their first real run, and both were then found to have a blind spot by exactly these scripts. `falsify_check_flags.py` caught a rule that removed a false positive and silently created a false negative in the same edit — visible only because the catch count dropped from 7 to 6 ([FINDINGS §59.1](FINDINGS.md)).

## Tests — no hardware, no device, no simulation

```bash
uv run scripts/test_axis_map.py && uv run scripts/test_park_target.py
```

⚠️ **Those two are 34 checks of the 549 that exist.** Everything matching `scripts/test_*.py` is headless and passes as of 2026-08-15; run them all with a loop rather than naming two. ⭐ **`scripts/test_fake_arm.py` is the newest and the one to read first** — it drives a simulated arm that *lags* at four speeds and asserts each lands inside the following-error band measured on the real hardware ([FINDINGS §59.0](FINDINGS.md)).

Of the original two, the ones that matter most: the hand-dialled `spacemouse_map.json` is compared against the **old** formula over 500 random inputs, so this refactor cannot have silently thrown away the bench time that produced it; and a 7-joint park pose against a 6-DoF `--no-gripper` robot no longer raises.

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
