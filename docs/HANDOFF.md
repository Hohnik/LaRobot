# Handoff — start here if you have no context

> **Written 2026-08-10 ~14:00 CEST, end of session 2.** This file exists so a fresh agent, or Julien in a
> month, can rebuild the *whole* picture without reading the chat that produced it.
>
> **Read in this order:** this file → [FINDINGS.md](FINDINGS.md) → [COMMANDS.md](COMMANDS.md) →
> [ROADMAP.md](ROADMAP.md). The README is the live state; `git log` carries the reasoning for every change,
> and the commit messages are deliberately long because they hold the *why*.
>
> ## ⭐ If you are a fresh agent, the four things that matter most
>
> 1. **§2 separates what is CONFIRMED ON HARDWARE from what is only verified in simulation.** Respect that
>    line. Three changes here passed their tests and then failed on first contact with the arm — one of
>    them dropped 4.3 kg. "It compiles and has tests" is not "it works".
> 2. **§4 is the working contract**, and rule 1 is absolute: *the agent never runs anything that can move
>    the arm.* Scripts that enable motors but send no setpoint are yours; anything that sends a setpoint is
>    Julien's. Rule 8 is how to write to him, and it has cost real time twice.
> 3. **[FINDINGS §0](FINDINGS.md) is the single most useful page in the repo.** This stack fails by lying,
>    not by crashing: every defect catalogued there produced a confident, plausible, wrong answer and not
>    one raised an exception. Check values for plausibility, never merely for the absence of an exception.
> 4. **§5.5 is the task list**, ordered, with the reasoning for the order. Item 1 is the next thing to build.
>
> Run `uv run scripts/test_*.py` first — **143 headless tests, no hardware needed** — to confirm the tree
> is sound before changing anything.

---

## 1. What this is, in one paragraph

Julien and a friend are building a **bimanual YAM robot setup** for a university robotics programme, following
a 382-line German plan (`docs/Setup-Plan.md`) built on the ABC / ASPIRE / ENPIRE papers. The professor's
summer milestones are: a stable local ABC stack, **SpaceMouse teleoperation for 1-2 tasks**, SFT of those
tasks, and a working simulation. **This repo is the very first crawl step: get a SpaceMouse driving a YAM arm.**
It is a separate repo from Julien's `Mind Understanding` learning workspace on purpose — that is an open-ended
learning programme, this is an engineering build with hardware and deadlines. It is indexed in that repo's
`canon/SOURCES.md` §4 so nothing is lost.

## 2. Where things actually stand

**✅ Working and PROVEN ON HARDWARE by Julien:**

- SpaceMouse decoded and verified on all six axes; **both** pucks usable.
- Both YAM arms identified over CAN **without energising anything**, both driven.
- **macOS drives the CAN bus** — the SDK is Linux/SocketCAN-only; worked around, not abandoned.
- **100 Hz control loop with ~3× headroom** (real 7-motor cycles, 25 s, 0 missed replies).
- **Gravity compensation** — the arm holds its own 4.3 kg, worst drift **0.61°** over 12 s.
- **Hand-guiding**, and **cartesian SpaceMouse teleop** on the real arm.
- **Full axis remapping**, tuned on the arm — any puck axis drives any motion, in either
  direction, per arm and **per control frame**. Julien's live map is a genuine permutation.
- **PARK**, including `q` → `p` → `d` as a hands-free shutdown.
- **Gripper open/close from the puck buttons**, learned by pressing them.
- **Two arms driven simultaneously** — as two separate terminal sessions, one each.

**✅ Built and verified in simulation, NOT yet felt on hardware:**

- **Control frames** (`v`): world / tool / camera. Tool frame follows the wrist, which is what
  makes driving from a camera view work. Each frame keeps its **own** map.
- **Pure rotation no longer drags the tool point** — it used to wander up to 44 cm ([§18](FINDINGS.md)).
- **Speed throttle near the workspace edge** — the fix for "it lags at high speed" ([§20](FINDINGS.md)).
- **Camera view at 30 fps** (`scripts/camera_view.py`), in a **window or in the terminal**.
  ⚠️ The agent physically cannot test anything camera-related — see [FINDINGS §21.1](FINDINGS.md).
- **Mirror-mode engagement logic** (`src/mirror.py`) — copy or mirror, staged engagement,
  14 tests. ⚠️ **The script that opens both arms and runs it does not exist yet.**

**⛔⭐ READ THIS FIRST — the arm fell on 2026-08-10, and the cause was advice in these docs.**

**`--no-gripper` swaps the gravity-compensation model, not just motor 7.** The bare arm XML carries a
**1 microgram** end effector; the real 0.695 kg gripper is merged in from the gripper XML, which
`NO_GRIPPER` replaces with a stub. In GUIDE mode `kp = 0`, so computed gravity torque is the *only* thing
holding 4.3 kg up — and the elbow was **39% short**. The arm folded forward while the status line read a
calm `hottest 35°C` for 33 seconds. **`--no-gripper` is not a safe subset of normal operation; it is a
different, less accurate robot.** Now fixed (`ee_mass=0.695`, residual 0.19 Nm) and GUIDE prints live
drift — but prefer running **with** the gripper. Full account: [FINDINGS §11](FINDINGS.md).

**⛔ Known broken / deliberately disabled:**

- **The gripper is controlled again**, after the 2π frame fix ([FINDINGS §3.5](FINDINGS.md)) — but that fix
  is **verified numerically, not yet on hardware.** Motor 7 was cooked three times before it. `--no-gripper`
  is the escape hatch, and `build_robot()` refuses to start if the frame is wrong (two independent gates,
  both before any control loop).
- **SpaceMouse axis directions** are still being dialled in — but the *tooling* for it is now complete:
  `scripts/map_axes.py` needs **no hardware at all**, and `x`/`y`/`z`/`1`/`2`/`3` still flip live.
  Current state `X←x+ Y←y− UP←z− ROLL←roll+ PITCH←pitch+ YAW←yaw+`.
- **PARK is fixed but untested on hardware** — three separate defects now: it was cancelled by any
  unrecognised key (Enter included); a length mismatch against a `--no-gripper` robot **raised and dropped
  the arm**; and it bypassed the gripper clamp. See §5.5 item 2.
- **No git remote.** Everything exists only on Julien's Mac. See §6.

## 3. The commands that matter

```bash
cd ~/Developer/Projects/yam-robotics && uv run scripts/teleop_session.py --yes --arm B
```

```
MODES     g GUIDE (weightless)   t TELEOP   h HOLD   p PARK   s save park pose
CONTROLS  m  set up the mouse — the arm MOVES, one isolated axis, half speed
FRAME     v  world / tool / camera — what "forward" means (tool follows the wrist)
DIRECTION x y z  flip translation        1 2 3  flip rotation
SPEED     - / +  linear             , / .  rotation          [ / ]  gripper step
GRIPPER   o open   c close          b  assign the PUCK BUTTONS (hold to move jaws)
OTHER     r  wrist rotation on/off   ?  help    q  QUIT → then p park, g guide, d disable
```

Arms are **B** and **G**, matching the labels on the hardware. `--arm arm1` is gone and fails loudly.

```bash
uv run scripts/camera_view.py --list          # which index is the arm-mounted camera
uv run scripts/camera_view.py --index 0 --big # live view; keys 1-5 change resolution
```

⛔ **First camera run needs macOS permission** — System Settings → Privacy & Security → Camera.

No shift keys, and unrecognised keys do nothing. Full inventory in [COMMANDS.md](COMMANDS.md).

## 4. ⭐ How to work on this, and why

These are not preferences, they were arrived at by things going wrong.

1. **The agent never runs anything that can move the arm.** Hand the command over. The working line:
   *scripts that enable motors but send no setpoint → agent; anything that sends a setpoint → Julien.*
2. **Dry run by default.** Every script that transmits needs `--yes` and prints its full plan without it.
3. **Announce before running, do not pause.** Say what is about to run, then run it.
4. ⛔ **Never warn-and-continue on a hazard you have correctly identified.** This was done once — the code
   detected stale gripper limits, printed a warning, and carried on. That is what burned the motor. Refuse.
5. **Prefer a test that could falsify the claim.** Two arms were "verified" identical by evidence that could
   not distinguish that from reading one arm twice; the fix was to find a measurement (per-unit `inertia`)
   that would have *differed* if the claim were wrong.
6. ⛔ **This stack fails by lying, not by crashing.** Every defect catalogued in
   [FINDINGS §0](FINDINGS.md) produced a confident, plausible, wrong answer and **not one raised an
   exception.** **Check values for plausibility, not just for the absence of an exception.**
7. ⛔ **Ask of every guard: what path reaches the hazard without passing through you?** All four defects
   found in session 3 ([FINDINGS §9](FINDINGS.md)) were guards, tests or messages that were written once
   and never re-derived against the thing they guard — a clamp PARK went around, a refusal a weaker copy
   undermined, a temperature monitor that aggregated away its own signal.

8. ⭐ **How to WRITE to Julien, because it has cost real time twice.** Two independent
   requirements, and both must hold:
   - **Density** — cut anything that does not change what he thinks or does. No
     ceremonial time-tracking block, no "anything else?" section by default, no
     tables recapping what he can read in the commits. One item per line in lists.
   - **Comprehensibility** — *define every term at first use*, build from what he
     already knows, and never let a name stand in for an idea. He blew up at
     *"mink wraps that as a QP"* — three unexplained things in five words — and at an
     IK explanation that used "inverse" in two different senses in adjacent
     sentences without saying so. **Short and impenetrable is worse than long and
     clear.** The model he pointed at is `canon/topics/ewc/content.md` chapter 1 in
     his Mind Understanding repo: it defines a term before using it, builds ideas as
     a lineage, and gives confusable pairs their own section.
   *(Full version, with his exact words, in the agent memory store under
   `explaining-to-julien.md` — ⚠️ which is per-machine and absent from clones, hence
   this summary here.)*

## 4.5 The rig, as of 2026-08-11

- **Power: wall sockets only. THERE IS NO E-STOP.** Julien confirmed it. The only way to cut power in a
  hurry is the mains plug, so *keep a hand near it* and prefer the software stops — `h` for HOLD, `q` for the
  consent flow. This is also why every new motion path here is slow, bounded and interruptible: there is no
  hardware backstop underneath the software one.
- Both arms and **both SpaceMice run off Julien's laptop**. Two arms, two CANables, two independent buses.
- **Both arms are calibrated** (`config/gripper_limits.json` holds `B` *and* `G` since 2026-08-11).
- ⭐ **Two separate terminal sessions, one per arm, already work simultaneously** — Julien drove both arms at
  once that way on 2026-08-10. That is a genuinely useful data point: it rules out CAN, USB and CPU
  contention as blockers for bimanual, and leaves the single-process refactor as the *only* remaining work.

**Health check after the overnight power cycle (2026-08-11), all agent-safe, nothing energised beyond a
register read:**

| check | B | G |
|---|---|---|
| motors on the bus | 7/7, gear ratios 40/40/40/10/10/10/10 | 7/7, same |
| errors | all `0x1 (normal)` | all `0x1 (normal)` |
| temperatures | 27-30 °C | 27-30 °C |
| joints 1-6 | ≈ 0 — the parked pose, mechanically supported | ≈ 0 |
| saved jaw limits still valid? | ✅ yes, after the automatic −2π shift | ✅ yes, no shift needed |
| normalised jaw position | **0.034** — nearly closed, only just inside the band | 0.516 — mid-stroke |

⚠️ **B's jaws sit at 0.034**, so there is very little closing travel before the clamp stops it. Harmless,
but do not read "the gripper won't close further" as a fault.

## 5. The three traps that will bite you first

1. **Never select hardware by index.** Adapter enumeration order changed *twice* in one session. Everything
   resolves by **serial** and re-verifies after opening. The two SpaceMice have **empty serials**, so they are
   assigned by asking the operator to move the one they want.
2. **Coordinate frames.** `get_yam_robot()` applies a ±2π wrap correction at every construction based on
   where a motor happens to be; `DMChainCanInterface` used directly does not. **Cached raw motor positions
   are therefore frame-dependent.** This cost a motor. See `reconcile_gripper_limits()`.
3. **Teardown order is not optional.** Stop the control thread → disable the motors → close the bus. Both
   vendor `close()` methods get this wrong, and one of them announces success it did not achieve.

## 5.5 ⭐ THE TASK LIST — start here

**Everything single-arm now works and has been confirmed by Julien on hardware.** What remains is new
work, not repair. In the order I would do it, with the reasoning:

| # | task | why, and what is already known |
|---|---|---|
| 1 | ⭐⭐ **Mirror mode — the SCRIPT. The logic is done.** | Julien's idea, and **the right first two-arm feature**: ✅ **`src/mirror.py` + 14 tests are DONE** — `MirrorLink` handles copy/mirror, staged engagement and the stop-rather-than-chase guard, with no robot handle so it is fully testable without an arm. ❌ **What is missing is the script** that opens both arms, reads B and commands G. That is the same two-arm process `ArmSession` needs, so **build them together**. ✅ Julien answered the design question: **both modes exist, `copy` is the default, and the arms are side by side** — so copy is correct today. ⚠️ **`MIRROR_SIGNS` is a geometric PREDICTION, not a measurement** — reflecting through a vertical plane should negate base_yaw, wrist_roll and gripper_twist and leave the three pitches alone. Expect to adjust it the first time `mirror` is used. |
| 2 | ⭐ **`ArmSession` + one script for both arms** | Fully designed in [ROADMAP step 6](ROADMAP.md). Neither hardware nor compute is the blocker — two arms on two buses from one loop is proven, two IK solves cost 0.100 ms/cycle against a 10 ms deadline, and Julien has already driven both arms at once as two processes. The blocker is that `teleop_session.py` (~1150 lines) holds one arm's state in one function's locals. ⭐ **Make `--arms B` run the N-arm code with N=1 first**, so the refactor is verified against a feel he already knows, separately from the two-arm risk. Prerequisites **done**: per-arm *and* per-frame maps, and `pick_device_by_wiggle(exclude=…)` |
| 3 | **Live telemetry on screen** | His clarification: camera fps, motor temperatures, poses **in units a human can act on**, gripper angles. ⚠️ The requirement is *understandable*, not *complete* — raw radians and quaternions fail it; degrees, centimetres and named axes pass |
| 4 | **Debug logs with more than one view** | *"we don't always need access to all of the data when we're debugging specific parts."* Not one firehose: one structured record per cycle, plus filtered views (thermal only, IK only, input only). Design not started |
| 5 | **Recorder → MCAP in ABC's schema** | ⏸️ **Deferred by Julien** while a friend finishes the plan. Building now would guess at a schema about to be specified. Get it wrong and every demo must be re-collected |
| 6 | ⭐⭐ **The D405 wrist cameras — THEY HAVE ARRIVED. This is the next big piece** | Julien mounted one on **arm B** provisionally, the other is with **arm G**. ⚠️ **Neither is plugged in** — verified 2026-08-11 by an unbounded scan of all 14 USB devices: two CANables, two SpaceMice, the C920, an ethernet adapter and five hubs, and **no Intel/RealSense device at all**. So the first step is physical, not software. He gave the manual's link: `intelrealsense.com/get-started`. See §8 below for what to research and what to expect |
| 6b | **Camera latency — probably NOT worth more software effort** | Julien perceives ~0.2 s. **Measured: the draw cost is ~2 ms**, so render, terminal and grabber are all irrelevant. The rest is the C920 itself — sensor readout, onboard MJPEG encode, USB transport — typically 100-200 ms for a consumer webcam and not removable in software. Resolution is the only lever (key `1` = 320×180). ⛔ **Confirm the 2 ms is still ~2 ms, then stop**; the real answer is the D405 wrist cameras. [FINDINGS §21.3](FINDINGS.md) |
| 7 | **Give this repo a git remote** | ~57 commits exist on one Mac only. Julien has deliberately deferred pushing; not forgotten |

⚠️ **Untested on hardware, all built and verified in simulation or headlessly.** Treat the first run of
each as a test: the speed throttle near the workspace edge, and all of `src/mirror.py`.
✅ Confirmed working by Julien since being built: control frames (`v`), per-frame maps, the pure-rotation
fix, and the camera at 30 fps in both window and terminal.

> ### ⭐ A LIVE THREAD IN THE OTHER REPO — do not lose it
>
> Julien asked for a **proper explanation of inverse kinematics**, taught as a structured path rather than
> summarised, and ruled that it belongs in his **`Mind Understanding`** learning repo rather than here —
> where IK is already indexed as a topic. He explicitly parked it: *"I haven't really read through most of
> your answers yet regarding the inverse kinematic answer and the writing in general. So do the most
> sensible thing, and we'll come back to it later once we fixed these main issues."*
>
> ⚠️ That repo's rules apply there and differ from this one's: `canon/` is **curated and read-only** for
> session agents, so an IK topic must be **proposed** via `agents/<name>/REPORT.md` §Proposals, not
> created unilaterally. See that repo's `CLAUDE.md` and `state/NOW.md`.

## 6. What to do next

**Immediately:**

1. ⭐ **Give this repo a git remote.** ~40 commits exist only on one Mac, including everything above.
   Julien's own private GitHub first; `Hohnik/LaRobot` is planned separately — see README §7.5 for the
   fork-and-PR approach and the "clean" checklist. **Do not push to a collaborator's `main`.**
2. **Work the §5.5 task list** — the axis map first (no hardware needed, so it costs nothing), then
   verify PARK and the gripper **on hardware**, in that order. Run the tests first; they are 2 seconds:
   `uv run scripts/test_axis_map.py && uv run scripts/test_park_target.py`

**Then, in roadmap order:** simultaneous bimanual teleop (the hard half is proven) → **recorder → MCAP in
ABC's exact schema** (get this right and the whole training half works unmodified; get it wrong and every
demo must be re-collected) → cameras.

## 7. Session log

| session | date | outcome |
|---|---|---|
| 1 | 2026-08-07 | Hardware enumerated. SpaceMouse readable. **Wrongly concluded the CAN protocol was unknown and macOS unusable.** |
| 2 | 2026-08-10, 09:30-14:00 CEST (40 min lunch) | Both prior conclusions refuted. Arm identified, driven, gravity-compensated, hand-guided and **teleoperated with a SpaceMouse**. Gripper disabled after cooking motor 7 three times. ~30 commits. |
| 3 | 2026-08-10, ~14:25-15:xx CEST | **No hardware touched.** Full axis remapping built (`src/axis_map.py`, `scripts/map_axes.py`) with 25 tests. Four defects found **by reading** and fixed — see FINDINGS §9. The world-frame axis semantics measured in simulation instead of assumed. 34 headless tests now exist where there were none. |
| 4 | 2026-08-10, ~15:20-16:xx CEST | ⛔ **First hardware run of session-3 code, and it went badly.** `mjpython --view` could not start; **the arm fell** in GUIDE because `--no-gripper` swaps the gravity model; and the new MAP mode **destroyed the hand-dialled axis map** (recovered from git). All three diagnosed to root cause and fixed, all three documented in [FINDINGS §11](FINDINGS.md). MAP mode replaced by **CONTROLS mode**, designed by Julien: the arm moves, one isolated axis at a time, and only keys edit the map. 43 headless tests. |
| 5 | 2026-08-11, morning | Hardware re-checked after an overnight power cycle (all clean, no recalibration needed). **PARK confirmed working by Julien.** Fixed: the gripper buttons, which shipped broken (`b` only worked in one mode while its own hint printed in another); `q` now offers **`p` park** and the park pose defaults to the session's starting pose, making `q p d` hands-free. ⭐ **Diagnosed and fixed the "incoherent motion":** pure rotation was translating the tool point **44 cm**. The obvious singularity hypothesis was **refuted by measurement**; the cause was an unconditionally-integrated orientation goal plus an `orientation_cost` that was 10× too high — which was making rotation *worse* as well. 92 headless tests. |
| 8 | 2026-08-11, evening | **Kitty images fixed** — they showed nothing because `f=100` means PNG and the renderer sent JPEG, with `q=2` suppressing the error that would have said so. `--term-test` added to make a silent display path speak. ⭐ **The D405 arrived and was measured**: serial `255323071773` (a real one, unlike the SpaceMice), USB SuperSpeed, and **it also enumerates as a plain UVC camera** — so OpenCV may open it with no SDK at all. `pyrealsense2` has no macOS wheels at any version (verified), but `librealsense` is a prebuilt Homebrew bottle. 143 headless tests. |
| 7 | 2026-08-11, afternoon | **Camera terminal view** — aspect-ratio stretch fixed (it ignored the source aspect entirely), real C920 modes offered down to 320×180 (a UVC camera silently substitutes the nearest mode, which is why 424×240 became 640×360), and **iTerm2/kitty inline images implemented** so the block renderer is a fallback rather than the only option. `b` had been a two-way toggle whose sides could be identical, so it looked broken. **Latency measured at ~2 ms of draw cost — the rest is the camera hardware.** ⭐ **Mirror-mode logic built** (`src/mirror.py`), whose own tests caught a hidden 5 rad/s jump at the guard handover and a length mismatch. 123 → 138 headless tests. |
| 6 | 2026-08-11, midday | Arms **renamed B and G** to match their physical labels, config migrated with every value verified byte-identical. **Per-frame control maps** — each frame owns its wiring and `m` describes the frame you are actually in, with tool-frame labels measured from the model. New frames are **seeded from the world map** so nothing is re-tuned from scratch. **Camera fixed: the 5 fps was my own frame-draining loop**, not USB bandwidth — `grab()` blocks on macOS, so "draining" waited for five frames. **Speed lag diagnosed as a singularity problem**, not a speed problem, and throttled at the source. 109 headless tests. |

**Time accounting:** session 2 ran 09:30 → ~14:00 with a 12:35-13:15 break — **~3 h 45 m of working time.**
⚠️ Earlier estimates in this session were badly wrong (~2.4× over) because per-turn effort was being summed
instead of wall-clock read. Read the clock.

⭐ **Session 3's lesson: the bench is not where the cheap defects are.** Nothing was plugged in, and it
still turned up a path that would have released a raised arm (PARK with `--no-gripper`), a thermal test
that could not have detected the thing it was testing, a warn-and-continue in the exact wording of the
one rule this project wrote in blood, and a simulator that could not reproduce the mapping it exists to
de-risk. All four were reachable by reading the code against its own documentation.

⛔ **Session 4's lesson is the counterweight, and it is sharper: reading does not find what only hardware
knows, and "it compiles and has tests" is not "it works".** Everything session 3 built passed 34 tests, three
dry runs and a simulated IK loop — and the first contact with the arm produced three failures in one attempt,
one of which dropped 4.3 kg. Two specific process faults worth carrying:

1. **`ls` is not verification.** `mjpython --view` was recommended on the strength of the binary existing.
   It could not start. *Verify the consequence, not the mechanism* — quoted in the same turn it was violated.
2. **A flag named for one thing changed another.** `--no-gripper` was chosen *because* it sounded like the
   smaller, safer experiment. It silently replaced the dynamics model. **Before recommending a flag as
   "safer", read what it actually switches** — the name is not the contract.


---

## 8. ⭐ The Intel RealSense D405 — MEASURED, 2026-08-11. Read this before touching it.

**One camera is connected and healthy.** Julien mounted one provisionally on **arm B**; the second is with
**arm G** and is **not plugged in** (only one serial appears on the bus).

### What was measured, not assumed

| | value | why it matters |
|---|---|---|
| product | `Intel(R) RealSense(TM) Depth Camera 405` | it is the D405, confirmed |
| **serial** | **`255323071773`** | ⭐ **a REAL serial.** Unlike the two SpaceMice, which report empty serials and forced the wiggle-to-assign hack, two D405s can be told apart properly. **Select by serial, never by index** (FINDINGS §0 #5) |
| USB IDs | VID `0x8086` (32902), PID `0x0B5B` (2907) | for `ioreg`/`lsusb`-style checks |
| `bcdDevice` | `20721` = `0x50F1` | **probably firmware 5.15.1**, unverified — confirm with `rs-enumerate-devices` |
| **link speed** | **SuperSpeed (5 Gbps), `Device Speed = 3`** | ⭐ it negotiated **USB 3**, not USB 2. Bandwidth is not a problem for one camera. Re-check when the second is added |
| **UVC** | `Intel(R) RealSense(TM) Depth Camera 405  Depth` → `UVC Camera VendorID_32902 ProductID_2907` | ⭐⭐ **see below — this is the shortcut** |

### ⭐⭐ The shortcut: it is also a plain UVC camera

macOS lists the D405 as a standard **UVC camera**, which means **OpenCV can open it with no SDK at all**.
`scripts/camera_view.py --list` should now show it as an extra index.

**That is very likely enough for teleop today.** Driving the arm from the camera's point of view needs a
*picture*, not a point cloud — and the whole control-frame machinery (`v` → tool frame) is already built and
waiting. ⭐ **Try this before spending an hour on the SDK.**

⚠️ Caveats: the entry macOS shows is the **Depth** stream, which over UVC is 16-bit and will look wrong
rendered as ordinary colour. Whether the RGB stream appears as a separate index is **not yet known** — check
`--list`. And a UVC-only path gives no depth alignment, no intrinsics and no camera controls.

### The SDK situation — measured, and the prediction held

**`pip install pyrealsense2` is impossible here.** Wheels exist only for `manylinux1_x86_64`,
`manylinux2014_aarch64` and `win_amd64` — **no macOS build at any version**, including the older 2.55 that
once had one. Verified with `uv pip install --dry-run` on both current and pinned versions.

⭐ **But `librealsense` IS available from Homebrew as a prebuilt bottle** — `stable 2.58.3 (bottled)`,
dependencies `glfw` and `libusb`, not currently installed. A *bottle* means no source compilation for the
C++ library and its tools:

```bash
brew install librealsense      # then:
rs-enumerate-devices           # confirms the camera, reports firmware
realsense-viewer               # GUI: streams, depth, and the camera's own controls
```

⚠️ **The Homebrew formula does not necessarily build the PYTHON bindings** — those usually need a source
build with `-DBUILD_PYTHON_BINDINGS=ON`. So the likely ladder, cheapest first:

1. **UVC via OpenCV** — nothing to install. Probably enough for teleop.
2. **`brew install librealsense`** — prebuilt tools, confirms hardware and firmware, gives a viewer.
3. **Source build with Python bindings** — only if depth data is genuinely needed in Python.

⭐ **This is the same shape as the CAN SDK problem** ([FINDINGS §2](FINDINGS.md)): a vendor SDK that assumes
a platform we are not on. That was solved by patching from *outside* while keeping `third_party/` a clean
upstream checkout. **Read §2 before choosing an approach here** — and note it also ends with the reminder
that Linux remains right for the final rig, so effort spent fighting macOS should be proportionate.

### What already exists and must not be rebuilt

The `camera` control frame (correct for the D405's modelled 25° flange cant), both viewers, the frame-rate
and latency instrumentation, and the finding that the C920's ~200 ms latency is sensor-and-encode rather
than software ([FINDINGS §21.3](FINDINGS.md)). ⭐ **A D405 may be much faster** — it is a machine-vision
camera, not a consumer webcam — so **re-measure rather than assume that carries over.**

⚠️ **The agent still cannot open a camera stream.** macOS camera permission is per-application
([FINDINGS §21.1](FINDINGS.md)); enumeration via `ioreg`/`system_profiler` works, opening does not. Plan for
measurements to be commands Julien runs, and put the diagnostics *inside* the program.
