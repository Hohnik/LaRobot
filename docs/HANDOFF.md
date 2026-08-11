# Handoff — start here if you have no context

> **Written 2026-08-10 ~14:00 CEST, end of session 2.** This file exists so a fresh agent, or Julien in a
> month, can rebuild the *whole* picture without reading the chat that produced it.
>
> **Read in this order:** this file → [FINDINGS.md](FINDINGS.md) → [COMMANDS.md](COMMANDS.md) →
> [ROADMAP.md](ROADMAP.md). The README is the live state; `git log` carries the reasoning for every change,
> and the commit messages are deliberately long because they hold the *why*.

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

**✅ Working and proven on hardware:**

- SpaceMouse decoded and verified on all six axes.
- Two YAM arms identified over CAN **without energising anything**, both driven.
- **macOS drives the CAN bus** — the SDK is Linux/SocketCAN-only and that was worked around, not abandoned.
- **100 Hz control loop measured with ~3× headroom** (real 7-motor cycles, 25 s, 0 missed replies).
- **Gravity compensation**: the arm holds its own 4.3 kg, worst drift **0.61°** over 12 s.
- **Hand-guiding** (zero gravity) and **cartesian SpaceMouse teleop** on the real arm.
- Bimanual gripper motion from one loop across two independent CAN buses.
- An interactive session with live mode switching, a rate limiter, and temperature monitoring.

- **Full axis remapping** — any puck axis can drive any motion, taught by gesture
  (`scripts/map_axes.py`, or `m` mid-session). 34 headless tests.

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

## 3. The two commands

```bash
cd ~/Developer/Projects/yam-robotics && uv run scripts/map_axes.py
```

⛔ Cannot move anything — no CAN bus, no robot, no motor. Decide which puck direction drives which
motion, with the arms unplugged if you like. Then:

```bash
cd ~/Developer/Projects/yam-robotics && uv run scripts/teleop_session.py --yes --arm B
```

```
MODES     g GUIDE (weightless)   t TELEOP   h HOLD   p PARK   s save park pose
DIRECTION x y z  flip X/Y/UP               1 2 3  flip ROLL/PITCH/YAW
REMAP     m  MAP mode — rebind axes; the arm holds still and the puck moves nothing
SPEED     - / +  linear             , / .  rotation          [ / ]  gripper step
GRIPPER   o open   c close          r  wrist rotation on/off
OTHER     ?  help                   q  QUIT (asks before releasing the arm)
```

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

## 5.5 ⭐ THE TASK LIST FOR THE NEXT SESSION — start here

Julien's own words, 2026-08-10 ~14:15, at the end of a working session. **Everything below is small; the
system works.** He said: *"as soon as I will have the axes as I want them, it might already work quite well
to be able to control one of the arms."*

| # | task | why / detail |
|---|---|---|
| 1 | ⭐ **Set up the mouse controls, in CONTROLS mode on the real arm** | `teleop_session.py --yes --arm B`, then `m`. The arm **moves**, one isolated axis at a time at half speed; push a direction, watch, press `f` to reverse it or `1`-`6` to reassign it. ⛔ **Do NOT use `--no-gripper` for this** — see §2 and FINDINGS §11.1. `scripts/map_axes.py` still exists for no-hardware sign tweaks but it cannot show you what a direction *is*, which is the actual difficulty |
| 2 | **Verify PARK on hardware** | ⚠️ **FOUR defects now, and the fourth was the one that actually mattered.** Julien reported it broken twice. The first three (cancelled by any key; a 7-vs-6 crash that **disabled the motors mid-air**; a bypassed gripper clamp) were all real but none explained why it did not move. The fourth: **PARK commanded `measured + one step`, so the position error — and therefore the torque — was capped at 0.23° forever. A treadmill.** See [FINDINGS §15](FINDINGS.md); it now commands a *trajectory* like TELEOP always did, and stalls out loud instead of printing a number that is not changing. `s`, move away, `p` |
| 2b | ⛔ **G has never had its jaws calibrated** | `config/gripper_limits.json` holds `B` only, so `--arm G` refuses to start with the gripper. That refusal is correct — but it printed a command **without `--arm`**, which would have re-calibrated B. Fixed. Run `uv run scripts/calibrate_gripper.py --yes --arm G` once, ~10 s, jaws only |
| 3 | **Verify the gripper stays cool** | The 2π frame fix (FINDINGS §3.5) is verified numerically but **not yet on hardware.** The status line now prints **`jaw NN°C` separately from `hottest`** — watch *that* for **60 s** in TELEOP. A plateau near idle (31-36 °C) is the pass; a steady climb means quit and use `--no-gripper`. ⛔ Watching `hottest` is **not** this test: motors 2/3 carry the 4.3 kg and sit at 41-42 °C all session, so a gripper climbing 33 → 41 °C is invisible inside a `max()` |
| 4 | ~~**Axis remapping**~~ | ✅ **Done and TUNED ON THE ARM, 2026-08-10.** Julien's live map is a real permutation — `X←y+ Y←x+ UP←z− ROLL←pitch+ PITCH←roll+ YAW←yaw−` — so the feature earned its place; sign flips alone could not have expressed it. Set up in **CONTROLS mode** (`m`): the arm moves one isolated axis at a time, `f` reverses the control you just used, `1`-`6` **swap** it with another motion. Per-arm maps exist (`--fork-map`), shared by default |
| 5 | ⭐⭐ **Two arms, two SpaceMice** — **what Julien asked for next** | **Fully designed in [ROADMAP step 6](ROADMAP.md); not built.** Neither hardware nor compute is the blocker: two arms on two buses from one loop is proven, and **two IK solves cost 0.100 ms/cycle** (measured) against a 10 ms deadline with ~6.2 ms of CAN. The blocker is that `teleop_session.py` holds one arm's state in one function's locals. Plan: extract `ArmSession`, run N of them, and ⭐ **make `--arms B` exercise the N-arm code with N=1 first**, so the refactor is verified separately from the two-arm hardware risk. Two prerequisites are **already done**: per-arm axis maps (`AxisMapStore` + `--fork-map`) and puck assignment with `exclude=` |
| 6 | **Cameras — real-time, no perceptible latency** | Julien's requirement, 2026-08-10: the C920 plus the wrist D405s, and *"they need to be real time, no latency type of setup."* ⚠️ Nothing here has been designed yet, and it is the one item that could disturb the 100 Hz loop — the ~6.2 ms/cycle CAN measurement was taken with **nothing else competing for CPU**. Expect cameras to want their own process and a shared-memory or timestamped-queue handoff rather than inline capture |
| 7 | **Live telemetry on screen** | Julien clarified 2026-08-11, and this is the *small* half of "output the data": while driving he wants **camera fps, motor temperatures, and poses in units a human can act on** — plus, once the camera view is in use, **the gripper's angles**. ⚠️ The requirement is "understandable", not "complete". Raw radians and raw quaternions fail it; degrees, centimetres and named axes pass it |
| 8 | **Debug logs, with more than one view** | Also his, same day: *"there should be different ways to view logs, or there should be multiple logs… we don't always need access to all of the data when we're debugging specific parts."* So **not** one firehose file. Likely shape: one structured record per cycle written once, plus **filtered views** over it — thermal only, IK only, input only — so a session can be replayed narrowly. Design not started |
| 9 | **Recorder → MCAP in ABC's exact schema** | Setup-Plan §6.1. Get it right and the whole training half works unmodified; get it wrong and every demo must be re-collected. ⏸️ **Deferred by Julien 2026-08-11** — a friend is still writing the plan, so building now would guess at a schema that is about to be specified |

⚠️ **Items 2 and 3 are code changes that have never been run against the arm.** They compile, they have
tests, and item 3 is verified numerically against two independent failures — but *"verified in principle"*
is not *"verified"*. **Treat the first run as a test, not a demonstration.**

⛔ **And item 1 is the counter-example to that caution, so read it before trusting a "free" step.** It was
written here as *"now free — decide the map with zero hardware risk, arrive at the arm with the mapping
already right."* **That was wrong**, and Julien said so immediately: you cannot decide a direction is wrong
until you have watched the arm go that way, and no amount of off-line tooling supplies that. The whole
premise — that the map is a property of the input device — was mine and it was false. **The map is a
property of the device *and* how the arm is turned on his desk**, and only one of those is in a file.

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
| 5 | 2026-08-11, morning | Hardware re-checked after an overnight power cycle (all clean, no recalibration needed). **PARK confirmed working by Julien.** Fixed: the gripper buttons, which shipped broken (`b` only worked in one mode while its own hint printed in another); `q` now offers **`p` park** and the park pose defaults to the session's starting pose, making `q p d` hands-free. ⭐ **Diagnosed and fixed the "incoherent motion":** pure rotation was translating the tool point **44 cm**. The obvious singularity hypothesis was **refuted by measurement**; the cause was an unconditionally-integrated orientation goal plus an `orientation_cost` that was 10× too high — which was making rotation *worse* as well. 92 headless tests. |
| 4 | 2026-08-10, ~15:20-16:xx CEST | ⛔ **First hardware run of session-3 code, and it went badly.** `mjpython --view` could not start; **the arm fell** in GUIDE because `--no-gripper` swaps the gravity model; and the new MAP mode **destroyed the hand-dialled axis map** (recovered from git). All three diagnosed to root cause and fixed, all three documented in [FINDINGS §11](FINDINGS.md). MAP mode replaced by **CONTROLS mode**, designed by Julien: the arm moves, one isolated axis at a time, and only keys edit the map. 43 headless tests. |

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
