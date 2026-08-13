# Handoff — start here if you have no context

> **Written 2026-08-10, kept current — last updated 2026-08-12, end of session 19.** This file exists so a fresh agent, or Julien in a month, can rebuild the *whole* picture without reading the chat that produced it.
>
> **Read in this order:** this file → [FINDINGS.md](FINDINGS.md) → [COMMANDS.md](COMMANDS.md) → [ROADMAP.md](ROADMAP.md). ⚠️ **§2 below is the live state, not the README** — the README's "what works right now" is session 2's snapshot and says so. `git log` carries the reasoning for every change, and the commit messages are deliberately long because they hold the *why*.
>
> ## ⭐ If you are a fresh agent, the four things that matter most
>
> 1. **§2 separates what is CONFIRMED ON HARDWARE from what is only verified in simulation.** Respect that line. Three changes here passed their tests and then failed on first contact with the arm — one of them dropped 4.3 kg. "It compiles and has tests" is not "it works".
> 2. **§4 is the working contract**, and rule 1 is absolute: *the agent never runs anything that can move the arm.* Scripts that enable motors but send no setpoint are yours; anything that sends a setpoint is Julien's. Rule 8 is how to write to him, and it has cost real time twice.
> 3. **[FINDINGS §0](FINDINGS.md) is the single most useful page in the repo.** This stack fails by lying, not by crashing: every defect catalogued there produced a confident, plausible, wrong answer and not one raised an exception. Check values for plausibility, never merely for the absence of an exception.
> 4. **§5.5 is the task list**, ordered, with the reasoning for the order. Item 1 is the next thing to build.
>
> Run `for f in scripts/test_*.py; do uv run "$f"; done` first — **308 headless tests, no hardware needed** — to confirm the tree is sound before changing anything. Also `uv run scripts/check_links.py` (docs cross-reference each other constantly; one broken pointer is in `Setup-Plan.md` and is not ours).
>
> ## ⭐ Where this actually stands, 2026-08-12 — read this paragraph if you read nothing else
>
> **Single-arm teleop is finished and confirmed on hardware.** GUIDE, TELEOP, HOLD, PARK, CONTROLS, the gripper, the axis map, control frames, saved poses and blended multi-pose runs all work and Julien has driven them. The camera view works and cameras are identified by measurement.
>
> ⭐⭐ **AND THE BIGGEST QUESTION IN THE PROJECT JUST CHANGED ITS ANSWER.** After driving a full session on 2026-08-12 he reported that **executing a task with the SpaceMouse is very hard**, that setting the scene up by hand is trivial by comparison, and that an automated scene reset is close to circular — *"if the robot can just place the object at a random location, then it can already control itself."* That refutes the recommendation this repo had written down about where demos come from.
>
> ⭐ **Everything about that now lives in ONE place: [ROADMAP.md](ROADMAP.md) §6.6, "Where the training data comes from".** ⛔ **Read `ROADMAP.md` §6.6 before building any part of the recorder (step 5), and before touching the waypoint runner.** It is written in plain language because Julien reads it himself, and it carries: what he decided, the plan he liked (**hand-guide a whole movement in GUIDE, then replay it to record**), his refinement that **noise should be set per waypoint** rather than once for the whole path, his idea to **label good and bad stretches with a microphone**, a free trick for detecting whether a grab worked, what **diffusion** as the model choice settles, what **provenance** requires of the recorder, and a table of **what is built and what is not**. ⛔ Almost none of it is built.
>
> ## ⭐⭐ AND THEN THE TARGET STACK WAS READ, AND IT SETTLED THREE THINGS — [ROADMAP.md](ROADMAP.md) §9
>
> Julien sent the project's stack plan and its three papers late on 2026-08-12. **All of it was read from the actual sources**, because two of the three papers are newer than any model's training data. ⛔ **Read [ROADMAP.md](ROADMAP.md) §9 before any restructuring and before the recorder.** The three things it settled:
>
> 1. ⛔⭐ **`amazon-far/abc` stores 14 states and 14 actions per timestep — two arms in ONE timeline.** So **both arms from one script stops being the next nice feature and becomes a prerequisite of the data format.** Two terminal sessions cannot produce that file. See task 0c below.
> 2. ✅ **ABC's actions are joint positions, which is what we already command.** An earlier worry about the literature preferring end-effector movements does not apply here.
> 3. ⭐ **ABC ships a large public dataset and pretrained checkpoints**, and the milestone is fine-tuning. ⚠️ **So the "50 to 200 demos" estimate may be several times too high** for our case. Unverified, and worth checking before planning a collection session.
>
> ⭐ **And the architecture answer to "what do we rebuild towards":** all three papers want the robot as a **callable library** with structured traces, an automatic success signal, long unbroken recordings and a language instruction. We have a 2000-line interactive `main()`. `src/arm_session.py` is already the first step the right way, and *"the class decides, the script narrates"* turns out to be the shape the research needs.
>
> ## ⬜⬜ THE NEXT JOB, AND IT IS THE ONLY BIG ONE LEFT: wire `ArmSession` in
>
> ⛔ **There is no bimanual command yet. `--arms` does not exist.** `teleop_session.py` is single-arm from top to bottom; asking for two arms today would just error. Julien asked for the command on 2026-08-12 and the honest answer was that it does not exist.
>
> **What IS done:** `src/arm_session.py` — one arm's state and mode machine, **17 tests against a fake robot**, written so N of them run in one loop. State, mode transitions, park stepping with the ramp, the queue, per-arm thermal guard. Its rule is *the class decides, the script narrates* — no method prints — which is why it can be proven without hardware.
>
> **What is left, concretely:** `main()` is ~1000 lines holding ONE arm's state as locals — `robot`, `teleop`, `mode`, `gripper_value`, `prev_q`, `home_ee`, `park_target`, `park_path`, `park_s`, `guide_ref`, `park_cmd`. Replace those with a list of `ArmSession`, and iterate.
>
> ⭐ **Do it in this order — it is the whole de-risking plan and it is not optional:**
> 1. `--arms B` runs the N-arm code with **N=1**. Julien confirms it *feels identical*. The restructure is then verified against a feel he already knows, separately from any two-arm risk.
> 2. Add the `a` selector (B → G → BOTH) and per-arm status rows. Still one arm connected.
> 3. `--arms B,G`, starting in **HOLD**, gripper enabled, desk clear.
> 4. Only then GUIDE and CONTROLS on two arms — GUIDE last, because that is the mode where a dynamics-model error becomes a *falling* arm, and `g` on two arms is 8.6 kg at once.
> 5. Mirror mode on top: `src/mirror.py` + its 14 tests already exist and need only the two-arm process.
>
> ⚠️ **The decisions are already made** — [ROADMAP step 6](ROADMAP.md) has the table: mode keys apply to the *selected* arm, driving always applies to *all* arms, start in HOLD and refuse `--start-mode guide` when N>1, and **a fault on one arm stops both** (a chain death on B must not leave G sagging). Prerequisites that are already built: per-arm and per-frame axis maps, and `pick_device_by_wiggle(exclude=…)` so one puck cannot be assigned to both arms.
>
> ⚠️ **Nothing is pushed** (working contract rule 9). A snapshot branch `julien/yam-teleop-wip` exists on `Hohnik/LaRobot` from 2026-08-12 and is **not kept in sync** — it was taken once, for his colleagues.

---

## 1. What this is, in one paragraph

Julien and a friend are building a **bimanual YAM robot setup** for a university robotics programme, following a 382-line German plan (`docs/Setup-Plan.md`) built on the ABC / ASPIRE / ENPIRE papers. The professor's summer milestones are: a stable local ABC stack, **SpaceMouse teleoperation for 1-2 tasks**, SFT of those tasks, and a working simulation. **This repo is the very first crawl step: get a SpaceMouse driving a YAM arm.** It is a separate repo from Julien's `Mind Understanding` learning workspace on purpose — that is an open-ended learning programme, this is an engineering build with hardware and deadlines. It is indexed in that repo's `canon/SOURCES.md` §4 so nothing is lost.

## 2. Where things actually stand

**✅ Working and PROVEN ON HARDWARE by Julien:**

- SpaceMouse decoded and verified on all six axes; **both** pucks usable.
- Both YAM arms identified over CAN **without energising anything**, both driven.
- **macOS drives the CAN bus** — the SDK is Linux/SocketCAN-only; worked around, not abandoned.
- **100 Hz control loop with ~3× headroom** (real 7-motor cycles, 25 s, 0 missed replies).
- **Gravity compensation** — the arm holds its own 4.3 kg, worst drift **0.61°** over 12 s.
- **Hand-guiding**, and **cartesian SpaceMouse teleop** on the real arm.
- **Full axis remapping**, tuned on the arm — any puck axis drives any motion, in either direction, per arm and **per control frame**. Julien's live map is a genuine permutation.
- **PARK**, including `q` → `p` → `d` as a hands-free shutdown.
- **Gripper open/close from the puck buttons**, learned by pressing them.
- **Two arms driven simultaneously** — as two separate terminal sessions, one each.

**✅ Built and verified in simulation, NOT yet felt on hardware:**

- **Control frames** (`v`): world / tool / camera. Tool frame follows the wrist, which is what makes driving from a camera view work. Each frame keeps its **own** map.
- **Pure rotation no longer drags the tool point** — it used to wander up to 44 cm ([§18](FINDINGS.md)).
- **Speed throttle near the workspace edge** — the fix for "it lags at high speed" ([§20](FINDINGS.md)).
- **Camera view at 30 fps** (`scripts/camera_view.py`), in a **window or in the terminal**. ⚠️ The agent physically cannot test anything camera-related — see [FINDINGS §21.1](FINDINGS.md).
- **Mirror-mode engagement logic** (`src/mirror.py`) — copy or mirror, staged engagement, 14 tests. ⚠️ **The script that opens both arms and runs it does not exist yet.**
- ⭐ **Cameras are identified by MEASUREMENT.** `--camera d405` / `--camera c920` selects by name, and the name↔index mapping is established by asking each index for a resolution only one camera supports. ⛔ It is **not** read off any list: macOS's enumeration order is not OpenCV's, and assuming it was got two of four cameras wrong on 2026-08-11 — the full account, and why the checks that were in place did not catch it, is [FINDINGS §22](FINDINGS.md). ⚠️ **Two D405s share every mode and cannot be told apart this way** — that matters as soon as the second one is plugged in.
- **The terminal view can get sharper**, its keys are no longer invisible, and they now offer the selected camera's own modes rather than a hard-coded C920 list. ⛔ **The kitty protocol is PNG-only** (~25x iTerm2's JPEG cost), which is the real ceiling on detail in Ghostty — measured table in [FINDINGS §21.4](FINDINGS.md). Flicker fixed by double-buffering the image and by not redrawing frames the terminal already has ([§21.5](FINDINGS.md)) — ⚠️ **reasoned and tested but not yet confirmed by eye.**

**⛔⭐ READ THIS FIRST — the arm fell on 2026-08-10, and the cause was advice in these docs.**

**`--no-gripper` swaps the gravity-compensation model, not just motor 7.** The bare arm XML carries a **1 microgram** end effector; the real 0.695 kg gripper is merged in from the gripper XML, which `NO_GRIPPER` replaces with a stub. In GUIDE mode `kp = 0`, so computed gravity torque is the *only* thing holding 4.3 kg up — and the elbow was **39% short**. The arm folded forward while the status line read a calm `hottest 35°C` for 33 seconds. **`--no-gripper` is not a safe subset of normal operation; it is a different, less accurate robot.** Now fixed (`ee_mass=0.695`, residual 0.19 Nm) and GUIDE prints live drift — but prefer running **with** the gripper. Full account: [FINDINGS §11](FINDINGS.md).

**⛔ Known broken / deliberately disabled:**

- **The gripper is controlled again**, after the 2π frame fix ([FINDINGS §3.5](FINDINGS.md)) — but that fix is **verified numerically, not yet on hardware.** Motor 7 was cooked three times before it. `--no-gripper` is the escape hatch, and `build_robot()` refuses to start if the frame is wrong (two independent gates, both before any control loop).
- **SpaceMouse axis directions** are still being dialled in — but the *tooling* for it is now complete: `scripts/map_axes.py` needs **no hardware at all**, and `x`/`y`/`z`/`1`/`2`/`3` still flip live. Current state `X←x+ Y←y− UP←z− ROLL←roll+ PITCH←pitch+ YAW←yaw+`.
- **PARK is fixed but untested on hardware** — three separate defects now: it was cancelled by any unrecognised key (Enter included); a length mismatch against a `--no-gripper` robot **raised and dropped the arm**; and it bypassed the gripper clamp. See §5.5 item 2.
- **Still no remote of Julien's own** — but as of 2026-08-12 there *is* an off-machine copy: **`larobot/julien/yam-teleop-wip`**, 56 commits, pushed to his friend's public repo `Hohnik/LaRobot`. ⚠️ **That is not a backup he controls** — it is a branch in someone else's repository, and it can be deleted by someone who is not him. See §6.

## 3. The commands that matter

```bash
cd ~/Developer/Projects/yam-robotics && uv run scripts/teleop_session.py --yes --arm B
```

```
MODES     g GUIDE (weightless)   t TELEOP   h HOLD   p PARK   s save park pose
CONTROLS  m  set up the mouse — the arm MOVES, one isolated axis, half speed
FRAME     v  world / tool / camera — what "forward" means (tool follows the wrist)
DIRECTION x y z  flip translation        1 2 3  flip rotation
SPEED     - / +  linear             , / .  rotation      ö / ä  gripper step ([ / ])
EASE      e  cycle none / in / out / both / s-curve — works in ANY mode
GRIPPER   o open   c close          b  assign the PUCK BUTTONS (hold to move jaws)
OTHER     r  wrist rotation on/off   ?  help    q  QUIT → then p park, g guide, d disable
```

⭐ **`ö` and `ä` are aliases of `[` and `]`, added 2026-08-12** — on a German QWERTZ layout the brackets are **AltGr+8 / AltGr+9**, a three-finger chord for a knob adjusted while the arm is moving. Both spellings work everywhere. ⛔ They arrive at all only because `KeyReader` now decodes UTF-8 across reads: `ö` is two bytes, and the old one-byte reader turned one keypress into two replacement characters and no key ([FINDINGS §27.7](FINDINGS.md)).

Arms are **B** and **G**, matching the labels on the hardware. `--arm arm1` is gone and fails loudly.

```bash
uv run scripts/camera_view.py --list          # which index is the arm-mounted camera
uv run scripts/camera_view.py --index 0 --big # live view; keys 1-5 change resolution
```

⛔ **First camera run needs macOS permission** — System Settings → Privacy & Security → Camera.

No shift keys, and unrecognised keys do nothing. Full inventory in [COMMANDS.md](COMMANDS.md).

## 4. ⭐ How to work on this, and why

These are not preferences, they were arrived at by things going wrong.

1. **The agent never runs anything that can move the arm.** Hand the command over. The working line: *scripts that enable motors but send no setpoint → agent; anything that sends a setpoint → Julien.*
2. **Dry run by default.** Every script that transmits needs `--yes` and prints its full plan without it.
3. **Announce before running, do not pause.** Say what is about to run, then run it.
4. ⛔ **Never warn-and-continue on a hazard you have correctly identified.** This was done once — the code detected stale gripper limits, printed a warning, and carried on. That is what burned the motor. Refuse.
5. **Prefer a test that could falsify the claim.** Two arms were "verified" identical by evidence that could not distinguish that from reading one arm twice; the fix was to find a measurement (per-unit `inertia`) that would have *differed* if the claim were wrong.
6. ⛔ **This stack fails by lying, not by crashing.** Every defect catalogued in [FINDINGS §0](FINDINGS.md) produced a confident, plausible, wrong answer and **not one raised an exception.** **Check values for plausibility, not just for the absence of an exception.**
7. ⛔ **Ask of every guard: what path reaches the hazard without passing through you?** All four defects found in session 3 ([FINDINGS §9](FINDINGS.md)) were guards, tests or messages that were written once and never re-derived against the thing they guard — a clamp PARK went around, a refusal a weaker copy undermined, a temperature monitor that aggregated away its own signal.

8. ⭐⭐ **How to WRITE to Julien. THIS RULE EXISTED, WAS FOLLOWED AS WRITTEN, AND STILL FAILED ON 2026-08-12** — he read a complete reply and understood none of it: *"I cannot understand a single thing of what you're trying to explain… the way you write is incomprehensible and feels cryptic. You could just explain it clearly and write way less text, in normal plain English."*

⛔ **The style is now named and non-negotiable: "ELI18".** Write plain English for a smart 18-year-old who knows the project but not the code. Short sentences, ordinary words, answer first, under ~400 words unless he asked for depth. **This applies to CHAT ONLY** — repo files keep the conventions you see here.

⭐ **The two failures that actually broke comprehension were NOT covered by the two requirements below, and they are the ones to watch:**
   - ⛔ **A reply structured as a correction of something he has never read.** The reply opened with *"the circularity objection I missed entirely"* — a phrase pointing at a document he had not seen, naming an idea it never defined. His answer: *"What are you talking about?"* **Say what a thing IS before saying what is wrong with it. Never assume he has read your prose, including prose from earlier today.**
   - ⛔ **A section reference with no file name.** The reply said `§6.6` in the one block he was told to read, so he had to hunt back through the message for the single earlier mention of `ROADMAP §6.6`. **Name the file every single time, including in summaries and recommendations.**

⚠️ **Banned in chat:** em-dashes · "it's not X, it's Y" · "not just X but Y" · "precisely" · "genuinely" · "load-bearing" · "the honest answer" · "smoking gun" · "surface" as a verb · stacked nouns as adjectives ("clean-scene replay pass") · bold inside a sentence · ⭐ ⛔ ⚠️ · ALL CAPS · dramatic one-line paragraphs. He pointed at the `claudish-to-english` critique for the full list (`github.com/gvzdv/claudish-to-english`, and `slhck.info/software/2026/06/22/claudish.html`). ⚠️ Reddit is blocked to both WebFetch and the in-app browser; those two sources carry the content.

⚠️ **It decays hardest immediately after heavy tool use or long reasoning**, which is exactly when the worst messages get written. Re-read the rule before composing, not before starting.

**The two original requirements still hold underneath all of that:**
   - **Density** — cut anything that does not change what he thinks or does. No ceremonial time-tracking block, no "anything else?" section by default, no tables recapping what he can read in the commits. One item per line in lists.
   - **Comprehensibility** — *define every term at first use*, build from what he already knows, and never let a name stand in for an idea. He blew up at *"mink wraps that as a QP"* — three unexplained things in five words — and at an IK explanation that used "inverse" in two different senses in adjacent sentences without saying so. **Short and impenetrable is worse than long and clear.** The model he pointed at is `canon/topics/ewc/content.md` chapter 1 in his Mind Understanding repo: it defines a term before using it, builds ideas as a lineage, and gives confusable pairs their own section. *(Full version, with his exact words, in the agent memory store under `explaining-to-julien.md` — ⚠️ which is per-machine and absent from clones, hence this summary here.)*

9. ⛔ **NOTHING IS PUSHED UNTIL JULIEN SAYS SO.** His ruling, 2026-08-12, immediately after the one push that has happened: *"From now on, we won't push until I'm happy with what we actually got. That was just to make my colleagues happy."* So `julien/yam-teleop-wip` on `Hohnik/LaRobot` is a **snapshot taken for someone else's benefit, not a branch we keep in sync.** Commit freely and often — local history is how the reasoning is preserved — but treat `git push` as an action that needs his word each time, the same way a setpoint does. ⚠️ Do not read "we already pushed once" as standing permission; that is precisely the inference this rule exists to block.

## 4.5 The rig, as of 2026-08-11

- **Power: wall sockets only. THERE IS NO E-STOP.** Julien confirmed it. The only way to cut power in a hurry is the mains plug, so *keep a hand near it* and prefer the software stops — `h` for HOLD, `q` for the consent flow. This is also why every new motion path here is slow, bounded and interruptible: there is no hardware backstop underneath the software one.
- Both arms and **both SpaceMice run off Julien's laptop**. Two arms, two CANables, two independent buses.
- **Both arms are calibrated** (`config/gripper_limits.json` holds `B` *and* `G` since 2026-08-11).
- ⭐ **Two separate terminal sessions, one per arm, already work simultaneously** — Julien drove both arms at once that way on 2026-08-10. That is a genuinely useful data point: it rules out CAN, USB and CPU contention as blockers for bimanual, and leaves the single-process refactor as the *only* remaining work.

**Health check after the overnight power cycle (2026-08-11), all agent-safe, nothing energised beyond a register read:**

| check | B | G |
|---|---|---|
| motors on the bus | 7/7, gear ratios 40/40/40/10/10/10/10 | 7/7, same |
| errors | all `0x1 (normal)` | all `0x1 (normal)` |
| temperatures | 27-30 °C | 27-30 °C |
| joints 1-6 | ≈ 0 — the parked pose, mechanically supported | ≈ 0 |
| saved jaw limits still valid? | ✅ yes, after the automatic −2π shift | ✅ yes, no shift needed |
| normalised jaw position | **0.034** — nearly closed, only just inside the band | 0.516 — mid-stroke |

⚠️ **B's jaws sit at 0.034**, so there is very little closing travel before the clamp stops it. Harmless, but do not read "the gripper won't close further" as a fault.

## 5. The three traps that will bite you first

1. **Never select hardware by index.** Adapter enumeration order changed *twice* in one session. Everything resolves by **serial** and re-verifies after opening. The two SpaceMice have **empty serials**, so they are assigned by asking the operator to move the one they want.
2. **Coordinate frames.** `get_yam_robot()` applies a ±2π wrap correction at every construction based on where a motor happens to be; `DMChainCanInterface` used directly does not. **Cached raw motor positions are therefore frame-dependent.** This cost a motor. See `reconcile_gripper_limits()`.
3. **Teardown order is not optional.** Stop the control thread → disable the motors → close the bus. Both vendor `close()` methods get this wrong, and one of them announces success it did not achieve.

## 5.5 ⭐ THE TASK LIST — start here

**Everything single-arm now works and has been confirmed by Julien on hardware.** What remains is new work, not repair. In the order I would do it, with the reasoning:

| # | task | why, and what is already known |
|---|---|---|
| 0 | ⚠️ **Confirm on the arm — the short list** | ✅ **Confirmed 2026-08-12:** Ctrl-C → park → disable · `h`/`t`/`h` switching *"instantly"* · `p 1 2 3 Enter` sequences · speed adjustable while moving · **blended corners** (*"works quite nicely"*) · the plan-and-confirm step · `-/+` and `,/.` while typing · the **fast settle** (parks reported `0.1s`, `0.9s`, and `0.020`/`0.036 rad off` — no four-second wait anywhere in his log) · **per-waypoint timings** (`slot 1 in 2.6s → next 2`) · **s-curve easing**, which he likes: *"the S curve works quite well"*. ⬜ **New and unconfirmed — the session-19 display fixes:** the **status line no longer welded onto every message** · the **HOLD banner surviving** · `e` working outside a park prompt · the **stale RUN row** clearing · **`ö`/`ä`** arriving at all. Still never seen: the **55 °C warning**, and the **blind-thermal stop**, which cannot be triggered without unplugging CAN mid-session — stated rather than assumed |
| 0a | ⭐⭐ **READ [ROADMAP.md](ROADMAP.md) §6.6 — where the training data comes from** | ⛔ **Not code, and it gates step 5 (the recorder) completely.** Build the recorder first and it gets designed around the wrong collection method. Julien has now answered the three questions that were open: **task = not decided yet, on purpose** (*"we first have to get the setup right"* — so build everything task-agnostic); **model = most likely diffusion**, which makes deliberately varied demos safe rather than harmful; **provenance = matters a lot**, meaning every episode must record the git commit, camera serials, calibration, collection method and a success flag. ⭐ Two things in `ROADMAP.md` §6.6 must not be lost: **noise on the grab waypoint poisons the dataset while noise on the approach waypoints is useful** (his own refinement, and it is right), and **a pause at each waypoint is a prerequisite** because blending is defined as not stopping and a grab needs a stop |
| 0b | ⛔⭐⭐ **BUILT 2026-08-13 AND NEVER RUN ON THE ARM — the first thing to try on hardware** — `ROADMAP.md` §6.6 | **How it works:** `w` records (every cycle, in every mode) · `w` again, then `0`-`9`, saves to `recordings/<n>.json` with the git commit and a timestamp · `l` then `0`-`9` shows the plan and waits for **Enter**, then ⛔ **parks to the recording's start pose first** and only then follows it in real time. `-`/`+` set the playback speed, capped from the recording's **own measured** top joint speed. `src/recording.py` + **37 tests**, all headless. ⚠️ **TREAT THE FIRST RUN AS A TEST** (working contract, session 4). The playback branch commands the arm and has never executed; its *decision* is the tested pure function `replay_step()`, but the wiring around it is unproven. ⭐ **Guards already in place:** the clock is **held** if the arm falls behind (`MAX_CURSOR_LAG`), a **4 s** stall ends the playback rather than waiting for ever, the **gripper is excluded** from the lag check (jaws legitimately sit off-target while gripping, and counting that would stall every playback that grips anything), jaw commands go **through the clamp** (a recording made on a mechanical stop would otherwise hold there, which cooked motor 7 three times), leaving the mode abandons the playback, and ⛔ **a recording error cannot propagate into the control loop** — unwrapped it would skip the consent flow and disable the motors, the exact path that dropped 4.3 kg. ⚠️ `src/recording.py` is the **motion** recorder, deliberately separate from the dataset recorder (step 5), so it does not have to guess at ABC's schema. Julien picked this feature: *"one good idea is definitely recording everything in the guide mode and then replaying it. That's a smart idea, definitely."* Stream `q(t)` while he hand-guides the arm, save it, replay it through the existing `src/motion.py` engine. ⭐ Why it beats waypoints: human timing and hesitation are preserved, no IK is involved, the path is achievable by definition because the arm physically went there, and his hands are only in frame during teaching. ⚠️ The unknown to check is whether the replay follows the taught path closely enough that image and action still line up — the park loop already reports the lag, so instrument it rather than assuming |
| 0b | ✅ **Smoothing between poses — done 2026-08-12, on by default** | Eases in and out over 0.2 rad (15% → 100% → 15%), both park paths, each leg of a sequence getting its own ramp. `--no-smooth` restores the constant rate. ⭐ **The opt-in caution in the original plan was wrong and checking it changed the decision:** "a deceleration bug shows up as overshoot" is true of a new integrator with velocity state, and false here — `advance_park_command` is `command + clip(target - command, -step, step)`, so a step is already bounded by the distance remaining and **scaling it down cannot overshoot**. Feel is still Julien's to judge on the arm; `PARK_RAMP` is the dial |
| 0c | ⭐⭐ **Wire `ArmSession` in — and as of 2026-08-12 this is the HIGHEST-VALUE work available, for three independent reasons** | ⭐ The three reasons, and any one of them would justify it: **(1)** it is the only way to drive both arms from one loop; **(2)** ⛔ **ABC's training file is 14 states and 14 actions wide, meaning both arms in one timeline, so two separate terminal sessions cannot produce a valid dataset at all** ([ROADMAP.md](ROADMAP.md) §9.2); **(3)** every paper in the stack plan wants the robot as a callable library, and this class is the first piece of one ([ROADMAP.md](ROADMAP.md) §9.5). ✅ The class is **built and tested** (`src/arm_session.py`, 17 tests, fake robot). ⬜ What remains is restructuring ~1000 lines of `main()` to use it — deliberately its own session, because mixing "write the class" with "restructure the loop" produces a diff nobody can review and only Julien can test. ⭐ Do it as [ROADMAP step 6](ROADMAP.md) says: **`--arms B` with N=1 first**, confirm it feels identical, and only then N=2 |
| 1 | ⭐⭐ **Mirror mode — the SCRIPT. The logic is done.** | Julien's idea, and **the right first two-arm feature**: ✅ **`src/mirror.py` + 14 tests are DONE** — `MirrorLink` handles copy/mirror, staged engagement and the stop-rather-than-chase guard, with no robot handle so it is fully testable without an arm. ❌ **What is missing is the script** that opens both arms, reads B and commands G. That is the same two-arm process `ArmSession` needs, so **build them together**. ✅ Julien answered the design question: **both modes exist, `copy` is the default, and the arms are side by side** — so copy is correct today. ⚠️ **`MIRROR_SIGNS` is a geometric PREDICTION, not a measurement** — reflecting through a vertical plane should negate base_yaw, wrist_roll and gripper_twist and leave the three pitches alone. Expect to adjust it the first time `mirror` is used. |
| 2 | ⭐ **`ArmSession` + one script for both arms** | Fully designed in [ROADMAP step 6](ROADMAP.md). Neither hardware nor compute is the blocker — two arms on two buses from one loop is proven, two IK solves cost 0.100 ms/cycle against a 10 ms deadline, and Julien has already driven both arms at once as two processes. The blocker is that `teleop_session.py` (~1150 lines) holds one arm's state in one function's locals. ⭐ **Make `--arms B` run the N-arm code with N=1 first**, so the refactor is verified against a feel he already knows, separately from the two-arm risk. Prerequisites **done**: per-arm *and* per-frame maps, and `pick_device_by_wiggle(exclude=…)` |
| 3 | **Live telemetry on screen** | His clarification: camera fps, motor temperatures, poses **in units a human can act on**, gripper angles. ⚠️ The requirement is *understandable*, not *complete* — raw radians and quaternions fail it; degrees, centimetres and named axes pass |
| 4 | **Debug logs with more than one view** | *"we don't always need access to all of the data when we're debugging specific parts."* Not one firehose: one structured record per cycle, plus filtered views (thermal only, IK only, input only). Design not started |
| 5 | **Recorder → MCAP in ABC's schema** | ⏸️ **Deferred by Julien** while a friend finishes the plan. Building now would guess at a schema about to be specified. Get it wrong and every demo must be re-collected. ⛔ **And as of 2026-08-12 it is blocked by item 0a as well**, on a second axis: the schema is his friend's to specify, but *what gets recorded* — nominal waypoint or actually-commanded pose, and whether perturbed replays are recorded at all — follows from [ROADMAP §6.6](ROADMAP.md). Recording the nominal target instead of the real command yields a dataset that looks correct and teaches nothing |
| 5b | ⭐ **Cameras: the second D405 breaks how they are identified** — full plan in [ROADMAP.md](ROADMAP.md) §7.1-7.3 | ⛔ **Read `ROADMAP.md` §7.1 before plugging in the second D405.** Cameras are identified by asking each for a picture size only one supports ([FINDINGS §22](FINDINGS.md)); **two identical D405s support the same sizes, so that method cannot separate them.** They have real serials, but the plain-webcam path cannot read a serial. ⭐ **UPDATE 2026-08-12: librealsense is INSTALLED and it works, but only with `sudo`** ([FINDINGS §28](FINDINGS.md)). macOS's own webcam driver holds the camera's control interface, and taking it back needs root. ⛔ **So this does not solve identification inside our code**, because a 100 Hz control loop must not run as root. Use `sudo rs-enumerate-devices -s` for inspection, keep streaming on the OpenCV path, and use the **wiggle method** for which-camera-is-on-which-arm ([FINDINGS §28.6](FINDINGS.md)) — the same trick already used for the two SpaceMice. ⛔⭐ **Also unresolved: the camera reports TWO DIFFERENT serial numbers** depending on whether librealsense or the USB descriptor is asked (`260322274021` vs `255323071773`, same evening, one camera on the bus). Settle it on the next hardware session before writing any serial into a config file. ⚠️ Also open: **measure the real mount** (the code's `camera` frame assumes a modelled 25° tilt that the photographs do not match — drive in `tool` until measured), **measure two-camera USB bandwidth** (fails as dropped frames, not as an error), and **re-measure latency** rather than assuming the C920's ~200 ms carries over. ⭐ **And the cable is wrong today**: it strains the plug when the wrist twists and hangs loose in the workspace. `ROADMAP.md` §7.3 has the fix and the USB length limits |
| 6 | ⭐⭐ **The D405 wrist cameras — and the cheap shortcut WORKS** | One is mounted on **arm B**, plugged in, and **measured** (serial `255323071773`, USB SuperSpeed); the second is with **arm G** and still unplugged — only one serial is on the bus. ⭐ **2026-08-11: OpenCV opens it over plain UVC and gets a real picture** — Julien's live view shows a textured photographic image and `--list` reports `colour`. *(An earlier note here said "depth only". That was inferred from the device's NAME — macOS calls it `… Depth` — and it was wrong; the pixels say otherwise. FINDINGS §22.)* **So driving from the wrist camera needs no SDK at all**, and `brew install librealsense` is an upgrade for depth data, intrinsics and camera controls rather than a prerequisite. Next: mount it properly, then `v` → **tool** frame (⛔ *not* `camera`, until the real mount transform is measured — COMMANDS). He gave the manual's link: `intelrealsense.com/get-started` |
| 6b | **Camera latency — probably NOT worth more software effort** | Julien perceives ~0.2 s. **Measured: the draw cost is ~2 ms**, so render, terminal and grabber are all irrelevant. The rest is the C920 itself — sensor readout, onboard MJPEG encode, USB transport — typically 100-200 ms for a consumer webcam and not removable in software. Resolution is the only lever (key `1` = 320×180). ⛔ **Confirm the 2 ms is still ~2 ms, then stop**; the real answer is the D405 wrist cameras. [FINDINGS §21.3](FINDINGS.md) |
| 7 | **A remote of Julien's own** | ⚠️ Partly addressed 2026-08-12 — 56 commits now sit on the branch `julien/yam-teleop-wip` in his friend's public repo, so the work is no longer on one Mac alone. That branch is not a backup he controls, so a private remote of his own remains open |

⚠️ **Untested on hardware, all built and verified in simulation or headlessly.** Treat the first run of each as a test: the speed throttle near the workspace edge, and all of `src/mirror.py`. ✅ Confirmed working by Julien since being built: control frames (`v`), per-frame maps, the pure-rotation fix, and the camera at 30 fps in both window and terminal.

> ### ⭐ A LIVE THREAD IN THE OTHER REPO — do not lose it
>
> Julien asked for a **proper explanation of inverse kinematics**, taught as a structured path rather than summarised, and ruled that it belongs in his **`Mind Understanding`** learning repo rather than here — where IK is already indexed as a topic. He explicitly parked it: *"I haven't really read through most of your answers yet regarding the inverse kinematic answer and the writing in general. So do the most sensible thing, and we'll come back to it later once we fixed these main issues."*
>
> ⚠️ That repo's rules apply there and differ from this one's: `canon/` is **curated and read-only** for session agents, so an IK topic must be **proposed** via `agents/<name>/REPORT.md` §Proposals, not created unilaterally. See that repo's `CLAUDE.md` and `state/NOW.md`.

## 6. What to do next

⛔ **§5.5 is the task list. This section does not repeat it** — it used to, and the copy went stale within a day: it was still asking for PARK and the gripper to be verified on hardware after Julien had confirmed both. **Two ordered task lists in one file is not thoroughness, it is a second thing to keep true.** The lesson is the same one this repo keeps relearning about guards: anything written once and never re-derived against what it describes will quietly start lying.

**The one item that is not in §5.5 because it is not engineering work:**

⭐ **A remote of Julien's own — still open, and now the *only* part of this still open.**

**What happened on 2026-08-12:** he asked for the work to reach his friend's repo immediately, explicitly waiving the "cleaned up and clear" bar in README §7.5 — *"I don't care if it's clean because they said they don't care either… we'll just work however we want and clean everything up as we go along."* So 56 commits went to **`Hohnik/LaRobot`** as the branch **`julien/yam-teleop-wip`**, whose name says what it is.

**What held, and was never in question:** ⛔ **`main` of a collaborator's repo was not touched.** A branch is reviewable and deletable; a push to their `main` is a fait accompli. That rule survives the waiver — it is about *their* repo, not about polish.

**What is still missing, and why it is not the same thing:** a branch in someone else's public repository is **not a backup Julien controls.** It can be deleted by someone who is not him, and nothing private can ever go there. His own private remote remains the real item. *(Skipped for now because creating it needs his GitHub account; `gh` is not installed here.)*

**Before this becomes a pull request**, README §7.5 still applies in full: ask Hohnik what structure he wants *first* — it is a social step, not a technical one — and only then open the PR, with Julien reviewing the diff. A pushed branch is not a proposal; opening the PR is.

## 7. Session log

| session | date | outcome |
|---|---|---|
| 1 | 2026-08-07 | Hardware enumerated. SpaceMouse readable. **Wrongly concluded the CAN protocol was unknown and macOS unusable.** |
| 2 | 2026-08-10, 09:30-14:00 CEST (40 min lunch) | Both prior conclusions refuted. Arm identified, driven, gravity-compensated, hand-guided and **teleoperated with a SpaceMouse**. Gripper disabled after cooking motor 7 three times. ~30 commits. |
| 3 | 2026-08-10, ~14:25-15:xx CEST | **No hardware touched.** Full axis remapping built (`src/axis_map.py`, `scripts/map_axes.py`) with 25 tests. Four defects found **by reading** and fixed — see FINDINGS §9. The world-frame axis semantics measured in simulation instead of assumed. 34 headless tests now exist where there were none. |
| 4 | 2026-08-10, ~15:20-16:xx CEST | ⛔ **First hardware run of session-3 code, and it went badly.** `mjpython --view` could not start; **the arm fell** in GUIDE because `--no-gripper` swaps the gravity model; and the new MAP mode **destroyed the hand-dialled axis map** (recovered from git). All three diagnosed to root cause and fixed, all three documented in [FINDINGS §11](FINDINGS.md). MAP mode replaced by **CONTROLS mode**, designed by Julien: the arm moves, one isolated axis at a time, and only keys edit the map. 43 headless tests. |
| 5 | 2026-08-11, morning | Hardware re-checked after an overnight power cycle (all clean, no recalibration needed). **PARK confirmed working by Julien.** Fixed: the gripper buttons, which shipped broken (`b` only worked in one mode while its own hint printed in another); `q` now offers **`p` park** and the park pose defaults to the session's starting pose, making `q p d` hands-free. ⭐ **Diagnosed and fixed the "incoherent motion":** pure rotation was translating the tool point **44 cm**. The obvious singularity hypothesis was **refuted by measurement**; the cause was an unconditionally-integrated orientation goal plus an `orientation_cost` that was 10× too high — which was making rotation *worse* as well. 92 headless tests. |
| 6 | 2026-08-11, ~14:00-14:35 | Arms **renamed B and G** to match their physical labels, config migrated with every value verified byte-identical. **Per-frame control maps** — each frame owns its wiring and `m` describes the frame you are actually in, with tool-frame labels measured from the model. New frames are **seeded from the world map** so nothing is re-tuned from scratch. **Camera fixed: the 5 fps was my own frame-draining loop**, not USB bandwidth — `grab()` blocks on macOS, so "draining" waited for five frames. **Speed lag diagnosed as a singularity problem**, not a speed problem, and throttled at the source. 109 headless tests. |
| 7 | 2026-08-11, ~15:00-15:47 | **Camera terminal view** — aspect-ratio stretch fixed (it ignored the source aspect entirely), real C920 modes offered down to 320×180 (a UVC camera silently substitutes the nearest mode, which is why 424×240 became 640×360), and **iTerm2/kitty inline images implemented** so the block renderer is a fallback rather than the only option. `b` had been a two-way toggle whose sides could be identical, so it looked broken. **Latency measured at ~2 ms of draw cost — the rest is the camera hardware.** ⭐ **Mirror-mode logic built** (`src/mirror.py`), whose own tests caught a hidden 5 rad/s jump at the guard handover and a length mismatch. 123 → 138 headless tests. |
| 8 | 2026-08-11, ~16:10-16:29 | **Kitty images fixed** — they showed nothing because `f=100` means PNG and the renderer sent JPEG, with `q=2` suppressing the error that would have said so. `--term-test` added to make a silent display path speak. ⭐ **The D405 arrived and was measured**: serial `255323071773` (a real one, unlike the SpaceMice), USB SuperSpeed, and **it also enumerates as a plain UVC camera** — so OpenCV may open it with no SDK at all. `pyrealsense2` has no macOS wheels at any version (verified), but `librealsense` is a prebuilt Homebrew bottle. 143 headless tests. |
| 9 | 2026-08-11, ~16:30-17:15 | **No hardware touched.** Fixed Julien's *"the resolution is stuck … pressing the numbers doesn't do anything"*: keys 1-6 changed the **capture** while the image sent to the terminal stayed pinned at 480 px, so they were **working perfectly and invisible**. Measured the PNG-vs-JPEG gap that makes kitty mode soft (~25x on both time and bytes) — the kitty protocol has no JPEG at all. Cameras given names. ⛔ **Two conclusions from this session were REFUTED in session 10 and are struck here so nobody inherits them:** ~~names paired to indices by macOS's list order, cross-checked~~ (the order is not OpenCV's — it was wrong about two of four cameras) and ~~the D405's UVC entry is depth only~~ (it delivers a colour picture; the claim came from reading the device's *name*). 143 → 156 headless tests. |
| 10 | 2026-08-11, ~17:20-18:0x | ⛔⭐ **Session 9's naming was wrong, and Julien's own falsification procedure is what caught it** — he covered each camera in turn and the C920 answered on index 0 where macOS lists the built-in. **Identity is now MEASURED**: each index is asked for a resolution only one camera supports, and whoever answers exactly is that camera ([FINDINGS §22](FINDINGS.md)). Three macOS enumerations all agree with each other and none is OpenCV's, so no list could ever have supplied this. Also fixed: **the probe read one frame at open**, so Apple's slow-exposing camera reported brightness 5 in a bright room and Continuity reported NO FRAME — warm-up, in the column used to identify cameras; **a black frame was reported as MONO depth/IR** — about an iPhone; **the number keys** now offer the selected camera's own modes (the old list was C920 modes, which collapse to three on the built-in — itself an unrecognised second report of the naming bug); and **the flicker**, which was delete-then-draw plus redrawing unchanged frames ~25 times a second. ⭐ **The D405's UVC stream is a real picture**, so the wrist view needs no SDK. 156 → 165 headless tests. |

| 11 | 2026-08-12, 09:54-10:3x | ⭐ **The work left the Mac.** Rig re-verified after an overnight break — both CANables by serial (`2081337C594E5018` = B, `20593383594E5018` = G), both SpaceMice, the C920 and the one D405 (`255323071773`), all unchanged; second D405 still unplugged. 166/166 headless tests. **56 commits pushed to `Hohnik/LaRobot` as `julien/yam-teleop-wip`** — Julien explicitly waived the "clean" bar (§6), and ⛔ their `main` was not touched. Secrets scan run against the tracked files first, because that repo is public: 43 files, nothing matching key/token/password patterns, no `.env`, nothing large. Full `github/gitignore` Python template adopted, with a warning block naming the three things that must **stay** tracked (`uv.lock`, `.python-version`, `config/*.json` — the last is measured calibration, not config). ⚠️ **New trap found:** `system_profiler SPUSBDataType` returns an **empty list** on this Mac while `ioreg -p IOUSB` shows 15 devices — see [FINDINGS §23](FINDINGS.md). |

| 12 | 2026-08-12, ~10:30-11:1x | **No hardware touched.** Read `teleop_session.py` end to end before restructuring it, and found ⛔ **three holes in the safety guards, none of which had ever fired** ([FINDINGS §24](FINDINGS.md)): the thermal guard **disarmed itself on any read error** and printed a calm `hottest 0°C`; the **55 °C warning was advertised in the startup plan and implemented nowhere**; and **Ctrl-C went around the consent flow** and disabled the motors, which on a raised arm is a sag. All three fixed — `ThermalGuard` is pure and has 14 tests, Ctrl-C is now the same request `q` is (twice forces it). 166 → 180 headless tests. ⚠️ **None of the three is confirmed on the arm yet.** The `ArmSession` refactor is deliberately *not* started: restructuring code with latent guard holes carries them forward invisibly. |

| 13 | 2026-08-12, ~11:00-12:0x | ⭐ **The camera identification was confirmed on hardware** — Julien's `--list` reproduced his covering test exactly ([FINDINGS §25](FINDINGS.md)). Four defects he found by using it, all fixed: a **2 fps stills mode offered as "best"** (the C920's 2560x1472 — measured), `--camera` **taking ~20 s** because it ran the full identification to answer a question about one device, the **720 px detail cap** being a constant where it had to be a measurement (now adaptive against the real draw cost), and `--term-test` **declaring Ghostty unsupported while Ghostty was drawing the image** (the kitty protocol needs an image id to reply to). ⭐ **Ctrl-C now parks to the start pose and disables** rather than asking — his ruling — which also let the duplicated park loop go. Saved-pose **slots** designed and their storage built + tested (6 tests, reads the existing calibration file unchanged); the key handling is deliberately **not** wired yet, see ROADMAP step 6.5. 188 → 194 headless tests. |

| 14 | 2026-08-12, ~11:30-12:4x | ⭐ **Ctrl-C parking confirmed working on the arm by Julien** — first press parked and disabled cleanly. Chasing his report that a later session *"kind of went back into a mode"* found ⛔ **the key reader was silently losing keys**: `select()` on `sys.stdin` while `read(1)` buffered them out of the descriptor's sight. Reproduced on a pty — type `pgh`, get `['p']` — and these are the keys that HOLD, quit, and stop a moving park ([FINDINGS §26](FINDINGS.md)). Also: the **park stall was a knife edge** (0.020 passed, 0.021 stalled, tolerance 0.02 — the controller's steady-state error), now split into `settled` vs `blocked`; parks **drain stale keys** before "any key stops it"; the SDK's shutdown traceback is suppressed narrowly; and the camera **reuses the probe's open handle** instead of opening twice. 194 → 205 headless tests. |

| 15 | 2026-08-12, ~12:50-13:4x | ⭐ **Julien confirmed `h`/`t`/`h` switching instantly on the arm** — the mode machinery and the fixed key reader both good. Built the **saved-pose slots and sequences** he asked for (`s 0-9`, `p` + digits + Enter, `+/-` park speed), on the interleaved park so the thermal guard runs throughout. ⛔ **His ruling split "the park pose" into two things:** slot 0 is the **base**, the only pose Ctrl-C returns to before releasing the motors, and waypoints 1-9 are ignored by Ctrl-C — because *a pose that is safe to be let go in is not the same as a pose you want to return to mid-task*, and they had been sharing one variable that `s` silently overwrote. 205 → 211 headless tests. Remaining in that feature: the smoothing ramp only. |

| 16 | 2026-08-12, ~13:50-15:0x | **Smoothing done and on by default** — and the reasoning that had made it opt-in turned out to be wrong: scaling an already-clamped step *down* cannot overshoot, so the risk that justified the flag does not exist in this shape. ⭐ **`ArmSession` built and tested** (`src/arm_session.py`, 17 tests on a fake robot) — the extraction that unblocks bimanual, with the rule that **the class decides and the script narrates** so none of it needs hardware to prove. ⚠️ **Not wired into `teleop_session.py`**, on purpose and in writing: that restructure is its own session, N=1 first. 211 → 233 headless tests. |

| 17 | 2026-08-12, ~14:20-15:3x | ⛔ **The smoothing built in session 16 was the wrong feature** — a speed ramp per leg, so the arm still stopped at every waypoint. Julien meant **corner blending**: one continuous motion curving *through* each pose. Built `src/motion.py` — `JointPath`, quadratic-Bézier corners, exact arc length, 12 tests — and rewired the park onto it, so there is **one motion engine** and the per-leg queue is gone. ⭐ Also the UX he specified: `p Enter` base, `p 1 Enter` one pose, `p 1 2 3 Enter` shows the plan and waits for a second Enter, and **`-/+` speed and `,/.` corners now work while typing**, not only while moving. ✅ `--term-test` answered: Ghostty replies `OK` to the kitty protocol and draws **only** the KITTY bars — **it does not implement iTerm2's**, so PNG is the ceiling and that question is closed. 233 → 245 headless tests. |

| 18 | 2026-08-12, ~15:20-16:3x | ⭐ **Blended corners confirmed working on the arm** — *"the smoothing seems to work quite nicely, actually."* Three complaints from that run, all fixed: the terminal **reprinted a whole block on every knob change** (now one live status line, `src/screen.py`, and `print` is shadowed in `main()` so the policy is one thing in one place) · the end of every park **waited 4 seconds** because `settled` shared the `blocked` timer (now 0.5 s vs 4 s) · and **easing is its own axis** — five profiles cycled with `e`, with Ctrl-C using `out` so a shutdown leaves at once and only lands softly. Each waypoint now reports its own seconds. 245 → 258 headless tests. |

| 19 | 2026-08-12, ~16:50-18:0x | ⭐⭐ **The session that changed where the training data will come from.** Julien drove a full ~4½-minute session — everything worked, s-curve easing *"works quite well"*, the fast settle and per-waypoint timings both confirmed — and then reported that ⛔ **the previous analysis of demo collection was "significantly wrong"**: the bottleneck is *executing* with the SpaceMouse, not resetting the scene, and an automated reset is near-circular. His proposal (teach poses by hand in GUIDE, replay to record) is analysed in full in **[ROADMAP §6.6](ROADMAP.md)**, including the two ways it can silently poison a dataset, a **third option that may beat both**, and a 20-minute measurement that settles it. ⭐ **All three of his UX complaints turned out to be ONE function** — `StatusLine.say()` cleared one row and then wrote a payload containing newlines, so every message overwrote part of the status row (`weightless°C`, `drivesC`, `cancelled.6.0s` in his own paste) and `_rows` desynchronised, which is the "duplicate print" and the eaten HOLD banner. **Four of the eight defects fixed were guards or tests that had stopped describing their subject** — including a screen test asserting on a payload shape no caller produces, and a keyboard test that pinned an arrow key silently changing a motion parameter and called it *"documented, not desired"*. Also `ö`/`ä` for the ease ramp, which needed a UTF-8 decoder before it could be a binding question at all. 258 → 267 headless tests. [FINDINGS §27](FINDINGS.md) |

| 20 | 2026-08-12, ~18:1x-19:xx | ⛔⭐ **NO CODE. Two things, and the first one changes every session from now on.** (1) **The way this project writes to Julien was rejected outright** — he read a full reply and understood none of it. The style is now named **"ELI18"** and the rule is in §4 item 8 above, with the two failures that actually broke it: **a reply structured as a correction of a document he had never read** (*"the circularity objection I missed entirely"* → *"What are you talking about?"*) and **a section reference with no file name** (`§6.6` instead of `ROADMAP.md` §6.6). ⚠️ **The old rule was followed as written and still failed**, so the scope widened from "explaining new topics" to every reply. Enforcing copy is `~/.claude/working-style/turn_reminder.md`, injected every turn. (2) ⭐ **[ROADMAP.md](ROADMAP.md) §6.6 rewritten in plain language and made the single place for data collection**, now carrying his answers (task undecided on purpose, diffusion, provenance matters), the plan he chose (**hand-guide a movement in GUIDE, replay it to record**), his **per-waypoint noise** refinement, his **microphone labelling** idea, a free grab-success detector using the gripper position we already read, and a table of what is built. Also **[ROADMAP.md](ROADMAP.md) §7.1-7.3 written**: what a D405 can actually do, ⛔ **the second D405 breaking camera identification**, and the cable routing with the USB length limits. |

| 20 | 2026-08-12, ~18:1x-19:xx | ⭐⭐ **NO CODE ON THE ARM. Three things, and the third one reshapes the roadmap.** (1) **The writing style was rejected and rebuilt** — see §4 item 8, now named "ELI18". (2) **Every doc reformatted to one line per paragraph** at his request, by `scripts/unwrap_markdown.py`, which refuses to write if any non-whitespace character moved. 1634 line breaks removed. Three of its 18 tests earned their place by catching real corruption, including a four-space indented diagram in `ROADMAP.md` step 1b that looks identical to a wrapped paragraph. (3) ⭐⭐ **The target stack was READ from source**, because two of its three papers postdate any model's training data: `amazon-far/abc` stores **14 states and 14 actions per timestep**, so two arms in one timeline, which makes the bimanual work a prerequisite of the data format rather than a feature; ABC's actions are **joint positions**, which we already command; and ABC ships **pretrained checkpoints**, so the demo count may be far lower than estimated. Also: **librealsense works with `sudo`** and the entitlement theory was wrong, ⛔ **one camera reports two different serial numbers**, and the terminal camera view has a **one-way ratchet** that shrinks the picture and can never grow back ([FINDINGS §29](FINDINGS.md), diagnosed from his two screenshots, `520 × 0.85 = 442` exactly). ⭐ **Then built the pure half of the feature he chose**: `src/recording.py`, a hand-taught movement that can be saved and played back, with 23 tests and no hardware needed. 267 → 308 headless tests. ⚠️ **He unplugged the hardware at the end of this session**, so everything hardware-shaped is queued for the next one. |

**Time accounting:** session 2 ran 09:30 → ~14:00 with a 12:35-13:15 break — **~3 h 45 m of working time.** ⚠️ Earlier estimates in this session were badly wrong (~2.4× over) because per-turn effort was being summed instead of wall-clock read. Read the clock.

⭐ **Sessions 6-10 are timed from the commit clock (`git log --date=format:'%H:%M'`), not from memory** — and doing that turned up two defects in this very table. Sessions 6, 7 and 8 were logged **out of order** (8, 7, 6), and session 8 was labelled *"evening"* when its commits are 16:13-16:29, which would have made session 9 at 16:30 look like it came first. **A log that is complete and mis-ordered still misleads** — the placement lesson again, in the one table whose entire job is sequence.

### ⭐ Where the time actually goes — measured over sessions 9 and 10, 2026-08-11

Julien asked for this explicitly, because *"just so you have an understanding of what takes time and why."* His own numbers: the first message went out at ~16:30, the opening check-in took ~2 minutes, the build itself ~32, and **he spent ~15 minutes reading the result, taking screenshots and testing** — roughly an hour of wall clock for one feature. Sessions 9 and 10 together ran 16:36 → 18:0x.

**Session 9, from the tool-call record** (16:36 → 17:14, ~38 min):

| phase | ~min | what it bought |
|---|---|---|
| reading before writing | 11 | `camera_view.py` is 828 lines and `test_camera_render.py` 318. Skipping this is how you fix the wrong thing |
| measuring | 3 | the PNG-vs-JPEG benchmark. **The single best minutes spent** — it produced the numbers that set the caps and killed a guess |
| writing code | 12 | 8 edits |
| writing tests | 5 | 13 new ones |
| documentation | 9 | 5 files, and it found 4 places the docs had silently drifted |

**Session 10** (17:20 → 18:0x): ~20 min of investigation *before a line of code* — six separate probes (AVFoundation ordering, two discovery-session orderings, per-device format lists) to settle a question two plausible stories were fighting over — then ~15 min of code, ~10 of tests, and the rest documentation.

**What is actually expensive, in order:**

1. ⭐ **Undoing a published wrong conclusion.** Session 9's naming bug cost ~10 minutes of *code* to fix and roughly **three times that in corrections**: FINDINGS §22 rewritten, four places in HANDOFF, two in ROADMAP, one in README, plus a struck-through session-log row. **The blast radius of an inference is every document that repeated it** — which is the "measure, don't infer" rule of this repo restated as a number, and the strongest argument for spending the 20 minutes on measurement first.
2. **This repo's documentation standard.** Docstrings that carry the *why*, commit messages that hold the reasoning, and FINDINGS entries are perhaps 40% of everything written. That is deliberate — it is why a contextless agent can resume — but it is the largest single line item, and the one to trim first if a session ever has to be short.
3. **Investigation round-trips.** Each measurement is a separate command with thinking either side. Cheap individually, and they are what separates a diagnosis from a guess.
4. **Writing the code.** Consistently the *smallest* part. Typing was never the bottleneck.

**Rough planning numbers, for deciding what fits in a sitting:** a known fix in a known place, 10-15 min · diagnose + fix + test + document a new defect, 30-45 · a session that has to overturn a conclusion already written into the docs, 45-60. ⚠️ Add Julien's own 10-20 minutes for anything he has to run on hardware — that time is real, it is on the critical path, and it is where the actual truth comes from.

⭐ **Session 3's lesson: the bench is not where the cheap defects are.** Nothing was plugged in, and it still turned up a path that would have released a raised arm (PARK with `--no-gripper`), a thermal test that could not have detected the thing it was testing, a warn-and-continue in the exact wording of the one rule this project wrote in blood, and a simulator that could not reproduce the mapping it exists to de-risk. All four were reachable by reading the code against its own documentation.

⛔ **Session 4's lesson is the counterweight, and it is sharper: reading does not find what only hardware knows, and "it compiles and has tests" is not "it works".** Everything session 3 built passed 34 tests, three dry runs and a simulated IK loop — and the first contact with the arm produced three failures in one attempt, one of which dropped 4.3 kg. Two specific process faults worth carrying:

1. **`ls` is not verification.** `mjpython --view` was recommended on the strength of the binary existing. It could not start. *Verify the consequence, not the mechanism* — quoted in the same turn it was violated.
2. **A flag named for one thing changed another.** `--no-gripper` was chosen *because* it sounded like the smaller, safer experiment. It silently replaced the dynamics model. **Before recommending a flag as "safer", read what it actually switches** — the name is not the contract.


---

## 8. ⭐ The Intel RealSense D405 — MEASURED, 2026-08-11. Read this before touching it.

**One camera is connected and healthy.** Julien mounted one provisionally on **arm B**; the second is with **arm G** and is **not plugged in** (only one serial appears on the bus).

### What was measured, not assumed

| | value | why it matters |
|---|---|---|
| product | `Intel(R) RealSense(TM) Depth Camera 405` | it is the D405, confirmed |
| **serial** | **`255323071773`** | ⭐ **a REAL serial.** Unlike the two SpaceMice, which report empty serials and forced the wiggle-to-assign hack, two D405s can be told apart properly. **Select by serial, never by index** (FINDINGS §0 #5) |
| USB IDs | VID `0x8086` (32902), PID `0x0B5B` (2907) | for `ioreg`/`lsusb`-style checks |
| `bcdDevice` | `20721` = `0x50F1` | **probably firmware 5.15.1**, unverified — confirm with `rs-enumerate-devices` |
| **link speed** | **SuperSpeed (5 Gbps), `Device Speed = 3`** | ⭐ it negotiated **USB 3**, not USB 2. Bandwidth is not a problem for one camera. Re-check when the second is added |
| **UVC** | `Intel(R) RealSense(TM) Depth Camera 405  Depth` → `UVC Camera VendorID_32902 ProductID_2907` | ⭐⭐ **see below — this is the shortcut** |

### ⭐ The shortcut WORKS — and an earlier claim here, that it did not, was wrong

macOS lists the D405 as a standard **UVC camera**, so OpenCV opens it with no SDK at all, and `camera_view.py --list` identifies it as index 1 on the current rig.

**The question this section used to leave open — "whether the RGB stream appears as a separate index" — has a better answer than expected.** macOS lists exactly **one** entry for the D405 and calls it `… Depth`, but what arrives over it is **an ordinary picture**: `--list` reports `colour`, and Julien's live view shows wall texture, wood grain and print on a t-shirt. A depth map has none of that. The D405's imagers are colour-capable, and that is what the single UVC entry carries.

⛔ **This section previously said the opposite — "depth only" — and it was committed before anyone looked at the pixels.** The claim was inferred from the word `Depth` in the device name. *A name is not a contract*: the same lesson as `--no-gripper`, chosen because it sounded like the safer experiment, which silently swapped the gravity model and dropped the arm ([FINDINGS §11.1](FINDINGS.md)).

⭐ **So the wrist view can be driven today**, with the control-frame machinery (`v` → tool frame) that is already built and waiting. A UVC-only path still gives no depth alignment, no intrinsics and no camera controls — that is what the ladder below is for, but it is an upgrade rather than a gate.

### The SDK situation — measured, and the prediction held

**`pip install pyrealsense2` is impossible here.** Wheels exist only for `manylinux1_x86_64`, `manylinux2014_aarch64` and `win_amd64` — **no macOS build at any version**, including the older 2.55 that once had one. Verified with `uv pip install --dry-run` on both current and pinned versions.

⭐ **But `librealsense` IS available from Homebrew as a prebuilt bottle** — `stable 2.58.3 (bottled)`, dependencies `glfw` and `libusb`, not currently installed. A *bottle* means no source compilation for the C++ library and its tools:

```bash
brew install librealsense      # then:
rs-enumerate-devices           # confirms the camera, reports firmware
realsense-viewer               # GUI: streams, depth, and the camera's own controls
```

⚠️ **The Homebrew formula does not necessarily build the PYTHON bindings** — those usually need a source build with `-DBUILD_PYTHON_BINDINGS=ON`. So the likely ladder, cheapest first:

1. ✅ **UVC via OpenCV** — **done, and it is enough to drive by.** Nothing to install; a colour picture arrives; `--camera d405 --term` works today.
2. **`brew install librealsense`** — an upgrade, not a gate. Prebuilt tools, confirms firmware, and `realsense-viewer` shows the depth stream and the camera's own controls. Worth doing when depth data, exposure control or intrinsics are actually wanted.
3. **Source build with Python bindings** — only if depth is needed *inside Python*. ⚠️ Its cost is a source build; do not start it before something concretely needs depth.

⭐ **This is the same shape as the CAN SDK problem** ([FINDINGS §2](FINDINGS.md)): a vendor SDK that assumes a platform we are not on. That was solved by patching from *outside* while keeping `third_party/` a clean upstream checkout. **Read §2 before choosing an approach here** — and note it also ends with the reminder that Linux remains right for the final rig, so effort spent fighting macOS should be proportionate.

### What already exists and must not be rebuilt

The `camera` control frame (correct for the D405's modelled 25° flange cant), both viewers, the frame-rate and latency instrumentation, and the finding that the C920's ~200 ms latency is sensor-and-encode rather than software ([FINDINGS §21.3](FINDINGS.md)). ⭐ **A D405 may be much faster** — it is a machine-vision camera, not a consumer webcam — so **re-measure rather than assume that carries over.**

⚠️ **The agent still cannot open a camera stream.** macOS camera permission is per-application ([FINDINGS §21.1](FINDINGS.md)); enumeration via `ioreg`/`system_profiler` works, opening does not. Plan for measurements to be commands Julien runs, and put the diagnostics *inside* the program.

| 21 | 2026-08-13, morning | ⭐ **Rig re-verified after the overnight unplug, then the hand-taught movement feature was wired in.** Health check, all agent-safe: both CANables by serial (`2081337C594E5018` = B, `20593383594E5018` = G), both SpaceMice, the C920, one D405 (`255323071773`); ⛔ **the second D405 is still not plugged in**. Both arms read 7/7 motors, everything near zero. ⭐ **Both arms' jaw limits reconcile** — B needs the usual −6.283 rad shift and normalises to **0.036**, G needs no shift and normalises to **0.072**, so no recalibration. ⚠️ **Both sets of jaws are nearly closed**, so there is very little closing travel; harmless, and it looks like a fault if unexpected. Then **`w` / `l` built and wired** (task 0b) with 37 headless tests. ⛔⭐ **A NameError was caught before it ever reached the arm**: `replay_step` was called and never imported, so the first playback would have raised inside the control loop with the motors live. Found by an AST scan for names used before assignment, run because no linter is installed and `py_compile` cannot see it — worth keeping as a habit for this file. 308 → 322 headless tests. |
