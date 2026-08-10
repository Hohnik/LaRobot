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

**⛔ Known broken / deliberately disabled:**

- **The gripper is not under control** — `NO_GRIPPER` is the default. This is the top open item.
  **[FINDINGS §3.5](FINDINGS.md) explains exactly why and how to fix it.** Motor 7 was cooked three times.
- **SpaceMouse axis directions** are still being dialled in. `x`/`y`/`z` flip them live and persist to
  `config/spacemouse_map.json`; current state is `[1, -1, -1, 1, 1, 1]`.
- **No git remote.** Everything exists only on Julien's Mac. See §6.

## 3. The one command

```bash
cd ~/Developer/Projects/yam-robotics && uv run scripts/teleop_session.py --yes --arm arm1
```

`g` guide · `t` teleop · `h` hold · `p` park · `s` save park · `x`/`y`/`z` flip an axis · `+`/`-` speed ·
`r` rotation · `q` quit (asks first). Full inventory in [COMMANDS.md](COMMANDS.md).

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
6. ⛔ **This stack fails by lying, not by crashing.** Eleven defects in one day produced confident, plausible,
   wrong answers and not one raised an exception. **Check values for plausibility, not just for the absence
   of an exception.** [FINDINGS §0](FINDINGS.md) catalogues all of them.

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
| 1 | ⭐ **Dial in the SpaceMouse axis directions** | The one thing standing between here and comfortable one-arm teleop. Drive, and flip whatever feels backwards: `x` `y` `z` for translation, `1` `2` `3` for roll/pitch/yaw. Persists to `config/spacemouse_map.json`. Current state `[1, -1, -1, 1, 1, 1]` |
| 2 | **Verify PARK now works** | It was cancelled instantly by any unrecognised key — including Enter. Fixed but **not yet tested on hardware.** `s` to save a pose, move away, `p` to return |
| 3 | **Verify the gripper stays cool** | The 2π frame fix (FINDINGS §3.5) is verified numerically but **not yet on hardware.** Watch `hottest` for ~30 s in TELEOP: a **plateau** is the pass, a steady climb means quit and use `--no-gripper` |
| 4 | **Axis *remapping*, if flipping is not enough** | Only sign flips exist. If Julien wants puck-Y to drive robot-X, that is a permutation and needs building |
| 5 | **Simultaneous bimanual teleop** | Hard half proven (`move_both_grippers.py`). Needs one process, two robots, two `CartesianTeleop`s, two pucks assigned up front. ~6.2 ms/cycle against a 10 ms budget |
| 6 | **Recorder → MCAP in ABC's exact schema** | Setup-Plan §6.1. Get it right and the whole training half works unmodified; get it wrong and every demo must be re-collected |

⚠️ **Items 2 and 3 are code changes that have never been run against the arm.** They compile and item 3 is
verified numerically against two independent failures, but *"verified in principle"* is not *"verified"*.
Treat the first run as a test, not a demonstration.

## 6. What to do next

**Immediately:**

1. ⭐ **Give this repo a git remote.** ~30 commits exist only on one Mac, including everything above.
   Julien's own private GitHub first; `Hohnik/LaRobot` is planned separately — see README §7.5 for the
   fork-and-PR approach and the "clean" checklist. **Do not push to a collaborator's `main`.**
2. **Fix the gripper** — [FINDINGS §3.5](FINDINGS.md) has the mechanism and three ranked options.
3. **Finish the axis directions.** 10 minutes of driving with `x`/`y`/`z`.

**Then, in roadmap order:** simultaneous bimanual teleop (the hard half is proven) → **recorder → MCAP in
ABC's exact schema** (get this right and the whole training half works unmodified; get it wrong and every
demo must be re-collected) → cameras.

## 7. Session log

| session | date | outcome |
|---|---|---|
| 1 | 2026-08-07 | Hardware enumerated. SpaceMouse readable. **Wrongly concluded the CAN protocol was unknown and macOS unusable.** |
| 2 | 2026-08-10, 09:30-14:00 CEST (40 min lunch) | Both prior conclusions refuted. Arm identified, driven, gravity-compensated, hand-guided and **teleoperated with a SpaceMouse**. Gripper disabled after cooking motor 7 three times. ~30 commits. |

**Time accounting:** session 2 ran 09:30 → ~14:00 with a 12:35-13:15 break — **~3 h 45 m of working time.**
⚠️ Earlier estimates in this session were badly wrong (~2.4× over) because per-turn effort was being summed
instead of wall-clock read. Read the clock.
