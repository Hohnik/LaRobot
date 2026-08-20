# yam-robotics — the YAM bimanual teleop walkthrough

> ## What this repo is
>
> **A finished, hardware-proven walkthrough of a bimanual YAM arm setup, built so a team can rebuild it from scratch** (Julien's ruling, [docs/FINDINGS.md](docs/FINDINGS.md) §67.0). Every feature here was built once, tested without hardware first, and then confirmed on the real arms. The findings, the dead ends and the measured numbers are all written down. The deliverable is the rebuild plan, [docs/PLAN.md](docs/PLAN.md), and the rebuild works from it.
>
> What it does today, all confirmed on hardware:
>
> - Two arms driven from one loop by SpaceMouse: the puck steers the gripper in cartesian space through inverse kinematics. The loop asks for 100 passes a second and reaches about 85 on the Mac. ⭐ That shortfall is macOS waking a sleep late, and the same loop reaches 97 on the Linux station ([docs/PERFORMANCE.md](docs/PERFORMANCE.md) section 2).
> - Hand-guiding: the arm goes weightless under gravity compensation and you move it by hand.
> - Waypoint runs that can grab. The run pauses where only the jaws move and reports whether something was gripped.
> - Recording and replaying hand-taught movements, with the puck as a scrub wheel for the playback clock.
> - Camera frames recorded beside every recording, and an exporter that turns a recording into an MCAP episode with images, in the exact topic shape of the team's setup guide (its section C3).
> - Composite runs: waypoints and recorded movements in one sequence.
> - Mirror mode (one arm follows the other), saved per-session settings, a simulator that lags like the real hardware and runs the whole loop with nothing attached, and safe-stop plus incident recording.
>
> ## Which file answers which question
>
> ⭐ Pick a row. You do not need to read anything else first.
>
> | I want to … | read | about |
> |---|---|---|
> | understand how the whole thing works, from nothing | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 20 min |
> | build the real station | [docs/PLAN.md](docs/PLAN.md) | 20 min |
> | see how this repo maps onto our team's repo, and what still has to meet | [docs/BRIDGE.md](docs/BRIDGE.md) | 10 min |
> | drive the arms today: every command and every key | [docs/COMMANDS.md](docs/COMMANDS.md) | look up what you need |
> | work on the Linux station | [docs/LINUX.md](docs/LINUX.md) | 10 min |
> | know what is worth making faster, and what the numbers say | [docs/PERFORMANCE.md](docs/PERFORMANCE.md) | 15 min |
> | understand why an arm is behind your hand, in mirror mode or in teleop | [docs/LAG.md](docs/LAG.md) | 12 min |
> | find out where the project stands right now | [docs/HANDOFF.md](docs/HANDOFF.md), top block only | 5 min |
> | know why some decision was made | [docs/FINDINGS.md](docs/FINDINGS.md), by section number | look it up |
> | see what is still open | [docs/ROADMAP.md](docs/ROADMAP.md) §8.2 | look it up |
> | read the first two sessions' history | [docs/HISTORY.md](docs/HISTORY.md) | nothing to catch up on |
>
> ⚠️ Three of those are written for agents rather than people. HANDOFF, FINDINGS and ROADMAP are dense on purpose. The other eight are written to be read.
>
> | something is wrong … | read |
> |---|---|
> | a motor will not answer, or an arm sags | this page, the bring-up checklist below |
> | a camera opens and records nothing, or records too few pictures | [docs/FINDINGS.md](docs/FINDINGS.md) §76 |
> | the arms are on the Linux station and something differs from the Mac | [docs/LINUX.md](docs/LINUX.md) §5 |
> | a recording looks wrong | `uv run checks/check_recordings.py` first, then [docs/COMMANDS.md](docs/COMMANDS.md) |
> | the follower arm lags, or mirror mode stops itself | [docs/LAG.md](docs/LAG.md) |
> | the test total changed | `uv run checks/run_tests.py` and compare, then [docs/FINDINGS.md](docs/FINDINGS.md) §70.4 |
> | you do not know a word used in any of these | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) §2 defines all of them |
>
> If you would rather read in one order than pick a row: this page → [docs/PLAN.md](docs/PLAN.md) → [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) → [docs/BRIDGE.md](docs/BRIDGE.md) → [docs/COMMANDS.md](docs/COMMANDS.md). Running it on the Linux station: [docs/LINUX.md](docs/LINUX.md).
>
> ## Day one: prove the rig works, in three commands
>
> ```bash
> uv sync                                      # installs everything, including the yam package
> uv run checks/check_platform.py              # ⭐ which machine is this, and what is missing?
> uv run checks/check_rig.py                   # what is on the USB bus — never transmits
> uv run apps/ping_motors.py --arm B --yes     # motor health: temps, faults, jaw shift
> uv run apps/teleop_session.py --arm B        # NO --yes: prints the full plan, moves nothing
> ```
>
> No hardware at all? `uv run apps/teleop_session.py --sim --yes --arms B,G` runs the whole loop on the simulator, and `uv run checks/run_tests.py` runs the whole suite and prints its total.
>
> ## Safety, in one place
>
> - ⛔ **There is NO e-stop.** Wall power is the only hard cut. Keep a hand near the plug; `h` (HOLD) and `q` (quit flow) are the software stops.
> - **Every transmitting script is a dry run without `--yes`** — it prints its plan and sends nothing.
> - **GUIDE mode (`g`) makes the arm weightless instantly**: you are holding 4.3 kg, and the gravity model is the only thing helping you ([docs/FINDINGS.md](docs/FINDINGS.md) §11 is what happens when it is wrong).
> - **Four speed limits sit in series and the smallest binds** — `--linear-scale`, `--teleop-speed`, `--max-speed`, `--max-lag`; the plan printout explains each ([docs/FINDINGS.md](docs/FINDINGS.md) §65.0). All are safety limits; raising them is the operator's call, never an agent's.
> - **The jaws cook motor 7 if commanded onto a stop** — every path goes through the gripper clamp and the stall latch; never bypass them ([docs/FINDINGS.md](docs/FINDINGS.md) §4).
> - **Nothing checks for the two arms colliding.** The operator is the guard, deliberately ([docs/ROADMAP.md](docs/ROADMAP.md) §8.2 item 25).
> - **An agent never runs anything that sends a setpoint.** Working contract: [docs/HANDOFF.md](docs/HANDOFF.md) §4. ⚠️ **Cameras differ by platform**: macOS grants capture per application, so an agent shell can never have it; **on the Linux station an agent CAN open cameras** (the gate is the `video` group), and it does ([docs/FINDINGS.md](docs/FINDINGS.md) §75.7).
>
> ## Hardware bring-up checklist (the page the rebuild asks for first)
>
> 1. **Power on, glance at the motor LEDs.** Green steady = enabled. **Red STEADY = disabled, the normal idle.** **Red FLASHING = a latched fault** — codes `8` over-voltage · `9` under-voltage · `A` over-current · `B` MOS overheat · `C` coil overheat · `D` loss of communication · `E` overload.
> 2. ⛔ **Read a fault BEFORE any power cycle**: `uv run apps/ping_motors.py --arm B --yes`. A mains cycle erases the code, and the code is the only record of what happened.
> 3. **`uv run checks/check_rig.py`** answers "what state is every device in" — presence is not readiness. `DFU in FS Mode` on a CAN adapter = its bootloader: unplug the hub from the Mac, **wait ten seconds**, replug. Motors latched `0xD` = the bus went away: cycle the arm's MAINS (after step 2), because motors run from the wall and no USB replug can reach them.
> 4. **After any power cycle, calibrate the jaws per arm**: `uv run apps/calibrate_gripper.py --yes --arm B` (then `--arm G`). ⚠️ `--arm` is not optional — without it you calibrate B whichever arm you meant.
> 5. **The jaw frame shifts by ±2π between sessions, per arm.** It depends on where the jaws sat at power-up; `build_robot()` reconciles it automatically and `ping_motors` prints it. Never write a shift direction into a document.
> 6. **The SpaceMice have EMPTY USB serials, and so does the C920.** Pucks are assigned by wiggle at session start (move the one you want); cameras: the two D405s have real serials, the C920 is selected by model name.
> 7. **First camera use needs macOS permission** for the terminal app: System Settings → Privacy & Security → Camera. Agent shells can never be granted it. ⚠️ **On Linux there is no per-app gate** — membership of the `video` group is the whole permission ([docs/LINUX.md](docs/LINUX.md) §3).
> 8. ⚠️ **Put the CAN adapters on a powered USB 3 hub, not behind a dock.** The one hard crash this rig had was the whole bus sagging away mid-session: all seven motors latched `0xD`, both adapters fell into their bootloaders ([docs/FINDINGS.md](docs/FINDINGS.md) §46.0). Not fixable in software.
>
> ## Layout
>
> ```
> apps/           things you RUN: teleop_session, camera_view, calibrate_gripper, ping_motors, …
>                 export_episode (the C3 log) · export_dataset (the C4 training set)
> checks/         read-only diagnostics + the falsifiers that prove the checkers can see failures
>                 two commands, two totals: run_tests.py and run_falsifiers.py
> tests/          the whole suite — one command prints the live total: uv run checks/run_tests.py
> src/yam/        the library (installed by uv sync): robot, can, session, teleop, motion,
>                 inputs/, fake/ (the lagging simulator), ui/
> config/         measured calibration — tracked on purpose, so it TRAVELS between machines
>                 (session_defaults.json is not; linux/ holds the two udev rules)
> src/yam/files.py  the one listing helper: a name starting with `.` is never our data
> docs/           ARCHITECTURE + PLAN + BRIDGE + PERFORMANCE + LINUX + COMMANDS are written
>                 to be READ by a person
>                 HANDOFF (live state) · FINDINGS (evidence) · ROADMAP are agent files, dense
>                 on purpose. `uv run checks/check_prose.py` holds the first group readable.
>                 HISTORY is this page's first two sessions, archived. Not the live state.
> third_party/    the vendored I2RT SDK, untouched
> recordings/     gitignored: taught movements and tracking logs live on the rig only
> ```
>
> ---
>
> ## The project's early history has moved
>
> The first two sessions (2026-08-07 to 2026-08-10) used to sit below this line and made up two thirds of this page. They are now [docs/HISTORY.md](docs/HISTORY.md), with every section number unchanged. Nothing in them is the live state, and the live state is [docs/HANDOFF.md](docs/HANDOFF.md).

---

**Where to go next**

Pick a row from the table at the top of this page. If you would rather be told: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) to understand the system, or [docs/PLAN.md](docs/PLAN.md) to build one.
