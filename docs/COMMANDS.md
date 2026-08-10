# Commands — the whole inventory, by how much they can move

> **Every script that can transmit is a dry run by default.** Leaving off `--yes` prints the full plan and
> sends nothing. That is the single convention to remember.
>
> **Everything takes `--arm arm1` or `--arm arm2`**, resolved by adapter *serial*, never by index.
> Default is `arm1`. Run them from `~/Developer/Projects/yam-robotics`.

---

## The three you actually need

```bash
# 1. Everyday driving: guide by hand, teleop with the SpaceMouse, park. All in one session.
uv run scripts/teleop_session.py --yes --arm arm1

# 2. After ANY power cycle: the jaw limits shift and must be re-measured. ~10 s, jaws only.
uv run scripts/calibrate_gripper.py --yes --arm arm1

# 3. Is the arm alive? Enables all 7, reads, disables. Nothing moves.
uv run scripts/read_arm_state.py --yes --arm arm1
```

### `teleop_session.py` — the keys

| key | mode / action |
|---|---|
| `g` | **GUIDE** — zero gravity, the arm is weightless, push it by hand |
| `t` | **TELEOP** — the SpaceMouse drives the end effector |
| `h` | **HOLD** — the arm holds its pose. The safe idle |
| `p` | **PARK** — drive slowly back to the saved pose. Any key aborts |
| `s` | save the current pose as the park pose |
| `o` / `c` | open / close the gripper |
| `[` / `]` | gripper step slower / faster |
| `r` | wrist rotation on / off *(on by default)* |
| `R` / `T` | rotation speed faster / slower |
| `x` `y` `z` | flip that axis when a direction feels backwards — **saved to `config/spacemouse_map.json`** |
| `+` / `-` | linear speed faster / slower |
| `?` | reprint the key list |
| `q` | **QUIT** — goes to HOLD and *asks*. Then `g` to go weightless and park by hand, `d` to disable |

Useful flags: `--start-mode hold|guide|teleop` · `--no-rotation` · `--linear-scale 0.2` · `--box 0.4`

⚠️ With two SpaceMice attached it asks you to **move the puck you want to assign to this arm**. They both
report an empty serial number, so there is nothing else to key an assignment off.

---

## Read-only — cannot move anything, no `--yes` needed

```bash
uv run scripts/probe_hardware.py        # enumerate HID, open the SpaceMouse, listen 5 s
uv run scripts/probe_can.py             # listen-only CAN watch; the transceiver cannot even ACK
uv run src/spacemouse_live.py --seconds 25 --until-complete   # live 6-axis readout
uv run scripts/teleop_sim.py --demo     # the FULL teleop loop against MuJoCo. No hardware at all
uv run scripts/teleop_sim.py            # ...driven by the real SpaceMouse, still simulation only
```

`teleop_sim.py --view` wants the MuJoCo viewer, which on macOS needs
`uv run mjpython scripts/teleop_sim.py --view`. Without it the run continues headless.

---

## Transmits, but cannot move a motor

These send CAN frames yet never enable anything — register reads only.

```bash
uv run scripts/identify_arm.py --yes --arm arm1              # motor models, sw versions, safety timeout
uv run scripts/identify_arm.py --arm arm1 --scan 1 8 --yes   # what is on this bus at all
uv run scripts/bench_can.py --yes --cycle --samples 8000     # control-rate measurement
```

**`--scan` is the first thing to run when something feels wrong.** If it reports 0 devices, the problem is
power or wiring, not software — see `FINDINGS.md` §8.

---

## Enables motors, sends no setpoint — the arm should not move

```bash
uv run scripts/ping_motors.py --yes --arm arm1      # per-motor position, torque, TEMPERATURE, error
uv run scripts/read_arm_state.py --yes --arm arm1   # all 7 through the whole-arm chain
```

`ping_motors.py` is also the temperature check. Idle is **31-36 °C**; **41-42 °C while holding a pose is
normal thermal equilibrium, not a fault.**

---

## ⛔ Moves the arm

```bash
uv run scripts/teleop_session.py --yes --arm arm1                 # ⭐ the one to use
uv run scripts/calibrate_gripper.py --yes --arm arm1              # jaws to both stops, gently, once
uv run scripts/hold_pose.py --yes --arm arm1                      # hold the pose; success = nothing moves
uv run scripts/hold_pose.py --yes --arm arm1 --zero-gravity       # weightless, hand-guide only
uv run scripts/teleop_gripper.py --yes --arm arm1                 # gripper twist + jaws only, no IK
uv run scripts/move_one_motor.py --yes --arm arm1 --delta 1.5 --cycles 3   # one motor, bounded
uv run scripts/move_both_grippers.py --yes                        # both arms' grippers, one loop
```

**Superseded but kept:** `teleop_arm.py` was the two-phase version. `teleop_session.py` replaces it — the
phases became live mode switches, and the snap bug it contained is fixed there, not here.

---

## Two arms

**Separately, today:** just change the flag. Each session assigns its own puck.

```bash
uv run scripts/teleop_session.py --yes --arm arm2
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
