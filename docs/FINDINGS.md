# Findings — everything learned about this rig, and how

> **Purpose.** `README.md` says what is true now; `ROADMAP.md` says what to do next and why.
> **This file is the latent knowledge** — the things that were expensive to learn, that no file records
> implicitly, and that a fresh agent would otherwise re-derive at cost or, worse, get wrong the same way.
>
> Everything here was **measured on the real hardware on 2026-08-10** unless it says otherwise.
> Where a number appears, it is a number that was actually read off the arm.

---

## 0. The one thing to internalise before touching anything

> ### ⛔ **This stack fails by lying, not by crashing.**

Nine separate defects in one day produced **confident, plausible, wrong answers** and **not one raised an
exception**:

| # | The lie | What it looked like |
|---|---|---|
| 1 | Transmit echoes decoded as motor replies | all 7 motors "replied" with a flawless set of zeros |
| 2 | `inertia` used in a model signature | every joint looked like a unique motor model |
| 3 | `FeedbackFrameInfo` vs `MotorInfo` | every field silently printed `?` while the motor answered perfectly |
| 4 | `error_code` annotated `int`, holds the string `'0x1'` | `!= 0x1` is always true → instant abort claiming "normal" |
| 5 | Adapter chosen by index | silently retargeted **the other robot** when arm 2 was plugged in |
| 6 | Two arms "verified" identical | evidence could not distinguish it from reading one arm twice |
| 7 | `DMChainCanInterface.close()` | shuts the bus, leaves every motor **enabled** |
| 8 | `MotorChainRobot.close()` | prints *"all torques set to zero"* — it sets none |
| 9 | Control thread dies silently | the loop kept commanding a corpse for **64 seconds**, printing healthy numbers |

**Practical rule: check values for plausibility, not merely for the absence of an exception.
And prefer a test that could falsify the claim over one that merely agrees with it.** #6 was only settled by
finding a measurement (per-unit `inertia`) that would have *differed* if the claim were wrong.

---

## 1. The hardware, as measured

**Two YAM arms, each with its own CANable adapter on its own CAN bus.** Both use motor IDs **1-7**, so
**nothing inside a CAN frame distinguishes the arms.** The adapter serial is the only discriminator.

| | serial | notes |
|---|---|---|
| `B` | `2081337C594E5018` | everything up to ~11:00 was this one |
| `G` | `20593383594E5018` | plugged in mid-session; **its adapter enumerated FIRST** |

⛔ **Never select the adapter by index.** `chain_channel('B')` returned `gsusb1` at 10:58 and `gsusb0` at
11:45 — **the enumeration order changed twice within one session.** `src/yam_can.py` resolves by serial and
re-verifies after opening.

**Proof the two arms are genuinely distinct** (not one arm read twice): `inertia` is per-unit calibration data
burned into each motor. All seven differ between the arms — e.g. joint 1: `1.7169109e-05` vs `1.6964389e-05`.

### Motors and joints

| motor | joint (from `yam.urdf`) | type | limits (rad) |
|---|---|---|---|
| 1 | `base_yaw` | DM4340 | −2.61799 … +3.14159 |
| 2 | `shoulder_pitch` | DM4340 | 0.0 … 3.66519 |
| 3 | `elbow_pitch` | DM4340 | 0.0 … 3.14159 |
| 4 | `forearm_pitch` | DM4310 | −1.69297 … +1.5708 |
| 5 | `wrist_roll` | DM4310 | −1.5708 … +1.5708 |
| 6 | `gripper_twist` | DM4310 | −2.0944 … +2.0944 |
| 7 | `gripper_jaws` | DM4310 | prismatic; see §3 |

**How the models were identified without energising anything:** the `gear_ratio` register. The Damiao part
number *is* the gear ratio — DM43**40** reports 40.0, DM43**10** reports 10.0. `sw_ver` partitions
identically as a cross-check. That layout is `yam_v1.yml` exactly ⇒ the arm is `yam` / `yam_pro` /
`yam_ultra_v1` (indistinguishable over CAN) and **not** `yam_ultra_v2`.

⚠️ **Decoding a motor with the wrong type does not raise, it mis-scales.** Position is safe (±12.5 rad on
both), but velocity reads **3× high** and torque **2.8× LOW** — under-reading force on the three heaviest
joints, which is the dangerous direction.

⚠️ **The URDF limits are directly comparable to raw motor positions** only because `get_robot.py` sets
`motor_offsets = [0.0] * n` and `yam_v1.yml` sets every direction to `+1`. Re-check if either changes.

### The parked pose is mechanically supported

`shoulder_pitch` and `elbow_pitch` read ~0.000, and the URDF puts **both of their mechanical limits at 0**.
The arm rests against its own stops rather than balancing, so energising from the parked pose cannot release a
joint into a fall. **This is why every first-contact test started there.**

---

## 2. macOS: why every patch in `src/yam_can.py` exists

I2RT's stack assumes Linux + SocketCAN throughout. Four things had to be solved, each for a distinct reason.

**1. `bustype` is an argument, not a constant.** `CanInterface.__init__` takes `bustype="socketcan"` and
passes it to `can.interface.Bus`. So `bustype="gs_usb"` reaches python-can untouched and the CANable works
over libusb. ⚠️ **But the layer the docs point you at cannot do this**: `get_yam_robot()` →
`DMChainCanInterface` picks the bus with `if "can" in channel:` and hardcodes socketcan, with no override
(`dm_driver.py:409`). Hence `chain_channel()` returns **`gsusb<N>`** — a name chosen precisely because it
fails that substring test — and `patch_dm_driver_for_gs_usb()` rewrites it one layer lower.

**2. `detach_kernel_driver` fails with EACCES on macOS** even though nothing holds the device (measured:
`is_kernel_driver_active(0)` is False and `claim_interface(0)` succeeds). Only the `USBError` is suppressed,
so a platform where the detach genuinely matters is unaffected.

**3. ⛔ Transmit echoes. Load-bearing.** A candleLight adapter echoes each sent frame back as a send
confirmation; python-can marks it `is_rx=False` but **does not drop it**. SocketCAN never loops a frame back
to the sending socket, so I2RT has no filter and takes the next frame as the reply. Decoding our own
request's `data[4:8]` — four zero bytes — produced a perfect set of zeros from all seven motors, reported as
success. Filtered at the bus layer where nothing can bypass it.

**4. Enable/disable desynchronisation.** `_send_message_get_response` retries by **re-sending** the command.
A late reply fails the arbitration-id check → the retry puts a *second* enable on the wire → two replies come
back → one is consumed, one is left in the buffer and read as the **next** motor's reply → which mismatches →
which retries. One hiccup snowballs down the chain and lands wherever the margin runs out. **The varying
failure point is the signature** — a genuinely dead motor fails in the same place every time.
I2RT's 3 ms inter-motor spacing is ample in-kernel; over libusb each transfer is ~0.45 ms and the margin is
thin. **A transport mismatch, not a bug in their code.** Fixed by draining before every enable/disable and
retrying the whole exchange after a fuller drain. Reliability went **1/3 → 8/8**.
⚠️ Deliberately **not** applied to `set_control`, which shares the function but runs the 100 Hz loop.

### Throughput — macOS is not the bottleneck

`bench_can.py --cycle --samples 8000`, real 7-motor cycles, 25 s sustained:

```
8000/8000 complete, 0 missed replies
cycle ms: mean 3.121  p50 3.116  p95 3.231  p99 3.318  p99.9 3.576  max 17.771
320 cycles/s sustained
would miss a 100 Hz deadline: 2/8000 (0.03%)
```

**~3× headroom over 100 Hz.** Session 1's "do not fight macOS for the control loop" is **refuted**. Linux
remains right for the final rig (RealSense, cuRobo, ABC training are Linux-first) and this says nothing about
the loop once cameras and inference compete for CPU — but the specific claim is dead.
⚠️ Measured with register reads, so it is a lower bound.

---

## 3. The gripper

- **Stroke: 6.57 motor-rad ↔ 0.096 m** of jaw travel (`linear_4310.yml`), matching the URDF's two prismatic
  tips at 0.0469 m each.
- **Measured limits (B, 2026-08-10): `+0.0704 … −5.0528`, usable stroke 5.123 rad = 78% of declared.**
  Saved in `config/gripper_limits.json`.
- **The jaws began the day parked hard against the `0` stop.** That is why every early jaw command in the
  positive direction tripped the torque limit within a fraction of a second while negative moved freely.
- ⚠️ **`get_yam_robot()` re-calibrates on EVERY construction** when `gripper_limits` is null, and
  `detect_gripper_limits` drives each stop and *holds* until the position stops changing. Julien: *"they move
  really quickly and quite hard… they seem to crash into the ends and then seem to try to push further."*
  **Passing `gripper_limits_override` is what disables it** (`get_robot.py:223-225`) — which is why
  `yam_robot.build_robot()` always loads the saved limits.
- There is **no speed parameter**. The routine is torque-controlled, so torque *is* the speed; lowering it
  lowers both. Ours uses 0.3 Nm vs I2RT's 0.5.
- After calibration the gripper is exposed as a **normalised 0…1 value**, not raw radians.
- ⛔⭐ **THE WORST BUG OF THE DAY: the limits are stored in one coordinate frame and used in another.**
  `get_yam_robot()` applies a **±2π wrap correction at every construction**, chosen from wherever the motor
  happens to be sitting at that instant (`get_robot.py:268-274`). `calibrate_gripper.py` builds a
  `DMChainCanInterface` directly and gets **no** such correction. So whether the saved numbers mean anything
  depends on the jaws' position when each ran.
  **The consequence is not a wrong number, it is a cooked motor:** `motor_chain_robot.py:390` force-clips
  every gripper command into `[min(limits), max(limits)]` *regardless of where the jaws are*. Measured: jaws
  at **−1.380** with a saved range of **[+1.231, +6.481]** → the gripper was commanded 2.6 rad away and held
  there against a mechanical stop → **43 °C → 65 °C in five seconds.**
  **Fix:** `reconcile_gripper_limits()` tries the saved range shifted by 0, +2π and −2π and returns whichever
  brackets the measured position; if none does, `build_robot` **refuses to start**. Verified: reconciling the
  failure case yields `[0.1979, −5.0524]` against the morning's independent calibration of
  `[0.0704, −5.0528]` — **the lower bound agrees to 0.0004 rad.**
  ⚠️ **Never "warn and continue" on this.** That is exactly what was done, and it is what burned the motor.
- ⛔ **A power cycle also invalidates the saved limits.** Measured 2026-08-10: before the power cycle the jaws
  calibrated to `+0.0704 … −5.0528`; afterwards they read **+1.6691 rad**, outside that range entirely. The
  motor's position reference shifts across power. A stale range makes the normalised value fall outside
  `[0,1]`, so every hold command pushes toward a stop — **which is precisely how motor 7 cooked, twice.**
  **Re-run `calibrate_gripper.py` after every power cycle.** `teleop_session.py` detects the mismatch and
  says so.
- ⛔ **Never command the gripper to 0.0 or 1.0.** Those *are* the mechanical stops, and holding a position at
  a stop is stall torque: full current, no motion, no cooling. Operator-requested values are clamped to
  **[0.02, 0.98]**.
  ⚠️ **The clamp is applied only to values the OPERATOR asks for, never to move the jaws to where they
  already are.** The earlier `[0.15, 0.85]` band was applied *on entering TELEOP*, which meant that if the
  jaws happened to sit outside it, the session **commanded them to move** the moment teleop began — a
  motion nobody asked for, into a mechanical stop when the limits were also mis-framed. Entry now takes
  the jaws exactly where they are.

---

## 3.5 ⭐ The gripper: two 2π frame errors, not a broken mechanism

**Motor 7 was cooked three times on 2026-08-10 and the first three explanations were all wrong.** The real
cause is a coordinate-frame mismatch, and Julien is the one who pushed back on giving up — correctly.

**Two separate 2π shifts are involved, and conflating them is what cost three attempts:**

| | shift | source |
|---|---|---|
| **(a) calibration frame** | ±2π | `calibrate_gripper.py` records limits through a bare `DMChainCanInterface`, which applies **no** wrap correction |
| **(b) runtime frame** | ±2π | `get_yam_robot()` adds ±2π to `motor_offset` at **every** construction, from the motor's momentary position (`get_robot.py:268-274`), and reports `raw − offset`. **The limits get no such treatment.** |

Fixing only (a) — which is what a first attempt did — leaves (b) intact, and the failure looks identical.

**The worked example, measured:**

```
jaws raw                       6.3235
6.3235 > π  ⇒ runtime reports  6.3235 − 2π = 0.0403
saved limits                   [6.481, 1.231]     (un-shifted)
normalised = (0.0403 − 6.481) / (1.231 − 6.481) = 1.227     ← outside [0,1]
```

A normalised position outside `[0,1]` is clipped onto the nearest limit by `motor_chain_robot.py:390`, so the
motor is commanded into **a stop it is already past** and pushes at 7.71 Nm indefinitely. Self-reinforcing:
7.7 Nm shoves the jaws further beyond what a 0.3 Nm calibration called the limit.

**`frame_correct_gripper_limits()` applies both shifts.** Verified on the two independent failures:

```
raw +6.3235  →  limits [0.1979, −5.0524]  normalised 0.0300  ✓
raw −1.3800  →  limits [0.1979, −5.0524]  normalised 0.3005  ✓
first calibration of the day, independently:  [0.0704, −5.0528]
```

Two different positions hours apart converging on one range, matching an independent measurement.

⛔ **And it is verified rather than trusted:** `build_robot` reads the normalised jaw position back from the
runtime after construction and **shuts everything down before the control loop starts** if it is outside
`[0,1]`. A prediction about a frame is exactly the kind of thing that should not be believed on argument.

**Escape hatch:** `--no-gripper` runs the six arm joints only and leaves motor 7 free.

### Lessons that generalise beyond the gripper

- **Two bugs with identical symptoms hide each other.** Fixing one and seeing no improvement is weak evidence
  that you fixed nothing — it may be evidence that there are two.
- ⛔ **A guard that cannot express a safe command is not a guard.** The stall guard "released" the command to
  the measured value — which clipped straight back onto the stop. It fired every cycle and changed nothing.
- **Cached raw motor positions are frame-dependent.** Anywhere a position is stored and re-used across
  process boundaries, ask which wrap correction was in force at each end.

---

## 3.6 (historical) The interim decision to disable the gripper

**Motor 7 was cooked three separate times on 2026-08-10.** Three different fixes were attempted and the first
two addressed the wrong layer. This is the true mechanism, and it is the single most important open item.

**The evidence that settled it**, printed by our own stall guard:

```
⚠️ GRIPPER STALLED (+7.71 Nm, not moving) — releasing it to 1.186
```

**`1.186`.** That is the *normalised* gripper position, and it is outside `[0, 1]` — **the jaws sit 18.6%
beyond the "fully open" end of their own calibrated range.** `command_joint_pos` clips it back to `1.0`,
which maps to the end stop, so the motor is commanded into a stop **it is already past** and pushes at
7.71 Nm indefinitely.

**It is self-reinforcing, which is why every run was worse than the last:** 7.7 Nm of runtime torque shoves
the jaws further beyond the limit that a 0.3 Nm calibration detected. The calibration finds a "stop" that
the runtime simply pushes through, so each session starts further outside the range than the one before.

**Why the earlier fixes could not work:**
- Clamping the command **above** `command_joint_pos` is bypassed: the vendor's clip is **below** it
  (`motor_chain_robot.py:390`).
- Releasing the command to the *measured* value is a no-op: `1.186` clips to `1.0`, the same stop. **A guard
  that cannot express a safe command is not a guard.**
- The ±2π reconciliation (§3) is correct and still necessary, but it fixes limits in the wrong **frame**, not
  limits of the wrong **size**. Both faults are real and independent.

**Current state: `GripperType.NO_GRIPPER` is the default.** Motor 7 is never enabled and never commanded; its
400 ms timeout leaves it damped and free. The six arm joints are entirely unaffected and teleop works fully.
`--gripper` opts back in.

> ### ⭐ To fix it properly (the next session's job)
> **Calibrate at the torque the runtime actually uses, not at 0.3 Nm.** The runtime reaches 7.7 Nm, so a
> limit found at 0.3 Nm is not a limit — it is where the jaws stop being easy to move. Options, in order of
> preference:
> 1. Calibrate at ~2-3 Nm and **inset** the saved range by a margin, so the runtime can never command the
>    true stop.
> 2. Reduce the gripper's `kp` (currently 20.0 from `linear_4310.yml`) so the runtime cannot generate 7.7 Nm
>    against a stop in the first place.
> 3. Command the gripper by **torque** rather than position — a gripper does not want a position controller.
>
> ⚠️ Whatever is chosen, **verify by leaving it holding for 60 s and watching the temperature plateau.**
> A gripper at equilibrium is the test; a gripper that "looks fine for 5 seconds" is not.

---

## 4. ⛔ The over-temperature incident, 2026-08-10 ~12:4x — read this before long sessions

**What happened.** During the first real cartesian teleop run, at t≈24 s:

```
ERROR:root:motor id: 7, error: motor over temperature at yam_real
```

I2RT's control thread raised and exited. **The teleop loop did not notice and kept running for another
64 seconds**, solving IK and calling `command_joint_pos` into a dead robot, printing plausible EE numbers the
whole time. Julien saw the arm stop responding while the terminal claimed motion. With no commands arriving,
the motors' own 400 ms timeout damped them and **the arm sagged slowly under gravity** — slowly enough to
catch by hand, which he did.

**Why motor 7 and not a big joint.** Motor 7 is the gripper, a small DM4310 with little thermal mass, and it
had spent the whole day being pushed **against a hard stop**: three teleop runs aborting at 1.2 Nm, I2RT's
0.5 Nm auto-calibration, then our 0.3 Nm calibration. Stall torque is the worst possible thermal case — full
current, no motion, no cooling. Then teleop commanded it to *hold* its position, which against a stop is more
stall torque. **The heat was cumulative across the session, which is why "I didn't even move it quickly"
is entirely consistent with an over-temperature fault.**

**Rules that follow:**
1. **Never command the gripper to hold a position at a hard stop.** Park it mid-stroke.
2. **Monitor temperature every cycle** and stop at 65 °C, below the firmware trip. `chain.read_states()`
   returns `temp_mos` / `temp_rotor` per motor. Idle is ~30-33 °C.
3. ⭐ **Check that the control thread is alive every cycle.** `chain.running` going False means commands are
   not arriving. A loop that cannot tell whether its commands land is worse than one that crashes.

---

## 5. Teardown — the order is not optional

```
1. stop the control thread   (chain.running = False, then wait ~0.15 s)
2. disable every motor        (while the bus is still open)
3. close the bus
```

Both vendor `close()` methods get this wrong. `DMChainCanInterface.close()` shuts the bus **without disabling
the motors**; `MotorChainRobot.close()` prints *"Robot closed with all torques set to zero"* and calls only
the former — while the 250 Hz thread is mid-`set_control`, which produced a thread-death traceback.
**Leaving motors enabled is what broke consecutive runs**: they time out into damping, and the next
`_motor_on()` takes the error-clearing path, which desynchronises §2.4.

`yam_robot.shutdown_robot()` does it correctly and **returns the IDs that actually confirmed** — never a
hopeful constant.

---

## 6. Recovering a drooped arm — yes, and nothing is lost

Julien asked whether a droop leaves the system miscalibrated. **It does not.** The encoders report true joint
positions at all times — proven this morning when a hand-twist of the gripper read back exactly
(0.0086 → −1.2312 rad) while every other motor stayed byte-identical.

**What was lost when the arm drooped was the control loop, not the knowledge of where the arm is.** So
recovery is simply: read the true current pose, then interpolate slowly to the target. `teleop_session.py`'s
PARK mode does exactly that at **0.40 rad/s**.
⚠️ What it *cannot* know is what is now in the way, which is why it moves slowly and `h` or `t` stops it.
⛔ **Correction, 2026-08-10: "any key aborts it" was the bug, not the feature.** Every unrecognised key —
Enter included — used to cancel PARK, so pressing `p` and then Enter out of habit killed the move in the
same keyboard batch and looked exactly like "park just went to hold". Only `h` and `t` stop it now.

---

## 7. Working rules established with Julien

- **The agent never runs anything that can move the arm.** Those are handed over as commands.
  The line: **scripts that enable motors but send no setpoint → agent. Anything that sends a setpoint →
  Julien.** (Widened from "anything that enables a motor" once enabling-without-setpoint had been shown ~15
  times to produce ≈0 torque.)
- **Announce before running, do not pause.** Say what is about to run, then run it.
- ⛔ **The workspace was the binding constraint, not the software**, until 12:1x when Julien cleared the desk.
- **`--yes` on every script that transmits.** Dry run is always the default and always prints the full plan.

---

## 8. State at the end of 2026-08-10

**Achieved:** SpaceMouse verified on all 6 axes · both arms identified and driven · bimanual gripper motion
from one loop over two buses · 100 Hz proven with 3× headroom · gravity compensation holding to **0.61°** ·
hand-guiding · **cartesian SpaceMouse teleop on the real arm** (EE moved 0.15 m under puck control).

**✅ RESOLVED — it was the wall power, exactly as the shared-cause reasoning predicted.** Julien
disconnected and reconnected the arms at the socket and both came straight back: all 7 motors on each bus,
every temperature 31-36 °C, no latched faults. **A power cycle is the recovery for a latched motor
over-temperature.** The diagnosis held because two independent arms on two independent buses cannot fail
together for independent reasons — the only shared thing was mains.

**Known-imperfect, deliberately deferred:**
- **SpaceMouse axis directions are wrong/unintuitive.** Not yet mapped to Julien's expectation. **The tooling
  for it is complete as of session 3** (`scripts/map_axes.py`, no hardware needed) — what remains is him
  driving it. See §9 and §10.
- **Two SpaceMice are now connected, and the ambiguity is SOLVED — by asking the hardware.** Both report an
  empty serial, so select-by-serial does not transfer from the CAN adapters. They differ only in USB
  `port_numbers` — `(1,3)` and `(1,4)` — which hidapi does not expose, and which tell a human nothing about
  which puck is under which hand. So `pick_device_by_wiggle()` opens all of them and uses **whichever one the
  operator moves.** Unambiguous, needs no config, survives replugging into any port, and costs five seconds.
- No git remote. Everything exists only on this Mac.

---

## 9. Four defects found by READING, 2026-08-10 (session 3) — no hardware involved

Nothing was plugged in for any of these. Each was found by checking the code against what the docs claimed
about it, which is worth noting on its own: **the bench is not where the cheap defects are.**

**1. ⛔ PARK with `--no-gripper` would have released a raised arm.** `config/park_pose.json` holds **7**
joints; `--no-gripper` builds a **6**-DoF robot. `park_target - measured` on mismatched shapes raises
`ValueError` — and that exception escaped the control loop, **skipped the "the arm is HOLDING, press g or d"
consent flow**, and fell into `finally`, which disables the motors. A raised arm sags. The path matters:
`--no-gripper` is exactly the escape hatch the gripper instructions tell you to fall back to, *so the
fallback was the broken one.* Symmetrically, a pose saved **in** a no-gripper session had 6 entries and broke
the next 7-DoF session. Fixed in `yam_robot.park_target_from()` — start from the measured pose and overlay
only the joints the saved pose carries, so no target is ever invented for a joint we know nothing about.
Tested: `scripts/test_park_target.py`.

**2. PARK was the one motion path that bypassed the gripper clamp.** It commanded the saved jaw value
directly. Harmless with the pose saved today (0.0366, inside the band) — but `s` saves wherever the jaws
happen to be, so saving with the jaws on a stop would later drive them back onto that stop and **hold** them
there. That is the stall condition from §4 rule 1, reachable through the one door the guard did not cover.

**3. ⭐ The gripper thermal test could not detect the thing it tested.** The status line printed only
`hottest` — the max over all seven motors. Motors 2/3 carry the arm's 4.3 kg and sit at **41-42 °C** in
normal equilibrium, while an idle motor 7 is **31-36 °C**. So motor 7 climbing 33 → 41 °C was **entirely
hidden inside the `max()`**, and "watch `hottest` plateau" agreed with the claim it was supposed to be able
to refute. §0's own rule, missed in the one place it was load-bearing. The jaw temperature is now printed
separately (`jaw NN°C`), and the session's peak jaw temperature is reported at exit.

**4. A warn-and-continue, in the exact wording of the rule against warn-and-continue.** A second
stale-limit check in `teleop_session.py` compared the raw jaw position against the **unshifted** limits from
the file, so it re-flagged precisely the cases `frame_correct_gripper_limits()` had legitimately reconciled:
at the measured raw **−1.380** it printed *"STALE GRIPPER LIMITS … re-run calibrate_gripper"* while the frame
was correct and the jaws normalised to **0.3005**. It then continued. Two harms, neither hypothetical: a real
warning and a false alarm became indistinguishable, and the remedy it advised is a routine that drives the
jaws into both mechanical stops. Deleted — `build_robot()` already gates this twice, better, and *before* any
control loop starts. **A duplicated weaker check is worse than no check**, because it launders the strong one.

**Also fixed:** the PARK progress report was an `elif` on the motion branch, so one cycle in every hundred
sent no command at all. Benign — the chain's own 250 Hz thread holds the last target — but not what the code
said it did, in the one mode a human watches rather than steers.

> ### The generalisation
> All four are the same shape: **a guard, a test or a message that was written once and then not re-derived
> against the thing it guards.** The clamp existed and PARK went around it. The refusal existed and a weaker
> copy undermined it. The temperature monitor existed and aggregated away the signal. ⭐ **Ask of every
> guard: what is the path that reaches the hazard without passing through you?**

---

## 10. The world frame, measured rather than assumed — 2026-08-10

`CartesianTeleop.step()` integrates the twist in the **world** frame (deliberately — body frame is named in
`src/teleop.py` as a later choice, not an ambiguity). What the world axes physically *are* had never been
checked, and `scripts/map_axes.py` prompts with them, so a wrong label would make the whole tool lie.

Measured in simulation, integrating a unit twist per component from the real saved park pose:

```
gravity                                  (0, 0, -9.81)        => +Z is up
joint 1 (base_yaw) rotates about world Z => the arm stands Z-up
twist [0.05,0,0] for 1 s  -> tcp moved [+0.0499,  0,       0     ]
twist [0,0.05,0] for 1 s  -> tcp moved [ 0,      +0.0499,  0     ]
twist [0,0,0.05] for 1 s  -> tcp moved [ 0,       0,      +0.0498]
roll / pitch / yaw        -> rotation about exactly that world axis,
                             tool point drifted <= 0.3 mm over 17 deg
```

| motion | world | meaning |
|---|---|---|
| `X` | `+X` | horizontal, straight out from the base at `base_yaw = 0` |
| `Y` | `+Y` | horizontal, 90° left of +X seen from above |
| `UP` | `+Z` | straight up, away from the table |
| `ROLL`/`PITCH`/`YAW` | about `+X`/`+Y`/`+Z` | the tool turns; **the tool point stays put** |

⭐ **Consequence worth having: a wrong rotation sign twists the wrist in place rather than flinging the
gripper across the desk.** That refines ROADMAP step 4's caution — rotation is the *less* dangerous sign to
get wrong, not the more.

⚠️ **Deliberately NOT claimed: which way is "forward" or "left".** That depends on how the arm is physically
turned on the desk, which no file in this repo records. Inventing a label would be exactly the confident,
plausible, wrong answer §0 is a list of. `map_axes.py` therefore describes the operator's own gesture back
to them from the reading, and never asserts a gesture-to-axis correspondence that has not been measured.

---

## 11. ⛔⭐ THE ARM FELL — 2026-08-10, session 3, and it was caused by advice in this repo

Three failures in one attempt. All three are mine, and the first is the important one.

### 11.1 `--no-gripper` silently breaks gravity compensation. The arm falls.

**What Julien saw.** He ran `teleop_session.py --yes --arm B --no-gripper --no-rotation`, which starts in
GUIDE. His words: *"only the lowest motor… was in weightless mode, and all of the other motors were turned
off. And therefore it just fell forward because the bottom motor didn't hold it in place."* The status line
read a calm `hottest 35°C` for **33 seconds** while the arm sank to its own stops (`q [0.21, 0., 0., …]` —
joints 2 and 3 at their zero limits).

**The mechanism, proven in simulation, not guessed:**

1. `GripperType.NO_GRIPPER` does **not** merely leave motor 7 unenabled. It swaps the *dynamics model*
   `get_yam_robot` uses for gravity compensation (`get_robot.py:186`, `combine_arm_and_gripper_xml`).
2. The bare arm XML gives its terminal body `mass="1e-6"` — **one microgram** (`yam.xml:38`). The real mass
   arrives by merging the gripper XML. Summing `linear_4310.xml`: `0.553219 + 0.0710042 + 0.0710042 =`
   **0.695 kg**, at the far end of the arm.
3. `zero_gravity_mode=True` sets **`kp = 0`** and commands zero torque (`motor_chain_robot.py:241`), so
   `motor_torques = joint_commands.torques + g * gravity_comp_factor + friction_comp` reduces to `g` alone
   (`:366`). **There is no position term to absorb a modelling error.**

Measured gravity torque at the saved park pose:

```
model mass          WITH gripper 4.987 kg      WITHOUT 4.292 kg     (missing 0.695 kg)
gravity torque WITH    [-0.00, -4.81,  6.34,  1.34, -0.07, -0.00] Nm
gravity torque WITHOUT [-0.00, -2.67,  3.88,  0.49, -0.00,  0.00] Nm
shortfall              [ 0.00, -2.14, +2.47, +0.85, -0.07, -0.00] Nm
                                       ^^^^^ joint 3 (elbow_pitch): 39% short
```

39% of the elbow's holding torque, unopposed. The arm folds forward. **Julien's observation was exactly
right, and his interpretation was nearly right:** the other motors were not off, they were commanded with
`kp = 0` and an under-computed gravity torque, which feels identical. Joint 1 felt free because `base_yaw`
rotates about the vertical and gravity never loads it — in *any* mode.

**⛔ THE RULE THAT FOLLOWS: `--no-gripper` is not a safe subset of normal operation. It is a different, less
accurate robot.** It was reached for as "the smallest possible experiment", and it is the opposite: it
removes the one thing that must not be removed.

**Fix:** `build_robot()` now passes `ee_mass=GRIPPER_MASS_KG` (0.695) on the no-gripper path. Worst residual
falls **2.465 → 0.188 Nm** (3% of the elbow's requirement), verified in simulation.
⚠️ **`ee_inertia` cannot be used** — the SDK emits an `ipos` attribute MuJoCo rejects (*"Schema violation:
unrecognized attribute: 'ipos'"*, it should be `pos`). That is a bug in the vendored tree, so the
centre-of-mass offset stays uncorrected and 0.188 Nm is the residual we cannot remove.

**Also fixed: GUIDE now prints live drift** from wherever it went weightless. The cause is gone, but the
instrument should have existed anyway — nothing on screen was measuring the one quantity that was failing.
*Same lesson as §9.3's jaw temperature: a readout must show what can fail, not what looks calm.*

> **Considered and rejected: an automatic sink-detector that forces GUIDE → HOLD.** In GUIDE, motion is
> *expected* — Julien is pushing the arm by hand — and there is no signal that distinguishes "he is lowering
> it" from "it is falling". Every threshold either false-fires during legitimate hand-guiding or is too slow
> to matter. Showing the number and fixing the cause beats automating a judgement the code cannot make.

### 11.2 The remap mode destroyed the hand-dialled axis map

The MAP mode written earlier that same session **bound whichever motion was selected the instant any clear
puck deflection arrived, and then auto-advanced to the next motion.** So the entirely natural act of *"let me
see what this does"* rewrote the map, cascading through several motions, each binding stealing a puck axis
and unbinding its previous owner. Then the session **saved it unconditionally on exit.**

Recovered from the terminal:

```
axis map saved → config/spacemouse_map.json
  — Y←roll− — ROLL←pitch− — YAW←yaw+
  ⚠️  UNBOUND, the arm will not perform these: X, UP, PITCH
```

His hand-dialled `[1, -1, -1, 1, 1, 1]`, produced on real hardware, was overwritten. It survived **only
because the file happened to be committed to git.**

**Three compounding faults, all now fixed:**

| fault | fix |
|---|---|
| deflection *edited* the map | **deflection observes, keys edit.** Nothing in CONTROLS mode writes the map except a keypress |
| auto-advance cascaded one wiggle into many bindings | there is no cursor and no advance any more |
| unconditional save on exit | saved **only if changed**, and the previous contents are copied to `config/spacemouse_map.prev.json` first |

> ⭐ **The generalisation, and it is the same as §9's:** *ask what path reaches the hazard without passing
> through your guard.* Here the hazard was data loss and there was no guard at all — because "explore" and
> "commit a change" had been collapsed into the same gesture.

### 11.3 `mjpython --view` cannot start, and I claimed it worked

```
failed to dlopen '/…/.venv/bin/python3': Library not loaded: @rpath/libpython3.12.dylib
```

`mjpython` is a launcher app bundle that dlopens the venv's interpreter, and the uv-managed CPython does not
place `libpython3.12.dylib` anywhere on mjpython's rpath. **It is present**, at
`~/.local/share/uv/python/cpython-3.12.12-macos-aarch64-none/lib/`, so one environment variable fixes it:

```bash
DYLD_FALLBACK_LIBRARY_PATH="$HOME/.local/share/uv/python/cpython-3.12.12-macos-aarch64-none/lib" \
  uv run mjpython scripts/teleop_sim.py --view
```

Verified: `mjpython OK, mujoco 3.11.0`.

⛔ **The process failure matters more than the fix.** I recommended this command after checking that the
`mjpython` *binary existed* — `ls` — and reported it as "verified". Presence is not function. That is
**"verify the consequence, not the mechanism"** (§0, and the founding-session lesson), violated in the same
turn that quoted it. *An `ls` is never verification of behaviour.*

---

## 12. CONTROLS mode — Julien's design, and why it replaced mine

Mine was: hold the arm still, select a motion, gesture to bind it. It was wrong for a reason he identified
immediately: **you cannot decide a direction is wrong until you have watched the arm go that way**, and the
same document that admits "+X and +Y are horizontal but which one points away from you is not recorded
anywhere" then asked him to bind X from memory. Incoherent.

His design, in his words: *"similar to teleoperate, just move the space mouse in different directions and
only the strongest direction is actually moved, and then I can press some key which reverses the direction of
that specific control."*

**Why it is better, point by point:**

| | his design | mine |
|---|---|---|
| the arm | **moves** — you see what each direction does | frozen; nothing to observe |
| cross-talk | **only the strongest axis is applied**, so the motion is attributable | a firm push moves 3 axes diagonally |
| what you must know | nothing — push and look | which motion index you meant, in the abstract |
| what a deflection does | **observes only** | *edited the map* — the data-loss bug in §11.2 |
| the edit | an explicit key on "the control I just used" | a cursor and an auto-advancing wizard |

**Implementation.** `isolate()` keeps only the largest-magnitude axis, with 1.3× hysteresis so two near-equal
axes cannot make the arm jitter between two motions. The session remembers the last axis that actually moved
— **with no timeout**, so `f` still works after the puck has sprung back to centre and his hand has left it.
`f` resolves puck axis → motion via `AxisMap.motion_driven_by()` and flips that motion's sign. `1`-`6`
reassign that same control to a different motion, taking **the direction he was last pushing** as the new
motion's positive sense, so "push the way you want it, then name the motion" reads the same as a gesture.

Speed is **half** teleop's (`CONTROLS_SCALE = 0.5`): it is the mode you enter with a mapping you have not yet
confirmed, so a wrong direction should be a slow wrong direction. Everything below the twist is the
*existing, hardware-proven* chain — IK, per-cycle joint-step clamp, joint limits, workspace box, `SafeRobot`
rate limiter. **CONTROLS mode adds a twist source, not a control path.**

### 12.1 What using it on the arm changed — 2026-08-10

It worked, and Julien's live map came out a genuine permutation:
`X←y+  Y←x+  UP←z−  ROLL←pitch+  PITCH←roll+  YAW←yaw−`. **Sign flips alone could not have expressed that**,
so the permutation half earned its place rather than being speculative generality. Two things came back:

**1. `1`-`6` now SWAP instead of steal-and-unbind.** *"Instead of only changing it to that specific thing and
then just deleting the other one, it would just swap whatever was on the other… that will make it a lot
easier."* He is right: the commonest edit is **two controls in each other's places**, and stealing left an
orphan he then had to notice and re-bind, with a motion silently dead in between. A straight exchange is also
an **involution** — the same key again undoes it — and preserves injectivity by construction. The sign
travels with the puck axis, because the unit being exchanged is the whole control (which axis, pushed which
way), not just the wiring.

**2. `,` and `.` were missing from the CONTROLS key handler**, so rotation speed could not be changed there
at all while linear could. The keys had been copied from the drive-mode handler and the second pair dropped.
⚠️ **The reason it took a hardware session to notice is the interesting part:** the status line showed only
the *resulting* speed of the active axis, so a key that did nothing was indistinguishable from a key that
worked. Both scales are now printed continuously. *Same shape as the jaw temperature and the GUIDE drift —
a readout must show the quantity a key is supposed to change.*

---

## 13. Bimanual prerequisites — built 2026-08-10, and one gap that would have bitten silently

**Per-arm axis maps (`AxisMapStore`).** Shared by default — Julien's *"probably the same, actually"* — with an
override created only by an explicit `--fork-map`. Defaulting to a map per arm would let the two silently
diverge, after which a puck that feels wrong on G is indistinguishable from a map that was never copied
across. ⛔ **Whatever reads it must print which scope it is editing**; tuning G and silently changing B
is the same shape as the bug in §11.2 — an edit whose blast radius was larger than the operator believed.
A legacy flat file still loads as the shared map, so nothing hand-dialled is lost.

**⛔ `pick_device_by_wiggle()` could assign the same puck to both arms.** Called twice without an `exclude`,
its single-device shortcut returns that device unconditionally, and with two attached nothing stopped the
operator moving the one they had already assigned. **Both failures are silent, and the symptom — two arms
following one hand — reads as a control bug rather than a device-assignment bug.** Exactly the class of the
CAN adapter chosen by index that silently retargeted the wrong robot (§0 #5). Now takes `exclude=[path, …]`
and says plainly when no unassigned puck remains.

**Two-arm IK, measured (it never had been):** `0.100 ms` mean per cycle, p99 `0.110 ms`, for two
`CartesianTeleop.step()` calls. Against a 10 ms deadline with ~6.2 ms of CAN for 14 motors, that is ~3.7 ms
spare. **IK is not the bimanual bottleneck**, and the assumption that it might be is now retired.

---

## 14. A test whose premise expires

`test_backward_compatible_with_hand_dialled_file` loaded `config/spacemouse_map.json` and asserted it was
still sign-only. It failed the moment Julien legitimately saved a permutation — **correctly**, but for a
useless reason: its own guard fired, not the property it was protecting.

**The property is about the file FORMAT, not about whatever is currently in the file.** It now runs against a
pinned fixture string, and a separate test checks that the live config loads, is injective, and round-trips.

⭐ **Generalises: a test whose subject is a file the user edits has a moving target.** Pin the fixture; test
the live artefact only for properties that must hold *whatever* it contains.

---

## 15. ⛔⭐ PARK was a treadmill — it commanded the measurement

**Julien reported PARK broken twice**, after it had been "fixed" once. The first round of fixes was real (a
7-vs-6 length crash, a bypassed gripper clamp, a skipped command cycle) but **none of them was why it did
not move**. This is.

```python
robot.command_joint_pos(q + np.clip(park_target - q, -stepmax, stepmax))   # q = MEASURED
```

It re-anchored to where the arm actually was, **every cycle**. So the commanded position was never more than
one step ahead of reality: `PARK_SPEED × dt = 0.40 × 0.01 =` **0.004 rad, about 0.23°.**

A position controller makes torque from the *error* between command and measurement. Capping that error at
0.23° caps the torque at `kp × 0.004` — not enough to overcome static friction plus 4.3 kg of arm. So the arm
does not move; because it does not move the measurement does not change; and because the measurement does not
change, the next cycle commands the same 0.23° offset. **A treadmill.**

⛔ **And it fails in this stack's signature style: it printed `parking… 1.2 rad to go` indefinitely, raised
nothing, and read as a controller that was merely slow.**

**TELEOP never had the bug, and that contrast is the proof:**

```python
step = q_target - prev_q                                   # prev_q = last COMMAND
q_target = prev_q + np.clip(step, -MAX_JOINT_STEP, MAX_JOINT_STEP)
```

It integrates from the **command**, never from the measurement, so when the arm lags its command keeps
advancing, the error grows, and the torque grows with it until the joint moves. PARK was the odd one out.

**Fix:** `advance_park_command()` — a trajectory that runs ahead of the arm as far as it needs to.
`SafeRobot.max_lag` (0.25 rad) is what stops it running away, which is exactly the right place for that guard
and was already there. Completion is judged on the **measured** pose, never the command, because the command
always arrives first. A **stall detector** now says so and holds if the measured error stops improving for
4 s — the silence is precisely how this survived two sessions.

`scripts/test_park_target.py` includes `test_the_old_formula_provably_could_not_converge`, which reproduces
the old expression and asserts that after 20 simulated seconds the command is still 0.004 rad from a stuck
arm. **The diagnosis is mechanical, not a story.**

> ⭐ **The generalisation, and it is the deepest one this project has produced so far:**
> **a controller must command a trajectory, not the thing it is measuring.** Feeding the measurement back
> into the command caps the error, and the error *is* the actuation. Anywhere you see
> `command = measured + something_small`, ask what makes it converge — often nothing does.

---

## 16. A refusal that named the wrong arm

G's first run refused correctly — it has never had its jaws calibrated, `config/gripper_limits.json`
holds `B` only — and then printed:

```
Run this once:  uv run scripts/calibrate_gripper.py --yes
```

**No `--arm G`.** Following that literally drives **B's** jaws into both mechanical stops, while the arm
you were trying to start stays uncalibrated and the same refusal comes back. The other two refusals in
`yam_robot.py` both interpolate `--arm {arm}`; this one did not.

⛔ **A remediation message that names the wrong target is worse than no message: it converts a clean refusal
into a wrong action.** Now fixed, and it also offers `--no-gripper` as the alternative.

---

## 17. The puck buttons drive the gripper

Julien: *"there are two buttons on the left and the right. One could be open, one could be closed… and then
pressing whatever the switch button was, I think f, could then switch it back."*

⛔ **The masks are learned by pressing, never assumed.** Which physical button sets which HID bit has never
been measured on this unit, and "assumed an identity that was never checked" is the single most repeated
failure in this file — the CAN adapter by index, the puck by index, the gripper limits in the wrong frame.
`b` in CONTROLS mode asks for OPEN, then CLOSE, and refuses to give one button both jobs (a button that both
opens and closes is a coin flip, not a control).

⭐ **`f` reverses the buttons, because `f` already means "reverse the control I just used".** If that was an
axis it flips the sign; if it was a button it swaps open and close. One rule, no new vocabulary — Julien
reached for `f` unprompted, which is the sign the rule is the right one. Like `swap()`, it is an involution.

Buttons are **hold-to-move** at 0.6 normalised units/s (~1.6 s for the full stroke), not step-per-press: a
gripper wants squeeze-and-hold. `o`/`c` remain as keyboard steps. The assignment lives in the axis map, so it
is **per-arm** for free, and an unset button writes no key at all rather than a `null`.

### 17.1 …and it shipped broken, in the most annoying possible way

Julien, next session: *"the space mouse buttons that should control the gripper don't do anything. And then
it says press b to set the gripper, and then b does nothing either."*

`b` was handled **inside the CONTROLS-mode branch**, while the *"press b to set the gripper buttons"* hint
printed from the button-reading block, which ran in **TELEOP as well**. So in TELEOP the hint appeared, `b`
fell through to the catch-all, and nothing happened. Buttons were also read only in teleop/map, so they were
dead in GUIDE and HOLD entirely.

⛔ **A message that tells you to press a key which does nothing where you are is the same defect as the
refusal that named the wrong arm (§16): the text is right, the context is wrong, and it costs a session to
find out.** Button assignment is a property of the **device**, not of the arm's mode, so it now sits above
the mode dispatch and works everywhere. The puck is read every cycle in every mode — which also stops HID
reports queueing up during GUIDE/HOLD and arriving in a burst at the next mode switch.

---

## 18. ⛔⭐ "It moves very incoherently in weird positions" — a pure rotation moved the tool point 44 cm

Julien, 2026-08-11: *"the inverse kinematics being weird and not working as intended, specifically when the
robot gets into weird positions, and then it starts moving very, very incoherently."*

### The first hypothesis was wrong, and measuring it saved a day

The obvious story — *near a singularity the Jacobian blows up, joint velocities explode, and the per-joint
`MAX_JOINT_STEP` clamp distorts the direction* — is **refuted**. Measured: mink's `lm_damping` keeps the
requested joint velocity around **0.45 rad/s against a 1.5 rad/s clamp**, so the clamp never binds, and at a
genuine singularity (`σ_min = 0.0001`, folded near the base) the arm asks for **0.03 rad/s** — it barely
moves *at all* rather than blowing up. Direction error from clamping: **0.00°** at every pose tested.

### What is actually happening

Reproducing the real control loop — IK, joint-step clamp, joint-limit clamp, workspace box — and commanding
**pure roll at 0.6 rad/s** from the park pose:

```
   t     |target-EE|    |IK q - commanded q|    tool point moved
 4.0 s     0.0004 m          0.0000 rad              0.000 m
 8.0 s     0.238  m          0.0800 rad              0.238 m
12.0 s     0.074  m          0.0800 rad              0.290 m      (peak 0.44 m)
```

**A pure rotation command translated the tool point 44 cm.** The chain:

1. A wrist joint reaches its limit — the tight ones are ±1.5708 rad. *(Confirmed by the second column: the
   gap between IK's internal joint state and the commanded one pins at exactly **0.0800**, which **is**
   `JOINT_LIMIT_MARGIN`. A joint is clamped at the margin while the IK believes it is at the true limit.)*
2. `CartesianTeleop.step()` advances `self.target` by the twist **unconditionally**. It never asks whether
   the arm followed, so the orientation goal runs arbitrarily far past anything reachable.
3. The QP now holds an impossible orientation target, and `position_cost` (1.0) and `orientation_cost` (0.5)
   are **traded against each other** — so it starts moving the **tool point** to partially satisfy a rotation
   it can never achieve.
4. The workspace box re-clamps translation, fighting the orientation task — hence the oscillation above.

### Two fixes, and the second one is the surprise

**(a) Anti-windup on the goal.** `_limit_lead()` bounds how far `target` may run ahead of the pose actually
achieved — 0.05 m and 0.25 rad, separately, because translation and rotation fail independently (the
workspace box already happened to bound translation, which is why only rotation misbehaved). **This is
`SafeRobot.max_lag` one layer up**, and it needs no model of *why* the arm cannot follow: joint limit,
singularity, rate limiter or an obstacle all present as an unclosable gap, and bounding the gap bounds them
all. Verified bounded, not merely slowed: the worst lead is **identical at 10 s and 80 s** (0.250060 rad).

**(b) `orientation_cost` 0.5 → 0.05.** Anti-windup alone cut the wander from 0.44 m to 0.40 m — barely
anything — because a *persistent* unreachable orientation error still bleeds into position. The cost ratio is
the real lever, and the measurement is counter-intuitive:

| `position:orientation` | pure-roll tool wander | rotation achieved |
|---|---|---|
| **1.0 : 0.5** ← the old default | **0.443 m** | **7.9°** |
| 1.0 : 0.2 | 0.034 m | 129.5° |
| **1.0 : 0.05** ← now | **0.002 m** | **134.6°** |
| 1.0 : 0.01 | 0.000 m | 18.2° |

⭐ **The old default was the worst of both worlds: it wandered 44 cm *and* achieved the least rotation.** A
*higher* orientation cost produced *less* rotation, because the effort went into satisfying an unreachable
orientation by translating, which drags the arm into a configuration that can rotate even less. Verified at
three starting poses; small rotations are unaffected and translation reach is unchanged (0.319 → 0.320 m).

> **The priority this encodes, in one line: never sacrifice where the tool IS to chase where it POINTS.**
> A wrist that cannot turn should simply not turn — it should not drag the whole arm across the desk.

**Also added:** the TELEOP status line now prints `⚠️ STUCK lead 5cm/14°` when the goal is pinned near its
limit. An arm that cannot follow used to present *only* as an arm behaving strangely.

⚠️ **`scripts/test_teleop_ik.py` reproduces the whole loop, not just `CartesianTeleop`** — the bug only
appears when the clamps interact with the IK, so testing the class in isolation would have missed it
entirely. One test deliberately restores `orientation_cost=0.5` and asserts the wander **comes back**: if
that ever stops failing, the cause has moved.

---

## 19. ⭐ Driving from the camera's point of view is a FRAME question, not a camera question

Julien, 2026-08-11, wanting to use the C920 as a stand-in for the wrist cameras that have not arrived:
*"I would provisionally like to use the Logitech camera… mounted on one of the arms as a test so that I can
try to learn to control the arm from the point of view of the camera to get the tilts right and stuff."*

**The interesting half of that request has nothing to do with cameras.** Until now every twist was
interpreted in the **world** frame — and `teleop.py` flagged the consequence from the very first version:

> *"World-frame integration: rotation pre-multiplies, so a twist means the same thing regardless of how the
> gripper happens to be oriented. Body frame would be more natural to a hand holding the puck, and is a
> deliberate later choice — not something to leave ambiguous now."*

This is that later choice, and the camera is what forces it. Looking **at** the arm, world frame is right:
"forward" is a fixed direction on the desk, predictable, and a wrong sign only nudges. Looking **through** a
wrist camera, world frame is wrong the moment you tilt: "push forward" then means forward *in the image*,
and the image turns with the wrist. **That is exactly what "get the tilts right" is.**

So `CartesianTeleop` gained `frame = world | tool | camera`, applied as `R_wf @ v` and `R_wf @ ω` before the
existing world-frame integration — which leaves the anti-windup and the workspace box untouched. `v` cycles
it live, and that is safe without a resync because a twist is a *velocity*: a frame change alters the
interpretation from the next cycle and leaves no stale cached state, unlike a mode change.

⛔ **`camera` is the MODELLED D405 mount and is WRONG for the hand-mounted C920.** The MJCF puts the D405 on
the flange at a 25° cant with `+Z` along the optical axis; a webcam cable-tied on by hand shares none of
that, and nobody has measured where it actually sits. **Use `tool` for the stand-in**, mount the camera
roughly looking the way the gripper points, and dial the remainder out with the axis map. Using `camera` for
an unmeasured mount would be inventing a transform — the single most repeated failure in this file.

### Camera capture: two facts worth keeping

**Almost all webcam "lag" is queued frames, not decode time.** A naive `read()` returns the *oldest* frame
in the driver's queue. `scripts/camera_view.py` grabs repeatedly (cheap, no decode) until the queue is dry
and decodes only the last one. It also sets **MJPG before the resolution** — left in uncompressed YUY2 the
C920 cannot fit 1080p through USB2 and collapses to a few fps, which *reads* as latency and is a bandwidth
problem. `--measure` reports the real frame interval so the claim is checked rather than asserted.

⚠️ **macOS gates camera access, and no code change fixes it.** First run prints
`OpenCV: not authorized to capture video (status 0)` until the permission is granted to the app running the
terminal — **System Settings → Privacy & Security → Camera**. Encountered 2026-08-11; the agent cannot
grant it.

⚠️ **OpenCV on macOS selects cameras by INDEX**, which collides with this repo's hard rule against selecting
hardware by index (§0 #5). AVFoundation offers no name-based alternative, so rather than pretend, `--list`
makes the ambiguity visible and suggests the honest disambiguation: cover the arm-mounted camera with a hand
and re-run — the index whose mean brightness collapses is the one on the arm.


---

## 20. ⭐ "It lags at high speed" is a singularity problem, not a speed problem

Julien, 2026-08-11: *"at high speeds the arm takes longer to follow the path that it's been told to move…
I can only really control it at speeds of less than half a meter per second."*

**Two hypotheses were tested and refuted before the real one.** It is not a constant per-speed cost, and it
is not a startup transient. Pushing +X at 0.25 m/s from the home pose, joint speed by cycle:

```
  cycle   1      0.66 rad/s          <- first cycle, fine
  cycle   2-10   0.67 rad/s          <- not a transient
  cycle  50-150  3.30 rad/s          <- it ESCALATES with time
```

Tracing it against `sigma_min`, the smallest singular value of the Jacobian — the standard measure of how
close the arm is to a configuration where some direction of motion becomes unreachable:

| cycle | joint rad/s | EE moved | `sigma_min` |
|---|---|---|---|
| 20 | 0.68 | 0.05 m | 0.170 |
| 100 | 1.34 | 0.25 m | 0.121 |
| 140 | **2.93** | 0.35 m | 0.048 |
| 180 | 0.12 | 0.38 m | **0.005** ← stalled |

**The same tip speed costs 0.68 rad/s in the middle of the workspace and 2.93 rad/s near full reach**, and
then the arm stops entirely. `SafeRobot` caps commands at 1.0 rad/s, so past that point the command is
throttled, the arm falls behind, and it reads as latency.

⭐ **So speed is not the cause — it only decides how quickly you arrive at the part of the workspace where
this happens.** That reframing matters, because the obvious fix (raise the cap) would not fix it. It would
move the wall a little further out and cost the guard that makes a wrong motion catchable, on a rig with
**no e-stop**.

**The fix is to ask for less.** `CartesianTeleop._apply_speed_scale()` measures the joint speed the solver
just requested and, if it exceeded `max_joint_rate` (0.9 rad/s, deliberately just under SafeRobot's 1.0),
scales the twist by exactly that ratio. Tip speed and joint speed are locally proportional, so it lands on
the allowed rate in a single step. Recovery is slower than reduction — 5% per cycle, ~0.2 s to full — because
reacting instantly in both directions oscillates at the boundary, which would feel worse than the lag.

**Measured, over 200 cycles (2 s):**

| commanded | cycles over the cap, before | after | worst lead |
|---|---|---|---|
| 0.12 m/s | 0 | 0 | 0.9 mm |
| 0.25 m/s | **86** | **0** | 5.1 mm |
| 0.40 m/s | 98 | 1 | 7.0 mm |
| 1.00 m/s | 42 | 2 | 7.3 mm |

At 0.25 m/s the rate limiter had been intervening on **43% of all cycles**. It now never does, and the
command stays within 7 mm of the arm instead of pinned at the 50 mm anti-windup bound.

⚠️ **The throttle costs time, not workspace.** A first test compared reach after a fixed number of cycles
and failed — correctly, since a throttled arm is behind at any given moment. Given time both converge on
**exactly 0.5194 m**. The test now asserts that distinction explicitly, because "it got slower" and "it can
no longer reach as far" are very different regressions and only one of them is acceptable.

Low speeds are untouched: at 0.12 m/s the scale never leaves 1.0, so normal driving is unchanged. The status
line prints `⚠️ SLOWED to N% (near the reach limit)` — without it, the throttle would present as
unexplained sluggishness, which is the same class of silent-failure this file exists to catalogue.
