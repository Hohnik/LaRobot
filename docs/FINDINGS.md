# Findings — everything learned about this rig, and how

> **Purpose.** `README.md` says what is true now; `ROADMAP.md` says what to do next and why. **This file is the latent knowledge** — the things that were expensive to learn, that no file records implicitly, and that a fresh agent would otherwise re-derive at cost or, worse, get wrong the same way.
>
> Everything here was **measured on the real hardware on 2026-08-10** unless it says otherwise. Where a number appears, it is a number that was actually read off the arm.

---

## 0. The one thing to internalise before touching anything

> ### ⛔ **This stack fails by lying, not by crashing.**

Nine separate defects in one day produced **confident, plausible, wrong answers** and **not one raised an exception**:

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

**Practical rule: check values for plausibility, not merely for the absence of an exception. And prefer a test that could falsify the claim over one that merely agrees with it.** #6 was only settled by finding a measurement (per-unit `inertia`) that would have *differed* if the claim were wrong.

---

## 1. The hardware, as measured

**Two YAM arms, each with its own CANable adapter on its own CAN bus.** Both use motor IDs **1-7**, so **nothing inside a CAN frame distinguishes the arms.** The adapter serial is the only discriminator.

| | serial | notes |
|---|---|---|
| `B` | `2081337C594E5018` | everything up to ~11:00 was this one |
| `G` | `20593383594E5018` | plugged in mid-session; **its adapter enumerated FIRST** |

⛔ **Never select the adapter by index.** `chain_channel('B')` returned `gsusb1` at 10:58 and `gsusb0` at 11:45 — **the enumeration order changed twice within one session.** `src/yam_can.py` resolves by serial and re-verifies after opening.

**Proof the two arms are genuinely distinct** (not one arm read twice): `inertia` is per-unit calibration data burned into each motor. All seven differ between the arms — e.g. joint 1: `1.7169109e-05` vs `1.6964389e-05`.

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

**How the models were identified without energising anything:** the `gear_ratio` register. The Damiao part number *is* the gear ratio — DM43**40** reports 40.0, DM43**10** reports 10.0. `sw_ver` partitions identically as a cross-check. That layout is `yam_v1.yml` exactly ⇒ the arm is `yam` / `yam_pro` / `yam_ultra_v1` (indistinguishable over CAN) and **not** `yam_ultra_v2`.

⚠️ **Decoding a motor with the wrong type does not raise, it mis-scales.** Position is safe (±12.5 rad on both), but velocity reads **3× high** and torque **2.8× LOW** — under-reading force on the three heaviest joints, which is the dangerous direction.

⚠️ **The URDF limits are directly comparable to raw motor positions** only because `get_robot.py` sets `motor_offsets = [0.0] * n` and `yam_v1.yml` sets every direction to `+1`. Re-check if either changes.

### The parked pose is mechanically supported

`shoulder_pitch` and `elbow_pitch` read ~0.000, and the URDF puts **both of their mechanical limits at 0**. The arm rests against its own stops rather than balancing, so energising from the parked pose cannot release a joint into a fall. **This is why every first-contact test started there.**

---

## 2. macOS: why every patch in `src/yam_can.py` exists

I2RT's stack assumes Linux + SocketCAN throughout. Four things had to be solved, each for a distinct reason.

**1. `bustype` is an argument, not a constant.** `CanInterface.__init__` takes `bustype="socketcan"` and passes it to `can.interface.Bus`. So `bustype="gs_usb"` reaches python-can untouched and the CANable works over libusb. ⚠️ **But the layer the docs point you at cannot do this**: `get_yam_robot()` → `DMChainCanInterface` picks the bus with `if "can" in channel:` and hardcodes socketcan, with no override (`dm_driver.py:409`). Hence `chain_channel()` returns **`gsusb<N>`** — a name chosen precisely because it fails that substring test — and `patch_dm_driver_for_gs_usb()` rewrites it one layer lower.

**2. `detach_kernel_driver` fails with EACCES on macOS** even though nothing holds the device (measured: `is_kernel_driver_active(0)` is False and `claim_interface(0)` succeeds). Only the `USBError` is suppressed, so a platform where the detach genuinely matters is unaffected.

**3. ⛔ Transmit echoes. Load-bearing.** A candleLight adapter echoes each sent frame back as a send confirmation; python-can marks it `is_rx=False` but **does not drop it**. SocketCAN never loops a frame back to the sending socket, so I2RT has no filter and takes the next frame as the reply. Decoding our own request's `data[4:8]` — four zero bytes — produced a perfect set of zeros from all seven motors, reported as success. Filtered at the bus layer where nothing can bypass it.

**4. Enable/disable desynchronisation.** `_send_message_get_response` retries by **re-sending** the command. A late reply fails the arbitration-id check → the retry puts a *second* enable on the wire → two replies come back → one is consumed, one is left in the buffer and read as the **next** motor's reply → which mismatches → which retries. One hiccup snowballs down the chain and lands wherever the margin runs out. **The varying failure point is the signature** — a genuinely dead motor fails in the same place every time. I2RT's 3 ms inter-motor spacing is ample in-kernel; over libusb each transfer is ~0.45 ms and the margin is thin. **A transport mismatch, not a bug in their code.** Fixed by draining before every enable/disable and retrying the whole exchange after a fuller drain. Reliability went **1/3 → 8/8**. ⚠️ Deliberately **not** applied to `set_control`, which shares the function but runs the 100 Hz loop.

### Throughput — macOS is not the bottleneck

`bench_can.py --cycle --samples 8000`, real 7-motor cycles, 25 s sustained:

```
8000/8000 complete, 0 missed replies
cycle ms: mean 3.121  p50 3.116  p95 3.231  p99 3.318  p99.9 3.576  max 17.771
320 cycles/s sustained
would miss a 100 Hz deadline: 2/8000 (0.03%)
```

**~3× headroom over 100 Hz.** Session 1's "do not fight macOS for the control loop" is **refuted**. Linux remains right for the final rig (RealSense, cuRobo, ABC training are Linux-first) and this says nothing about the loop once cameras and inference compete for CPU — but the specific claim is dead. ⚠️ Measured with register reads, so it is a lower bound.

---

## 3. The gripper

- **Stroke: 6.57 motor-rad ↔ 0.096 m** of jaw travel (`linear_4310.yml`), matching the URDF's two prismatic tips at 0.0469 m each.
- **Measured limits (B, 2026-08-10): `+0.0704 … −5.0528`, usable stroke 5.123 rad = 78% of declared.** Saved in `config/gripper_limits.json`.
- **The jaws began the day parked hard against the `0` stop.** That is why every early jaw command in the positive direction tripped the torque limit within a fraction of a second while negative moved freely.
- ⚠️ **`get_yam_robot()` re-calibrates on EVERY construction** when `gripper_limits` is null, and `detect_gripper_limits` drives each stop and *holds* until the position stops changing. Julien: *"they move really quickly and quite hard… they seem to crash into the ends and then seem to try to push further."* **Passing `gripper_limits_override` is what disables it** (`get_robot.py:223-225`) — which is why `yam_robot.build_robot()` always loads the saved limits.
- There is **no speed parameter**. The routine is torque-controlled, so torque *is* the speed; lowering it lowers both. Ours uses 0.3 Nm vs I2RT's 0.5.
- After calibration the gripper is exposed as a **normalised 0…1 value**, not raw radians.
- ⛔⭐ **THE WORST BUG OF THE DAY: the limits are stored in one coordinate frame and used in another.** `get_yam_robot()` applies a **±2π wrap correction at every construction**, chosen from wherever the motor happens to be sitting at that instant (`get_robot.py:268-274`). `calibrate_gripper.py` builds a `DMChainCanInterface` directly and gets **no** such correction. So whether the saved numbers mean anything depends on the jaws' position when each ran. **The consequence is not a wrong number, it is a cooked motor:** `motor_chain_robot.py:390` force-clips every gripper command into `[min(limits), max(limits)]` *regardless of where the jaws are*. Measured: jaws at **−1.380** with a saved range of **[+1.231, +6.481]** → the gripper was commanded 2.6 rad away and held there against a mechanical stop → **43 °C → 65 °C in five seconds.** **Fix:** `reconcile_gripper_limits()` tries the saved range shifted by 0, +2π and −2π and returns whichever brackets the measured position; if none does, `build_robot` **refuses to start**. Verified: reconciling the failure case yields `[0.1979, −5.0524]` against the morning's independent calibration of `[0.0704, −5.0528]` — **the lower bound agrees to 0.0004 rad.** ⚠️ **Never "warn and continue" on this.** That is exactly what was done, and it is what burned the motor.
- ⛔ **A power cycle also invalidates the saved limits.** Measured 2026-08-10: before the power cycle the jaws calibrated to `+0.0704 … −5.0528`; afterwards they read **+1.6691 rad**, outside that range entirely. The motor's position reference shifts across power. A stale range makes the normalised value fall outside `[0,1]`, so every hold command pushes toward a stop — **which is precisely how motor 7 cooked, twice.** **Re-run `calibrate_gripper.py` after every power cycle.** `teleop_session.py` detects the mismatch and says so.
- ⛔ **Never command the gripper to 0.0 or 1.0.** Those *are* the mechanical stops, and holding a position at a stop is stall torque: full current, no motion, no cooling. Operator-requested values are clamped to **[0.02, 0.98]**. ⚠️ **The clamp is applied only to values the OPERATOR asks for, never to move the jaws to where they already are.** The earlier `[0.15, 0.85]` band was applied *on entering TELEOP*, which meant that if the jaws happened to sit outside it, the session **commanded them to move** the moment teleop began — a motion nobody asked for, into a mechanical stop when the limits were also mis-framed. Entry now takes the jaws exactly where they are.

---

## 3.5 ⭐ The gripper: two 2π frame errors, not a broken mechanism

**Motor 7 was cooked three times on 2026-08-10 and the first three explanations were all wrong.** The real cause is a coordinate-frame mismatch, and Julien is the one who pushed back on giving up — correctly.

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

A normalised position outside `[0,1]` is clipped onto the nearest limit by `motor_chain_robot.py:390`, so the motor is commanded into **a stop it is already past** and pushes at 7.71 Nm indefinitely. Self-reinforcing: 7.7 Nm shoves the jaws further beyond what a 0.3 Nm calibration called the limit.

**`frame_correct_gripper_limits()` applies both shifts.** Verified on the two independent failures:

```
raw +6.3235  →  limits [0.1979, −5.0524]  normalised 0.0300  ✓
raw −1.3800  →  limits [0.1979, −5.0524]  normalised 0.3005  ✓
first calibration of the day, independently:  [0.0704, −5.0528]
```

Two different positions hours apart converging on one range, matching an independent measurement.

⛔ **And it is verified rather than trusted:** `build_robot` reads the normalised jaw position back from the runtime after construction and **shuts everything down before the control loop starts** if it is outside `[0,1]`. A prediction about a frame is exactly the kind of thing that should not be believed on argument.

**Escape hatch:** `--no-gripper` runs the six arm joints only and leaves motor 7 free.

### Lessons that generalise beyond the gripper

- **Two bugs with identical symptoms hide each other.** Fixing one and seeing no improvement is weak evidence that you fixed nothing — it may be evidence that there are two.
- ⛔ **A guard that cannot express a safe command is not a guard.** The stall guard "released" the command to the measured value — which clipped straight back onto the stop. It fired every cycle and changed nothing.
- **Cached raw motor positions are frame-dependent.** Anywhere a position is stored and re-used across process boundaries, ask which wrap correction was in force at each end.

---

## 3.6 (historical) The interim decision to disable the gripper

**Motor 7 was cooked three separate times on 2026-08-10.** Three different fixes were attempted and the first two addressed the wrong layer. This is the true mechanism, and it is the single most important open item.

**The evidence that settled it**, printed by our own stall guard:

```
⚠️ GRIPPER STALLED (+7.71 Nm, not moving) — releasing it to 1.186
```

**`1.186`.** That is the *normalised* gripper position, and it is outside `[0, 1]` — **the jaws sit 18.6% beyond the "fully open" end of their own calibrated range.** `command_joint_pos` clips it back to `1.0`, which maps to the end stop, so the motor is commanded into a stop **it is already past** and pushes at 7.71 Nm indefinitely.

**It is self-reinforcing, which is why every run was worse than the last:** 7.7 Nm of runtime torque shoves the jaws further beyond the limit that a 0.3 Nm calibration detected. The calibration finds a "stop" that the runtime simply pushes through, so each session starts further outside the range than the one before.

**Why the earlier fixes could not work:**
- Clamping the command **above** `command_joint_pos` is bypassed: the vendor's clip is **below** it (`motor_chain_robot.py:390`).
- Releasing the command to the *measured* value is a no-op: `1.186` clips to `1.0`, the same stop. **A guard that cannot express a safe command is not a guard.**
- The ±2π reconciliation (§3) is correct and still necessary, but it fixes limits in the wrong **frame**, not limits of the wrong **size**. Both faults are real and independent.

**Current state: `GripperType.NO_GRIPPER` is the default.** Motor 7 is never enabled and never commanded; its 400 ms timeout leaves it damped and free. The six arm joints are entirely unaffected and teleop works fully. `--gripper` opts back in.

> ### ⭐ To fix it properly (the next session's job)
> **Calibrate at the torque the runtime actually uses, not at 0.3 Nm.** The runtime reaches 7.7 Nm, so a limit found at 0.3 Nm is not a limit — it is where the jaws stop being easy to move. Options, in order of preference:
> 1. Calibrate at ~2-3 Nm and **inset** the saved range by a margin, so the runtime can never command the true stop.
> 2. Reduce the gripper's `kp` (currently 20.0 from `linear_4310.yml`) so the runtime cannot generate 7.7 Nm against a stop in the first place.
> 3. Command the gripper by **torque** rather than position — a gripper does not want a position controller.
>
> ⚠️ Whatever is chosen, **verify by leaving it holding for 60 s and watching the temperature plateau.** A gripper at equilibrium is the test; a gripper that "looks fine for 5 seconds" is not.

---

## 4. ⛔ The over-temperature incident, 2026-08-10 ~12:4x — read this before long sessions

**What happened.** During the first real cartesian teleop run, at t≈24 s:

```
ERROR:root:motor id: 7, error: motor over temperature at yam_real
```

I2RT's control thread raised and exited. **The teleop loop did not notice and kept running for another 64 seconds**, solving IK and calling `command_joint_pos` into a dead robot, printing plausible EE numbers the whole time. Julien saw the arm stop responding while the terminal claimed motion. With no commands arriving, the motors' own 400 ms timeout damped them and **the arm sagged slowly under gravity** — slowly enough to catch by hand, which he did.

**Why motor 7 and not a big joint.** Motor 7 is the gripper, a small DM4310 with little thermal mass, and it had spent the whole day being pushed **against a hard stop**: three teleop runs aborting at 1.2 Nm, I2RT's 0.5 Nm auto-calibration, then our 0.3 Nm calibration. Stall torque is the worst possible thermal case — full current, no motion, no cooling. Then teleop commanded it to *hold* its position, which against a stop is more stall torque. **The heat was cumulative across the session, which is why "I didn't even move it quickly" is entirely consistent with an over-temperature fault.**

**Rules that follow:**
1. **Never command the gripper to hold a position at a hard stop.** Park it mid-stroke.
2. **Monitor temperature every cycle** and stop at 65 °C, below the firmware trip. `chain.read_states()` returns `temp_mos` / `temp_rotor` per motor. Idle is ~30-33 °C.
3. ⭐ **Check that the control thread is alive every cycle.** `chain.running` going False means commands are not arriving. A loop that cannot tell whether its commands land is worse than one that crashes.

---

## 5. Teardown — the order is not optional

```
1. stop the control thread   (chain.running = False, then wait ~0.15 s)
2. disable every motor        (while the bus is still open)
3. close the bus
```

Both vendor `close()` methods get this wrong. `DMChainCanInterface.close()` shuts the bus **without disabling the motors**; `MotorChainRobot.close()` prints *"Robot closed with all torques set to zero"* and calls only the former — while the 250 Hz thread is mid-`set_control`, which produced a thread-death traceback. **Leaving motors enabled is what broke consecutive runs**: they time out into damping, and the next `_motor_on()` takes the error-clearing path, which desynchronises §2.4.

`yam_robot.shutdown_robot()` does it correctly and **returns the IDs that actually confirmed** — never a hopeful constant.

---

## 6. Recovering a drooped arm — yes, and nothing is lost

Julien asked whether a droop leaves the system miscalibrated. **It does not.** The encoders report true joint positions at all times — proven this morning when a hand-twist of the gripper read back exactly (0.0086 → −1.2312 rad) while every other motor stayed byte-identical.

**What was lost when the arm drooped was the control loop, not the knowledge of where the arm is.** So recovery is simply: read the true current pose, then interpolate slowly to the target. `teleop_session.py`'s PARK mode does exactly that at **0.40 rad/s**. ⚠️ What it *cannot* know is what is now in the way, which is why it moves slowly and `h` or `t` stops it. ⛔ **Correction, 2026-08-10: "any key aborts it" was the bug, not the feature.** Every unrecognised key — Enter included — used to cancel PARK, so pressing `p` and then Enter out of habit killed the move in the same keyboard batch and looked exactly like "park just went to hold". Only `h` and `t` stop it now.

---

## 7. Working rules established with Julien

- **The agent never runs anything that can move the arm.** Those are handed over as commands. The line: **scripts that enable motors but send no setpoint → agent. Anything that sends a setpoint → Julien.** (Widened from "anything that enables a motor" once enabling-without-setpoint had been shown ~15 times to produce ≈0 torque.)
- **Announce before running, do not pause.** Say what is about to run, then run it.
- ⛔ **The workspace was the binding constraint, not the software**, until 12:1x when Julien cleared the desk.
- **`--yes` on every script that transmits.** Dry run is always the default and always prints the full plan.

---

## 8. State at the end of 2026-08-10

**Achieved:** SpaceMouse verified on all 6 axes · both arms identified and driven · bimanual gripper motion from one loop over two buses · 100 Hz proven with 3× headroom · gravity compensation holding to **0.61°** · hand-guiding · **cartesian SpaceMouse teleop on the real arm** (EE moved 0.15 m under puck control).

**✅ RESOLVED — it was the wall power, exactly as the shared-cause reasoning predicted.** Julien disconnected and reconnected the arms at the socket and both came straight back: all 7 motors on each bus, every temperature 31-36 °C, no latched faults. **A power cycle is the recovery for a latched motor over-temperature.** The diagnosis held because two independent arms on two independent buses cannot fail together for independent reasons — the only shared thing was mains.

**Known-imperfect, deliberately deferred:**
- **SpaceMouse axis directions are wrong/unintuitive.** Not yet mapped to Julien's expectation. **The tooling for it is complete as of session 3** (`scripts/map_axes.py`, no hardware needed) — what remains is him driving it. See §9 and §10.
- **Two SpaceMice are now connected, and the ambiguity is SOLVED — by asking the hardware.** Both report an empty serial, so select-by-serial does not transfer from the CAN adapters. They differ only in USB `port_numbers` — `(1,3)` and `(1,4)` — which hidapi does not expose, and which tell a human nothing about which puck is under which hand. So `pick_device_by_wiggle()` opens all of them and uses **whichever one the operator moves.** Unambiguous, needs no config, survives replugging into any port, and costs five seconds.
- No git remote. Everything exists only on this Mac.

---

## 9. Four defects found by READING, 2026-08-10 (session 3) — no hardware involved

Nothing was plugged in for any of these. Each was found by checking the code against what the docs claimed about it, which is worth noting on its own: **the bench is not where the cheap defects are.**

**1. ⛔ PARK with `--no-gripper` would have released a raised arm.** `config/park_pose.json` holds **7** joints; `--no-gripper` builds a **6**-DoF robot. `park_target - measured` on mismatched shapes raises `ValueError` — and that exception escaped the control loop, **skipped the "the arm is HOLDING, press g or d" consent flow**, and fell into `finally`, which disables the motors. A raised arm sags. The path matters: `--no-gripper` is exactly the escape hatch the gripper instructions tell you to fall back to, *so the fallback was the broken one.* Symmetrically, a pose saved **in** a no-gripper session had 6 entries and broke the next 7-DoF session. Fixed in `yam_robot.park_target_from()` — start from the measured pose and overlay only the joints the saved pose carries, so no target is ever invented for a joint we know nothing about. Tested: `scripts/test_park_target.py`.

**2. PARK was the one motion path that bypassed the gripper clamp.** It commanded the saved jaw value directly. Harmless with the pose saved today (0.0366, inside the band) — but `s` saves wherever the jaws happen to be, so saving with the jaws on a stop would later drive them back onto that stop and **hold** them there. That is the stall condition from §4 rule 1, reachable through the one door the guard did not cover.

**3. ⭐ The gripper thermal test could not detect the thing it tested.** The status line printed only `hottest` — the max over all seven motors. Motors 2/3 carry the arm's 4.3 kg and sit at **41-42 °C** in normal equilibrium, while an idle motor 7 is **31-36 °C**. So motor 7 climbing 33 → 41 °C was **entirely hidden inside the `max()`**, and "watch `hottest` plateau" agreed with the claim it was supposed to be able to refute. §0's own rule, missed in the one place it was load-bearing. The jaw temperature is now printed separately (`jaw NN°C`), and the session's peak jaw temperature is reported at exit.

**4. A warn-and-continue, in the exact wording of the rule against warn-and-continue.** A second stale-limit check in `teleop_session.py` compared the raw jaw position against the **unshifted** limits from the file, so it re-flagged precisely the cases `frame_correct_gripper_limits()` had legitimately reconciled: at the measured raw **−1.380** it printed *"STALE GRIPPER LIMITS … re-run calibrate_gripper"* while the frame was correct and the jaws normalised to **0.3005**. It then continued. Two harms, neither hypothetical: a real warning and a false alarm became indistinguishable, and the remedy it advised is a routine that drives the jaws into both mechanical stops. Deleted — `build_robot()` already gates this twice, better, and *before* any control loop starts. **A duplicated weaker check is worse than no check**, because it launders the strong one.

**Also fixed:** the PARK progress report was an `elif` on the motion branch, so one cycle in every hundred sent no command at all. Benign — the chain's own 250 Hz thread holds the last target — but not what the code said it did, in the one mode a human watches rather than steers.

> ### The generalisation
> All four are the same shape: **a guard, a test or a message that was written once and then not re-derived against the thing it guards.** The clamp existed and PARK went around it. The refusal existed and a weaker copy undermined it. The temperature monitor existed and aggregated away the signal. ⭐ **Ask of every guard: what is the path that reaches the hazard without passing through you?**

---

## 10. The world frame, measured rather than assumed — 2026-08-10

`CartesianTeleop.step()` integrates the twist in the **world** frame (deliberately — body frame is named in `src/teleop.py` as a later choice, not an ambiguity). What the world axes physically *are* had never been checked, and `scripts/map_axes.py` prompts with them, so a wrong label would make the whole tool lie.

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

⭐ **Consequence worth having: a wrong rotation sign twists the wrist in place rather than flinging the gripper across the desk.** That refines ROADMAP step 4's caution — rotation is the *less* dangerous sign to get wrong, not the more.

⚠️ **Deliberately NOT claimed: which way is "forward" or "left".** That depends on how the arm is physically turned on the desk, which no file in this repo records. Inventing a label would be exactly the confident, plausible, wrong answer §0 is a list of. `map_axes.py` therefore describes the operator's own gesture back to them from the reading, and never asserts a gesture-to-axis correspondence that has not been measured.

---

## 11. ⛔⭐ THE ARM FELL — 2026-08-10, session 3, and it was caused by advice in this repo

Three failures in one attempt. All three are mine, and the first is the important one.

### 11.1 `--no-gripper` silently breaks gravity compensation. The arm falls.

**What Julien saw.** He ran `teleop_session.py --yes --arm B --no-gripper --no-rotation`, which starts in GUIDE. His words: *"only the lowest motor… was in weightless mode, and all of the other motors were turned off. And therefore it just fell forward because the bottom motor didn't hold it in place."* The status line read a calm `hottest 35°C` for **33 seconds** while the arm sank to its own stops (`q [0.21, 0., 0., …]` — joints 2 and 3 at their zero limits).

**The mechanism, proven in simulation, not guessed:**

1. `GripperType.NO_GRIPPER` does **not** merely leave motor 7 unenabled. It swaps the *dynamics model* `get_yam_robot` uses for gravity compensation (`get_robot.py:186`, `combine_arm_and_gripper_xml`).
2. The bare arm XML gives its terminal body `mass="1e-6"` — **one microgram** (`yam.xml:38`). The real mass arrives by merging the gripper XML. Summing `linear_4310.xml`: `0.553219 + 0.0710042 + 0.0710042 =` **0.695 kg**, at the far end of the arm.
3. `zero_gravity_mode=True` sets **`kp = 0`** and commands zero torque (`motor_chain_robot.py:241`), so `motor_torques = joint_commands.torques + g * gravity_comp_factor + friction_comp` reduces to `g` alone (`:366`). **There is no position term to absorb a modelling error.**

Measured gravity torque at the saved park pose:

```
model mass          WITH gripper 4.987 kg      WITHOUT 4.292 kg     (missing 0.695 kg)
gravity torque WITH    [-0.00, -4.81,  6.34,  1.34, -0.07, -0.00] Nm
gravity torque WITHOUT [-0.00, -2.67,  3.88,  0.49, -0.00,  0.00] Nm
shortfall              [ 0.00, -2.14, +2.47, +0.85, -0.07, -0.00] Nm
                                       ^^^^^ joint 3 (elbow_pitch): 39% short
```

39% of the elbow's holding torque, unopposed. The arm folds forward. **Julien's observation was exactly right, and his interpretation was nearly right:** the other motors were not off, they were commanded with `kp = 0` and an under-computed gravity torque, which feels identical. Joint 1 felt free because `base_yaw` rotates about the vertical and gravity never loads it — in *any* mode.

**⛔ THE RULE THAT FOLLOWS: `--no-gripper` is not a safe subset of normal operation. It is a different, less accurate robot.** It was reached for as "the smallest possible experiment", and it is the opposite: it removes the one thing that must not be removed.

**Fix:** `build_robot()` now passes `ee_mass=GRIPPER_MASS_KG` (0.695) on the no-gripper path. Worst residual falls **2.465 → 0.188 Nm** (3% of the elbow's requirement), verified in simulation. ⚠️ **`ee_inertia` cannot be used** — the SDK emits an `ipos` attribute MuJoCo rejects (*"Schema violation: unrecognized attribute: 'ipos'"*, it should be `pos`). That is a bug in the vendored tree, so the centre-of-mass offset stays uncorrected and 0.188 Nm is the residual we cannot remove.

**Also fixed: GUIDE now prints live drift** from wherever it went weightless. The cause is gone, but the instrument should have existed anyway — nothing on screen was measuring the one quantity that was failing. *Same lesson as §9.3's jaw temperature: a readout must show what can fail, not what looks calm.*

> **Considered and rejected: an automatic sink-detector that forces GUIDE → HOLD.** In GUIDE, motion is *expected* — Julien is pushing the arm by hand — and there is no signal that distinguishes "he is lowering it" from "it is falling". Every threshold either false-fires during legitimate hand-guiding or is too slow to matter. Showing the number and fixing the cause beats automating a judgement the code cannot make.

### 11.2 The remap mode destroyed the hand-dialled axis map

The MAP mode written earlier that same session **bound whichever motion was selected the instant any clear puck deflection arrived, and then auto-advanced to the next motion.** So the entirely natural act of *"let me see what this does"* rewrote the map, cascading through several motions, each binding stealing a puck axis and unbinding its previous owner. Then the session **saved it unconditionally on exit.**

Recovered from the terminal:

```
axis map saved → config/spacemouse_map.json
  — Y←roll− — ROLL←pitch− — YAW←yaw+
  ⚠️  UNBOUND, the arm will not perform these: X, UP, PITCH
```

His hand-dialled `[1, -1, -1, 1, 1, 1]`, produced on real hardware, was overwritten. It survived **only because the file happened to be committed to git.**

**Three compounding faults, all now fixed:**

| fault | fix |
|---|---|
| deflection *edited* the map | **deflection observes, keys edit.** Nothing in CONTROLS mode writes the map except a keypress |
| auto-advance cascaded one wiggle into many bindings | there is no cursor and no advance any more |
| unconditional save on exit | saved **only if changed**, and the previous contents are copied to `config/spacemouse_map.prev.json` first |

> ⭐ **The generalisation, and it is the same as §9's:** *ask what path reaches the hazard without passing through your guard.* Here the hazard was data loss and there was no guard at all — because "explore" and "commit a change" had been collapsed into the same gesture.

### 11.3 `mjpython --view` cannot start, and I claimed it worked

```
failed to dlopen '/…/.venv/bin/python3': Library not loaded: @rpath/libpython3.12.dylib
```

`mjpython` is a launcher app bundle that dlopens the venv's interpreter, and the uv-managed CPython does not place `libpython3.12.dylib` anywhere on mjpython's rpath. **It is present**, at `~/.local/share/uv/python/cpython-3.12.12-macos-aarch64-none/lib/`, so one environment variable fixes it:

```bash
DYLD_FALLBACK_LIBRARY_PATH="$HOME/.local/share/uv/python/cpython-3.12.12-macos-aarch64-none/lib" \
  uv run mjpython scripts/teleop_sim.py --view
```

Verified: `mjpython OK, mujoco 3.11.0`.

⛔ **The process failure matters more than the fix.** I recommended this command after checking that the `mjpython` *binary existed* — `ls` — and reported it as "verified". Presence is not function. That is **"verify the consequence, not the mechanism"** (§0, and the founding-session lesson), violated in the same turn that quoted it. *An `ls` is never verification of behaviour.*

---

## 12. CONTROLS mode — Julien's design, and why it replaced mine

Mine was: hold the arm still, select a motion, gesture to bind it. It was wrong for a reason he identified immediately: **you cannot decide a direction is wrong until you have watched the arm go that way**, and the same document that admits "+X and +Y are horizontal but which one points away from you is not recorded anywhere" then asked him to bind X from memory. Incoherent.

His design, in his words: *"similar to teleoperate, just move the space mouse in different directions and only the strongest direction is actually moved, and then I can press some key which reverses the direction of that specific control."*

**Why it is better, point by point:**

| | his design | mine |
|---|---|---|
| the arm | **moves** — you see what each direction does | frozen; nothing to observe |
| cross-talk | **only the strongest axis is applied**, so the motion is attributable | a firm push moves 3 axes diagonally |
| what you must know | nothing — push and look | which motion index you meant, in the abstract |
| what a deflection does | **observes only** | *edited the map* — the data-loss bug in §11.2 |
| the edit | an explicit key on "the control I just used" | a cursor and an auto-advancing wizard |

**Implementation.** `isolate()` keeps only the largest-magnitude axis, with 1.3× hysteresis so two near-equal axes cannot make the arm jitter between two motions. The session remembers the last axis that actually moved — **with no timeout**, so `f` still works after the puck has sprung back to centre and his hand has left it. `f` resolves puck axis → motion via `AxisMap.motion_driven_by()` and flips that motion's sign. `1`-`6` reassign that same control to a different motion, taking **the direction he was last pushing** as the new motion's positive sense, so "push the way you want it, then name the motion" reads the same as a gesture.

Speed is **half** teleop's (`CONTROLS_SCALE = 0.5`): it is the mode you enter with a mapping you have not yet confirmed, so a wrong direction should be a slow wrong direction. Everything below the twist is the *existing, hardware-proven* chain — IK, per-cycle joint-step clamp, joint limits, workspace box, `SafeRobot` rate limiter. **CONTROLS mode adds a twist source, not a control path.**

### 12.1 What using it on the arm changed — 2026-08-10

It worked, and Julien's live map came out a genuine permutation: `X←y+  Y←x+  UP←z−  ROLL←pitch+  PITCH←roll+  YAW←yaw−`. **Sign flips alone could not have expressed that**, so the permutation half earned its place rather than being speculative generality. Two things came back:

**1. `1`-`6` now SWAP instead of steal-and-unbind.** *"Instead of only changing it to that specific thing and then just deleting the other one, it would just swap whatever was on the other… that will make it a lot easier."* He is right: the commonest edit is **two controls in each other's places**, and stealing left an orphan he then had to notice and re-bind, with a motion silently dead in between. A straight exchange is also an **involution** — the same key again undoes it — and preserves injectivity by construction. The sign travels with the puck axis, because the unit being exchanged is the whole control (which axis, pushed which way), not just the wiring.

**2. `,` and `.` were missing from the CONTROLS key handler**, so rotation speed could not be changed there at all while linear could. The keys had been copied from the drive-mode handler and the second pair dropped. ⚠️ **The reason it took a hardware session to notice is the interesting part:** the status line showed only the *resulting* speed of the active axis, so a key that did nothing was indistinguishable from a key that worked. Both scales are now printed continuously. *Same shape as the jaw temperature and the GUIDE drift — a readout must show the quantity a key is supposed to change.*

---

## 13. Bimanual prerequisites — built 2026-08-10, and one gap that would have bitten silently

**Per-arm axis maps (`AxisMapStore`).** Shared by default — Julien's *"probably the same, actually"* — with an override created only by an explicit `--fork-map`. Defaulting to a map per arm would let the two silently diverge, after which a puck that feels wrong on G is indistinguishable from a map that was never copied across. ⛔ **Whatever reads it must print which scope it is editing**; tuning G and silently changing B is the same shape as the bug in §11.2 — an edit whose blast radius was larger than the operator believed. A legacy flat file still loads as the shared map, so nothing hand-dialled is lost.

**⛔ `pick_device_by_wiggle()` could assign the same puck to both arms.** Called twice without an `exclude`, its single-device shortcut returns that device unconditionally, and with two attached nothing stopped the operator moving the one they had already assigned. **Both failures are silent, and the symptom — two arms following one hand — reads as a control bug rather than a device-assignment bug.** Exactly the class of the CAN adapter chosen by index that silently retargeted the wrong robot (§0 #5). Now takes `exclude=[path, …]` and says plainly when no unassigned puck remains.

**Two-arm IK, measured (it never had been):** `0.100 ms` mean per cycle, p99 `0.110 ms`, for two `CartesianTeleop.step()` calls. Against a 10 ms deadline with ~6.2 ms of CAN for 14 motors, that is ~3.7 ms spare. **IK is not the bimanual bottleneck**, and the assumption that it might be is now retired.

---

## 14. A test whose premise expires

`test_backward_compatible_with_hand_dialled_file` loaded `config/spacemouse_map.json` and asserted it was still sign-only. It failed the moment Julien legitimately saved a permutation — **correctly**, but for a useless reason: its own guard fired, not the property it was protecting.

**The property is about the file FORMAT, not about whatever is currently in the file.** It now runs against a pinned fixture string, and a separate test checks that the live config loads, is injective, and round-trips.

⭐ **Generalises: a test whose subject is a file the user edits has a moving target.** Pin the fixture; test the live artefact only for properties that must hold *whatever* it contains.

---

## 15. ⛔⭐ PARK was a treadmill — it commanded the measurement

**Julien reported PARK broken twice**, after it had been "fixed" once. The first round of fixes was real (a 7-vs-6 length crash, a bypassed gripper clamp, a skipped command cycle) but **none of them was why it did not move**. This is.

```python
robot.command_joint_pos(q + np.clip(park_target - q, -stepmax, stepmax))   # q = MEASURED
```

It re-anchored to where the arm actually was, **every cycle**. So the commanded position was never more than one step ahead of reality: `PARK_SPEED × dt = 0.40 × 0.01 =` **0.004 rad, about 0.23°.**

A position controller makes torque from the *error* between command and measurement. Capping that error at 0.23° caps the torque at `kp × 0.004` — not enough to overcome static friction plus 4.3 kg of arm. So the arm does not move; because it does not move the measurement does not change; and because the measurement does not change, the next cycle commands the same 0.23° offset. **A treadmill.**

⛔ **And it fails in this stack's signature style: it printed `parking… 1.2 rad to go` indefinitely, raised nothing, and read as a controller that was merely slow.**

**TELEOP never had the bug, and that contrast is the proof:**

```python
step = q_target - prev_q                                   # prev_q = last COMMAND
q_target = prev_q + np.clip(step, -MAX_JOINT_STEP, MAX_JOINT_STEP)
```

It integrates from the **command**, never from the measurement, so when the arm lags its command keeps advancing, the error grows, and the torque grows with it until the joint moves. PARK was the odd one out.

**Fix:** `advance_park_command()` — a trajectory that runs ahead of the arm as far as it needs to. `SafeRobot.max_lag` (0.25 rad) is what stops it running away, which is exactly the right place for that guard and was already there. Completion is judged on the **measured** pose, never the command, because the command always arrives first. A **stall detector** now says so and holds if the measured error stops improving for 4 s — the silence is precisely how this survived two sessions.

`scripts/test_park_target.py` includes `test_the_old_formula_provably_could_not_converge`, which reproduces the old expression and asserts that after 20 simulated seconds the command is still 0.004 rad from a stuck arm. **The diagnosis is mechanical, not a story.**

> ⭐ **The generalisation, and it is the deepest one this project has produced so far:** **a controller must command a trajectory, not the thing it is measuring.** Feeding the measurement back into the command caps the error, and the error *is* the actuation. Anywhere you see `command = measured + something_small`, ask what makes it converge — often nothing does.

---

## 16. A refusal that named the wrong arm

G's first run refused correctly — it has never had its jaws calibrated, `config/gripper_limits.json` holds `B` only — and then printed:

```
Run this once:  uv run scripts/calibrate_gripper.py --yes
```

**No `--arm G`.** Following that literally drives **B's** jaws into both mechanical stops, while the arm you were trying to start stays uncalibrated and the same refusal comes back. The other two refusals in `yam_robot.py` both interpolate `--arm {arm}`; this one did not.

⛔ **A remediation message that names the wrong target is worse than no message: it converts a clean refusal into a wrong action.** Now fixed, and it also offers `--no-gripper` as the alternative.

---

## 17. The puck buttons drive the gripper

Julien: *"there are two buttons on the left and the right. One could be open, one could be closed… and then pressing whatever the switch button was, I think f, could then switch it back."*

⛔ **The masks are learned by pressing, never assumed.** Which physical button sets which HID bit has never been measured on this unit, and "assumed an identity that was never checked" is the single most repeated failure in this file — the CAN adapter by index, the puck by index, the gripper limits in the wrong frame. `b` in CONTROLS mode asks for OPEN, then CLOSE, and refuses to give one button both jobs (a button that both opens and closes is a coin flip, not a control).

⭐ **`f` reverses the buttons, because `f` already means "reverse the control I just used".** If that was an axis it flips the sign; if it was a button it swaps open and close. One rule, no new vocabulary — Julien reached for `f` unprompted, which is the sign the rule is the right one. Like `swap()`, it is an involution.

Buttons are **hold-to-move** at 0.6 normalised units/s (~1.6 s for the full stroke), not step-per-press: a gripper wants squeeze-and-hold. `o`/`c` remain as keyboard steps. The assignment lives in the axis map, so it is **per-arm** for free, and an unset button writes no key at all rather than a `null`.

### 17.1 …and it shipped broken, in the most annoying possible way

Julien, next session: *"the space mouse buttons that should control the gripper don't do anything. And then it says press b to set the gripper, and then b does nothing either."*

`b` was handled **inside the CONTROLS-mode branch**, while the *"press b to set the gripper buttons"* hint printed from the button-reading block, which ran in **TELEOP as well**. So in TELEOP the hint appeared, `b` fell through to the catch-all, and nothing happened. Buttons were also read only in teleop/map, so they were dead in GUIDE and HOLD entirely.

⛔ **A message that tells you to press a key which does nothing where you are is the same defect as the refusal that named the wrong arm (§16): the text is right, the context is wrong, and it costs a session to find out.** Button assignment is a property of the **device**, not of the arm's mode, so it now sits above the mode dispatch and works everywhere. The puck is read every cycle in every mode — which also stops HID reports queueing up during GUIDE/HOLD and arriving in a burst at the next mode switch.

---

## 18. ⛔⭐ "It moves very incoherently in weird positions" — a pure rotation moved the tool point 44 cm

Julien, 2026-08-11: *"the inverse kinematics being weird and not working as intended, specifically when the robot gets into weird positions, and then it starts moving very, very incoherently."*

### The first hypothesis was wrong, and measuring it saved a day

The obvious story — *near a singularity the Jacobian blows up, joint velocities explode, and the per-joint `MAX_JOINT_STEP` clamp distorts the direction* — is **refuted**. Measured: mink's `lm_damping` keeps the requested joint velocity around **0.45 rad/s against a 1.5 rad/s clamp**, so the clamp never binds, and at a genuine singularity (`σ_min = 0.0001`, folded near the base) the arm asks for **0.03 rad/s** — it barely moves *at all* rather than blowing up. Direction error from clamping: **0.00°** at every pose tested.

### What is actually happening

Reproducing the real control loop — IK, joint-step clamp, joint-limit clamp, workspace box — and commanding **pure roll at 0.6 rad/s** from the park pose:

```
   t     |target-EE|    |IK q - commanded q|    tool point moved
 4.0 s     0.0004 m          0.0000 rad              0.000 m
 8.0 s     0.238  m          0.0800 rad              0.238 m
12.0 s     0.074  m          0.0800 rad              0.290 m      (peak 0.44 m)
```

**A pure rotation command translated the tool point 44 cm.** The chain:

1. A wrist joint reaches its limit — the tight ones are ±1.5708 rad. *(Confirmed by the second column: the gap between IK's internal joint state and the commanded one pins at exactly **0.0800**, which **is** `JOINT_LIMIT_MARGIN`. A joint is clamped at the margin while the IK believes it is at the true limit.)*
2. `CartesianTeleop.step()` advances `self.target` by the twist **unconditionally**. It never asks whether the arm followed, so the orientation goal runs arbitrarily far past anything reachable.
3. The QP now holds an impossible orientation target, and `position_cost` (1.0) and `orientation_cost` (0.5) are **traded against each other** — so it starts moving the **tool point** to partially satisfy a rotation it can never achieve.
4. The workspace box re-clamps translation, fighting the orientation task — hence the oscillation above.

### Two fixes, and the second one is the surprise

**(a) Anti-windup on the goal.** `_limit_lead()` bounds how far `target` may run ahead of the pose actually achieved — 0.05 m and 0.25 rad, separately, because translation and rotation fail independently (the workspace box already happened to bound translation, which is why only rotation misbehaved). **This is `SafeRobot.max_lag` one layer up**, and it needs no model of *why* the arm cannot follow: joint limit, singularity, rate limiter or an obstacle all present as an unclosable gap, and bounding the gap bounds them all. Verified bounded, not merely slowed: the worst lead is **identical at 10 s and 80 s** (0.250060 rad).

**(b) `orientation_cost` 0.5 → 0.05.** Anti-windup alone cut the wander from 0.44 m to 0.40 m — barely anything — because a *persistent* unreachable orientation error still bleeds into position. The cost ratio is the real lever, and the measurement is counter-intuitive:

| `position:orientation` | pure-roll tool wander | rotation achieved |
|---|---|---|
| **1.0 : 0.5** ← the old default | **0.443 m** | **7.9°** |
| 1.0 : 0.2 | 0.034 m | 129.5° |
| **1.0 : 0.05** ← now | **0.002 m** | **134.6°** |
| 1.0 : 0.01 | 0.000 m | 18.2° |

⭐ **The old default was the worst of both worlds: it wandered 44 cm *and* achieved the least rotation.** A *higher* orientation cost produced *less* rotation, because the effort went into satisfying an unreachable orientation by translating, which drags the arm into a configuration that can rotate even less. Verified at three starting poses; small rotations are unaffected and translation reach is unchanged (0.319 → 0.320 m).

> **The priority this encodes, in one line: never sacrifice where the tool IS to chase where it POINTS.** A wrist that cannot turn should simply not turn — it should not drag the whole arm across the desk.

**Also added:** the TELEOP status line now prints `⚠️ STUCK lead 5cm/14°` when the goal is pinned near its limit. An arm that cannot follow used to present *only* as an arm behaving strangely.

⚠️ **`scripts/test_teleop_ik.py` reproduces the whole loop, not just `CartesianTeleop`** — the bug only appears when the clamps interact with the IK, so testing the class in isolation would have missed it entirely. One test deliberately restores `orientation_cost=0.5` and asserts the wander **comes back**: if that ever stops failing, the cause has moved.

---

## 19. ⭐ Driving from the camera's point of view is a FRAME question, not a camera question

Julien, 2026-08-11, wanting to use the C920 as a stand-in for the wrist cameras that have not arrived: *"I would provisionally like to use the Logitech camera… mounted on one of the arms as a test so that I can try to learn to control the arm from the point of view of the camera to get the tilts right and stuff."*

**The interesting half of that request has nothing to do with cameras.** Until now every twist was interpreted in the **world** frame — and `teleop.py` flagged the consequence from the very first version:

> *"World-frame integration: rotation pre-multiplies, so a twist means the same thing regardless of how the gripper happens to be oriented. Body frame would be more natural to a hand holding the puck, and is a deliberate later choice — not something to leave ambiguous now."*

This is that later choice, and the camera is what forces it. Looking **at** the arm, world frame is right: "forward" is a fixed direction on the desk, predictable, and a wrong sign only nudges. Looking **through** a wrist camera, world frame is wrong the moment you tilt: "push forward" then means forward *in the image*, and the image turns with the wrist. **That is exactly what "get the tilts right" is.**

So `CartesianTeleop` gained `frame = world | tool | camera`, applied as `R_wf @ v` and `R_wf @ ω` before the existing world-frame integration — which leaves the anti-windup and the workspace box untouched. `v` cycles it live, and that is safe without a resync because a twist is a *velocity*: a frame change alters the interpretation from the next cycle and leaves no stale cached state, unlike a mode change.

⛔ **`camera` is the MODELLED D405 mount and is WRONG for the hand-mounted C920.** The MJCF puts the D405 on the flange at a 25° cant with `+Z` along the optical axis; a webcam cable-tied on by hand shares none of that, and nobody has measured where it actually sits. **Use `tool` for the stand-in**, mount the camera roughly looking the way the gripper points, and dial the remainder out with the axis map. Using `camera` for an unmeasured mount would be inventing a transform — the single most repeated failure in this file.

### Camera capture: two facts worth keeping

**Almost all webcam "lag" is queued frames, not decode time.** A naive `read()` returns the *oldest* frame in the driver's queue. `scripts/camera_view.py` grabs repeatedly (cheap, no decode) until the queue is dry and decodes only the last one. It also sets **MJPG before the resolution** — left in uncompressed YUY2 the C920 cannot fit 1080p through USB2 and collapses to a few fps, which *reads* as latency and is a bandwidth problem. `--measure` reports the real frame interval so the claim is checked rather than asserted.

⚠️ **macOS gates camera access, and no code change fixes it.** First run prints `OpenCV: not authorized to capture video (status 0)` until the permission is granted to the app running the terminal — **System Settings → Privacy & Security → Camera**. Encountered 2026-08-11; the agent cannot grant it.

⚠️ **OpenCV on macOS selects cameras by INDEX**, which collides with this repo's hard rule against selecting hardware by index (§0 #5). AVFoundation offers no name-based alternative, so rather than pretend, `--list` makes the ambiguity visible and suggests the honest disambiguation: cover the arm-mounted camera with a hand and re-run — the index whose mean brightness collapses is the one on the arm.


---

## 20. ⭐ "It lags at high speed" is a singularity problem, not a speed problem

Julien, 2026-08-11: *"at high speeds the arm takes longer to follow the path that it's been told to move… I can only really control it at speeds of less than half a meter per second."*

**Two hypotheses were tested and refuted before the real one.** It is not a constant per-speed cost, and it is not a startup transient. Pushing +X at 0.25 m/s from the home pose, joint speed by cycle:

```
  cycle   1      0.66 rad/s          <- first cycle, fine
  cycle   2-10   0.67 rad/s          <- not a transient
  cycle  50-150  3.30 rad/s          <- it ESCALATES with time
```

Tracing it against `sigma_min`, the smallest singular value of the Jacobian — the standard measure of how close the arm is to a configuration where some direction of motion becomes unreachable:

| cycle | joint rad/s | EE moved | `sigma_min` |
|---|---|---|---|
| 20 | 0.68 | 0.05 m | 0.170 |
| 100 | 1.34 | 0.25 m | 0.121 |
| 140 | **2.93** | 0.35 m | 0.048 |
| 180 | 0.12 | 0.38 m | **0.005** ← stalled |

**The same tip speed costs 0.68 rad/s in the middle of the workspace and 2.93 rad/s near full reach**, and then the arm stops entirely. `SafeRobot` caps commands at 1.0 rad/s, so past that point the command is throttled, the arm falls behind, and it reads as latency.

⭐ **So speed is not the cause — it only decides how quickly you arrive at the part of the workspace where this happens.** That reframing matters, because the obvious fix (raise the cap) would not fix it. It would move the wall a little further out and cost the guard that makes a wrong motion catchable, on a rig with **no e-stop**.

**The fix is to ask for less.** `CartesianTeleop._apply_speed_scale()` measures the joint speed the solver just requested and, if it exceeded `max_joint_rate` (0.9 rad/s, deliberately just under SafeRobot's 1.0), scales the twist by exactly that ratio. Tip speed and joint speed are locally proportional, so it lands on the allowed rate in a single step. Recovery is slower than reduction — 5% per cycle, ~0.2 s to full — because reacting instantly in both directions oscillates at the boundary, which would feel worse than the lag.

**Measured, over 200 cycles (2 s):**

| commanded | cycles over the cap, before | after | worst lead |
|---|---|---|---|
| 0.12 m/s | 0 | 0 | 0.9 mm |
| 0.25 m/s | **86** | **0** | 5.1 mm |
| 0.40 m/s | 98 | 1 | 7.0 mm |
| 1.00 m/s | 42 | 2 | 7.3 mm |

At 0.25 m/s the rate limiter had been intervening on **43% of all cycles**. It now never does, and the command stays within 7 mm of the arm instead of pinned at the 50 mm anti-windup bound.

⚠️ **The throttle costs time, not workspace.** A first test compared reach after a fixed number of cycles and failed — correctly, since a throttled arm is behind at any given moment. Given time both converge on **exactly 0.5194 m**. The test now asserts that distinction explicitly, because "it got slower" and "it can no longer reach as far" are very different regressions and only one of them is acceptable.

Low speeds are untouched: at 0.12 m/s the scale never leaves 1.0, so normal driving is unchanged. The status line prints `⚠️ SLOWED to N% (near the reach limit)` — without it, the throttle would present as unexplained sluggishness, which is the same class of silent-failure this file exists to catalogue.


---

## 21. The terminal camera view: what is fixable in software and what is not

### 21.1 ⛔ The agent cannot test the camera. At all. Ever.

**macOS grants camera access per application.** The permission Julien granted covers his terminal; the agent's shell runs under a different process and gets `OpenCV: not authorized to capture video (status 0)` no matter what the code does. There is no flag, no entitlement and no code change that works around it.

**Consequences, and they shape how this file should be built:**

- Anything camera-related must be verified either by **Julien running a command**, or by making the logic a **pure function of an image** and testing that. Both camera bugs so far — the stretched aspect ratio and the 5 fps drain loop — lived in exactly such pure functions. `scripts/test_camera_render.py` exists for this.
- ⚠️ **`--probe` once measured a code path the viewer never ran** and reported a healthy 30 fps while the viewer delivered 5. *A measurement that does not exercise the real path measures nothing.* `--measure` now runs through `FrameGrabber`, the same class the viewer uses.
- The agent also cannot read the terminal's identity: its shell has no TTY and reports `TERM_PROGRAM` unset. **`--term-info` exists so the program reports what the agent cannot detect.**

### 21.2 A two-way toggle whose sides can be identical is not a toggle

`b` switched between `"blocks"` and `detect_term_mode()`. In any terminal the detector did not recognise, **both sides were `"blocks"`** and pressing it changed nothing — indistinguishable from a broken key. Kitty was worse: *detected* and then silently discarded because its protocol was unimplemented, so a kitty user was downgraded with no indication at all.

Now `b` **cycles** blocks → iterm → kitty (a three-way cycle cannot be a no-op), prints the mode it moved to, and says when that differs from what was detected. The kitty graphics protocol is implemented rather than detected-and-dropped. `detect_term_mode()` returns **the mode and the reason**, and the reason is printed on entry.

⭐ **Generalises past this key:** a fallback that is invisible is a bug report waiting to happen. Say which path was taken *and why*, especially when the fallback is the degraded one.

**✅ Confirmed 2026-08-11.** Julien's terminal is **Ghostty 1.3.1** (`TERM_PROGRAM=ghostty`, `TERM=xterm-ghostty`, `COLORTERM=truecolor`), and `--term-info` correctly reports *"Ghostty detected (speaks the kitty graphics protocol)"* with `best mode: kitty`. So `--term` already draws real images with no flag needed.

### 21.2.1 ⛔ `f=100` is PNG, not "some compressed image" — the blank-screen bug

Julien's screenshots showed **blocks working and kitty blank**. The cause: the first kitty renderer encoded **JPEG** and labelled it `f=100`.

In the kitty graphics protocol `f` takes exactly three values — `f=24` (raw RGB), `f=32` (raw RGBA) and `f=100` (**PNG**). **There is no JPEG.** So the terminal was handed JPEG bytes, told they were PNG, failed to decode, and said nothing — **because `q=2` had suppressed the very error that explains it.**

Two lessons, and the second is the more expensive one:

- **A format code is not a MIME type.** `f=100` names a specific container. Reading it as "compressed image" is the same class of error as assuming an SDK flag means what its name suggests — which is how `--no-gripper` dropped an arm (§11.1).
- ⚠️ **Suppressing errors cost far more than the noise it saved.** `q=2` is correct for a 30 fps redraw loop, and it turned a one-line diagnosis into a session of guessing. **`--term-test` now exists to send exactly one image with errors ENABLED** and print the terminal's reply verbatim. *When a display path can fail silently, ship the diagnostic that makes it speak.*

**PNG forces a size decision, because PNG of a photo is large.** Measured on a photo-like frame:

```
  1280x720   998 KB   31 ms encode   -> 40 MB/s at 30 fps. Impossible.
   640x360   283 KB    6.6 ms
   480x270  ~180 KB   ~3 ms          -> the default (--image-width)
   320x180    78 KB    1.6 ms        -> still 22x the detail of blocks
```

⭐ Even the smallest is a large win: a 67x19 cell grid is 67x38 = **2,546 pixels**, while 320x180 is **57,600**. `IMWRITE_PNG_COMPRESSION=1` is deliberate — level 1 costs ~1.6 ms where the default costs several times that, for a few percent of size.

⚠️ Two further kitty-protocol facts that would each have broken a 30 fps redraw loop, both found by reading the spec rather than by running it — the agent cannot test this:

- **Images persist until deleted.** One per frame at 30 fps would accumulate placements without bound. `a=d,d=A` clears the previous frame first.
- **The terminal replies to every image**, on **stdin** — which this viewer reads for keypresses. Without `q=2` every frame would inject escape bytes the key handler sees as junk input.

### 21.3 The remaining latency is the camera, and software cannot fix it

Measured by Julien on 2026-08-11 with the on-screen draw-cost readout: **~2 ms per frame.** So the render, the terminal and the grabber are all irrelevant to the ~0.2 s he perceives.

What is left is the **C920 itself** — sensor readout, onboard MJPEG encoding, and USB transport. For consumer webcams that is typically **100-200 ms**, and no software change removes it.

**The only lever that helps is resolution** (fewer pixels to read out, encode and transfer), which is why key `1` selects 320×180. Beyond that this needs different hardware; the D405 wrist cameras, when they arrive, are the real answer.

⚠️ **Do not spend more time optimising the software path.** It was measured at 2 ms against a ~200 ms budget. The next person to look at this should confirm that number is still ~2 ms and then stop.

### 21.4 ⛔⭐ "The resolution is stuck and the number keys do nothing" — 2026-08-11

Julien, session 9: *"the resolution is not great. It definitely doesn't let me go back up to 1920x1080 … When pressing the numbers, that doesn't matter. It doesn't do anything."*

**The keys were working perfectly. They were invisible.** `1`-`6` change the **capture** resolution — what the camera sends the Mac — and the viewer then handed the terminal an image resized to `--image-width`, which defaulted to a **fixed 480 px**. So a 1080p capture and a 480p capture produced a pixel-identical picture, and the only evidence anything had happened was a `1280x720` → `1920x1080` string in a status line nobody reads while looking at a picture.

⚠️ **This is the same defect shape as `b` toggling between two identical states ([21.2](#212-a-two-way-toggle-whose-sides-can-be-identical-is-not-a-toggle)), and it is worth naming as a class: a control whose effect is not observable is indistinguishable from a broken one.** The fix is never only "make it work" — it is "make the effect visible". Every number a key can now change is on the status line.

**The fix: three ceilings, smallest wins.** The image sent to the terminal is now `min(pane in real pixels, what was actually captured, the protocol's budget)`:

1. **The pane.** Pixels beyond what the terminal can display are scaled straight back out again — pure cost.
2. **The capture.** Upscaling before transmission invents nothing and costs bytes. This is the ceiling that makes keys `1`-`6` visible: a bigger capture now genuinely produces a bigger image.
3. **The protocol's budget**, which is the interesting one.

#### The budget, measured — and why Ghostty is soft where iTerm2 would not be

⛔ **The kitty graphics protocol has exactly one compressed format, and it is PNG.** `f` takes `24` (raw RGB), `32` (raw RGBA) or `100` (PNG). **There is no JPEG.** iTerm2's inline-image escape, by contrast, carries whatever the image is — JPEG included.

MEASURED 2026-08-11, best of five, on a synthetic but realistic 16:9 frame (gradients, hard edges, text and sensor grain; a flat wall is cheaper, pure noise dearer):

| width | kitty: PNG level 1 | iTerm2: JPEG q60 | PNG as % of a 33 ms frame |
|---|---|---|---|
| 480 px | 3.8 ms, 277 KB/frame | 0.1 ms, 16 KB | 11% |
| 640 px | 6.7 ms, 391 KB | 0.3 ms, 26 KB | 20% |
| 960 px | 16.1 ms, 1266 KB | 0.6 ms, 46 KB | 48% |
| 1280 px | 28.8 ms, 2259 KB | 1.1 ms, 70 KB | 87% |

**~25x on time, ~30x on bytes.** That single protocol fact is the whole reason the terminal view is soft, and it is not a bug anyone can fix in this repo. Hence the caps: **720 px for kitty/Ghostty, 1280 px for iTerm2.** Sizes are `KB/frame`; multiply by 30 for the per-second load into a pty that also has to draw them.

⚠️ Compression level is **not** the lever. At 640 px: level 0 = 2.1 ms but 676 KB; level 1 = 6.7 ms, 391 KB; level 6 = 28.8 ms for 371 KB. Level 6 costs 4x the time for 5% of the size. Level 1 stays.

⛔ **ANSWERED 2026-08-12: Ghostty does NOT accept iTerm2's escape** (§25.4). Does yours? If it does, the sharpness ceiling doubles for free. `--term-test` now sends **one image in each protocol** so the answer is a look at the screen rather than a guess.

#### The second silent guess: the character cell was assumed to be exactly 2:1

The grid geometry needs to know how tall a character cell is relative to its width, and it hard-coded `2`. Whenever the font disagrees, a 16:9 picture is displayed stretched — **the same bug Julien caught in a screenshot in session 7, in a second disguise.**

Terminals already know: `TIOCGWINSZ` returns `ws_xpixel`/`ws_ypixel` beside the row and column counts. kitty and Ghostty fill them in; Apple Terminal reports zeros; a piped or captured run has no terminal at all. So the cell is now **measured** where possible, and where it is not, the status line prints `ASSUMED` — because a fallback you cannot see is indistinguishable from a bug, which is this section's whole theme.

### 21.5 ⛔ The flicker — two causes, and one of them was wasting half the machine

Julien, 2026-08-11: *"the image in the terminal is flickering because some frames seem to not be drawn or something like that."*

**Cause 1 — delete-then-draw.** Every kitty frame began with `a=d,d=A` (**delete all images**) and only then transmitted the new one. Between the delete and the new image being decoded there is nothing on screen, so the picture was blanked 30 times a second. Whether that reads as flicker depends on how fast the terminal decodes — which is why it got worse as the image was allowed to grow.

The fix is double buffering, exactly as in any graphics pipeline: two image ids used alternately, the new image **placed over** the old one, and only then the old id deleted from underneath. There is never a moment with nothing on screen. Deleting still has to happen — `d=I` frees the image data too, and without it a 30 fps stream leaks one image per frame into the terminal's memory.

**Cause 2 — redrawing frames the terminal already had.** ⭐ This one is worth internalising. The display loop ran as fast as it could while the camera delivered 30 fps: with an ~18 ms draw it went round about **55 times a second**, so nearly half of every second was spent re-encoding and re-transmitting a **pixel-identical** picture. The loop tracked a frame sequence number and used it only to count fps — never to decide whether drawing was worth doing. Skipping unchanged frames halves the terminal's load and removes half the flashes at the same time.

⚠️ Both fixes are reasoned and structurally tested; neither can be *seen* by the agent ([§21.1](#211--the-agent-cannot-test-the-camera-at-all-ever)). Confirmation is Julien's eye, and it should be asked for explicitly rather than assumed.

### 21.6 The number keys were offering another camera's modes

*"The numbers when I press them don't allow for all the quality options. They cycle between about three, and not in the correct order either."*

`SIZES` in `camera_view.py` is a list of **C920** modes, and it was used for every camera. A UVC device asked for a mode it does not have substitutes the nearest one it does, so on the MacBook Air camera — whose seven modes are 640x480, 1280x720, 1552x1552, 1760x1328, 1328x1760, 1920x1080, 1080x1920 — keys 1 to 4 all land on 640x480. **Three distinct results from six keys, and its portrait modes are why the order looked wrong too.**

Keys are now bound to the selected camera's **own** modes, read from AVFoundation: deduplicated, ascending, at most six spread across the range, bounds-checked, so key `6` is always the best that camera can do. On the C920 that is **2560x1472**, which was never on offer before — the hard-coded list stopped at 1920x1080.

⭐ **The symptom was also evidence.** "Only three distinct sizes" is the signature of the MacBook Air camera, not the C920 — so the complaint about the number keys was, in hindsight, an independent report that the tool was driving the wrong camera. It was not recognised as such at the time. **When two complaints arrive together, check whether one is a symptom of the other before fixing them separately.**

---

## 22. ⛔⭐ Which camera is which — and how a careful, checked, published inference was still wrong

> **Read this section for the method, not just the answer.** It is the cleanest example in this repo of [§0](#0-the-one-thing-to-internalise-before-touching-anything) — the stack failing by lying — being reproduced *by the code written to prevent it*.

**The problem.** Four cameras are visible on this Mac: the built-in one, the D405 on arm B, the C920, and Julien's iPhone over Continuity. **OpenCV opens them by integer index and reports no name at all** — verified on OpenCV 5.0, where `cv2.videoio_registry` enumerates *backends* and never *devices*. Indices also move on replug, which is [§0 #5](#0-the-one-thing-to-internalise-before-touching-anything) — *an adapter chosen by index silently retargeted the other robot* — with a different cable.

### What was tried first, and why it looked sound

macOS *will* name cameras (`system_profiler -json SPCameraDataType`), and ⭐ it needs no camera permission, so even the agent can run it — the only reason naming was approachable at all ([§21.1](#211--the-agent-cannot-test-the-camera-at-all-ever)). The first implementation paired macOS's n-th camera with OpenCV's n-th index.

It was **not** done casually. The pairing was labelled an inference in the code, in the docs and in the chat; the device counts were checked to agree (macOS listed 4, indices 0-3 opened, and OpenCV's own `out device of bound (0-3): 4` confirmed the count); a falsifier was coded — a D405 cannot deliver a frame wider than 1280, so a shuffled order would strand it on a 1920-px index — and a falsification procedure was published: *cover each camera and see which index goes dark.*

**Julien ran it. It failed.**

| | macOS says | reality |
|---|---|---|
| index 0 | MacBook Air Camera | **HD Pro Webcam C920** |
| index 1 | RealSense D405 | RealSense D405 ✅ |
| index 2 | HD Pro Webcam C920 | **MacBook Air Camera** |
| index 3 | iPhone (Continuity) | iPhone (Continuity) ✅ |

Two of four names were wrong, and the tool was confident about both. He had already driven a whole session with `--camera c920` while looking at his laptop's own camera.

⭐ **Three lessons, and the third is the one that generalises.**

1. **The falsifier only covered the case it was written for.** It knew one fact — the D405's width — so it could only catch a permutation that moved the D405. The D405 happened to be in the right place. **A check that can only fire for one of N possibilities is not a check on the claim; it is a check on a corner of it.**
2. **Agreement between sources is not independence.** `system_profiler` and AVFoundation agree because they read the same CoreMedia list. Counting them as two confirmations was counting one fact twice — the same error as [§1](#1-the-hardware-as-measured)'s "two arms verified identical by evidence that could not tell them apart from one arm read twice".
3. ⛔ **Nothing that could be read off a list would have worked.** Three separate macOS enumerations — `system_profiler`, `AVCaptureDevice.devicesWithMediaType:`, and an `AVCaptureDeviceDiscoverySession` asked in two different device-type orders — return the **same** order, and it is not OpenCV's. The information simply is not in any list.

*(One observation, offered only as a lead if the measurement below ever fails: OpenCV's order looks like USB cameras sorted by location ID first, then built-in, then Continuity — `0x1120000` (C920) < `0x1210000` (D405). One data point. Do not build on it.)*

### ⭐ The fix: ask the hardware a question only one camera can answer

Every camera advertises a set of modes, and AVFoundation will list them. On this rig they differ sharply:

| camera | modes | one that is **its alone** |
|---|---|---|
| MacBook Air Camera | 7 | `1552x1552` (square) |
| RealSense D405 | 6 | `848x480` |
| HD Pro Webcam C920 | 18 | `160x90` |
| iPhone (Continuity) | 4 | `1920x1440` |

Ask an index for a mode only one camera owns. If it comes back **exactly**, that camera is on that index. A camera cannot deliver a mode it does not have — **it substitutes the nearest one it does**, which is how `424x240` turned into `640x360` in session 7. That measured substitution behaviour is the foundation the scheme rests on, and it is stated in `identify_indices()` where it can be re-checked: if a future OpenCV ever echoes the request instead of the result, every index matches every mode, which surfaces as *everything ambiguous* rather than as a wrong name.

**Deliberately not cached.** A replug can reorder indices without changing anything a cache could key on, so a stored map is the same failure with a longer fuse. Identification costs a few seconds at startup; the viewer then runs for minutes.

**What it refuses to answer, out loud:** an index that matches two cameras · two indices claiming the same camera · a listed camera no index answers for (normal for Continuity when the phone is asleep) · and ⚠️ **two D405s, which share every mode and therefore cannot be told apart this way at all.** The second D405 is on the desk waiting to be plugged in, so that case is not hypothetical — when it arrives, tell them apart by covering one, and select with `--index`.

### ⛔ Two measurement bugs that made the evidence murky in the first place

Both were in the code that was supposed to help identify cameras, and both produced confident wrong readings rather than errors.

1. **The probe read one frame the instant the camera opened.** Apple's built-in camera takes roughly half a second to expose and Continuity longer, so the built-in camera reported **brightness 5 while pointing at a bright room**, and the iPhone reported `NO FRAME`. Both numbers were about sensor warm-up — in the column the operator is told to use to tell cameras apart. It now reads until a frame has actual variation, and reports how long that took.
2. **A black frame was called mono.** Three all-zero channels are identical, so `frame_is_mono` printed `MONO — depth/IR, not a picture` about **an iPhone**. A frame with no variation carries no colour information; the honest answer is "cannot say", which is now what it returns.

### ⭐ The D405 over UVC gives a PICTURE, not a depth map — an earlier claim here, corrected

This section briefly claimed the opposite, on the strength of the device's **name**: macOS lists exactly one entry for the D405 and calls it `Intel(R) RealSense(TM) Depth Camera 405  Depth`, from which it was concluded that plain UVC reaches only a depth stream and that `brew install librealsense` was therefore required before the wrist camera could be driven by eye.

**Wrong, and the tool's own output said so.** `--list` reported index 1 as `colour`, and the live view shows a textured photographic image of the room — wall pattern, wood grain, print on a t-shirt. A depth map has no texture. The D405's imagers are colour-capable, and what arrives over the single UVC entry is an ordinary picture with a cold white balance.

⚠️ **The error was reasoning from a label instead of from the pixels** — a name is not a contract, which is the same lesson as `--no-gripper` in [§11.1](#111---no-gripper-silently-breaks-gravity-compensation-the-arm-falls), where a flag named for one thing changed another and dropped the arm.

**So:** the "no SDK needed" shortcut **does** work for teleop. `librealsense` is still the route to depth data, intrinsics, alignment and camera controls — but it is an upgrade, not a prerequisite, and the wrist view can be driven today.

### Using it

```bash
uv run scripts/camera_view.py --list                    # names, indices, and the checks
uv run scripts/camera_view.py --camera c920 --term      # select by name, not index
```

`--camera` accepts any part of the name, plus the aliases `d405`, `realsense`, `c920`, `iphone`, `builtin`, and a `vid:pid`. It **refuses** on no match or an ambiguous one and never falls back to index 0.

---

## 23. ⛔ `system_profiler SPUSBDataType` reports an empty bus while 15 devices are plugged in

Found 2026-08-12, checking that the rig was unchanged after an overnight break.

```
$ system_profiler -json SPUSBDataType
{ "SPUSBDataType" : [ ] }          # and the plain-text form prints nothing at all
```

Every device was in fact connected. `ioreg` says so:

```
$ ioreg -p IOUSB -l -w 0 | grep '"USB Product Name"'
   2x CANable 2.5 Candlelight · 2x SpaceMouse Compact · HD Pro Webcam C920
   Intel(R) RealSense(TM) Depth Camera 405 · AX88179A · 5 hubs · USB BillBoard
```

⛔ **This is [§0](#0-the-one-thing-to-internalise-before-touching-anything) in its purest form.** The command does not fail, does not warn, and does not exit non-zero. It returns a well-formed, empty, confident answer — and the natural reading of that answer is *"nothing is plugged in"*. A session that opens by checking the rig this way would conclude the arms had been unplugged and go looking for a hardware fault that does not exist.

⚠️ It is not a stale-cache fluke: it was run twice, in both JSON and plain-text form, with everything connected. Reproducible. Whether it is a macOS regression or a permissions change was not chased, because the workaround is one line and the answer would not change what we do.

**Use `ioreg` for USB enumeration.** It is also what earlier sessions used for the unbounded USB scans, so nothing here needs rewriting:

```bash
ioreg -p IOUSB -l -w 0 | grep '"USB Product Name"'                    # what is attached
ioreg -p IOUSB -l -w 0 | grep -E '"USB (Product Name|Serial Number)"' # with serials
```

⭐ **Serials are the point.** `2081337C594E5018` is arm **B** and `20593383594E5018` is arm **G** ([§1](#1-the-hardware-as-measured)), and the D405 is `255323071773`. Checking those, rather than a device count, is what makes "the rig is unchanged" a measurement instead of an impression. The two SpaceMice still report **no serial at all**, which is why they are still assigned by asking the operator to wiggle one.

**The general lesson, which is the same one this repo keeps paying for:** an empty result and a failed query are indistinguishable unless the tool distinguishes them for you. When a check returns "nothing", ask what a *broken* check would have returned — and if the answer is "the same thing", the check has told you nothing at all.

---

## 24. ⛔⭐ Three holes in the safety guards, found by READING — 2026-08-12

Found while reading `teleop_session.py` end to end before restructuring it. **None had ever fired on hardware**, which is why none is in [§0](#0-the-one-thing-to-internalise-before-touching-anything)'s table — and is also exactly why they were worth finding first. This is the same exercise as [§9](#9-four-defects-found-by-reading-2026-08-10-session-3--no-hardware-involved), and it found the same class of thing: **guards with a path around them, and messages that promise what the code does not do.**

⚠️ **All three fixes are tested headlessly and NONE is confirmed on the arm.** Session 4 is the standing warning here — three changes that passed 34 tests, three dry runs and a simulated IK loop produced three failures on first hardware contact, one of which dropped 4.3 kg.

### 24.1 The thermal guard disarmed itself on any read error

The temperature block wrapped the read **and every decision that followed** in one `try`, whose handler was:

```python
except Exception:
    temps, hottest, jaw_temp = [], 0.0, None
```

So **any** failure of `chain.read_states()` — a CAN hiccup, a decode error, a short state list — reported the hottest motor as **0 °C**. Then `if hottest >= TEMP_STOP` could not fire, the thermal stop was gone for that cycle, and the status line printed a calm `hottest 0°C`. Nothing warned. If the read failed persistently, the session would run to completion with **no thermal protection at all**, reassuring the operator the whole way.

Motor 7 has been cooked three times on this rig ([§3.5](#35--the-gripper-two-2π-frame-errors-not-a-broken-mechanism)), so this is not a theoretical guard.

**Fixed** by `ThermalGuard` in `src/yam_robot.py`: only the *read* is wrapped, never the decisions; a failed read is `None` and never a temperature; blindness is announced once and becomes a **stop** after 100 cycles (1 s at 100 Hz), because a session that cannot see temperatures has lost the thing between the gripper and a stall burn. The readout now shows `hottest ??°C ⚠️BLIND` rather than a number.

### 24.2 The 55 °C warning was advertised and never issued

The startup plan has always printed `temperature : warn 55°C, stop 65°C`. An exhaustive grep over `scripts/` and `src/` found `TEMP_WARN` used in exactly two places: its own definition, and that line of the plan. **The warning did not exist.** Only the 65 °C stop was ever implemented, so the operator's first notice of a heating motor was the session ending.

Same defect class as [§16](#16-a-refusal-that-named-the-wrong-arm), where a refusal named the wrong arm: the text is right, the behaviour is absent, and only somebody at the bench finds out. ⭐ **A constant that is printed but never compared against is a promise, and this repo should grep for those on purpose.**

### 24.3 Ctrl-C released the arm, going around the consent flow

This session exists partly because *"quitting released the arm on a timer"* and **a 5 s countdown is not consent** — so `q` was changed to go to HOLD and wait for an explicit second key. That fix is real and it works.

**Ctrl-C went around all of it.** `SIGINT` raised past the consent flow, past the `with KeyReader()`, into the outer handler, and straight into `finally` — which calls `shutdown_robot()` and disables the motors. On a raised arm that is a sag, and ⚠️ **Ctrl-C is precisely what a person presses when something looks wrong**, which is the worst possible moment to release 4.3 kg.

Working contract rule 7, verbatim: *what path reaches the hazard without passing through your guard?* Here it was the interrupt, and the guard was the newest code in the file.

**Fixed** with a SIGINT handler rather than a `try/except`, so the ~540-line loop body did not have to be re-indented for it: the first Ctrl-C sets a flag **and restores the default handler**, so the loop stops at the top of its next cycle and falls into the same consent flow `q` uses — and a **second** Ctrl-C is a genuine force quit, which is what pressing it twice means. There is a deliberate remaining gap: a Ctrl-C during `build_robot()`, before the handler is installed, still exits the old way. That is correct — the arm has only just been energised and is where the operator left it.

### ⭐ What to take from this, beyond the three fixes

**Every one of these was found by reading the code against its own documentation**, and each is invisible to testing that only exercises the happy path: the thermal hole needs a CAN failure, the warning needs a hot motor, the Ctrl-C hole needs someone to panic. The pattern worth repeating on any file that guards hardware:

1. For each constant that names a limit, **grep for where it is compared.** If it only appears in its definition and in a printed message, it is decoration.
2. For each `except`, ask **what the handler makes the next check believe.** A handler that substitutes a safe-looking default has silently answered a safety question.
3. For each guard, ask **which exits skip it** — `break`, `return`, exceptions, and signals. Signals are the one people forget, because they do not appear in the code.

---

## 25. ⭐ The camera identification held up, and four things around it did not

**2026-08-12. First, the good news, because it is the part worth trusting.** Julien ran `--list` and the measurement reproduced his hand experiment exactly:

```
✅ index 0 delivered 160x90,     which only C920 webcam offers.
✅ index 1 delivered 424x240,    which only RealSense D405 offers.
✅ index 2 delivered 1080x1920,  which only MacBook Air Camera offers.
✅ index 3 delivered 1920x1440,  which only Julien's iPhone Camera offers.
```

That is the wiring he found by covering each camera, and the exact opposite of what three macOS enumerations claim ([§22](#22--which-camera-is-which--and-how-a-careful-checked-published-inference-was-still-wrong)). **Ask the hardware a question only one device can answer.** It works, it is fast, and it needs no list to be correct.

Everything below is a defect he found by *using* it.

### 25.1 A 2 fps stills mode was offered as "the best this camera can do"

*"When I press number six the frame rate drops to like two frames per second."*

MEASURED: the C920 advertises **2560x1472 at 2.0 fps**. Every other mode it has runs at
30. `key_sizes()` sorted modes by pixel count and put the largest on key 6, which for a **live view** is the wrong idea entirely — the right one is *the sharpest mode that still moves*. AVFoundation reports a max frame rate per format, so this is a measurement and not a matter of taste: modes below `MIN_LIVE_FPS` (15) are now excluded.

⭐ The general shape: **"biggest" and "best" are the same only when one dimension matters.** Here there were two, and the code only knew about one.

### 25.2 `--camera c920` took ~20 seconds to open one named camera

*"It takes like twenty seconds for the camera to start running, which shouldn't be the case because I deliberately said which camera I want."*

Correct. `--camera` called the full `identify_indices()`, which opens **every** camera and asks **every** camera's question at **every** index — 4 opens and 16 reconfigurations to answer a question about one device. It also woke his iPhone over Continuity every single time, which is the slowest thing on the bus and was never wanted.

**Now:** one question per index, stopping at the first exact match (safe *because* the mode is unique to one camera), plus `config/camera_index_hint.json` remembering where that camera was last found.

⚠️ **The hint is an ordering of the search, not a cached answer**, and that distinction is what keeps it consistent with §22's refusal to cache identity. The camera at the remembered index is asked the same question as any other; a stale hint costs one extra open and falls through to the scan. **A cache that is verified on every use is not a cache of the answer.**

### 25.3 The detail cap was a constant where it needed to be a measurement

*"The max resolution that can be sent is 720x405 … it doesn't really make any sort of difference."*

720 px came from the PNG encode benchmark in [§21.4](#214--the-resolution-is-stuck-and-the-number-keys-do-nothing--2026-08-11). That benchmark was right and the conclusion drawn from it was too narrow: on his machine encoding is ~8 ms of a ~40 ms draw, and **the rest is writing ~650 KB into a pty**. That ratio depends on the terminal, the font size, the window and the machine's load — none of which a constant measured on one afternoon can know.

The viewer now **climbs toward what it actually sustains**: spend at most half the frame interval drawing, back off quickly (×0.85), climb slowly (×1.15), with the ceiling set by the pane and the capture rather than by any protocol budget. Same discipline as the rest of the rig — *measure the consequence rather than predicting it*.

### 25.4 ⛔ The diagnostic itself produced a confident wrong answer

`--term-test` reported **"the terminal said NOTHING → it does not implement this protocol"** about Ghostty 1.3.1 — which was, at that moment, drawing the camera view in kitty mode.

**The kitty protocol keys its response to an image id** (`i=`) or number (`I=`). The test sent neither, so the terminal had nothing to answer *about* and correctly stayed silent. The test then read silence as absence.

⭐ Worth keeping for its own sake: **silence is not evidence unless you asked a question that requires an answer.** The same shape as [§23](#23--system_profiler-spusbdatatype-reports-an-empty-bus-while-15-devices-are-plugged-in)'s empty USB list, and it appeared here in the code written to diagnose exactly this class of problem. Fixed by giving the test image an id; "no reply" now says plainly that it proves nothing and asks the operator to look at the screen.

✅ **ANSWERED 2026-08-12, and the answer is no.** With the image-id fix in place, `--term-test` got a clean `ESC _G i=771;OK ESC \` back — so Ghostty implements the kitty protocol properly — and Julien reported seeing **one** set of bars, the KITTY set. The iTerm2 image never appeared.

⛔ **So PNG is the ceiling on this terminal, and that is a closed question rather than a hope.** The ~25x cheaper JPEG path is unavailable in Ghostty, which is exactly why the sent-image size is decided by the adaptive controller measuring the real draw cost ([§25.3](#253-the-detail-cap-was-a-constant-where-it-needed-to-be-a-measurement)) rather than by a constant chosen from encode benchmarks. If sharper is ever wanted badly enough, the lever is a different terminal — iTerm2 and WezTerm both take JPEG — not more code here.

---

## 26. ⛔⭐ The key reader was swallowing keys — including the ones that stop the arm

**2026-08-12, found by chasing something else entirely.** Julien reported that after a park stalled, the session *"kind of went back into a mode"*. It had, and the cause was not in the park.

### The defect

`KeyReader.get()` did this:

```python
if select.select([sys.stdin], [], [], 0)[0]:
    return sys.stdin.read(1)
```

**Those two lines do not see the same thing.** `sys.stdin` is a `TextIOWrapper` over a `BufferedReader`: `read(1)` returns one character, but the buffer underneath first pulls **everything available** off the file descriptor. `select()` then asks the *descriptor* whether data is waiting, the descriptor says no — and the remaining keys sit in Python's buffer, invisible, until the next keystroke happens to arrive and flushes them out.

**Reproduced on a pty:**

```
keys typed:      pgh
first  drain(): ['p']
second drain(): []
```

`drain()`'s own docstring promises *"every pending keypress"*. It returned one of three and lost the rest.

### Why this one matters more than a dropped keystroke usually would

These keys are **`h` for HOLD**, **`q` for the quit-and-consent flow**, and **"any key" to stop a park while the arm is moving.** ⛔ **A burst of keys is exactly what a person produces when they want the arm to stop**, and a burst was precisely the broken case.

It also explains the reported symptom directly: a swallowed key surfaces later, next to input that had nothing to do with it, so a mode appears to change by itself. In his log a park announced itself and reported `park stopped.` in the same breath — cancelled by a keystroke typed seconds earlier, for something else.

**Fixed** by reading the descriptor with `os.read(fd, 1)`, so `select` and the read are talking about the same buffer. `scripts/test_keyboard.py` covers it on a **real pty** — 6 tests — because the bug lived in the seam between two real objects and a mocked stdin would have passed while the arm still ate keystrokes.

⚠️ **The same defect was in `--term-test`**, and it is how that command reported `⛔ ERROR` about a terminal that had answered correctly: it read a single `\x1b` of a longer reply and judged the rest missing. Same fix.

⭐ **The general lesson, and it is not really about terminals:** `select()` answers questions about a *file descriptor*, and every buffered reader layered on top of one holds data the descriptor no longer knows about. Mixing the two is a silent data-loss bug in any language. Ask both questions of the same object.

### 26.1 The park stall was a knife edge, not a fault

Same session, two consecutive runs, same arm and same pose:

```
⭐ PARK reached (0.020 rad off) → HOLD        # interleaved park
⛔ PARK STALLED — 0.021 rad still to go       # Ctrl-C park, next session
```

`PARK_TOLERANCE` is **0.02**. The arm was landing either side of the threshold by a thousandth of a radian — and that is not a fault, it is **the noise floor**. A position-controlled arm holding itself against gravity settles where its stiffness balances the load, a fraction of a degree short of the commanded pose. `park_target_from`'s own comment said so all along: *"it must allow for a position controller's steady-state error."* 0.02 rad (1.1°) turned out to sit **on** that error rather than safely above it.

⭐ **The fix is not a bigger number.** Loosening the tolerance would also make a genuinely obstructed arm look parked — and the obstruction might be a hand. What separates the two cases is **how far away it stopped**:

| stopped improving, and | verdict | why |
|---|---|---|
| **close** (< 0.06 rad) | `settled` — success | the remaining error is the controller, not an obstacle |
| **far** | `blocked` — hold and ask | something is in the way, or the pose is unreachable |

One threshold was doing two jobs; it is now two thresholds doing one each (`park_verdict()`, pure, 5 tests). Both park paths — the interleaved one and the blocking one — now share the single rule, so they can no longer disagree about the same arm at the same pose.

### 26.2 Two smaller things from the same log

- **A park could be cancelled by a keystroke typed before it started.** "Any key stops it" must mean a key pressed *at the moving arm*, not one left over from teleop or from the menu that led there. Both park paths now drain stale input first.
- ⭐ **"Smooth motion" meant two different mechanisms, and I built the wrong one.** Julien asked for smoothing between saved poses. I implemented a trapezoidal **speed ramp along each leg** — ease in, cruise, ease out — which makes a single move gentler and leaves the arm **stopping dead at every waypoint**. He meant **corner blending**: one continuous motion that curves *through* the corner.

Both are real features, both were wanted, and the word "smooth" covers both. ⚠️ The tell was there in the original request — *"move from point one to point two … in a smooth curve it would go to the next point"* describes a **path**, not a speed — and I read it as a speed because that was the cheaper thing to build. **When a request could name two different mechanisms, say which one you are building before building it.** One sentence would have saved the round trip. Both now exist and are independent: the ramp decides *how fast* the cursor moves, the blend decides *what shape* it follows.

- **Every clean exit printed a traceback.** The SDK's `robot_server` thread raises `motor chain is not running` when we stop the chain — i.e. on success, immediately before `motors confirmed disabled`. ⚠️ Harmless, and worth removing anyway: a scary traceback on every successful exit is a training exercise in ignoring tracebacks, and this project depends on people reading the ones that matter. Suppressed **narrowly** — only during our own shutdown, only that thread, only that exception, only that message; anything else prints in full. If it does, that protocol carries JPEG — ~25× cheaper in time and ~30× in bytes than the PNG the kitty protocol forces — and the sharpness ceiling roughly doubles for free. One look at the screen settles it.
---

## 27. ⛔⭐ One function explained all three of the complaints from session 19 — 2026-08-12

Julien's report after driving the arm through GUIDE / TELEOP / PARK for ~4½ minutes: *"everything worked as it should. However, a couple things was still a bit weird. Hold sometimes gets overwritten … I feel like the second hold, or the first hold, was overwritten"*, plus an observation that some output looked duplicated and an explicit invitation to work out **where** it had duplicated.

⭐ **All of it was `StatusLine.say()`, and the evidence was in the paste he sent back.** Every message line in it carries a fragment of the status line welded onto its end:

```
⭐ MODE: GUIDE — arm is weightless°C  jaw   33°C  q [-0.49  1.    0.88 …]
⭐ MODE: TELEOP — SpaceMouse drivesC  jaw   33°C  q [-0.49  0.96  0.84 …]
⭐ MODE: PARK → slot 0, 0.98 rad … Press h or t to stop. 0.216 -0.081  0.352]
  run cancelled.6.0s  hottest   39°C  jaw   33°C  q [-0.49  0.96 …]
  PARK to which?  0 = base, 1-9 = a waypoint, Enter = base. 0.98  0.76  0.27 …
```

`weightless°C`, `drivesC`, `to stop. 0.216`, `cancelled.6.0s` — the tail after each message is **the surviving right-hand end of the status row underneath it.**

### 27.1 The defect

`say()` cleared **one** row and then wrote its payload. But `print` is shadowed for the whole of `main()`, and its ~60 call sites were written for the builtin, so they all look like `print("\n⭐ MODE: HOLD\n")`. The payload therefore *contains newlines*:

1. `\x1b[K` clears the hint row.
2. The payload's first line (empty) emits `\n` → **the cursor moves onto the status row, which is never cleared.**
3. `⭐ MODE: HOLD` is written over its first 13 columns. The rest of the status survives.
4. `_rows` is then wrong by the number of embedded newlines, so the next `_rewind()` moves the cursor to a row the live block was never on, and repaints the block there — **leaving the previous copy on screen.**

⭐ **That fourth step is the "duplicate print", and it is not a copy-paste artefact.** His log contains the same three rows twice with only the timestamp differing — `[HOLD] t= 198.0s` and `[HOLD] t= 256.0s`, **58 seconds apart**, each under an identical `⭐ PARK reached in 0.9s (0.036 rad off …)` and `RUN 1 → 2 → 3 … Enter=go`. A stale copy of a live row, still holding the timestamp it had when it was orphaned.

⭐ **And `⭐ MODE: HOLD` is the shortest banner in the program** — 13 columns against a ~90-column status row. It therefore overwrote the least and looked, correctly, like *the one that got eaten*. Julien's *"hold sometimes gets overwritten"* was a precise description of a real mechanism, not an impression.

### 27.2 The test that should have caught it, and why it could not

```python
screen.say("⭐ MODE: PARK")          # the test
print("\n⭐ MODE: PARK → …\n")       # every actual call site
```

**The test passed a single-line message. No call site in the program produces one.** It asserted `out.count("\n") == 1`, which is exactly the property that breaks the moment the payload has newlines of its own — so the assertion was measuring the thing it was supposed to protect, on input that could not violate it.

⛔ This is working contract rule 7 again — *what path reaches the hazard without passing through your guard?* — in a new place. The guard was a **test**, written against the interface rather than against its callers. The repo already knew this shape: a thermal test that could not detect the thing it tested (§9), a refusal that named the wrong arm (§16), a hint advertising a key that did nothing where it was printed (§17.1). **A test is a guard, and it decays the same way.**

⭐ **The fix that makes it hard to repeat:** `test_screen.py` now replays the escape codes into a grid of rows (`rows_of()`) and asserts on **what each row ends up holding**, not on the stream. `assert "MODE: HOLD" in out` was true throughout the bug. The text was never missing; only its *position* was wrong, and only a model of the screen can see that.

### 27.3 Two more ways the same row accounting could break, both now closed

- **A message longer than the terminal wraps**, consuming two physical rows where the class counts one. `say()` never truncated, and the PARK banner is ~110 columns. ⭐ **Truncation is now applied only when a live block exists** — with nothing live there is nothing to rewind over, so the startup plan and HELP still print in full. Getting this wrong in the other direction would silently cut the information he reads to decide whether to pass `--yes`.
- **Width was measured with `len()`.** `⭐` is one character and **two columns**; `⚠️` is two characters (`⚠` + VS16) and two columns. Every line in this program has emoji in it, so a line "fitted" to 99 columns could be 105 wide, wrap, and desynchronise the same arithmetic. ⚠️ First attempt keyed the VS16 promotion on east-asian-width `A` (ambiguous) — but `⚠` is `N`, *neutral*, so the rule missed the commonest symbol in the codebase. The test caught it. **Ambiguous-width characters (`→ · °`) are still counted as one column, which is right outside a CJK locale and wrong inside one**; `width()` keeps a spare column for that.

### 27.4 ⛔ The stale hint row — and it is why `e` "did nothing"

`hint()` was **never cleared anywhere in the file.** So once a run had been typed, the row

```
RUN 1 → 2 → 3 · speed 0.78 (-/+) · corners smooth 0.15 (,/.) · ease s-curve over 0.20 (e, [/]) · Enter=go
```

stayed live for the rest of the session — after the run was cancelled, through mode changes, sitting above a `[HOLD]` status. It is a *live* row, repainted every cycle, which is the other half of why the same block appears twice in his paste.

⭐ **Worse than untidy, because it advertises keys.** His log reads:

```
  run cancelled.
  (key 'e' does nothing — press ? for the list)
  (key 'e' does nothing — press ? for the list)
```

He cancelled a run, the stale row went on offering `(e, [/])`, he pressed `e` twice, and was told the key does nothing. **Both halves were defects:** the row should have gone, and `e` should have worked. `e` was bound only inside the "typing a park sequence" branch, although the ease profile describes how the *next* park will move, which is meaningful in any mode. It is now global.

⚠️ **`e` is still handled in the pending branch as well, and that duplication is deliberate:** an unrecognised key while typing a sequence *cancels the run*, so a key bound only further down the dispatch would abort the very move it was meant to configure.

### 27.5 Two live rows, and the ones that were fighting for the status row

`hint()` exists so a knob change repaints instead of scrolling. Two places still wrote their transient readout through `print(…, end="")`, which the shadowed `print` routes to `screen.set` — **the heartbeat row**:

- the park-sequence echo (`park sequence: 1 → 2   (another digit, or Enter)`), and
- the once-a-second park progress (`moving… 2.31 rad of path left, …`).

Both replaced the temperature readout and were then wiped by the next repaint. ⛔ The second is the worse one: during the one motion where an operator most wants to watch a temperature climb, the temperature row was the one being painted over. Both are hints now.

### 27.6 ⛔ An arrow key was silently changing a motion parameter

An arrow key sends `ESC` `[` `A`. The key reader returned those as **three keypresses**, and **`[` is bound** — gripper step in the drive modes, ease-ramp length while a run is being typed. So pressing ↑ halved a motion parameter and then printed `(key 'A' does nothing)`, which reads as the program having lost track of itself.

⚠️ **A test asserted this behaviour and called it *"documented, not desired"*.** Writing a defect into a test does not make it safe; it makes it permanent. That wording is the tell — a test recording behaviour nobody wanted, with a comment explaining why it was fine. Escape sequences now arrive as one non-printable token, which the session's `k.isprintable()` filter drops in silence. A bare `ESC` still arrives as `ESC`, because it cancels a pending park.

### 27.7 ⭐ `ö` and `ä` — and why the reader could not have seen them

Julien: *"I don't like the fact that the brackets are used because I have a German keyboard, and they're awkward to reach. Maybe ä and ö could be used."*

He is right, and it is worse than awkward: **on a German QWERTZ layout `[` and `]` are AltGr+8 and AltGr+9** — a three-finger chord, on a rig whose input design rule is *no shift keys*, for a knob adjusted while 4.3 kg is moving.

⛔ **But the keys could not simply be bound, because the reader was structurally incapable of receiving them.** `KeyReader.get()` did `os.read(fd, 1)` and decoded that **single byte** with `errors="replace"`. `ö` is two bytes in UTF-8 (`0xC3 0xB6`), `ä` is `0xC3 0xA4` — so each half decoded independently to `U+FFFD` and **one keypress produced two replacement characters and no key.** No exception, no log line: this file's opening rule, in the input layer.

Fixed with a UTF-8 **incremental** decoder held on the instance, because the two bytes of one character can land in different reads and a per-call decoder would mangle a key that arrived intact. Tested by writing the halves separately into a pty.

⭐ **Why `ö`/`ä` and not something else** — the alternatives were checked rather than assumed. Unshifted keys free on a German layout: `ö`, `ä`, `ü`, `#`, `<`, `ß`, and `+` (already "faster"); `^` and `´` are **dead keys** that emit nothing until a second press. `ö` and `ä` are adjacent, on the home row, in the physical position US layouts give `;` and `'`. ⚠️ **They are ALIASES — `[` and `]` still work**, because they are in every doc and a US keyboard (a colleague's, or a clone of this repo) must not lose the feature.

### 27.8 A mode key pressed twice reported itself as unknown

`(key 'g' does nothing — press ? for the list)` appears in his log **immediately after `⭐ MODE: GUIDE`**, which reads as the program having lost track of its own state. It had not: the mode branches are guarded by `mode != "guide"`, so a second press fell past them to the catch-all for unrecognised keys. Now answered with `already in GUIDE`.

⚠️ **The mode is deliberately not re-entered.** `enter_guide()` re-arms gravity compensation and re-takes the drift reference; that cannot be tested from the bench, so this fixes the *message* and changes nothing the motors see.

### ⭐ What to take from this one

**Four of the eight items above were guards or tests that had stopped describing their subject** — a screen test asserting on a payload shape no caller produces, a keyboard test pinning an unwanted behaviour, a hint outliving the state it described, a mode branch whose fall-through reported the wrong thing. None of them failed. They all passed, kept passing, and stopped meaning anything.

⛔ **The corollary for this repo's method: "245 tests pass" is a statement about the tests as much as about the code.** Session 4's lesson was that reading does not find what only hardware knows. This is its twin: **a test does not find what its input cannot express.** The cheapest defence found here was to assert on the *user-visible artefact* — the grid of rows a terminal ends up displaying — rather than on the call that produced it.

---

## 28. ⛔⭐ librealsense on macOS: installed, cannot open the camera, and does not need to — 2026-08-12

### 28.1 What was measured

`brew install librealsense` succeeded: version 2.58.3, prebuilt bottle, no compiling, dependencies `glfw` and `libusb` already present. Then `rs-enumerate-devices -s` failed, identically, in **both** the agent's shell and Julien's own terminal:

```
failed to claim usb interface: 0, error: RS2_USB_STATUS_ACCESS
acquire_power failed: failed to set power state
libusb_init failed with status: -99 (attempt 1)
Could not create device - failed to set power state
No device detected. Is it plugged in?
```

⭐ **The fact that it fails in HIS terminal too is the useful half.** macOS camera permission is granted per application, and his terminal has it (`camera_view.py` works there). So permission is **not** the cause, and [§21.1](FINDINGS.md) does not explain this one.

The camera itself is healthy. `ioreg` reports serial `255323071773`, `Device Speed = 3` (USB SuperSpeed, 5 Gbps), `kUSBCurrentConfiguration = 1`.

### 28.2 ⭐ The USB tree shows FOUR interfaces, which extends §8

`ioreg -p IOService -w0 -r -n "Intel(R) RealSense(TM) Depth Camera 405"`:

| interface | macOS's name for it | what is attached |
|---|---|---|
| 0 | `Intel(R) RealSense(TM) Depth Camera 405  Depth@0` | matched, no visible client |
| 1 | `… Depth@1` | matched, no visible client |
| 2 | `… Y@2` | matched, no visible client (Y = the infrared/mono imagers) |
| 3 | `… RGB@3` | ⭐ **`UVCAssistant`** — macOS's own UVC driver |

⭐ **This settles why OpenCV gets a real photograph rather than a depth map.** [§8](FINDINGS.md) recorded that macOS lists exactly one entry for the D405 and calls it `… Depth`, and inferred that the colour picture arrives over that single entry. At the USB level there is a **distinct `RGB` interface**, and macOS's UVC driver has claimed exactly that one. The colour picture is the RGB interface doing its job.

⚠️ **An open question worth one command.** If the `Depth` or `Y` interfaces also appear as *capture devices*, some depth or infrared data may be reachable through OpenCV with no SDK at all. `uv run scripts/camera_view.py --list` answers it: one D405 entry, or more than one? ⛔ Julien has to run it; the agent cannot open a camera ([§21.1](FINDINGS.md)).

### 28.3 ⛔ Google Chrome was holding BOTH cameras, after he believed everything was closed

The same `ioreg` output shows, under the C920 **and** under the D405:

```
+-o Google Chrome  <class AppleUSBHostDeviceUserClient, id 0x…, active, retain 7>
```

⭐ **This is what "deep check whether something is using the camera" looks like, and the answer was yes.** Julien had closed the viewer, FaceTime and Photo Booth, and reported *"I think I properly closed everything… still nothing."* Chrome keeps a user client open on both cameras regardless. Eliminate it before drawing any conclusion about librealsense:

```bash
osascript -e 'quit app "Google Chrome"'; sleep 3; rs-enumerate-devices -s
```

### 28.4 ⭐⭐ ANSWERED THE SAME DAY: `sudo` works, and the second theory was wrong

```
$ sudo rs-enumerate-devices -s
Device Name        Serial Number     Firmware Version
RealSense D405     260322274021      5.15.1.55
```

Two candidates had been written down, with one command to separate them:

- **(a) Privilege.** libusb's macOS backend has to take a USB interface away from the kernel driver that already holds it, and that can require root. ✅ **This was it.**
- **(b) Entitlement.** The device node carries `IOServiceDEXTEntitlements = (("com.apple.developer.driverkit.transport.usb"))`, and a Homebrew-built binary cannot carry that entitlement. ⛔ **Wrong**, and wrong in an instructive way: it was plausible, self-consistent, and it predicted that root *could not* help, because root does not grant entitlements. Root helped.

⭐ **The lesson is [§0](FINDINGS.md) pointed the other way for once, and it still applies.** A mechanism that fully explains the symptom is not a result. This one cost nothing only because it was written down as a hypothesis with that warning attached rather than as a finding. ⚠️ **Chrome was a red herring too** — quitting it changed nothing, so §28.3 survives as a lesson about checking rather than as a cause.

⭐ **Firmware `5.15.1.55` confirms a prediction.** [§8](FINDINGS.md) read `bcdDevice = 20721 = 0x50F1` and guessed *"probably firmware 5.15.1, unverified"*. It was right.

⚠️ **What this does and does not unlock.** Running an inspection tool as root is fine. ⛔ **Running a 100 Hz control loop as root is not.** So this does not mean our Python code can read serials or depth. The practical shape: `sudo rs-enumerate-devices` for inspection and firmware, the ordinary OpenCV path for streaming, and the wiggle method in §28.6 for working out which camera is on which arm.

### 28.5 ⛔⭐ ONE CAMERA, TWO DIFFERENT SERIAL NUMBERS. Unresolved.

| source | serial reported |
|---|---|
| `sudo rs-enumerate-devices -s` (librealsense, reads the camera's own firmware) | **`260322274021`** |
| `ioreg` USB descriptor, and [§8](FINDINGS.md) on 2026-08-11 | **`255323071773`** |

Both readings come from the same evening, minutes apart, with exactly **one** RealSense on the bus (confirmed by `ioreg`). The firmware version matching the prediction made from that same USB descriptor says it is one physical camera.

⛔ **So one camera answers the question "what is your serial number" with two different numbers, depending on which tool asks.** Anyone writing "select the camera by serial" into a config file must record *which* serial, or the value is right in one tool and wrong in the other while both look plausible. Same class of trap as [§5](FINDINGS.md) trap 2, where cached raw motor positions were frame-dependent and it cost a motor.

✅ **SETTLED AS A FACT ON 2026-08-13, though not as a mechanism.** Julien ran both commands back to back in one shell, on one camera, with the second D405 still unplugged. So the "a second camera was briefly involved" story is ruled out: **this single camera really does answer with two different numbers depending on which tool asks.** ⚠️ *Why* is still unverified. The likely reason is that the two tools read different places, one being the camera's own firmware and one being the USB descriptor, but nothing here proves that. ⭐ **The practical rule is what matters and it does not depend on the mechanism: whoever writes a serial into a config file records which command produced it.** The comparison, for reference:

```bash
sudo rs-enumerate-devices -s; ioreg -p IOUSB -w0 -l | grep -A2 RealSense | grep Serial
```

That also answers whether two D405s can be told apart by either number.

### 28.6 ⭐⭐ The way around it is still the better plan

What librealsense was wanted for, and how much each part actually depends on it:

| wanted | depends on librealsense? | how much we care |
|---|---|---|
| depth | yes | ⚠️ optional. Most image-based policies use colour only |
| lens numbers (intrinsics) | yes | only needed to convert a pixel into a direction in 3D |
| exposure and gain control | yes | nice to have |
| **telling two identical D405s apart** | ⛔ **this was the one that mattered** | see below |

⭐ **The last one has a solution that needs no SDK, and this repo already invented it for exactly the same problem.** The two SpaceMice report **empty serial numbers**, so they cannot be selected by identity; they are assigned by asking the operator to move the one they want (`pick_device_by_wiggle`, `src/spacemouse.py`). Two D405s cannot be told apart by capability, because they support identical capture modes ([§22](FINDINGS.md)). So assign them the same way: **make one camera's picture change and see which index changes.**

1. ⭐ **Version 1, no arm, no risk:** ask the operator to wave a hand in front of one camera. Roughly three seconds, and it is the same interaction he already knows from the pucks.
2. **Version 2, automatic:** command a small, bounded wrist twist on one arm and see which camera's image moves. ⚠️ It moves a motor, so it is Julien's to run under working-contract rule 1, and the motion must be tiny.

⭐ **This is a better answer than a serial lookup, for the reason [§0](FINDINGS.md) keeps restating.** A serial tells you *which camera this is*. What the code actually needs to know is *which arm this camera is bolted to*, and no serial can answer that: swap the two brackets and every serial-based mapping is silently wrong while every value still looks plausible. The wiggle measures the thing we care about.

---

## 29. ⭐ The terminal camera view shrinks itself and can never grow back — 2026-08-12

Julien, on two screenshots taken minutes apart: *"they get worse over time, which is a bit weird… it has the highest quality when I use it for FaceTime or something. It just lowers for the terminal for some reason."*

⭐ **The camera is fine and nothing is degrading.** The terminal renderer is shrinking the picture deliberately, and then cannot climb back out. It is a one-way ratchet, which is exactly what "gets worse over time" feels like from outside.

### The arithmetic, from his own two screenshots

The controller in `run_terminal` ([`scripts/camera_view.py`](../scripts/camera_view.py)) measures its own draw cost every 0.4 s and adjusts the width it sends:

- `target` = half the frame interval = `0.5 × 1000/30` = **16.7 ms**
- `draw > 16.7 ms` → shrink, `× 0.85`
- `draw < 0.6 × 16.7 = 10 ms` → grow, `× 1.15`
- anything between 10 and 16.7 ms → **do nothing**

His screenshots: `sent 520x292` at `draw 13.3 ms`, and later `sent 442x249` at `draw 10.5 ms`.

⭐ **`520 × 0.85 = 442.0` exactly.** So precisely one shrink step happened. And at 442 the draw cost is 10.5 ms, which sits **inside the dead band and above the 10 ms grow threshold** — so it can never climb back.

⛔ **Every width whose draw cost lands between 10 and 16.7 ms is a fixed point.** One transient hiccup (another app waking, a thermal blip, a scroll) knocks it down a step, and it stays there for the rest of the session. The dead band was added to stop oscillation and it also removed every path back up.

### Why FaceTime looks better, and it is not the camera

FaceTime draws pixels onto the screen. The terminal path has to encode **every frame as a PNG** and write it into the terminal. Ghostty implements only the kitty protocol, and that protocol has **no JPEG at all** ([§21.4](FINDINGS.md)); PNG of a photograph costs roughly 25× the encode time and 20× the bytes of a JPEG. So the terminal is the ceiling here, permanently.

⭐ **Immediate workaround, and it costs nothing:** drop `--term` and use the window.

```bash
uv run scripts/camera_view.py --camera d405 --big
```

### The fix, written down and NOT applied

⚠️ Julien deprioritised it (*"not that relevant, though, because I can still see everything… we can think about that later"*), so this is a ready recipe rather than a change:

1. **Shrink only after two consecutive over-target readings**, so a single hiccup cannot move it. This is the part that removes the ratchet.
2. **Raise the grow threshold** from `0.6 × target` to about `0.85 × target`, so the dead band is narrow and recovery actually happens.
3. **Reduce both step sizes** (say `× 0.93` down and `× 1.06` up) so the residual oscillation is too small to see.

⚠️ Also visible in his screenshots and already warned about on screen: capture is `1280x720` while only ~450 px is sent. Shrinking the *capture* would cut the resize cost and might on its own let the controller climb.

---

## 30. ⛔⭐ Four defects in the first hardware run of the recording feature — 2026-08-13

Julien recorded and played back several movements on arm B, across four sessions. The feature worked: *"w and l work great. I was able to record. I was able to play it back. I was able to save the recordings, play back the recordings in different speeds."* Four things were wrong, and three of them were only visible because the console prints numbers at two different moments.

### 30.1 ⛔⭐ Pressing `w` did not stop the recording. It stopped when the slot digit was pressed.

His report: *"when I recorded, the recordings played for like two seconds longer than I actually recorded. It was just standing still for that time."*

⭐ **The console had already printed the evidence, twice, and the two numbers disagreed.** The stop message reports the length at the `w` press; the save message reports it again after the digit:

| slot | announced at the `w` press | what landed in the file | appended afterwards | joint travel in that tail |
|---|---|---|---|---|
| 1 | 7.9 s / 690 samples | 9.70 s / 850 samples | **1.80 s** | 0.37 rad |
| 3 | 3.4 s / 294 samples | 7.78 s / 679 samples | **4.38 s** | 0.11 rad |
| 4 | 2.7 s / 233 samples | 6.02 s / 522 samples | **3.32 s** | 0.70 rad |

⛔ **The cause.** The per-cycle sampler runs when `take is not None`. Pressing `w` set `pending = "take_save"` and printed a summary, and left `take` in place. So the recording kept growing for exactly as long as he took to answer the prompt, and the arm was nearly still while he read it. Recording 3 is **56% padding**.

⭐ **The fix is one line of intent: move the recording to a second name and set `take = None` immediately.** The sampler stops on that statement, and the prompt then saves something frozen. `take_to_save` holds it.

⚠️ **The general lesson, and it is worth more than the fix.** A modal prompt that leaves the thing it is asking about still running is a defect shape, not a one-off. The same pattern existed in the park-sequence prompt and was harmless there only because typing a digit changes nothing physical. ⭐ **And note what caught it: printing the same quantity at two moments in the same flow.** Neither number looked wrong alone.

### 30.2 ⛔ Every hand-taught recording is faster than any planned motion is allowed to be

Measured from his three files, joint speeds in rad/s:

| recording | max | p99 | p95 | median |
|---|---|---|---|---|
| 1 (teleop) | 0.78 | 0.68 | 0.59 | 0.29 |
| 3 (guide) | 2.87 | 2.67 | 1.99 | 0.04 |
| 4 (guide) | 3.31 | 2.36 | 2.00 | 0.49 |

`MAX_JOINT_STEP * CONTROL_HZ` is **1.5 rad/s**, which is the fastest this code lets any planned motion command. So both hand-guided recordings exceed it by roughly 1.6x to 1.8x, and driving with the SpaceMouse does not.

⭐ **So playback at 1.00x asks for more than the arm is allowed to do, the loop holds its clock to let the arm catch up, and the playback comes out longer than the recording.** His two runs took 10.1 s for a 7.8 s recording and 8.5 s for a 6.0 s one. Both effects were in play at once: this one, plus the padding from §30.1.

⛔ **The old readout could not explain any of it.** `safe_time_scale()` floored its answer at 1.0, so every hand-taught recording reported "max 1.00x" and then ran slower than 1x anyway. **The number was correct and reported in a form that could not describe what he was watching.** The floor was a *policy* ("1x is always allowed") living inside a function whose job is to measure. It has been removed; the session applies the policy now.

⭐ **Julien's own proposal, adopted:** *"maybe one x should just be the original speed, and then you could go up and down. So max speed would just be limited by the actual safety things we have or the motors. Maybe we need an extra system for max speeds in general."* There is now one named ceiling, `MAX_PLANNED_JOINT_SPEED`, **derived** from the teleop clamp rather than picked, so the two cannot drift apart. A playback starts at the fastest speed that will actually track, and the plan line states the taught speed, the ceiling and a warning when the first exceeds the second.

### 30.3 ⚠️ The maximum joint speed is set by a single sample, and it distorts the decision

Recording 4: maximum **3.31**, 99th percentile **2.36**. ⭐ **One sample is dragging the maximum up by 40%.** At 100 Hz a single noisy reading of 0.033 rad does exactly that, and a weightless arm being pushed by hand is where such a reading comes from.

So sizing a playback speed off the maximum lets one bad sample veto a whole recording. `Trajectory.joint_speed(percentile)` now exists. ⛔ **Both numbers are kept and both are shown.** The percentile decides the speed; the maximum is still reported, because hiding a real fast movement behind an average is the failure this file is named after.

### 30.4 ⛔ A message told him to press keys that did something else, and he changed a motion parameter by accident

His report: *"the German characters ö and ä don't quite work as I think they should. When I press e outside of park and then the characters, then they change the gripper step speed, which in itself might be cool, but not necessary currently and all the time."*

The chain: `e` cycles the ease profile in any mode, and its message ended `(ö/ä adjusts how long)`. Outside a park prompt, `ö`/`ä` were bound to the **gripper step**. His log shows the result, `gripper step 0.200 per press`, which is the ceiling. **Every later `o` or `c` would then move the jaws a fifth of their travel.**

⛔ **This is [§27.4](FINDINGS.md) again, at a different keystroke.** That entry recorded a stale hint advertising a key that did nothing where it was shown. This is a live hint advertising a key that does *something else* where it is shown, which is worse: the first wastes a press, the second changes a setting silently.

⭐ **The fix is one meaning per key, not a cleverer message.** `ö`/`ä` (and `[`/`]`) now adjust the ease ramp everywhere. The gripper step became `--gripper-step`, on his judgement that it does not need to be live. ⚠️ **A message that has to explain which of two things a key does today is a design admitting it is wrong.**

### 30.5 ⭐ And his question deserved a real answer: what is easing for, outside a park?

*"The easing outside of parking, I don't really know what that means. Does it work for recording, or does it work for teleoperating? Or does it not work outside of parking, really? What's the point of that?"*

**Neither.** Easing shapes how a **planned** move starts and stops, which means `p` runs and the Ctrl-C park, and nothing else. Driving by hand has no plan to shape. A playback follows the timing it was taught, so easing does not apply there either. Pressing `e` elsewhere configures the next planned move.

⭐ **So the message now says where the effect lives**, every time the key is touched: `ease s-curve over 0.20 rad · affects p runs and Ctrl-C only · ö/ä = how long`. A knob whose effect you cannot see has to name its own scope.

### ⭐ What confirmed itself in the same run, at no cost

- **The session-19 display fixes hold.** Not one message in four sessions of log has a fragment of the status line welded onto it. That was the defect behind *"hold sometimes gets overwritten"* ([§27](FINDINGS.md)).
- **`ö`/`ä` arrive at all**, so the UTF-8 decoder works ([§27.7](FINDINGS.md)).
- **`e` works outside a park prompt**, which was the other half of §27.4.
- **Parking to a recording's start pose and handing over to playback works**, four times, including one interrupted by Ctrl-C with a clean park and disable.
- ⚠️ Still never seen: the `⭐ MODE: HOLD` banner on its own (he never pressed `h` alone), the 55 °C warning, and the blind-thermal stop.

---

## 31. ⛔⭐ Three more things from the same day, and one of them may be free depth — 2026-08-13

### 31.1 ⛔⭐ The control loop runs at about 87 Hz, and nothing said so for weeks

⭐ **It was found because a playback summary did not add up.** Recording 6 is 3.6 s. The summary read *"3.6s of movement at 1.00x, plus 0.4s waiting for the arm to catch up"* and then *"PLAYBACK finished in 4.6s"*. **0.6 s is missing**, and 4.0 / 4.6 = 0.87.

⛔ **The cause.** `dt = 1.0 / CONTROL_HZ` is a constant, and the bottom of the loop sleeps `max(0, dt - elapsed)`. A cycle that overruns is therefore **not compensated**: nominal time falls behind the wall clock and never catches up. Every quantity computed from `dt` is then in nominal seconds while the operator is watching real ones.

**What that affected:**

| thing | consequence |
|---|---|
| **playback duration** | a 3.6 s recording took 4.6 s. Julien read this as the feature being wrong, and roughly two thirds of the difference was this rather than the lag holds |
| **park speed** | ⚠️ a park at "0.40 rad/s" actually moves at about 0.35. **Not changed**, on purpose: it is slower than stated, which is the safe direction, and he has tuned his speed preferences against the current behaviour. Changing it would silently speed every park up by 15% |
| **the reported waiting time** | understated by the same 13% |

⭐ **Fixed for the playback only**, which now advances on a measured `real_dt`, clamped to 0.1 s so one long stall cannot make the cursor jump. ⭐ **And two instruments were added so this cannot hide again:** the loop rate appears in the status line whenever it falls below 92% of `CONTROL_HZ`, and the playback summary now checks that its own numbers reconcile with the wall clock and says so when they do not.

⛔ **The lesson is the one this file keeps paying for.** Nothing was broken enough to notice. The loop ran, the arm moved, every number on screen looked plausible. **It was caught by printing two quantities that had to agree and noticing that they did not** — the same method that caught the recording padding in [§30.1](FINDINGS.md) on the same day. ⭐ **Printing a redundant number is cheap and it is the only thing that has found this class of defect twice.**

⚠️ **Why the loop is slow has not been measured.** Candidates: the per-cycle CAN read, the IK solve in TELEOP, the status line, or `robot.get_joint_pos()` costing more than assumed. The readout now makes it visible, so the next session can watch which mode is slow.

### 31.2 ⭐⭐ The D405 at 848x480 is probably DEPTH, arriving over plain UVC with no SDK

His screenshots on 2026-08-13, all from `--camera d405 --term`:

| capture | what the picture looked like |
|---|---|
| 640x480 | ✅ a clean colour photograph of the room |
| 1280x720 | ✅ a clean colour photograph |
| **848x480** | ⛔ **smooth diagonal coloured bands**, twice, and nothing recognisable |

⭐ **Smooth diagonal banding with cycling colour is the signature of 16-bit data read as 8-bit BGR triplets.** 848 pixels of 16-bit is 1696 bytes a row. Read as `848 x 3` it wants 2544. The mismatch shears every row sideways, and consecutive byte pairs cycle through blue, green and red, which is exactly the pattern in both images.

⭐⭐ **If that is right, `848x480` is the D405's depth stream and it is reachable with no SDK at all.** That answers the open question left in [§28.2](FINDINGS.md), and it matters because `librealsense` only works as root on this Mac ([§28.4](FINDINGS.md)), which rules it out for the control loop. **848x480 is a native RealSense depth resolution and no colour webcam offers it**, which is also why `--list` uses a RealSense-only mode to identify the camera in the first place.

⚠️ **This is an inference from two images plus arithmetic. It has not been measured.** ⭐ **The number that settles it is the pixel format, so the readout now shows it.** Run `--camera d405 --term`, press `5` for 848x480, and read the four letters after the capture size. If they differ from what 640x480 and 1280x720 report, the inference holds.

⛔ **Do not "fix" 848x480 into a picture before this is settled.** If it is depth, the right change is to read it as 16 bits and colour-map it, which is a feature. Forcing it into an 8-bit photograph would throw away the depth this project has been trying to reach since [§8](FINDINGS.md).

### 31.3 ✅ The image no longer shrinks over time

His two 848x480 screenshots read `sent 699x396` and then `sent 710x402`. ⭐ **The width went UP between them**, which is the behaviour [§29](FINDINGS.md) was rewritten to restore. The old controller could only ever go down from a width whose cost landed in its dead band.

### 31.4 The HOLD banner after a park or playback is deliberate

Julien: *"this normal HOLD mode keeps on showing up, but when hold gets activated after the recording, it doesn't. Is that intended? If this is sensible and more UX UI friendly, then leave it as is."*

✅ **It is intended, and it is left as is.** Pressing `h` prints `⭐ MODE: HOLD` because nothing else announces the change. A park or a playback ending already says where it went, in the same line that reports how it went: `⭐ PARK reached in 0.7s (0.023 rad off) → HOLD`. Printing a second banner underneath would be one event described twice.

⚠️ **Checked rather than assumed:** every path that sets `mode = "hold"` prints something. The `h` key, both park outcomes, both playback outcomes, the thermal stop and the quit flow. **There is no route into HOLD that happens silently.**

---

## 32. ⛔⭐ Both CAN adapters were in DFU mode, and the error could not say so — 2026-08-13

Julien's session died at startup:

```
⛔ RuntimeError: No candleLight CAN adapter found. Is the arm's CANable plugged in?
```

Both adapters were plugged in. ⭐ **`ioreg` shows why, and it is unambiguous:**

```
"USB Product Name" = "DFU in FS Mode"    "USB Serial Number" = "2081337C594E"
"USB Product Name" = "DFU in FS Mode"    "USB Serial Number" = "20593383594E"
```

Those serials are the first 12 characters of the two known CANables (`2081337C594E5018` and `20593383594E5018`). **Both boards had entered DFU mode**, the chip's built-in firmware-update bootloader. Measured identifiers: **VID `0x0483`** (STMicroelectronics), **PID `0xDF11`**, `Device Speed = 1` (full speed), `bcdDevice = 512`. That is the standard STM32 DFU bootloader.

### Why the message could not describe it

`GsUsb.scan()` looks for candleLight devices and correctly found none, because a board in DFU mode presents the bootloader instead. ⚠️ **The old text was not wrong. It was unable to describe the situation**, so it sent him to check cables while the cause was a device state. That cost a round trip.

⭐ **Two properties of DFU mode make this specifically hard to recognise:**

1. **The product name changes completely**, from `CANable 2.5 Candlelight` to `DFU in FS Mode`, so nothing in a device listing looks like a CAN adapter.
2. ⛔ **The serial is TRUNCATED to 12 characters.** So even a listing that showed it would not match `ARM_SERIALS`, and a human scanning for `2081337C594E5018` would miss `2081337C594E`.

### ⭐ The fix in the code, and it pays for itself immediately

`src/yam_can.py::adapters_in_dfu_note()` runs when the adapter scan comes back empty. It looks for `0x0483:0xDF11` over the same libusb that `GsUsb.scan()` uses, and if it finds any, says so, lists the truncated serials, warns that they will not match `ARM_SERIALS`, and gives the fix. ⚠️ **It never raises**: a diagnostic that fails must not replace the error it is explaining.

### What to actually do

⭐ **First try: unplug both CANables and plug them back in, without holding any button on the board.** A normal power-up runs the application firmware rather than the bootloader, so this usually clears it. Then confirm:

```bash
ioreg -p IOUSB -w0 -l | grep -i candlelight
```

⚠️ **If they come back in DFU again, the firmware needs re-flashing with `dfu-util`**, which is a much bigger job and has not been done on this rig.

### ⛔⭐ 32.0 UPDATE, same day: replugging did NOT clear it, and the likely reason is a BOOT jumper

Julien replugged and `ioreg` still shows both boards as `DFU in FS Mode`. ⭐ **That narrows the cause a great deal, because DFU mode is entered at reset and persists until the next reset.** If a plain power-up still lands in the bootloader, then something is asking for the bootloader *every single time*.

⭐⭐ **The CANable has a BOOT jumper or button on the board, and its whole purpose is exactly this.** The vendor firmware instructions say to *"move the boot jumper into the boot position as labelled on the PCB and then plug it into your computer"*, and afterwards *"return the boot jumper to its original position"*. **A jumper left in the boot position forces DFU on every power-up, and no amount of replugging will ever clear it.**

⛔ **So look at the two boards before considering anything else.** Both being affected at once fits a physical cause far better than a software one: they sit together on one small hub, so one knock or one bag can reach both.

⚠️ **And a second thing to check, which would explain a failed replug even with the jumpers correct.** DFU mode only clears on a **reset**, which needs power actually removed. ⛔ **Unplugging the hub from the Mac does not necessarily cut power to the hub's downstream ports.** If the hub stayed powered, the boards never lost power and never reset. **Unplug each CANable from the hub itself**, wait a few seconds, and plug it back.

### ⭐ The recovery ladder, cheapest first

1. ⭐ **Look at the BOOT jumper or button on each board.** If either is in the boot position, move it back. This is free and it is the most likely answer.
2. **Unplug each CANable from the hub directly**, not the hub from the Mac, so the board genuinely loses power. Wait a few seconds.
3. **Try a different port, and try one board on its own**, to rule out the hub.
4. ⚠️ **Only if all of that fails does the firmware need re-flashing.** `brew install dfu-util`, then something of the shape:

   ```bash
   dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D <candlelight-for-canable-2.5>.bin
   ```

   ⛔ **DO NOT run that without the right firmware file for a CANable 2.5.** The wrong image bricks the adapter, and this has never been done on this rig. Sources: the `candle-usb/candleLight_fw` project and `canable.io`. ⚠️ With two boards attached, select one with `-S <serial>` or the command may flash the wrong one.

### ⛔ Why they entered DFU is still UNKNOWN, and that matters if it recurs

No cause was established. The candidates, none of them verified:

- **The board's BOOT button**, pressed or knocked. Both boards sit on one small hub, so one knock could reach both.
- **A power event on that shared hub.** If the chip resets and the application does not start, it stays in the bootloader. Both being affected together fits a shared power cause better than anything else.
- **A USB DFU_DETACH request from software.** ⚠️ Nothing in this repo sends one, and the gs_usb protocol has no bootloader jump, so this is the least likely.

⭐ **If it happens again, record these before touching anything:** whether it was one adapter or both · whether anything was re-plugged or knocked · whether a session had just exited cleanly · and whether the hub itself re-enumerated. **That is the data that would separate a power cause from a button.**

### ⭐ The pattern this makes, and it is now three for three

Every confusing failure in two days has been **a device that is present but not in the state the code assumes**:

| | the device was | the code assumed |
|---|---|---|
| [§28.4](FINDINGS.md) | on the bus, claimed by macOS's own driver | librealsense could claim it |
| [§28.5](FINDINGS.md) | reporting two different serial numbers | a device has one serial |
| **§32** | on the bus, running its bootloader | present means ready |

⛔ **So "is it plugged in?" is the wrong first question on this rig.** The useful one is *"what state is it in?"*, and every device-not-found message should be able to answer it. This one now can.

### 32.1 ⚠️ The camera in the same session — NOT reproduced, and it looks healthy

He also reported the D405 could not be found. **Everything measurable about it is fine**, checked while the CAN adapters were still in DFU mode:

- It is on the bus, serial `255323071773`, **SuperSpeed** (`Device Speed = 3`), on a SuperSpeed hub.
- `locationID` is `0x01210000`, **the same port as this morning**, so it has not moved.
- All four USB interfaces are registered and matched, and `UVCAssistant` is attached to the `RGB` interface.

⚠️ **The USB enumeration ORDER did change** between the two readings, which is exactly the hazard [§22](FINDINGS.md) exists for. ⭐ **But the camera code survives that by construction**: `config/camera_index_hint.json` keys on the stable macOS `unique_id` rather than on an index, and the code re-verifies by asking the camera at that index a question only it can answer, falling through to every other index if the answer is wrong.

⛔ **So this is unexplained and needs his error text.** `ioreg` cannot show it, and the agent cannot open a camera ([§21.1](FINDINGS.md)). The command that would settle it:

```bash
uv run scripts/camera_view.py --list
```

⚠️ One candidate worth checking in that output: **Google Chrome holds a device user client on both cameras**, visible in the `IOService` plane. It also held them earlier while the camera worked, so it is not obviously the cause, and it is the cheapest thing to eliminate.

### 32.2 ⚠️ Changing the controls did NOT break anything, but it changed BOTH arms

He asked whether editing the controls could have caused this. ✅ **It could not.** The axis map is a JSON file of puck-to-motion assignments and it cannot affect whether a USB device enumerates.

⭐ **It did do something he should know about, though.** `config/spacemouse_map.json` changed under the `"shared"` key, which means **the edit applies to arm B as well as G**. The world frame's rotation wiring moved:

| | before | after |
|---|---|---|
| source | `[1, 0, 2, 5, 3, 4]` | `[1, 0, 2, 4, 3, 5]` |
| sign | `[1, 1, -1, -1, 1, -1]` | `[1, 1, -1, 1, 1, -1]` |

In the plan line that reads `ROLL←yaw− PITCH←roll+ YAW←pitch−` before and `ROLL←pitch+ PITCH←roll+ YAW←yaw−` after. ✅ **The previous version is safe in `config/spacemouse_map.prev.json`**, which the session writes before saving. `--fork-map` gives G its own map if the arms should differ.

### 32.3 ⛔ A process finding: `git add -A` can sweep his measured calibration into my commits

`config/spacemouse_map.json` was modified **after** my last commit, so it happened to escape. ⚠️ **It would not have if the timing had differed by ten minutes.** Every commit in this repo has used `git add -A`, so an axis map or a gripper calibration that Julien tuned between commits gets included in an unrelated change with a message that never mentions it.

⛔ **That is the same class of defect as everything in [§0](FINDINGS.md): the record would look correct and be wrong.** A month later, `git log` on the map file would blame a commit about a camera bug for a change he made with his hands.

⭐ **The rule from now on: check `git status config/` before committing, and give a config change its own commit with its own message.** `config/` holds *measured* values, not settings, so a change to it is evidence and deserves to be recorded as such.

---

## 33. ⛔⭐ THE RIG IS BACK UP, and three written claims about it had already gone stale — 2026-08-13, 15:22

⭐ **All three findings in this section come from re-deriving what the documents assert, rather than from new hardware work.** Nothing was energised and nothing moved. That is the point: [§0](FINDINGS.md)'s rule is about values, and a document is a value too.

### 33.0 ✅ Both CAN adapters are out of DFU mode, and the recovery ladder was never needed

⛔ **[§32](FINDINGS.md) and the handoff both opened with "THE RIG IS DOWN".** That is no longer true. Measured at 15:22:

```
"USB Product Name" = "CANable 2.5 Candlelight"   "USB Serial Number" = "2081337C594E5018"   (arm B)
"USB Product Name" = "CANable 2.5 Candlelight"   "USB Serial Number" = "20593383594E5018"   (arm G)
```

**Three things separate this from the DFU state, and all three flipped back:**

| | in DFU (§32) | now |
|---|---|---|
| product name | `DFU in FS Mode` | `CANable 2.5 Candlelight` |
| serial | truncated to 12 chars | full 16 chars, matching `ARM_SERIALS` |
| `idVendor` | `0x0483` (STMicroelectronics bootloader) | `7504` = `0x1D50` (candleLight application firmware) |

⭐ **And it was confirmed through the real code path, not only through `ioreg`.** `uv run scripts/probe_can.py --seconds 3` opened the adapter, computed the 1 Mbit/s bitrate from a 160 MHz clock, and reported **`✓ listen-only granted`**. That proves libusb can claim the board and the gs_usb protocol answers, which a device listing cannot. Zero frames in 3 s is the expected reading for a healthy idle arm, for the reason written at the top of that script.

⛔⭐ **WHY THEY RECOVERED IS UNKNOWN, and that is now the second unknown on the same fault.** §32 already recorded that the *cause* was never established. The *recovery* is equally unexplained: no agent touched the boards, and the recovery ladder in [§32.0](FINDINGS.md) was never run. **Julien did something between 14:55 and 15:22 and it is not written down.** ⚠️ The ladder's step 1 hypothesis — a BOOT jumper left in the boot position — predicts that only moving a jumper clears it, so if he simply replugged again, that hypothesis is **wrong** and the real cause is still live. **This question is in [HANDOFF §5.5](HANDOFF.md) task 0 and it is worth 30 seconds of his time**, because the difference decides whether this recurs mid-session.

### 33.1 ✅⭐ The `w` freeze fix IS confirmed on hardware — by recordings that were already on disk

⛔ **The handoff said: "the saved recordings in `recordings/` (slots 1, 3, 4, 5, 6) are all PADDED and should be discarded rather than used."** ⭐ **Two of the five are padded. Three are clean, and they are the evidence that the fix works.**

⛔⭐ **READ THIS BEFORE THE TABLE — it is a dated record, not the current state.** Julien recorded over slots **3 and 4** at 16:34 and 16:35, about an hour after this was written, so the two padded files below **no longer exist** and every recording now on disk is clean. That is the third time in one day that a written measurement outlived the file it described ([§34.7](FINDINGS.md)). ⭐ **The table stays exactly as measured**, because it is the *evidence* that the `w` freeze fix works, and a decision's evidence does not expire. For the current state run **`uv run scripts/check_recordings.py`**.

Measured with `Trajectory.trailing_still_seconds()` at 15:22, before those two re-recordings:

| slot | commit | recorded | duration | padding | share | verdict |
|---|---|---|---|---|---|---|
| 3 | `e89b745` | 09:34 | 7.78 s | **4.46 s** | **57.3 %** | ⛔ before the fix |
| 4 | `e89b745` | 09:35 | 6.02 s | **2.64 s** | **43.9 %** | ⛔ before the fix |
| 1 | `0e268ed` | 12:55 | 7.72 s | 0.03 s | 0.5 % | ✅ after the fix |
| 5 | `0e268ed` | 12:36 | 1.63 s | 0.00 s | 0.0 % | ✅ after the fix |
| 6 | `0e268ed` | 12:41 | 3.55 s | 0.25 s | 7.0 % | ✅ after the fix |

⭐ **The commit column is what settles it.** The `w` freeze landed in `0e268ed` at 10:01 (`take_to_save`, frozen and waiting for its slot digit). Slots 3 and 4 were saved at 09:34 and 09:35 under `e89b745` and carry 4.46 s and 2.64 s — which independently reproduces [§30.1](FINDINGS.md)'s figures of *"1.8 to 4.4 seconds"* and *"recording 3 was 56% padding"* (57.3 % here). Slots 1, 5 and 6 were saved nearly three hours later under `0e268ed` and carry none.

⭐ **All five are hardware recordings.** `method` reads `live:guide`, which is written in exactly one place, `teleop_session.py`, and that script has no simulation, fake or mock mode — `build_robot()` resolves a real adapter by serial and refuses otherwise. So these are the real arm, in GUIDE, with a hand on it.

⛔ **So [HANDOFF §5.5](HANDOFF.md) task 0 drops from three unverified changes to two.** Item (1), *"`w` now freezes the recording immediately"*, is confirmed. ⚠️ **What is confirmed is that the multi-second padding is gone from the saved file**, which is the harm. Whether the sample count in the on-screen stop message matches the saved count is still unobserved, and it is free to check during the next recording.

⚠️ **Slot 6's 0.25 s is not residual padding.** The defect produced 1.8 to 4.4 s; a quarter of a second is the arm coming to rest before the key was pressed. The measured gap between the two cases is more than an order of magnitude, which is why one threshold separates them cleanly.

⛔⭐ **A padded tail is NOT motionless, and this is the trap that made the check look impossible.** A weightless arm held by a hand wobbles at a **flat 0.032 to 0.038 rad/s** for as long as it is held — the two padded tails sit dead level on that floor for seconds. A first attempt using 0.02 rad/s as "still" reported **zero padding on all five files** and would have confirmed the wrong answer. The threshold is 0.05 rad/s and it sits above the wobble floor by measurement, not by taste.

⭐ **It is now a script rather than a sentence: `uv run scripts/check_recordings.py`.** No hardware, reads only, and it prints the table above from whatever is actually on disk. Four tests cover the function, including the one that fails if the threshold is put back below the wobble floor.

### 33.2 ⛔⭐ The speed table in `joint_speed`'s docstring went stale in under three hours, and nothing could see it

**The docstring in `src/recording.py` gave slot 1 as `max 0.78, p99 0.68, p95 0.59, median 0.29`.** Re-measured now, slot 1 is `max 0.49, p99 0.43, p95 0.38, median 0.20`. ⭐ **Slots 3 and 4 still reproduce to the digit**, so the code is right and the file changed.

⭐ **What happened:** the table was written into commit `0e268ed` at **10:01**. Julien recorded over slot **1** at **12:55**. ⛔ **`recordings/` is gitignored**, so the file those four numbers describe no longer exists anywhere and cannot be recovered.

⛔ **The general defect, and it applies to every number this repo writes about a recording: saving by slot digit overwrites silently.** Any measurement of "recording N" expires the moment N is reused, and the document keeps asserting it. **The fix is cheap and already half in place** — quote `commit` and `recorded_at` beside every measurement. Those two fields are the only reason the mismatch was detectable at all, and they exist because Julien asked for provenance on 2026-08-12 ([ROADMAP §6.6](ROADMAP.md)).

⭐⭐ **Provenance has now paid for itself twice in one day, before any dataset exists.** Once to prove which recordings postdate the `w` fix (§33.1), once to explain a stale table (§33.2). That is a stronger argument for [ROADMAP §6.6](ROADMAP.md)'s provenance requirement than the requirement's own rationale.

### 33.3 ⭐ The pattern, and it is [§0](FINDINGS.md) pointed at the documents

Three claims, all written carefully, all wrong within a day of being written:

| the claim | why it went wrong |
|---|---|
| "THE RIG IS DOWN" | a hardware state changed and no one re-checked it |
| "slots 1, 3, 4, 5, 6 are all padded" | written from *when the defect existed*, not from the files |
| slot 1 is `max 0.78, p99 0.68` | the file was overwritten three hours later |

⛔ **None of them was careless, and that is the finding.** Each was true when written. [HANDOFF §4](HANDOFF.md) rule 7 already says this about guards — *ask of every guard what path reaches the hazard without passing through you* — and [§32.3](FINDINGS.md) said it about commits. **The same rule applies to any sentence containing a measurement: it is a cached value with no invalidation.**

⭐ **The practical defence, and it is the one this repo already uses everywhere else: make it a script.** `check_recordings.py` cannot go stale, because it reads the files. `check_links.py` cannot go stale, because it resolves the links. ⛔ **A measurement written in prose and never re-derived should be treated as an assertion about the past, and dated.** Where a number matters, either date it with its provenance or replace it with the command that recomputes it.

---

## 34. ⭐⭐ TWO HARDWARE RUNS SETTLED THE STACK, AND ANSWERED "HOW FAST CAN THE ARMS MOVE" — 2026-08-13, 16:34-16:35

⭐ Julien recorded and played back two movements on arm B and pasted both terminal logs. **All three of the stacked unverified changes are now confirmed**, one real defect came out of the logs, and the per-joint table gives the **first measurement on this rig of what the arm can physically follow**. Nothing here needed a new script run on the arm.

### 34.0 ✅✅ All three changes are confirmed, and each has its own evidence

| # | the change | how it was confirmed | verdict |
|---|---|---|---|
| 1 | `w` freezes the recording immediately | The stop message and the save message report the **same** sample count and duration, twice: `3.7s, 326 samples` → `3.7s, 326 samples → 3.json`, and `5.4s, 471 samples` → `5.4s, 471 samples → 4.json`. Both files measure **0.00 s** of trailing still time. | ✅ |
| 2 | Playback runs in measured time | Run B: a **5.39 s** recording played at 1.00x **finished in 5.4 s** with **0.0 s** of waiting. The old defect ran about 15% long. | ✅ |
| 3 | A per-joint table prints after each playback | It printed in both runs, with six rows each. | ✅ |

⭐⭐ **And the code's own reconciliation check passed, which is the part worth noticing.** [§31.1](FINDINGS.md) added a guard that recomputes `elapsed − planned − waiting` and complains if it does not come out near zero. Run A: `7.2 − 3.7 − 3.5 = 0.0`. Run B: `5.4 − 5.4 − 0.0 = 0.0`. **No warning fired in either run, and the warning is real code that would have fired.** That is a redundant number agreeing with itself, which is the only class of evidence that has ever caught this defect ([§31.1](FINDINGS.md)'s own closing line).

⚠️ **Run A looks like a failure and is not.** It played a **3.7 s** recording in **7.2 s** and spent **48%** of the run waiting. The recording was taught at **3.20 rad/s** (99th percentile) while any planned motion here is allowed **1.5 rad/s**, so 1.00x asks for more than the arm may be commanded to do and the loop holds its clock. ⭐ **`replay_plan_line()` warns about exactly this before Enter is pressed** — *"taught 3.2 rad/s exceeds the 1.5 allowed, so 1.00x will lag"* — and the session had auto-selected **0.47x** for that recording. Playing at 1.00x took four presses of `+`. **The warned-about thing happened, which is the system working.**

### 34.1 ⭐⭐ HOW FAST CAN THE ARMS MOVE — answered, and the prediction in the task list was WRONG

⛔ **The prediction on record was:** *"Expect the three wrist joints to look far worse than the shoulder and elbow, since their position gain is 10 against 80. If they do, the gains are the limit rather than the 1.5 rad/s clamp."* ⭐ **The gains are real and the prediction is refuted.**

**The gains, read out of the vendor config** (`third_party/i2rt/i2rt/robots/config/yam_v1.yml`, and our code passes `arm_type=ArmType.YAM` explicitly at two call sites in `src/yam_robot.py`):

```
kp: [80.0, 80.0, 80.0, 10.0, 10.0, 10.0]
kd: [ 5.0,  5.0,  5.0,  1.5,  1.5,  1.5]
```

⛔⭐ **SUPERSEDED IN PART — read [§35.2](FINDINGS.md) before using any number below this line.** A third run at 17:21 served as held-out data, and refitting with all three moved two of this section's headline figures: the delay ratio between the soft and stiff joints went **1.13x → 0.97x**, the offset ratio went **1.7x → 2.16x**, and the first joint to reach the 0.15 rad threshold went **`gripper_twist` at 2.16 rad/s → `forearm_pitch` at 1.89 rad/s**. ⭐ **The conclusion of this section is unchanged and got sharper**, so the fit below stays as the record of what two runs supported. ⚠️ **The held-out test also measured this model's real precision: ±25% on any single point.** The prediction table at the end of this section was tested and came out optimistic.

**The raw measurements, both runs, exactly as printed:**

| joint | kp | run A worst lag @ speed | run A top speed / lag | run B worst lag @ speed | run B top speed / lag |
|---|---|---|---|---|---|
| base_yaw | 80 | 0.117 @ 1.30 | 2.43 / 0.101 | 0.040 @ 0.44 | 0.54 / 0.040 |
| shoulder_pitch | 80 | 0.181 @ 3.75 | 3.75 / 0.181 | 0.076 @ 0.82 | 0.97 / 0.073 |
| elbow_pitch | 80 | 0.107 @ 1.76 | 1.99 / 0.081 | 0.056 @ 0.32 | 0.40 / 0.045 |
| forearm_pitch | 10 | 0.170 @ 2.38 | 2.93 / 0.169 | 0.116 @ 0.69 | 1.03 / 0.090 |
| wrist_roll | 10 | 0.175 @ 3.22 | 3.22 / 0.175 | 0.100 @ 0.54 | 0.84 / 0.073 |
| gripper_twist | 10 | 0.172 @ 2.12 | 2.95 / 0.169 | 0.082 @ 0.68 | 0.68 / 0.082 |

**Least squares on all four points per joint, fitting `lag = offset + delay × speed`:**

| joint | kp | kd/kp (s) | offset (rad) | delay (s) | R² | speed at the 0.15 rad hold |
|---|---|---|---|---|---|---|
| base_yaw | 80 | 0.0625 | 0.035 | 0.0339 | 0.59 | 3.41 rad/s |
| shoulder_pitch | 80 | 0.0625 | 0.041 | 0.0372 | 1.00 | 2.92 rad/s |
| elbow_pitch | 80 | 0.0625 | 0.042 | 0.0269 | 0.73 | 4.01 rad/s |
| forearm_pitch | 10 | 0.1500 | 0.078 | 0.0334 | 0.81 | 2.17 rad/s |
| wrist_roll | 10 | 0.1500 | 0.064 | 0.0341 | 0.91 | 2.52 rad/s |
| gripper_twist | 10 | 0.1500 | 0.057 | 0.0430 | 0.89 | **2.16 rad/s** |

⭐⭐ **THE RESULT, AND IT IS A DIFFERENT SHAPE OF ANSWER THAN EXPECTED.** The lag splits cleanly into two parts, and **only one of them follows the gains**:

- **The speed-dependent part is a DELAY of about 0.033 s, and it is nearly the same on every joint.** Mean 0.0327 s on the kp=80 joints against 0.0369 s on the kp=10 joints. **A ratio of 1.13.** The prediction from kp alone was **8.00**. Even the more careful prediction from `kd/kp` was **2.40**. Both overshoot badly.
- **The standing offset DOES follow the gains.** 0.039 rad mean on the kp=80 joints against 0.066 rad on the kp=10 joints, a ratio of **1.7**. This is the same quantity PARK reports as *"as close as the arm holds itself under load"* (0.020 and 0.027 rad in these two runs), and a fixed torque requirement divided by a smaller gain is exactly where it should show up.

⭐ **Why `kp` alone was the wrong model, and it is worth understanding rather than memorising.** Following error while moving is not set by `kp` on its own. With position commands only, the damping term acts as a brake proportional to speed, so the error scales as **`kd/kp`** — and `kd` drops alongside `kp` (5 → 1.5), so the ratio that matters is 0.15/0.0625 = **2.4**, not 8. ⚠️ **And even 2.4 overpredicts by a factor of two**, which says the delay is dominated by something shared by all six joints rather than by any per-joint gain. The candidates, none of them measured: the loop period itself (12 ms at the 83 Hz these runs ran at, so 0.033 s is about 2.7 cycles), the CAN request/response round trip, and the SDK's command pipeline.

⭐ **THE PRACTICAL ANSWER: the arm tracks a commanded path up to roughly 2.2 rad/s.** The first joint to exceed the 0.15 rad threshold at which the playback holds its clock is **`gripper_twist`, at 2.16 rad/s**. Above that, a playback still completes — it just takes longer than the recording, and says so.

⚠️ **FOUR REASONS TO READ THAT NUMBER AS A FIRST ESTIMATE AND NOT A SPECIFICATION:**

1. **It is two recordings, in two different parts of the workspace.** Gravity load changes with pose, and load changes the lag at the same speed.
2. **`base_yaw`'s fit is poor (R² 0.59) and its run-A points go the wrong way** — 0.117 rad of lag at 1.30 rad/s but only 0.101 at 2.43 rad/s. **Lag is therefore not a function of instantaneous speed alone.** Acceleration and pose both matter, and this is the visible proof.
3. **The coverage is uneven by construction.** The playback holds its clock once the arm falls behind, so the fast samples are not an even sweep.
4. **The four points per joint come from two summary rows each, not from every cycle.** A saved tracking file ([§34.4](FINDINGS.md)) is the fix, and from now on every playback writes one.

⭐⭐ **A FALSIFIABLE TEST THAT COSTS ONE PLAYBACK AND NO NEW CODE.** Play slot 3 (taught 3.20 rad/s) at each speed the `-`/`+` keys can actually reach, and compare the waiting time against the model:

| speed | commanded p99 | predicted worst lag | prediction |
|---|---|---|---|
| 0.47x (the auto-selected default) | 1.50 rad/s | 0.128 rad | tracks, ~0 s waiting |
| 0.59x | 1.88 rad/s | 0.140 rad | tracks, ~0 s waiting |
| 0.73x | 2.34 rad/s | 0.158 rad | **the marginal case — a little waiting** |
| 0.92x | 2.93 rad/s | 0.183 rad | waits |
| 1.00x | 3.20 rad/s | 0.195 rad | waits — **measured 0.181, so the model is 8% conservative** |

⛔ **The sharp one is 0.73x.** If it waits a little, the model holds. If it tracks cleanly, the model is pessimistic and the real ceiling is higher. ⭐ **Playing below 1.00x is slower than the movement was taught by hand**, so it asks the arm for nothing it has not already physically done with a person holding it.

### 34.2 ⚠️⭐ The 1.5 rad/s ceiling is computed from a loop rate this rig does not reach

`MAX_PLANNED_JOINT_SPEED = MAX_JOINT_STEP * CONTROL_HZ` = `0.015 × 100` = **1.5 rad/s**. The teleop clamp it comes from is **per cycle**: `q_target = prev_q + clip(step, ±MAX_JOINT_STEP)`.

⛔ **The loop does not run at 100 Hz.** These two runs reported **83 Hz** and **84 Hz** on the status line, and the `⚠️ nnHz` warning fired correctly in both (its threshold is 92 Hz). **So the real ceiling on a teleop-driven joint is `0.015 × 83` ≈ 1.25 rad/s, not 1.5.**

⭐ **Nothing unsafe follows from this, and the direction matters.** Both 1.25 and 1.5 sit below the ~2.2 rad/s the arm can actually track ([§34.1](FINDINGS.md)), so the constant is conservative and the error makes it more so. ⚠️ **But it is a number that means something other than what it says**, which is [§0](FINDINGS.md)'s whole subject, and `safe_time_scale` divides by it to choose a playback speed.

⚠️ **A SECOND, SEPARATE INSTANCE OF THE SAME NOMINAL-VERSUS-REAL CONFUSION, and it is still live.** `q_target = teleop.step(twist, dt)` integrates the SpaceMouse twist using the **nominal** `dt` of 0.01 s while the loop takes 0.012 s. A twist is a velocity, so **hand-driven motion runs at about 83% of the speed the display claims.** The file's own comment says *"anything that has to match real time must use `real_dt`, not `dt`"*. ⛔ **Not changed here, on purpose:** it would alter a feel Julien has already tuned, and at high speed settings the per-cycle clamp probably dominates anyway so the change might do nothing. **His call, and it needs a hardware run to judge.**

⚠️ **Also unexplained: the loop rate fell from ~87 Hz to 83-84 Hz.** [§31.1](FINDINGS.md) measured ~87 Hz on 2026-08-13. Candidates, none tested: the **second D405 that appeared on the bus** ([§34.5](FINDINGS.md)), general machine load, or the per-cycle tracking log. ⭐ The tracking file now records `loop_hz`, so the next run measures this instead of noticing it.

### 34.3 ⛔ DEFECT FOUND IN THE LOGS: "PARK reached in 0.0s" was reporting the wrong clock

**Run A printed this, and both lines are from the same park:**

```
  ⭐ slot recording start in 4.4s
⭐ PARK reached in 0.0s (0.020 rad off) → HOLD
```

⛔ **A park that had just taken 4.4 seconds reported 0.0 seconds.** The cause is two clocks sharing one variable. `park_leg_t` is reset every time the cursor passes a waypoint, because Julien asked for each leg to report its own duration. **The last leg's mark is passed at the very end of the path**, so on a single-leg park — which is every playback, since it parks to the recording's start pose — the reset lands moments before arrival, and the arrival message then measured from the reset.

⭐ **Fixed.** `park_start_t` is set once in `begin_path` and never reset. The arrival line now reports the total, and adds the settling time when it is a distinguishable part of it. Run A would now read `PARK reached in 4.4s`; run B, which took 2.2 s of path plus 0.5 s of settling, would read `PARK reached in 2.7s, 0.5s of that settling`.

⚠️ **THE ARM WAS NEVER IN DANGER, AND CHECKING THAT WAS THE FIRST THING WORTH DOING.** "Reached in 0.0s" could mean the park declared success without moving, which would leave playback to command the recording's opening pose as a jump — the exact hazard the park-first design exists to prevent. **It did not happen, and the tracking table proves it:** the worst lag in run A was 0.181 rad, and had the arm still been 1.23 rad from the start pose the first lag reading would have been about 1.23. ⭐ **A display defect and a safety defect look identical in a log until a second, independent number is checked.**

### 34.4 ⭐ The tracking table is now a file, because a paste is not a record

Every playback now writes `recordings/tracking/<slot>_<timestamp>.json`: the per-joint rows plus the arm, slot, commit, playback speed, taught speed, elapsed and waiting time, worst lag, measured `loop_hz`, and the recording's own metadata.

⛔ **Why: the measurement in [§34.1](FINDINGS.md) existed only inside a chat window.** Every number in it was hand-copied out of a terminal. The analysis that produced the delay model could not have been repeated a week later, and this project has now watched three separate written measurements outlive their source inside one day ([§33.3](FINDINGS.md)).

⭐ **Named by timestamp, not by slot.** The recordings themselves are saved by slot digit and overwrite silently, which is what destroyed the files behind two earlier tables ([§33.2](FINDINGS.md), [§34.7](FINDINGS.md)). A tracking file cannot collide with another. ⚠️ The write is wrapped so a failure prints a warning and never ends a session: the arm is in HOLD at that moment and a missing diagnostic is not worth a traceback.

### 34.5 ⛔⭐ A SECOND D405 IS ON THE BUS AND THE C920 IS GONE — nothing announced either

**Measured at 16:52, by two independent methods (`pyusb` enumeration and `ioreg`), which agree:**

| | serial | bus / addr |
|---|---|---|
| RealSense D405 | `255323071773` | 1 / 3 |
| RealSense D405 | `260323072846` | 1 / 8 |

At 15:22 the bus held **one** D405 (`255323071773`) and an **HD Pro Webcam C920**. Now it holds **two** D405s and no C920. ⛔ **Two documents are therefore wrong as written:** [HANDOFF §5.5](HANDOFF.md) item 6 says the second D405 *"is with arm G and still unplugged — only one serial is on the bus"*, and item 5b tells the reader to **read [ROADMAP §7.1](ROADMAP.md) before plugging in the second D405.** That step was skipped, presumably because it happened at the bench rather than in a session.

⭐⭐ **AND ONE THING THE DOCUMENTS FEARED TURNS OUT TO BE FINE.** [ROADMAP §7.1](ROADMAP.md) worried that two identical D405s cannot be told apart, because the identification trick in [§22](FINDINGS.md) asks each camera for a picture size only one model supports and two D405s support the same sizes. **They report distinct USB serial numbers, and `pyusb` reads them with no root at all** — which is a genuine improvement on the position recorded in item 5b, that *"the plain-webcam path cannot read a serial"*. ⛔ **What is still unsolved is the mapping**, and it is the harder half: macOS's USB enumeration order is not OpenCV's device-index order ([§22](FINDINGS.md)), so knowing that serial `2603…` exists does not say which OpenCV index opens it. The wiggle method ([§28.6](FINDINGS.md)) remains the answer for which-camera-is-on-which-arm.

⭐ **This is also the finding that justifies `scripts/check_rig.py`.** The bench changed under an agent that was mid-session, and every document describing the cameras became wrong, silently. One command now prints the state of both adapters, both pucks, every camera and the whole USB topology. **Run it at the start of a session.**

⚠️ **The serial confusion in item 5b is now half-explained and half-open.** It recorded `260322274021` from librealsense against `255323071773` from the USB descriptor, *"same evening, one camera on the bus"*, and treated it as one camera reporting two serials. **The second camera's USB serial is `260323072846`, which shares the `2603` prefix and not the digits.** ⛔ So it is still not settled, and the honest reading is that nobody knows which number belongs to which camera. Settle it with both cameras attached: run `sudo rs-enumerate-devices -s` and compare against `uv run scripts/check_rig.py`.

### 34.6 ⚠️⭐ The DFU cause, after his answer — the jumper theory is probably dead and the fault is unexplained

**Julien, asked what he did between 14:55 and 15:22:** *"I think I only replugged them, but I'm not a hundred percent sure. I don't remember what happened, but I also thought it was quite weird."*

⭐ **That is enough to weaken the leading hypothesis a great deal.** [§32.0](FINDINGS.md) argued that a BOOT jumper left in the boot position forces the bootloader on **every** power-up, so **no amount of replugging can ever clear it**. The boards cleared. **If he only replugged, the jumper explanation is wrong.**

⚠️ **His uncertainty is not a dead end, because one candidate already on record explains everything with no mystery left.** [§32.0](FINDINGS.md) also flagged that **unplugging the hub from the Mac does not necessarily cut power to the hub's downstream ports.** DFU only clears on a genuine reset, which needs power actually removed. So: an earlier attempt that moved the *hub* would not have reset the boards, and a later attempt that moved the *CANables themselves* would. **That fits the whole sequence — a failed replug and then a successful one — and requires nothing unusual.**

⛔ **The remaining candidate is worse and cannot be ruled out: an intermittent entry into DFU at power-up.** A floating or marginally-driven BOOT0 pin, or a slow supply rail on the shared hub, lets noise decide at reset. That would recur at random, and it would recur mid-session.

⭐ **The one question that separates them, and it is worth 30 seconds:** **when it cleared, did you unplug the CANables from the hub itself, or the hub from the Mac?** Cables from the hub, and this is closed as a power-cycle that did not happen. Anything else, and it is unexplained and expected to recur.

⭐ **The mechanical defence is now in place either way:** `uv run scripts/check_rig.py --raw` prints which adapters are in DFU, which are absent, and the full bus topology, which is exactly the data [§32](FINDINGS.md) asks to capture before touching anything.

### 34.7 ⛔ Slots 3 and 4 were overwritten, which is the third instance in one day

The two recordings behind [§33.1](FINDINGS.md)'s padding table were **replaced at 16:34 and 16:35** by these very runs. `recordings/` is gitignored, so both files are gone permanently.

| slot | was | is now |
|---|---|---|
| 3 | 7.78 s, 4.46 s padded, p99 2.67 | 3.73 s, 0.00 s padded, p99 3.20 |
| 4 | 6.02 s, 2.64 s padded, p99 2.40 | 5.39 s, 0.00 s padded, p99 0.79 |

⭐ **Written measurements about recordings have now gone stale three times in one day**, twice in text this session wrote itself. [§33.3](FINDINGS.md) predicted this and prescribed the remedy: date a number with its provenance, or replace it with the command that recomputes it. **Both stale tables have been handled the second way** — the `joint_speed` docstring no longer carries a live table at all, and [§33.1](FINDINGS.md)'s table is now explicitly labelled as dated evidence. ⛔ **The lesson is stronger than "be careful": a table of live data inside a document is a cache with no invalidation, and writing it more carefully does not help.**

---

## 35. ⭐⭐ A THIRD RUN TESTED THE SPEED MODEL AS HELD-OUT DATA, AND TWO PUBLISHED NUMBERS NEEDED CORRECTING — 2026-08-13, 17:21

⭐ Julien ran a long session on arm B: three parks including a three-waypoint sequence, a recording, and a playback at **0.607x**. **Everything built in [§34](FINDINGS.md) is confirmed**, the playback happened to be an accidental version of the sharp test [§34.1](FINDINGS.md) asked for, and **fitting the model again with three runs moved two of its headline numbers.**

### 35.0 ✅ The PARK timer fix is confirmed, on three parks, and the arithmetic reconciles

⭐ **This is a redundant-number check, which is the only kind that has ever caught anything here.** The per-leg times and the total are printed independently, so they have to add up:

| park | legs printed | total printed | do they reconcile? |
|---|---|---|---|
| 3-waypoint sequence 2→3→1 | 3.0 + 3.6 + 4.2 = **10.8 s** | **11.3 s**, 0.5 s of that settling | ✅ 10.8 + 0.5 = 11.3 |
| single waypoint, slot 1 | 2.3 s | **3.0 s**, 0.7 s of that settling | ✅ 2.3 + 0.7 = 3.0 |
| single, to a recording's start | 2.6 s | **2.6 s**, no settling shown | ✅ settling was under the 0.05 s floor |

⛔ **Before the fix all three would have read `PARK reached in 0.5s`, `0.7s` and `0.0s`.** The third case also confirms the suppression rule works: the tail is omitted rather than printing a distracting `0.0s of that settling`.

### 35.1 ✅⭐ The saved tracking file is better than the table it replaces, and it earned its keep immediately

`recordings/tracking/3_2026-08-13T17-21-58+02-00.json` exists and holds everything. **Two things it has that the terminal does not:**

1. **Full precision.** `0.15787` rather than `0.158`, which matters when the threshold being tested is `0.15`.
2. ⭐ **A seventh row, `gripper_jaws`.** The terminal skips any joint whose top speed is under 0.01 rad/s, so the jaws never appear on screen. The file records them: worst lag `0.00015` rad at a top speed of `0.00536` rad/s. ⚠️ **That is a useful negative:** the jaws were commanded almost nothing during this playback, which confirms the gripper column of the recording was near-constant and that `n_compare=N_ARM` (leaving the jaws out of the keeping-up check) changed nothing here.

⭐ It also carries `loop_hz: 87.0`, `max_cursor_lag: 0.15`, `max_planned_joint_speed: 1.5`, the playback speed `0.607`, the taught speed `2.4701`, and the recording's own metadata. **Every number needed to re-derive the analysis is in one file.** ⛔ **And reading it found a defect nothing else would have** — [§35.4](FINDINGS.md).

### 35.2 ⭐⭐ THE SPEED MODEL, TESTED ON HELD-OUT DATA AND THEN REFIT — and the corrected answer

**Run C: recording 3 re-recorded (5.53 s, 482 samples, taught 2.47 rad/s at the 99th percentile), played at 0.607x, loop at 87 Hz, motors at 42-43 °C.** Result: finished in **9.694 s** against 9.1 s of movement, so **0.587 s of waiting, 6.1% of the run**, worst lag **0.15787 rad**.

⭐⭐ **THIS WAS EFFECTIVELY THE SHARP TEST, and it did not need arranging.** [§34.1](FINDINGS.md) asked for a playback that would land the worst joint right on the 0.15 rad threshold, to see whether the model held. **The measured worst lag was 0.158 rad, which is exactly on it.** ⛔ **But it happened at a lower commanded speed than the model predicted**, which is the informative part and is why no further speed test is needed.

**STEP 1 — the held-out test. Runs A and B produced the model; run C had no say in it.**

| joint | commanded | measured lag | predicted | error |
|---|---|---|---|---|
| base_yaw | 0.38 | 0.028 | 0.048 | **+72%** |
| shoulder_pitch | 1.03 | 0.059 | 0.079 | +35% |
| elbow_pitch | 0.32 | 0.054 | 0.051 | −7% |
| forearm_pitch | 1.16 | **0.156** | 0.116 | **−26%** |
| wrist_roll | 0.98 | 0.084 | 0.097 | +16% |
| gripper_twist | 1.42 | **0.158** | 0.121 | **−24%** |

⭐ **Mean signed error +4.8%, mean absolute error 24.6%.** So the model is unbiased on average and **wrong by about a quarter on any single point.** ⛔ **Read that as the real precision of this whole measurement.** It predicts the trend and it must not be used to justify a specific speed to within better than ±25%.

**STEP 2 — refit with all three runs, 33 speed-and-lag pairs:**

| joint | kp | offset (rad) | delay (s) | fit rms | reaches 0.15 rad at |
|---|---|---|---|---|---|
| base_yaw | 80 | 0.024 | 0.0395 | 0.020 | 3.20 rad/s |
| shoulder_pitch | 80 | 0.042 | 0.0359 | 0.012 | 3.00 rad/s |
| elbow_pitch | 80 | 0.046 | 0.0250 | 0.010 | 4.18 rad/s |
| forearm_pitch | 10 | 0.100 | 0.0267 | 0.020 | **1.89 rad/s** |
| wrist_roll | 10 | 0.061 | 0.0343 | 0.013 | 2.59 rad/s |
| gripper_twist | 10 | 0.081 | 0.0363 | 0.019 | **1.91 rad/s** |

⛔⭐ **TWO NUMBERS PUBLISHED IN [§34.1](FINDINGS.md) MOVE, AND BOTH MOVE IN THE SAME DIRECTION — the split between the soft and stiff joints is CLEANER than two runs suggested, not messier:**

| | published from 2 runs | refit from 3 runs |
|---|---|---|
| delay ratio, soft ÷ stiff | 1.13x | **0.97x** |
| offset ratio, soft ÷ stiff | 1.7x | **2.16x** |
| mean delay, stiff / soft | 0.0327 / 0.0369 s | **0.0335 / 0.0324 s** |
| mean offset, stiff / soft | 0.039 / 0.066 rad | **0.037 / 0.080 rad** |

⭐⭐ **The conclusion of [§34.1](FINDINGS.md) survives and gets sharper.** The speed-dependent part is a **delay of 0.033 s that is now indistinguishable between the two gain groups** (ratio 0.97, so if anything the soft joints are marginally *faster*). The gains show up **only** in the constant part, and that gap widened to a factor of **2.16**.

⚠️ **Do NOT read the 2.16 as confirming the `kd/kp` prediction of 2.40, even though the numbers are close.** `kd/kp` is the coefficient of the *speed* term in that theory, and the speed term is the one that shows **no** gain dependence at all. Matching it against the *constant* term would be reading a coincidence as a mechanism. ⭐ **What the constant plausibly is:** a torque requirement divided by `kp`. Same torque would give 8x; the wrist joints carry far less mass than the shoulder, so their torque requirement is smaller and partly cancels their smaller gain. **That story is consistent and it is untested.**

**STEP 3 — is speed doing any work at all, or is it all a constant?** A straight line beats a flat line for **every one of the six joints** (fit rms 0.010-0.020 against 0.021-0.045). ⭐ So the speed term is real, and a pure-friction model is not enough.

⭐⭐ **STEP 4 — THE CORRECTED PRACTICAL ANSWER, and it endorses the 1.5 rad/s clamp rather than changing it.**

| commanded speed | joints the fit puts past 0.15 rad |
|---|---|
| 1.0 rad/s | none |
| 1.2 rad/s | none |
| **1.5 rad/s (the clamp)** | **none** |
| 2.0 rad/s | forearm_pitch, gripper_twist |
| 2.5 rad/s | forearm_pitch, gripper_twist |

⛔ **But the scatter is what a person actually experiences, and run C proves it: `forearm_pitch` measured 0.156 rad of lag at only 1.16 rad/s commanded.** The fit puts that at 0.131. With a fit rms of 0.020 rad and a slope of 0.027 s, ±0.020 rad is worth roughly ±0.7 rad/s of crossing speed. **So individual moments cross 0.15 anywhere between about 1.2 and 2.5 rad/s, depending on the pose.**

⭐ **Which is the right answer to Julien's original question, and it is a comfortable one:** *the 1.5 rad/s clamp sits inside the scatter band.* Most motion at the clamp tracks; some waiting at the top end is normal and expected. **Raising the clamp would move it above the band and make waiting the rule rather than the exception, so it should stay at 1.5.** ⛔ **No further speed test is needed for this decision.** The active sweep in [ROADMAP §7.5](ROADMAP.md) is now only worth building to separate the delay's three candidate causes, which is a different question.

⚠️⚠️ **A CONSEQUENCE FOR THE DATASET WORK THAT NOBODY HAS WRITTEN DOWN YET.** When a playback waits, **the replay is slower than the demonstration was.** Run C spent 6.1% of its time waiting, so its timing is 6% stretched and unevenly so, concentrated wherever the fast joints were moving. ⛔ **[ROADMAP §6.6](ROADMAP.md) already requires recording the pose that was actually commanded rather than the nominal plan, which handles positions. It says nothing about timing.** A dataset built from replays therefore carries **human paths at slightly non-human timing**, and how much that matters depends on the model. **Flagged as an open question for the recorder, not resolved here.**

### 35.3 ✅⭐ `check_rig.py` proved itself twice within twenty minutes of existing

1. ⭐ **It caught arm G missing and refused.** `G 20593383594E5018 ⛔ ABSENT from the bus`, and `⛔ VERDICT: not ready — adapter G is not attached`, exit code 1. Julien had unplugged it for a colleague. **The old failure mode was a session dying at startup with a message about cables.**
2. ⛔⭐ **Arm B's USB address changed from `bus 0 addr 4` to `bus 0 addr 5`** between two runs twenty minutes apart, with no cable touched on B. **This is [§1](FINDINGS.md)'s "never select hardware by index" rule caught in the act** — the address moved, the serial did not, and everything in this repo resolves by serial so nothing noticed. *An enumeration order that changes twice in one session was measured on 2026-08-10; it is still changing.*

### 35.4 ⛔ A PROVENANCE DEFECT, found by reading the saved file rather than the screen

The tracking file records the recording's own metadata, and it says:

```
"method": "live:hold"
```

⛔ **The movement was hand-guided in GUIDE.** Julien pressed `w` while in HOLD, then pressed `g` to go weightless, then guided the arm. `method` is written at the keypress, so it names **the mode the recording started in and nothing else.**

⚠️ **This is worse than a cosmetic error, because provenance is the field [ROADMAP §6.6](ROADMAP.md) says matters most.** *"Being able to reproduce everything"* was his own requirement. **A dataset that mislabels how a demonstration was produced is worse than one that omits it**, since a wrong label is trusted and a missing one is investigated.

⭐ **Fixed.** Every mode the recording passes through is now collected, and at stop time the recording gains a `modes` list. When more than one mode occurred, `method` becomes `live:hold+guide`. **Both fields are kept** so anything already reading `method` keeps working. ⚠️ **Not yet seen on the arm**, and it rides along with any recording.

### 35.5 ⚠️⭐ THE DFU CAUSE: both hypotheses are now damaged, and the entry cause is unexplained

**Julien's answer:** *"I also unplugged the hub from the Mac, not the CANable cables from the hub."*

⛔ **That kills BOTH candidate explanations, which is not the outcome [§34.6](FINDINGS.md) expected:**

| hypothesis | what it predicted | what happened |
|---|---|---|
| a BOOT jumper left in the boot position | **no** replug can ever clear it | a replug cleared it ⇒ **dead** |
| the hub kept its downstream ports powered, so the boards never reset | unplugging the *hub* would NOT clear it; only unplugging the *cables* would | he unplugged the hub and it cleared ⇒ **dead as stated** |

⭐ **What is left, and it is the least satisfying option: entry into DFU is intermittent.** Something at power-up occasionally lands the chip in its bootloader instead of its firmware, and a later power cycle happens not to. A floating or weakly-driven BOOT0 pin and a slow-rising supply rail on the shared hub both do this. ⛔ **It will recur, at random, possibly mid-session.**

⚠️ **One weaker variant is worth keeping, because it changes what to do.** [§32.0](FINDINGS.md) noted that DFU only clears on a genuine reset, which needs power actually gone. **A replug that is too quick may not discharge the board.** So the earlier failed attempt and the later successful one may differ only in how long the plug was out. ⛔ **Practical rule from now on: unplug, WAIT ABOUT TEN SECONDS, then plug back in.** It costs nothing and it removes one variable.

⭐ **And the data to capture if it recurs is now one command.** Run `uv run scripts/check_rig.py --raw` **before touching anything**: it answers whether it is one adapter or both, whether they are absent or in DFU, and whether the hub re-enumerated. Then note whether the session had just exited cleanly and whether anything was knocked.

### 35.6 ⚠️⭐ ARM G IS SHARED WITH A COLLEAGUE — an operational fact with a real planning consequence

**Julien, 2026-08-13:** *"I briefly unplugged the arm G from USB so my colleague can use it. Whenever we need both, just let me know, I can instantly get it back."*

⭐ **Arm G is not reliably available**, and that lands squarely on the biggest remaining job. [HANDOFF §5.5](HANDOFF.md) item 0c is the `ArmSession` restructure, whose whole point is driving both arms from one process.

⭐⭐ **Good news, and it is why this changes nothing about the plan's order.** The de-risking sequence in [ROADMAP step 6](ROADMAP.md) starts with **`--arms B` running the N-arm code with N=1**, and that needs only arm B. **So the restructure can be built and its first milestone confirmed with arm G in someone else's hands.** Only steps 3 to 5 need both. ⛔ **Ask for arm G before starting step 3, not before starting the work.**

⚠️ **The camera on arm G stays plugged in even while its CAN adapter is out.** Both D405 serials are still on the bus with arm G absent, so a camera count does not tell you whether an arm is available. `check_rig.py` reports the two independently.

### 35.7 ⛔ Slot 3 was overwritten a FOURTH time, and this one finally did not cost anything

Slot 3 is now the 5.53 s recording from 17:21. It has been overwritten at 16:34 and again at 17:21, and slot 1 at 12:55, and slot 4 at 16:35.

⭐⭐ **The difference this time: the measurement survived.** `recordings/tracking/3_2026-08-13T17-21-58+02-00.json` still holds every number measured from the file that was replaced, at full precision, with the commit and both timestamps. **That is [§34.4](FINDINGS.md) working as intended on its first real test.** The measurements from 16:34 and 16:35 have no such file, because the saving did not exist yet, which is exactly why [§34.1](FINDINGS.md)'s table had to be dated instead.

⭐ **The remaining exposure is the recordings themselves, and it is his to decide.** A slot digit is a convenient way to save and a lossy way to keep. **Options, none taken:** refuse to overwrite without a confirmation keypress; copy the old file aside before writing; or name files by timestamp and let the digits be shortcuts to the most recent. ⚠️ **The first is the smallest change and the most annoying; the third is the most correct and changes how `l` lists things.** Worth one minute of his opinion rather than an agent's choice.

---

## 36. ✅⭐ THE RED BLINKING LIGHTS ARE NOT A FAULT, and step 0 of the restructure is done — 2026-08-13, 18:00

### 36.0 ✅⭐⭐ Both arms are healthy, measured, and identical — so the lights are not reporting a motor error

Julien reconnected arm G and reported **red lights blinking**. ⛔ **Nothing in this repo documents what any LED on this rig means**, so the lights were treated as an unknown and the motors were asked directly instead.

**Two checks, cheapest and safest first.**

⭐ **Check 1, `identify_arm.py --arm G --yes` — register reads only, and it cannot energise anything.** It uses arbitration ID `0x7FF` sub-command `0x33`, which asks a motor's firmware for a stored value and cannot command motion under any circumstances. All seven motors answered:

- Gear ratios **40/40/40/10/10/10/10**, matching [§1](FINDINGS.md).
- ⭐ **Joint 1's `inertia` reads `1.6964389942586422e-05`**, which is arm **G**'s own per-unit calibration value from [§1](FINDINGS.md) (B's is `1.7169109e-05`). **So the serial-based selection addressed the arm it claimed to.** That is the measurement [§0](FINDINGS.md) rule 5 asks for: one that would have *differed* if the claim were wrong.
- **Safety timeout `8000` on every motor, enabled.** A motor with no command for 400 ms enters damping mode by itself.

⭐ **Check 2, `ping_motors.py --arm G --yes` — enables each motor for one frame, reads its reply, disables it.** It sends **no setpoint**, which puts it on the agent's side of [§4](HANDOFF.md) rule 1. The reply carries the error code and both temperatures, and there is no other way to read them.

| | arm G | arm B |
|---|---|---|
| motors online | 7/7 | 7/7 |
| **error codes** | **all `0x1 (normal)`** | **all `0x1 (normal)`** |
| temperatures | 31-35 °C | 31-35 °C |
| velocities | ≤ 0.022, so nothing moving | ≤ 0.022 |
| torques | ≤ 0.035 Nm | ≤ 0.035 Nm |

⭐⭐ **CONCLUSION: no motor is reporting a fault, on either arm, and the two arms are indistinguishable in health.** The warn threshold is 55 °C and the stop is 65 °C, so the temperatures are cool.

⚠️ **What the lights ARE remains undocumented, and the most likely explanation is the safety timeout.** Every motor is set to enter damping mode after 400 ms with no command. An arm that is powered but not being commanded sits in exactly that state permanently. **A blinking LED on a powered, uncommanded motor is the expected indication rather than a warning.** ⛔ **That is a hypothesis and it is not verified** — no datasheet is vendored here and nothing in the repo describes the LEDs. **The cheap test is a glance: if arm B's lights look the same while both arms sit idle, the state is normal.**

⭐ **The gripper positions look alarming and are fine, for the reason [§2](FINDINGS.md) warns about.** Arm G's motor 7 reads `-3.3343` rad against saved limits `[0.1417, -5.0864]`, which contains it. Arm B's motor 7 reads `0.0158` against saved limits `[6.4811, 1.2308]`, which does **not** contain it — until the ±2π wrap correction is applied, giving `[0.198, -5.052]`, which does. ⛔ **Cached raw motor positions are frame-dependent**, `ping_motors` reads the raw frame, and this is the trap that once cost a motor.

⭐ **Worth adding to `check_rig.py` later:** it reports device state and cannot report motor state, because it never transmits. The two-command sequence above is the motor-level answer, and [§5.5](HANDOFF.md) task 0 now carries it.

### 36.1 ⚠️ The USB addresses shuffled again, and the two arms have now swapped positions

| when | arm B | arm G |
|---|---|---|
| 16:52 | bus 0 addr **4** | bus 0 addr **6** |
| 17:40 | bus 0 addr **5** | absent |
| 18:00 | bus 0 addr **5** | bus 0 addr **4** |

⛔ **Arm G now sits at the address arm B held an hour ago.** Anything selecting by index would now be driving the wrong robot, which is [§1](FINDINGS.md)'s warning happening for the fourth observed time. Everything here resolves by serial, so nothing noticed.

### 36.2 ✅⭐ STEP 0 OF THE RESTRUCTURE IS DONE: `ArmSession` now models the park the script actually has

⛔ **The gap, from [ROADMAP §6.1](ROADMAP.md):** the class was committed at 14:16 on 2026-08-12 with a queue of legs and a per-leg speed ramp, which stops dead at every waypoint. `teleop_session.py` replaced that at **15:15 the same day** with a single blended `JointPath`, and that commit's message opens *"the smoothing I built before was the wrong thing"*.

⭐ **What the class has now:** one blended path with an arc-length cursor, waypoint marks that are *reported* rather than stopped at, the cursor waiting when the arm falls behind, arrival gated on the cursor reaching the end rather than on the error, and the two separate park clocks. `step_path()` returns a `ParkStep` carrying every number the script prints, and prints nothing.

⭐ **Tests: 17 → 21, and three of the originals asserted the superseded behaviour.** The one worth naming is `test_each_leg_gets_its_own_ease_in`, which asserted the arm re-eases at every waypoint. **That is the opposite of what blending is for**, so its replacement asserts the cursor does *not* stop at an intermediate waypoint.

⚠️⚠️ **THE LESSON IS BIGGER THAN THE CLASS: THIS FILE WENT STALE IN ONE HOUR AND NOBODY NOTICED FOR A DAY, WITH ALL 17 TESTS PASSING THE WHOLE TIME.** The tests passed because they asserted the old design. ⛔ **An unwired class is a copy of a design, and a copy drifts** — nothing enforces that a change to the script lands in it too. **This is the same defect as [§33.3](FINDINGS.md)'s stale measurements, in code rather than in prose**, and the same remedy does not apply: a script cannot re-derive a design. **The only real fix is to finish the wiring, after which there is one copy.**

⚠️ **Two of the new tests were wrong before they were right, and both mistakes are easy to repeat:**

1. One assumed the **gripper column does not count toward the park error.** It does, like any other joint. A run meant to start at its own final target actually started 0.5 away and tested nothing.
2. One sampled a **fixed `t=4.5` for a blocked verdict.** The stall timer starts when the *cursor* stops advancing, not when the park starts, so a fixed time is a guess about the easing profile and the path length. It read `moving`, which was the stall timer working correctly.

Both now carry a comment saying so.

### 36.3 ⭐⭐ THE SCALE OF STEP 1, MEASURED RATHER THAN ESTIMATED

The documents have said *"~1000 lines of `main()`"* since 2026-08-12. **Measured:**

- **`main()` spans lines 500-2305 of `teleop_session.py`: 1806 lines.**
- **338 references** to the 20 state names that have to become `arm.<field>`.

| name | refs | | name | refs |
|---|---|---|---|---|
| `mode` | 93 | | `park_ramp` | 17 |
| `robot` | 43 | | `park_speed` | 15 |
| `teleop` | 37 | | `park_path` | 14 |
| `gripper_value` | 13 | | `park_s` | 12 |
| `ease_idx` | 10 | | `prev_q` | 9 |
| `park_leg_t` | 9 | | `park_marks` | 9 |
| `park_cmd` | 8 | | `park_best_err` | 8 |
| `park_progress_t` | 8 | | `blend_idx` | 8 |
| `guide_ref` | 7 | | `home_ee` | 6 |
| `park_target` | 6 | | `park_start_t` | 6 |

⛔ **`mode` alone is 93 sites, and every one of them is in code that commands 4.3 kg on a rig with no emergency stop.** ⭐ **This measurement is the argument for doing step 1 as its own session and nothing else** — the repo already said so on instinct, and now there is a number behind it. It is also the argument for step 1 landing as **one mechanical change with no behaviour change at all**, so that `--arms B` at N=1 tests exactly one thing: whether 338 substitutions were made correctly.

### 36.4 ⚠️ `park_speed_factor()` is now used by nothing but its own tests

`easing_factor()` superseded it in the script on 2026-08-12, and [§36.2](FINDINGS.md) removed the class's last use. It still has 9 tests in `scripts/test_park_target.py`. ⭐ **Left in place deliberately**, under Julien's standing rule to change nothing that does not have to change. **Recorded so it is a decision rather than an oversight**, and so that whoever eventually removes it knows the tests go with it.

### 36.5 ⛔⭐⭐ A SYSTEMATIC DIFF OF `ArmSession` AGAINST THE SCRIPT — the park was only the first gap, and [§36.2](FINDINGS.md)'s "step 0 is done" was premature

⛔ **[ROADMAP §6.1](ROADMAP.md)'s audit found the park gap by searching for one word, `JointPath`. It did not diff the class against the script.** Doing that properly, by counting how often each per-arm behaviour appears in each file:

| behaviour | in the class | in the script | verdict |
|---|---|---|---|
| gripper stall guard (`GRIPPER_STALL*`) | **0** | 6 | ⛔ **missing, and it is a SAFETY guard** |
| teleop per-cycle clamp (`MAX_JOINT_STEP`) | **0** | 4 | ⛔ missing, decision needed |
| joint-limit clamp (`JOINT_LIMIT_MARGIN`) | **0** | 3 | ⛔ missing, decision needed |
| workspace box (`args.box`) | **0** | 3 | ✅ stays in the script, it is a cartesian idea |
| CONTROLS wizard (`"map"`, `last_active_axis`) | **0** | 8, 9 | ✅ stays in the script, it is interactive |
| `stall_since` | 2 | 8 | ⛔ **the variable was there with nothing writing to it** |

⛔⭐ **THE WORST ONE: the gripper stall guard was absent, and the class carried a dead `stall_since` variable that made it look present.** The script releases the jaws to their measured position when they push above 1.0 Nm while moving under 0.05 rad/s for 0.4 s. **That guard exists because motor 7 was cooked three times.** Pushing at full current without moving is the worst thermal case there is: full current, no motion, no cooling.

⭐ **Now built, as `gripper_stall_release(t)`, with six tests.** It returns the jaw value to back off to rather than applying one, because *the class decides and the script narrates*. It reports nothing when the chain read failed, which is the same **"cannot see it, cannot judge it"** rule the thermal guard uses after [§24.1](FINDINGS.md). It is silent on a six-motor `--no-gripper` arm, because indexing a seventh motor inside the control loop is the exact shape of bug that once dropped 4.3 kg.

⚠️ **A smaller untruth, also found and fixed: the class docstring claimed "the same five modes… `guide`, `teleop`, `hold`, `park`, `map`".** It has four and never sets `map`. CONTROLS is an interactive wizard and belongs in the script. ⛔ **A docstring that overstates what a file does is how the next reader mis-plans a restructure**, which is what happened here.

⭐⭐ **AND ONE REAL DESIGN DECISION IS NOW OPEN, with a recommendation.** The two clamps that limit what may be commanded — `MAX_JOINT_STEP` per cycle and the joint-limit margin — currently live **only in the teleop branch**. ⛔ **Working-contract rule 7 asks of every guard: what path reaches the hazard without passing through you?** PARK already went around the gripper clamp once for precisely this reason ([§9](FINDINGS.md)). **Recommendation: move both into `ArmSession`'s single command path**, so every mode is clamped by construction rather than by remembering. ⚠️ **Not done here**, because it changes what actually gets commanded to the arm and that deserves its own reviewable step with Julien's word on it.

⭐ **The lesson, and it is the third form of the same one this repo keeps meeting.** [§33.3](FINDINGS.md) found stale *measurements* in prose. [§36.2](FINDINGS.md) found a stale *design* in code, with its tests passing. **This is a stale *inventory*: a list of what a component covers, which nobody re-derived against the thing it describes.** All three were true when written. ⛔ **The defence that worked was the same each time: count something, do not recall it.** A ten-line search settled in seconds what two careful readings had missed.

---

## 37. ⛔⭐⭐ THERE IS A 1.0 rad/s CEILING ON EVERY COMMAND AND NOBODY HAD MENTIONED IT — 2026-08-13, evening

> Julien: *"I still don't understand if we can increase the max speed, and if the motors allow for much higher speeds than what we currently have. Because it still sounded like all of the recordings I did were speed limited for some reason."*

⭐ **He was right, and the limit is lower than anything this repo has been calling "the limit".**

### 37.0 ⛔⭐⭐ `SafeRobot(max_speed=1.0)` clamps every command from every mode

`src/yam_robot.py` builds the robot as `SafeRobot(get_yam_robot(...))` at **two** call sites, neither passing `max_speed`. The default is **1.0 rad/s per joint**, and the clamp sits **below all control logic**:

```python
def __init__(self, robot, max_speed: float = 1.0, max_lag: float = 0.25):
...
budget  = self.max_speed * dt
limited = self._last_cmd + np.clip(q - self._last_cmd, -budget, budget)
limited = np.clip(limited, measured - self.max_lag, measured + self.max_lag)
```

⛔ **So the numbers this repo has been quoting are not the binding ones:**

| the number | where | is it what binds? |
|---|---|---|
| `MAX_PLANNED_JOINT_SPEED` = **1.5** rad/s | teleop branch of `teleop_session.py` | **no** — 1.0 bites first |
| `park_speed`, adjustable up to **1.5** rad/s | park | **no** — 1.0 bites first |
| playback speed multiplier, any value | replay | **no** — 1.0 bites first |
| **`SafeRobot.max_speed` = 1.0 rad/s** | below everything | ⭐ **YES** |

⭐ **This answers his question directly: every recording he played back was capped at 1.0 rad/s per joint, whatever speed the plan announced.** Nothing on screen says so, which is why it felt like an unexplained limit.

### 37.1 ⛔⭐⭐ AND IT CORRECTS THE SPEED ANALYSIS PUBLISHED EARLIER TODAY — [§34.1](FINDINGS.md) and [§35.2](FINDINGS.md)

`TrackingLog` measures `speed` from the **target** the replay computed and `lag` as `|target − measured|`. **The target is computed before `SafeRobot` clamps it.** So every "commanded speed" in those tables is a *requested* speed, and any request above 1.0 rad/s was never actually sent.

⛔ **The claim "the arm tracks up to about 1.9 rad/s" is therefore wrong, and so is the conclusion built on it.** The arm has **never been commanded above 1.0 rad/s** in any measurement this repo holds.

⭐⭐ **Re-reading run C's table against the 1.0 line splits it cleanly, which the earlier model could not do:**

| joint | requested | lag | vs the 1.0 ceiling |
|---|---|---|---|
| base_yaw | 0.38 | 0.028 | below |
| elbow_pitch | 0.32 | 0.054 | below |
| wrist_roll | 0.98 | 0.084 | below |
| shoulder_pitch | 1.03 | 0.059 | at it |
| **forearm_pitch** | **1.26** | **0.149** | **above** |
| **gripper_twist** | **2.04** | **0.156** | **above** |

**Every joint asked for less than 1.0 rad/s tracked within 0.09 rad. Both joints asked for more sat at the 0.15 rad threshold where the playback holds its clock.** Run B, whose fastest request was 1.03, showed a worst lag of 0.090. That is the same split.

⭐ **So the honest reading of all three runs is simpler than the delay model:** below 1.0 rad/s the arm follows with under 0.09 rad of error; above it, the request runs away from a command that cannot exceed 1.0, and the gap grows until the cursor waits.

⚠️ **And it explains the scatter the earlier fit could not.** The gap grows for as long as the request exceeds the ceiling, so it depends on **how long** the excess lasts, not on the instantaneous speed. `elbow_pitch` in run A was asked for 1.99 rad/s and lagged only 0.081, because that request was brief. `base_yaw` showed lag **falling** as speed rose. Both are natural under a duration model and awkward under a per-joint delay.

⛔ **The "0.033 s delay, identical on every joint" is now most likely the shared rate limiter rather than a transport delay.** A single constant shared by all six joints is exactly what one clamp below all of them produces. ⚠️ **That is the leading explanation and it is not proven** — separating a rate clamp from a transport delay needs a run with `max_speed` raised, which has never happened.

⭐ **What the data still supports, and it is worth keeping:** the arm's real tracking error at or below 1.0 rad/s is **0.03 to 0.09 rad**, and the gains do not shape it much. That part stands because it comes from requests the clamp never touched.

### 37.2 ⭐⭐ CAN WE GO FASTER? Yes. Here is the order, and what to watch

⭐ **The motors are nowhere near their limit, and his own recordings prove it.** Hand-guiding reached **2.4 to 3.7 rad/s** at the 99th percentile, with a person pushing the arm. **The hardware moves at those speeds; only our software refuses to command them.**

**Raise them in this order, one at a time, testing each:**

| # | change | from → to | why this order |
|---|---|---|---|
| 1 | `SafeRobot(max_speed=…)` in `src/yam_robot.py` | 1.0 → **1.5** | It is the only one that binds today. Nothing else changes until it moves |
| 2 | `MAX_PLANNED_JOINT_SPEED` | 1.5 → 2.0 | Only meaningful once (1) is above it |
| 3 | `MAX_CURSOR_LAG` (the playback hold) | 0.15 → leave | Raise last, and only if playbacks still wait when they should not |

⚠️ **What to watch, in order of how likely it is to bite:**

1. **Following error.** `SafeRobot.max_lag` is 0.25 rad and the worst lag ever measured is 0.181. Faster motion means more lag, so this becomes the next thing to hit. ⭐ It is a *torque* limit as well, because the position gain multiplies it, so raising it raises the force the arm can apply.
2. **The inverse-kinematics loop.** Faster teleop means a larger step per cycle, and a joint hitting its limit is what made the arm move incoherently ([§18](FINDINGS.md), `_limit_lead`).
3. ⚠️ **Heat is probably NOT the constraint at these speeds.** The hottest reading across a 337-second session with three parks, a recording and a playback was **43 °C**, against a warning at 55 and a stop at 65. **Motion is not what cooks these motors; holding still against a stop is** ([§4](FINDINGS.md), motor 7 three times). Faster motion may even run cooler, because a moving motor is not stalled.

⛔ **All three numbers are safety limits and they are Julien's to raise, not an agent's.** Each is one line. Each needs a hardware run afterwards.

### 37.3 ⛔ RETRACTED: my own recommendation to move the clamps into one place was WRONG, and checking took ten minutes

**What I proposed earlier today** ([§36.5](FINDINGS.md)): move `MAX_JOINT_STEP` and `JOINT_LIMIT_MARGIN` out of the teleop branch into `ArmSession`'s single command path, on the grounds of working-contract rule 7 — *what path reaches the hazard without passing through you?*

⛔ **Both halves are wrong, for different reasons.**

1. ⭐ **The universal clamp already exists, one layer lower.** `SafeRobot` wraps the robot and rate-limits **every** command from **every** mode, plus a following-error limit anchored to the measured position. Its own docstring says it sits *"BELOW all control logic"* so *"the buggy code cannot reach around"*. **Rule 7's question already has a good answer.** `MAX_JOINT_STEP` in the teleop branch is an *additional, tighter* limit for the one path that can produce a jump.
2. ⛔ **Applying `JOINT_LIMIT_MARGIN` to PARK would be actively wrong.** The margin exists to keep the **IK** away from joint limits, because a joint pinned at its limit is what makes the QP move the tool point in directions nobody asked for ([§18](FINDINGS.md)). Park targets come from `s <digit>`, which saves a **measured** pose — a pose the arm physically held. Clamping park to *limit minus margin* would **refuse to return to a pose the arm has already been in.**

⭐ **The correct statement is narrower and it is worth keeping:** the gripper clamp is the one that genuinely must be universal, and it already is, applied inside `park_target_from` *"so no caller can forget it"* after PARK bypassed it once.

⚠️ **The lesson: rule 7 is a question, not a verdict.** I pattern-matched "a guard lives in one branch" to "a guard is being bypassed", and recommended changing a safety path without first checking whether a lower layer already covered it. **The check was one search.**

### 37.4 ⚠️ `check_rig.py` called an unplugged dock "a fault"

With everything unplugged it printed *"the USB bus reports nothing at all, which is itself a fault"*. `usb.core` lists external devices, so **zero is the correct reading when the dock is unplugged**, which is what Julien's desk looks like after he goes home. ⭐ Corrected to say nothing is attached and that this is expected. ⛔ **I wrote that line without ever seeing the case it describes**, which is the same defect as any other untested message.

### 37.5 ⭐⭐ WHY THE ARM STOPS BEFORE IT LOOKS FULLY EXTENDED — three separate limits, and only one of them is the answer

> Julien: *"when I control something in teleoperate, it stops moving in the direction I want it to move even though the arm hasn't even close to fully extended."*

**Three different things could produce that, and they are not the same:**

| limit | value | anchored to | does it match his description? |
|---|---|---|---|
| ⭐ **workspace box** | **±0.30 m** cube | ⛔ **wherever the arm was when TELEOP was entered** | ✅ **yes — this is almost certainly it** |
| joint limits minus margin | `JOINT_LIMIT_MARGIN` = 0.08 rad | each joint's own URDF limit | partly, at extremes only |
| `SafeRobot` rate + lag limits | 1.0 rad/s, 0.25 rad | the measured pose | no, these slow motion rather than stopping it |

⛔⭐ **The box re-centres every time TELEOP is entered** (`home_ee = teleop.ee_position()` on entry). **So the wall is 30 cm from wherever he happened to press `t`, and it moves every session.** That fits "not even close to fully extended" exactly: the YAM reaches far more than 30 cm, and if teleop was entered near the middle of a reach, the wall arrives early.

⚠️ **And nothing on screen shows where the wall is.** The status line reports temperature, loop rate and joint angles. It does not report how much box is left, so hitting the wall reads as the arm refusing to move.

⭐ **Three fixes, cheapest first, none of them done:**

1. **Show it.** Put "0.28 / 0.30 m from centre" on the status line. The wall stops being invisible. **~5 lines, no behaviour change, and it makes the other two decisions informed rather than guessed.**
2. **Anchor it to the base**, or to the park pose, instead of to wherever teleop started. Then the wall is in a fixed, learnable place.
3. **Make it adjustable live**, like `park_speed` and the ease ramp already are.

⭐⭐ **His own idea, and it is a good one for later:** *"maybe the clamp limit should be recordable… where I can once go through all of the motions that are necessary to build, like, a bounding box or bounding space."* **Assessment: right instinct, and worth doing after 1-3.** A recorded workspace matches the task instead of an arbitrary cube. ⚠️ **Two cautions.** A hull is only as safe as the recording, so a region he forgets to visit becomes unreachable later, which is annoying rather than dangerous. And a *convex* hull of a recorded path can include poses the arm cannot actually reach, so it must be checked against the joint limits rather than trusted. ⭐ **The cheap version of his idea: record the corners he cares about as saved poses, and take the box as their bounding volume plus a margin.** That reuses `s <digit>`, which already exists.

### 37.6 ⭐ THE RED LIGHTS — the full timeline, which changes the reading of [§36.0](FINDINGS.md)

**Julien's account, 2026-08-13 evening.** The blinking started **after he lent arm G to a colleague**, who *"did some code execution stuff"* and *"tried to connect them as well, and he got them to connect"*. The lights then began blinking red while still plugged in, stayed blinking after the colleague unplugged it, stayed blinking when Julien plugged it back in, and **later stopped**. He is reporting from memory and the rig is now unplugged, so none of this can be re-checked tonight.

⭐⭐ **The important part: the blinking survived being unplugged from USB.** The CAN adapter loses power when USB is unplugged. **The motors do not** — they run from the wall ([§4.5](HANDOFF.md): power is wall sockets only, there is no e-stop). **So the lights were showing motor state, not adapter state**, and whatever the motors were in persisted while they stayed powered.

⚠️ **A hypothesis that was not considered in [§36.0](FINDINGS.md), and it is now the leading one:** `ping_motors.py --yes` sends an **enable** frame to each motor and reads the reply. **Running it may itself have cleared a latched state.** The ping at 18:00 reported `err=0x1 (normal)` everywhere, which is consistent both with "already fine" and with "cleared by the enable frame, and the reply read after". ⛔ **Nothing distinguishes those two from the data collected**, and the honest position is that the check may have changed what it measured.

⛔ **The new risk this exposes: a third party ran unknown code against these motors.** That is not a criticism, it is a fact that changes what to check. **DM motors have writable registers** — `0x55` writes one and `0xAA` saves it to flash, so a change can survive a power cycle. Nothing in this repo ever writes a register, but another tool might.

⭐ **What was checked, and it is reassuring as far as it goes.** `identify_arm.py` read seven registers on arm G at 18:00 and all match the recorded baseline: gear ratios 40/40/40/10/10/10/10, `timeout` 8000 on every motor, and joint 1's `inertia` equal to arm G's own recorded value. ⚠️ **Seven registers is not all of them.** Control mode, and the `PMAX`/`VMAX`/`TMAX` scaling limits, were **not** read, and a wrong `VMAX` would silently mis-scale every velocity reading rather than raising anything.

⭐⭐ **What to do tomorrow, and it is cheap:** run `identify_arm.py --yes` on **both** arms and diff them against each other. The two arms should agree on every register except the per-unit `inertia` values. **A difference is the signal.** [HANDOFF §5.5](HANDOFF.md) task 0 carries this.

---

## 38. ✅⭐⭐ THE REGISTER DIFF §37.6 ASKED FOR, RUN ON BOTH ARMS — AND THE RISK IT CANNOT COVER — 2026-08-14, 09:10

> [§37.6](FINDINGS.md) ended with *"run `identify_arm.py --yes` on both arms and diff them against each other… A difference is the signal."* That was done, with three more registers than `identify_arm.py` reads, and it is now `scripts/check_arms_match.py` so it is a command rather than a claim.

### 38.0 ✅ Every readable register is identical on both arms, and nothing has moved since it was first recorded

**140 reads: 10 registers × 7 motors × 2 arms.** Register reads only (`0x7FF` sub-command `0x33`), nothing energised, agent-safe. Run three times across three separate bus opens; **every value identical to the last digit each time**, which is itself worth having, because the transmit-echo defect in `src/yam_can.py` produces a flawless set of zeros that looks like success.

| what | result |
|---|---|
| motors replying | **14/14**, both arms, all 10 registers |
| `timeout` | **8000 on every one of the 14 motors** — the safety timeout is enabled on both arms. `0` would mean disabled, which I2RT warn can produce uncontrolled torque from a failed gravity-compensation loop |
| `gear_ratio` | 40/40/40/10/10/10/10 on both arms, unchanged |
| registers that differ between the arms | **only `inertia` and `flux`**, and both are per-unit measured data — see [§38.2](FINDINGS.md) |
| ⭐ `inertia` against the baseline in [§1](FINDINGS.md) | **matches to every recorded digit, four days later.** B joint 1 `1.7169109923997894e-05`, G joint 1 `1.6964389942586422e-05`. §1 recorded `1.7169109e-05` and `1.6964389e-05` on 2026-08-10 |

⭐ **That last row is the one that answers the actual worry.** The concern was that the colleague's tool might have written and *saved* a register on arm G. A re-calibration would have produced fresh per-unit values, which would still *look* like per-unit scatter — so an arm-vs-arm diff alone could not have caught it. **The four-day-old recorded value matching exactly is what rules it out.**

⭐ **Three registers were read here that `identify_arm.py` has never read:** `OT_value`, `flux` and `gear_eff`. The script takes its register list *from* the vendored SDK's own `register_addr_map` rather than copying it, so it cannot fall behind the driver the way a written list would.

### 38.1 ⛔⭐⭐ WHAT THE DIFF CANNOT COVER, AND IT IS THE RISK §37.6 ACTUALLY NAMED

§37.6 asked for *"control mode, and the `PMAX`/`VMAX`/`TMAX` scaling limits"*. **None of those four was read, and none of them can be, through this path.** Three reasons, and the third is the important one:

1. The SDK's `register_addr_map` (`i2rt/motor_config_tool/utils.py`) holds exactly ten entries — `KT_value`, `OT_value`, `master_id`, `id`, `timeout`, `inertia`, `sw_ver`, `flux`, `gear_ratio`, `gear_eff`. **Not one of them is a scaling limit or a control mode**, and no address for those is published anywhere in this checkout.
2. ⛔ **Guessing an address would be worse than not reading it.** A read of the wrong register returns four perfectly plausible bytes, and labelling them `VMAX` is the confident-plausible-wrong failure this whole document exists to catalogue ([§0](FINDINGS.md)).
3. ⭐⭐ **And the scaling is not read from the motor in the first place.** `MotorConstants` in `i2rt/motor_drivers/utils.py` hardcodes `POSITION_MAX = 12.5`, `VELOCITY_MAX = 45` and `TORQUE_MAX = 54` in Python, per motor family, and every feedback frame is decoded with those. **If a motor's stored limits were changed, the motor would encode its feedback on one scale while the SDK decodes it on another, and every position, velocity and temperature reading would be silently wrong by a constant factor.** No register read detects that, because the register the SDK trusts is a line of Python.

⭐ **What DOES bound it, from data already on record and at no cost — the gripper limit reconciliation.** `config/gripper_limits.json` holds each arm's jaw limits as *measured raw motor radians*: B `[6.4810788128480965, 1.2308308537422743]`, G `[0.1417181658655675, -5.08640421149004]`. On 2026-08-13 at 18:00, G's motor 7 read `-3.3343` and sits inside G's band; B's read `0.0158` and sits inside B's once the `+2π` wrap is applied, giving `6.2990` ([§36.0](FINDINGS.md)).

- **A gross change is ruled out.** Had `POSITION_MAX` doubled from 12.5 to 25, every decoded position would double, and G's jaw would have read about `-6.67` — **outside** its `[-5.086, 0.142]` band. The reconciliation would have failed. It did not.
- ⚠️ **A small change is not ruled out.** A 10% scaling error puts G's jaw at `-3.67`, still comfortably inside the band. So this bounds the error to roughly tens of percent, and no further.

⛔⚠️ **And one thing that looks like evidence and is not: "both arms read ≈ 0 on joints 1-6 at rest".** Zero is scale-invariant. Any multiplicative change leaves zero at zero, so a parked arm reading zero **cannot** detect a scaling error on those six joints. [§36.0](FINDINGS.md) reports those near-zero readings as part of a healthy picture, which is fair, but they carry no information about this particular risk. The gripper is the only joint sitting at a large non-zero value, which is exactly why it is the only one that bounds anything.

### 38.2 ⭐ `flux` DIFFERS BETWEEN THE ARMS, AND THE FALSIFYING TEST SAYS THAT MEANS NOTHING

**`flux` — the permanent-magnet flux linkage — had never been read in this repo before today.** It differs between the arms on all seven motors, by 0.11% to 1.79%. The first run's verdict was therefore **7 registers differ that should not**, which is alarming and wrong.

⛔ **Explaining a difference away is the exact shape of a confirmation-bias error**, so the claim was tested rather than argued. **The falsifier: a register holding *measured* per-motor data scatters even between two motors of the identical model on the identical arm. A *configured* constant does not** — every motor of that model would report one number, so a within-arm spread of exactly zero beside a non-zero between-arm difference would mean somebody wrote it.

| register | widest spread WITHIN one arm, same model | widest difference BETWEEN the arms | reading |
|---|---|---|---|
| `inertia` | **46.29%** (G, DM4340, joints 1-3) | 2.88% (joint 5) | measured, and known to be since [§1](FINDINGS.md) |
| `flux` | **2.51%** (B, DM4310, joints 4-7) | 1.77% (joint 2) | ⭐ measured — it scatters *more* within one arm than between them |

⭐ **So `flux` joins `inertia` on the expected-to-differ list, and the evidence is recomputed on every run rather than written down here.** That is [§33.3](FINDINGS.md)'s rule applied to a judgement rather than to a number: the script prints the two percentages every time, so if `flux` ever *stops* scattering within an arm, the exemption fails on its own.

⚠️ **`inertia`'s 46.29% deserves a caveat.** It is dominated by joint 2 reading `2.6e-05` against joints 1 and 3 at `1.7e-05`, which is a real physical difference between joint positions rather than unit-to-unit manufacturing scatter. It still falsifies "configured constant", since a configured constant would be identical. The tighter pairwise comparison says the same thing more cleanly: B's joints 1 and 3 differ by 1.4%, and B-vs-G on joint 1 differs by 1.2%.

### 38.3 ⛔ TWO DEFECTS IN MY OWN SCRIPT, ONE FOUND BY READING ITS OUTPUT AND ONE BY A TEST

1. ⛔ **The verdict printed *"nothing has moved since the baseline"* on the run where no baseline existed and nothing had been compared.** The count of moved registers was `0`, and `0` is also what "nothing moved" looks like. **Reporting "we looked and found nothing" and "we did not look" as the same sentence is this document's whole subject.** Fixed with a separate `baseline_checked` flag, and pinned by a test.
2. ⛔ **The falsifier reported an unmeasurable spread as 0.00% and concluded "this register looks CONFIGURED".** With only one motor of a given model in the read there is no within-arm spread to compute, and `0.0` was standing in for "not measurable". On a partial read that would have manufactured a false alarm about a healthy motor. **Caught by a test written specifically to check that the falsifier can fail correctly**, which is the only kind of test that means anything about a falsifier.

⭐ **The pattern in both is one thing:** a count of zero is ambiguous between *absent* and *unexamined*, and every instance of that ambiguity in this repo has been a defect.

### 38.4 ⭐⭐ THE ARM-VS-ARM DIFF HAS A HOLE, AND A BASELINE CLOSES IT

⛔ **A tool that wrote the same register on *both* arms leaves them agreeing with each other perfectly, and the diff §37.6 asked for reports a clean verdict.** That was still the right check on 2026-08-13, because only arm G was lent out. It is not the right check in general, and both arms will be shared eventually.

⭐ **So the script asks a second, stronger question: has anything changed since we last looked?** `--save-baseline` writes all 140 values to **`config/motor_registers.json`** with the commit and the timestamp; later runs diff against it, and there **nothing** may differ, per-unit values included. A measured constant burned into a motor has no reason to move between two readings, so if one does, something wrote it. `config/` is where measured evidence lives rather than settings ([§32.3](FINDINGS.md)), and the file is committed.

⚠️ **The baseline was laid down at 09:10 on 2026-08-14, which is *after* the colleague ran his code.** It therefore cannot answer anything about that event — [§38.0](FINDINGS.md)'s four-day-old `inertia` match is what does. The baseline is for the next time.

⭐ **`src/provenance.py` was extracted in the same change.** `git_commit()` and `dt_now()` had lived inside `scripts/teleop_session.py`, and the register baseline is the third thing to record provenance after the recorder and the tracking log. Three call sites is where a copied helper starts to drift, and `src/spacemouse.py` exists because a device fix once landed in only one of two copies. ⚠️ `teleop_session.py` still has its own identical pair; they collapse into the module during the `ArmSession` restructure, which rewrites that file anyway. Doing it today would edit the script Julien is about to test, for no gain.

**370 → 384 headless tests.**

---

## 39. ⛔⭐⭐ WHAT THE LEDS MEAN — AND `ping_motors.py` HAS BEEN ERASING THE FAULT IT WAS SENT TO READ — 2026-08-14, 09:40

> [§36.0](FINDINGS.md) recorded a real gap: *"What the LEDs mean is undocumented anywhere in this repo."* It is now documented, from the vendor's own manual, and **the answer refutes what §36.0 concluded.** Chasing it also turned up a flag that was documented, advertised in `--help`, and wired to nothing.

### 39.0 ⭐⭐ THE LED TABLE, FROM THE VENDOR MANUAL

**Source: DAMIAO "DM-J4340-2EC reduction motor User Manual V1.0", 2024.03.14, section "Indicator status".** Both motor families on these arms are covered — joints 1-3 are DM4340, joints 4-7 DM4310, and the DM4310 manual says the same thing.

| light | ERR bit | what it means |
|---|---|---|
| **green, steady** | 1 | enable mode, normal working status |
| ⭐ **red, STEADY** | 0 | **disabled mode. This is NOT a fault.** |
| ⛔ **red, FLASHING** | — | **a latched fault.** The code says which |

**The fault codes, verbatim from the manual:** `8` overpressure (over-voltage) · `9` undervoltage · `A` overcurrent · `B` MOS overheating · `C` motor coil overheating · `D` loss of communication · `E` overload.

✅ **The vendored SDK's table agrees exactly** (`i2rt/motor_drivers/utils.py::MotorErrorCode`): `0x0` disabled, `0x1` normal, `0x8`-`0xE` as above. Two independent sources, same table, so this is settled rather than inferred.

⛔⭐ **AND IT REFUTES [§36.0](FINDINGS.md).** That section reasoned: *"Every motor is set to enter damping mode after 400 ms with no command. An arm that is powered but not being commanded sits in exactly that state permanently. A blinking LED on a powered, uncommanded motor is the expected indication rather than a warning."* **It was flagged as an unverified hypothesis, and it is wrong.** A powered, uncommanded motor is in **disabled** mode, and disabled mode is **red STEADY**. **Flashing red is a fault, full stop.** So arm G really was holding a latched fault when Julien saw those lights.

⛔⭐⭐ **A second, sharper trap in the same area: the SDK calls `0x1` "normal", and the manual calls the same thing "enable mode".** Those are not the same claim. **`err=0x1` means "this motor is enabled right now"** — it says nothing about whether a fault was latched a moment earlier. [§36.0](FINDINGS.md) read `err=0x1 (normal)` on all 14 motors and concluded the lights were not reporting a fault. **The reading was taken by a command that had just enabled the motor**, so `0x1` was guaranteed by the act of measuring.

### 39.1 ⛔⛔⭐⭐ `motor_on()` CLEARS FAULTS IN A LOOP, SILENTLY — AND THE FLAG THAT WAS SUPPOSED TO STOP IT WAS DEAD

`dm_driver.py:152-182`, the vendor's `motor_on()`:

```python
motor_info = self.parse_recv_message(message, MotorType.DM4310, ignore_error=True)
if int(motor_info.error_code, 16) != MotorErrorCode.normal:
    while int(motor_info.error_code, 16) != MotorErrorCode.normal:
        logging.info(f"motor {motor_id} error: {motor_info.error_message}")
        self.clean_error(motor_id=motor_id)     # sends 0xFB
        ...
        message = self._send_message_get_response(id, motor_id, data)
```

**So enabling a motor clears any latched fault, repeatedly, until the code reads normal.** And two things make it silent:

1. `motor_on()` opens with `logging.getLogger().setLevel(logging.ERROR)` and restores the level on the way out. **The line naming the fault is `logging.info`, and the parser's own `logging.warning` naming it is also below ERROR.** Both are suppressed for exactly the duration of the clearing.
2. The value returned at the end is the **post-clear** reading, so the caller sees a healthy motor.

⛔⛔ **And `scripts/ping_motors.py` claimed to control this and did not.** Its docstring said *"`--attempt-error-clear` (default off) controls whether we let it try at all"*, and `--help` said the same. **The string `attempt_error_clear` appeared exactly twice in the file: once in that docstring, once in the `add_argument` call. It was never read.** The clear loop ran on every single `--yes`.

⭐⭐ **This closes the open question in [§37.6](FINDINGS.md), which could only hedge.** That section wrote: *"`ping_motors.py --yes` sends an enable frame… Running it may itself have cleared a latched state… Nothing distinguishes those two from the data collected."* **It is no longer a maybe.** The code is written to clear, in a loop, with the diagnosis suppressed. **That is why arm G's red lights stopped after the 18:00 ping on 2026-08-13, and why the fault type is permanently unknown.**

⚠️ **The fault type is lost and cannot be recovered.** Given Julien's account — a colleague *"tried to connect them as well, and he got them to connect"* — `0xD loss of communication` is the natural candidate, since that is what a CAN client dropping off produces and the 400 ms timeout is its mechanism. ⛔ **That is a guess and it must stay labelled as one.** Any of `0x8`-`0xE` would have produced the same flashing red light.

### 39.2 ✅ THE FIX: a fault is now REPORTED rather than erased, and the flag means what it says

**Default behaviour is now "report and leave it alone".** `--attempt-error-clear` restores the vendor behaviour.

- `src/yam_can.py` gains `MotorFaultNotCleared`, `do_not_clear_motor_faults()`, `describe_motor_error()` and the `MOTOR_LED_FOR_ERROR` table, plus two wrappers installed by `patch_dm_driver_for_gs_usb()` beside the existing drain hardening.
- ⭐ **The error nibble is decoded before delegating to the vendor's parser, not read off its result.** Two reasons, and both are why the obvious implementation fails: with `ignore_error=False` the parser **raises**, so there is no result; and the log level is already forced to ERROR, so no handler ever sees the message.
- ⭐ **`clean_error` was intercepted rather than the enable path rewritten**, because `motor_on` is already wrapped here with bus-draining and retries that exist because of a **measured** cascade failure — three consecutive runs failing at motors 4, then 7, then succeeding. Bypassing `motor_on` to read the code directly would have thrown that away.

⚠️⚠️ **The default policy is still to clear, and that is deliberate rather than timid.** `clean_error` has a second caller: `DMChainCanInterface`'s own motor-recovery routine (`dm_driver.py:639`), which runs **during a real session** and must be able to clear and re-enable. Making refusal the global default would turn a recoverable motor into a dead arm. **Only a diagnostic opts out, and only around its own read.** `test_the_default_policy_still_clears` guards it.

⭐ **The risk profile is unusually good and worth stating exactly: a motor reporting `0x1` takes byte-for-byte the same path as before.** The new branch can only execute on a motor that is already faulted, which is precisely the case that was being mishandled. Every run anyone has ever made was the `0x1` case.

⚠️⭐ **One real consequence was checked rather than assumed, because `src/yam_can.py` is imported by the session Julien runs.** The parser wrapper sits in the **100 Hz control loop**: `DMSingleMotorCanInterface.set_control()` (`dm_driver.py:284`) parses a reply every cycle for every motor, and that method belongs to the very class being patched. **Measured 2026-08-14: 0.035 µs added per call, 0.24 µs per 7-motor cycle, 0.0024% of the 10 ms budget.** The loop's actual shortfall is 83-87 Hz against 100, which is **1500-2000 µs** — four orders of magnitude larger. ⭐ **So this cannot contribute to [ROADMAP §8.2](ROADMAP.md) item 14, and that is a measurement rather than a reassurance.**

⚠️ **A design alternative was considered and rejected: install the parser wrapper only inside the context manager, for zero control-loop cost.** It is worse. `DMChainCanInterface` runs its control loop **in a thread**, so swapping a class method while that thread is parsing frames is a real race, and 0.24 µs is not worth buying one.

### 39.3 ⭐ WHAT THIS MEANS AT THE BENCH, IN ONE PARAGRAPH

**Glance at the arms before running anything.** All lights **red and steady** is a healthy, powered, uncommanded rig — nothing wrong. **Any light flashing red is a real latched fault**, and `uv run scripts/ping_motors.py --arm <B|G> --yes` will now **name it and leave it in place** instead of quietly erasing it. Write the code down; it is the only record. Clear it deliberately with `--attempt-error-clear` when you are done reading it.

### 39.4 ⛔ THE PATTERN, AND IT IS WORKING-CONTRACT RULE 7 AGAIN

**A guard that was written once and never re-derived against the thing it guards.** The flag was added in good faith, with an accurate description of a real hazard, and the wiring was never done — so the description became a false statement that read as a safety feature. Rule 7 asks *what path reaches the hazard without passing through you?* Here the answer was *every path*, because the guard was not in any of them.

⭐ **Third instance of the same shape in two days.** [§36.5](FINDINGS.md): `ArmSession` carried a dead `stall_since` variable that made a missing gripper stall guard look present. [§38.3](FINDINGS.md): a verdict that reported "nothing changed" when nothing had been compared. **A guard, a variable and a message, each describing something that was not there.** The defence that has actually worked all three times is a test that asserts the guard can *fail*, rather than one that asserts it passes.

**384 → 396 headless tests.**
