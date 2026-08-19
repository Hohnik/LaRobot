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
Run this once:  uv run apps/calibrate_gripper.py --yes
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
uv run apps/camera_view.py --list                    # names, indices, and the checks
uv run apps/camera_view.py --camera c920 --term      # select by name, not index
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

⚠️ **An open question worth one command.** If the `Depth` or `Y` interfaces also appear as *capture devices*, some depth or infrared data may be reachable through OpenCV with no SDK at all. `uv run apps/camera_view.py --list` answers it: one D405 entry, or more than one? ⛔ Julien has to run it; the agent cannot open a camera ([§21.1](FINDINGS.md)).

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

The controller in `run_terminal` ([`scripts/camera_view.py`](../apps/camera_view.py)) measures its own draw cost every 0.4 s and adjusts the width it sends:

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
uv run apps/camera_view.py --camera d405 --big
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
uv run apps/camera_view.py --list
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

⭐ **And it was confirmed through the real code path, not only through `ioreg`.** `uv run apps/probe_can.py --seconds 3` opened the adapter, computed the 1 Mbit/s bitrate from a 160 MHz clock, and reported **`✓ listen-only granted`**. That proves libusb can claim the board and the gs_usb protocol answers, which a device listing cannot. Zero frames in 3 s is the expected reading for a healthy idle arm, for the reason written at the top of that script.

⛔⭐ **WHY THEY RECOVERED IS UNKNOWN, and that is now the second unknown on the same fault.** §32 already recorded that the *cause* was never established. The *recovery* is equally unexplained: no agent touched the boards, and the recovery ladder in [§32.0](FINDINGS.md) was never run. **Julien did something between 14:55 and 15:22 and it is not written down.** ⚠️ The ladder's step 1 hypothesis — a BOOT jumper left in the boot position — predicts that only moving a jumper clears it, so if he simply replugged again, that hypothesis is **wrong** and the real cause is still live. **This question is in [HANDOFF §5.5](HANDOFF.md) task 0 and it is worth 30 seconds of his time**, because the difference decides whether this recurs mid-session.

### 33.1 ✅⭐ The `w` freeze fix IS confirmed on hardware — by recordings that were already on disk

⛔ **The handoff said: "the saved recordings in `recordings/` (slots 1, 3, 4, 5, 6) are all PADDED and should be discarded rather than used."** ⭐ **Two of the five are padded. Three are clean, and they are the evidence that the fix works.**

⛔⭐ **READ THIS BEFORE THE TABLE — it is a dated record, not the current state.** Julien recorded over slots **3 and 4** at 16:34 and 16:35, about an hour after this was written, so the two padded files below **no longer exist** and every recording now on disk is clean. That is the third time in one day that a written measurement outlived the file it described ([§34.7](FINDINGS.md)). ⭐ **The table stays exactly as measured**, because it is the *evidence* that the `w` freeze fix works, and a decision's evidence does not expire. For the current state run **`uv run checks/check_recordings.py`**.

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

⭐ **It is now a script rather than a sentence: `uv run checks/check_recordings.py`.** No hardware, reads only, and it prints the table above from whatever is actually on disk. Four tests cover the function, including the one that fails if the threshold is put back below the wobble floor.

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

⚠️ **The serial confusion in item 5b is now half-explained and half-open.** It recorded `260322274021` from librealsense against `255323071773` from the USB descriptor, *"same evening, one camera on the bus"*, and treated it as one camera reporting two serials. **The second camera's USB serial is `260323072846`, which shares the `2603` prefix and not the digits.** ⛔ So it is still not settled, and the honest reading is that nobody knows which number belongs to which camera. Settle it with both cameras attached: run `sudo rs-enumerate-devices -s` and compare against `uv run checks/check_rig.py`.

### 34.6 ⚠️⭐ The DFU cause, after his answer — the jumper theory is probably dead and the fault is unexplained

**Julien, asked what he did between 14:55 and 15:22:** *"I think I only replugged them, but I'm not a hundred percent sure. I don't remember what happened, but I also thought it was quite weird."*

⭐ **That is enough to weaken the leading hypothesis a great deal.** [§32.0](FINDINGS.md) argued that a BOOT jumper left in the boot position forces the bootloader on **every** power-up, so **no amount of replugging can ever clear it**. The boards cleared. **If he only replugged, the jumper explanation is wrong.**

⚠️ **His uncertainty is not a dead end, because one candidate already on record explains everything with no mystery left.** [§32.0](FINDINGS.md) also flagged that **unplugging the hub from the Mac does not necessarily cut power to the hub's downstream ports.** DFU only clears on a genuine reset, which needs power actually removed. So: an earlier attempt that moved the *hub* would not have reset the boards, and a later attempt that moved the *CANables themselves* would. **That fits the whole sequence — a failed replug and then a successful one — and requires nothing unusual.**

⛔ **The remaining candidate is worse and cannot be ruled out: an intermittent entry into DFU at power-up.** A floating or marginally-driven BOOT0 pin, or a slow supply rail on the shared hub, lets noise decide at reset. That would recur at random, and it would recur mid-session.

⭐ **The one question that separates them, and it is worth 30 seconds:** **when it cleared, did you unplug the CANables from the hub itself, or the hub from the Mac?** Cables from the hub, and this is closed as a power-cycle that did not happen. Anything else, and it is unexplained and expected to recur.

⭐ **The mechanical defence is now in place either way:** `uv run checks/check_rig.py --raw` prints which adapters are in DFU, which are absent, and the full bus topology, which is exactly the data [§32](FINDINGS.md) asks to capture before touching anything.

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

⭐ **And the data to capture if it recurs is now one command.** Run `uv run checks/check_rig.py --raw` **before touching anything**: it answers whether it is one adapter or both, whether they are absent or in DFU, and whether the hub re-enumerated. Then note whether the session had just exited cleanly and whether anything was knocked.

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

⛔ **`mode` alone is 93 sites, and every one of them is in code that commands 4.3 kg on a rig with no emergency stop.** ⭐ **This measurement is the argument for doing step 1 as its own session and nothing else** — the repo already said so on instinct, and now there is a number behind it. It is also the argument for step 1 landing as **one mechanical change with no behaviour change at all**, so that `--arms B` at N=1 tests exactly one thing: whether the substitutions were made correctly.

> ### ⛔⭐⭐ CORRECTION, 2026-08-14: THE NUMBERS ABOVE COUNT COMMENTS AND STRINGS, AND OVERSTATE THE WORK BY ABOUT A THIRD
>
> **The table above came from a text search, and a text search for `mode` matches the word wherever it appears** — including **35 times in this very function's own comments** and 3 times inside printed strings. **Only a bare `mode` used as a variable has to be rewritten.**
>
> Counted again with an abstract syntax tree, which sees variables and ignores prose:
>
> | | text search | actual variable uses |
> |---|---|---|
> | all 20 names inside `main()` | **333** *(and 338 five commits ago, so this is the method that produced it)* | ⭐ **247** |
> | `mode` alone | **93** | ⭐ **48** |
>
> **So the edit is about 27% smaller than published, and the headline "93 sites of `mode`" is really 48.** `main()` is now **1819** lines, having grown 13 since.
>
> ⚠️ **The conclusion does not change.** 247 mechanical edits across 1819 lines is still large, still deserves its own session, and is still the argument for no behaviour change so the hardware test asks exactly one question. **What changes is how much the number can be trusted.**
>
> ⛔ **The lesson, and it is the fourth of this shape this week, under a heading that says "MEASURED RATHER THAN ESTIMATED".** A figure produced by the wrong instrument reads as *more* reliable than a guess, because it arrives with a specific number attached. [§38.3](FINDINGS.md) conflated "we found nothing" with "we did not look", [§39.1](FINDINGS.md) had a flag wired to nothing, [§40.1](FINDINGS.md) had a safety guard with no test. **This one conflated "the word appears" with "the code refers to it".** ⭐ **Re-derive with an instrument that matches the claim** — for "how many edits", that is a parser and not a grep.

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

**Glance at the arms before running anything.** All lights **red and steady** is a healthy, powered, uncommanded rig — nothing wrong. **Any light flashing red is a real latched fault**, and `uv run apps/ping_motors.py --arm <B|G> --yes` will now **name it and leave it in place** instead of quietly erasing it. Write the code down; it is the only record. Clear it deliberately with `--attempt-error-clear` when you are done reading it.

### 39.4 ⛔ THE PATTERN, AND IT IS WORKING-CONTRACT RULE 7 AGAIN

**A guard that was written once and never re-derived against the thing it guards.** The flag was added in good faith, with an accurate description of a real hazard, and the wiring was never done — so the description became a false statement that read as a safety feature. Rule 7 asks *what path reaches the hazard without passing through you?* Here the answer was *every path*, because the guard was not in any of them.

⭐ **Third instance of the same shape in two days.** [§36.5](FINDINGS.md): `ArmSession` carried a dead `stall_since` variable that made a missing gripper stall guard look present. [§38.3](FINDINGS.md): a verdict that reported "nothing changed" when nothing had been compared. **A guard, a variable and a message, each describing something that was not there.** The defence that has actually worked all three times is a test that asserts the guard can *fail*, rather than one that asserts it passes.

**384 → 396 headless tests.**

---

## 40. ⛔⭐⭐ THE ±2π GRIPPER SHIFT IS NOT A PROPERTY OF AN ARM, AND ARM G FLIPPED OVERNIGHT — 2026-08-14, 09:45

> Read from Julien's own ping output rather than from a new experiment. He ran `ping_motors.py` on both arms; six of the fourteen readings say something, and one of them contradicts two documents.

### 40.0 ⛔⭐⭐ ARM G'S GRIPPER FRAME MOVED A FULL TURN BETWEEN TWO SESSIONS

| when | arm G motor 7 raw | shift needed | jaws |
|---|---|---|---|
| 2026-08-13 18:00 ([§36.0](FINDINGS.md)) | `-3.3343` | **none** | 66.5% open |
| 2026-08-14 09:45 | **`+3.0982`** | **+2π** | 63.6% open |

**Nothing was recalibrated in between.** The raw reading changed by **6.4325 rad**, which is 2π of encoder frame plus about 0.15 rad of actual jaw movement. ⭐ **A 6.43 rad physical move is impossible: the whole stroke is 5.228 rad.** So the frame moved and the jaws barely did.

⛔⭐ **Two documents recorded "G needs no shift" as though it described arm G.** [HANDOFF §4.5](HANDOFF.md)'s health-check table and its session-21 log row both say it. **It describes where the jaws sat when the robot was last built, and `get_yam_robot()` picks the wrap correction from exactly that.** Both are corrected, and the table now carries the command that re-derives it. **Eighth instance of the [§33.3](FINDINGS.md) staleness pattern, and the first one about a safety mechanism.**

✅ **Nothing is broken and no recalibration is needed.** `reconcile_gripper_limits()` found the `+2π` frame, and it is the *only* one that brackets `+3.0982`, so the answer is unambiguous. Arm B is unchanged: `+0.0154` today against `+0.0158` yesterday, both needing `−2π`, both giving 3.5% open.

⭐ **This is the mechanism that cooked motor 7 three times, demonstrating itself live and being handled correctly.** That is the best possible outcome for a guard, and it is the first time the guard has been observed doing its job on a frame that actually changed.

### 40.1 ✅⭐ `ping_motors.py` NOW PRINTS THE FRAME, AND `reconcile_gripper_limits()` FINALLY HAS TESTS

⛔ **The function had no tests at all.** It is the fix for the worst bug of 2026-08-10, its failure mode is a destroyed motor, and nothing checked it. **That is the fourth instance of [§39.4](FINDINGS.md)'s pattern**, a safety guard with nothing asserting it can fail. `scripts/test_gripper_frame.py`, 14 tests, now covers it, including both arms on both days as regression data.

⭐⭐ **The invariant that makes the answer trustworthy, and it is not free.** The function tries shifts `0`, `+2π`, `−2π` and returns the **first** that brackets the position. That is only safe if at most one ever can. Each shifted band is `stroke + 2 × margin` wide and consecutive bands sit 2π apart, so they stay disjoint while `stroke + 2 × margin < 2π`:

    5.250 + 0.6 = 5.85  <  6.283      leaving 0.43 rad of headroom

⚠️ **A longer-stroke gripper, or a larger margin, would make two frames overlap and the function would silently return whichever it tried first.** A test now asserts this and fires if that day comes.

⚠️ **And one thing that reads as stronger evidence than it is: "the limits reconciled" is a weak statement.** Sampling the whole circle, only about **7%** of positions fail to reconcile, because the bands cover 5.85 rad out of every 6.283. **So a successful reconciliation means the frame arithmetic worked. It does not mean the calibration is still good.** It is a frame check and nothing more. *(Found by writing a test with a "clearly stale" value that turned out to reconcile perfectly well.)*

### 40.2 ⭐ EVERY RESTING READING IS A QUANTISATION STEP, WHICH IS WHY 0.0220 rad/s IS NOT DRIFT

`uint_to_float(k, -M, M, 12)` is `M × (2k − 4095) / 4095`, and `2k − 4095` is **always odd**. **So there is no code for exactly zero**, and a motor at rest reports `±M/4095`:

| | velocity step | torque step | position step |
|---|---|---|---|
| **DM4340** (joints 1-3) | 0.002442 rad/s | 0.006838 Nm | 0.000381 rad |
| **DM4310** (joints 4-7) | 0.007326 rad/s | 0.002442 Nm | 0.000381 rad |

✅ **All 28 velocity and torque readings in Julien's output are exact odd multiples of the right step for their motor type.** The largest, `0.0220 rad/s`, is **three steps** from the zero code. **Both arms were standing still.** `ping_motors.py` now says so in steps rather than in rad/s, because a number like 0.0220 reads like drift and is not.

⚠️⚠️ **A collision worth knowing: DM4340's velocity step and DM4310's torque step are the same number, 0.002442**, because `VELOCITY_MAX` is 10 on one and `TORQUE_MAX` is 10 on the other. Do not read one for the other.

⛔ **A tempting idea that does NOT work, recorded so nobody spends an hour on it.** It looks as though the quantisation step could probe the stored `VMAX`/`TMAX` limits that [§38.1](FINDINGS.md) says are unverifiable. **It cannot.** At rest the motor emits the integer code nearest zero *whatever* its own stored limit is, so the decoded value depends only on the constant the SDK assumes. **A near-zero reading is scale-invariant, exactly as [§38.1](FINDINGS.md) already said.** Only a known non-zero physical quantity can probe it.

⚠️ **A second tempting idea, also weaker than it looks: using the step as a fingerprint to verify the motor-type map.** The torque step *does* discriminate cleanly, because 28 and 10 are not in an integer ratio, while velocity does not, because 30 and 10 are (an odd multiple of 30/4095 is also an odd multiple of 10/4095). All 14 motors are pinned to one type by torque. ⛔ **But the pinned type is simply the type the SDK used to decode, which `YAM_MOTOR_TYPES` chose — so pinning it proves nothing about the map.** The independent check on the map is the `gear_ratio` register, which `identify_arm.py` already reads. **Building a "type check" on the quantisation would be a guard describing something that is not there, which is the pattern of [§39.4](FINDINGS.md).** Not built, on purpose.

### 40.3 ✅ WHAT THE PING CONFIRMED, INCLUDING THE FIX FROM AN HOUR EARLIER

- **All 14 motors answer, every error code `0x1`, temperatures 29-34 °C** against a 55 °C warning. Slightly cooler than yesterday's 31-35.
- ⭐⭐ **"No motor is holding a fault" is a trustworthy statement for the first time.** [§39.1](FINDINGS.md)'s fix landed an hour before this run, so error clearing was **off** and a latched fault would have been reported instead of erased. **Under the old code the same line would have been printed either way.**
- ⚠️ **The LED observation was not recorded.** Julien ran the ping before reporting on the lights, so the chance to see them in their pre-ping state is gone for this occasion. **It cost nothing this time, because the ping proves there was no fault to see.** The manual's table ([§39.0](FINDINGS.md)) is still unconfirmed against this actual hardware, which is a one-glance job whenever the arms are idle and powered.

**396 → 410 headless tests.**

---

## 41. ✅⭐⭐ BOTH BENCH TESTS PASSED, AND THE BOX ANSWER CAME WITH A DIAGNOSTIC MESSAGE THAT NAMES THE WRONG CAUSE — 2026-08-14, 11:00

> Julien ran both tests from [HANDOFF](HANDOFF.md)'s bench plan. Both passed. The second one printed more than expected, and analysing it in simulation turned up a message asserting something the data does not support.

### 41.0 ✅ TEST A — the provenance label works on hardware

`2.json`, recorded 2026-08-14T10:59 under commit `3238a93`: **`method` reads `live:hold+guide`.** He pressed `w` in HOLD, switched to GUIDE, guided the arm by hand for 15.7 s, and the label names both modes. **[§35.4](FINDINGS.md)'s fix is confirmed.**

⭐ **The `w` freeze fix is confirmed for a fourth time, and by the strongest evidence available:** the stop message and the save message both read **15.7 s, 1365 samples**. Identical. That is the redundant-number check that has caught every defect in this class.

⭐ **A free measurement fell out of it:** 1365 samples over 15.70 s is **86.9 Hz**, which is an independent reading of the control loop rate from the recording itself. The status line said 84 Hz in the same session. Both sit in the 83-88 band this rig has been running at, so [ROADMAP §8.2](ROADMAP.md) item 14 gains a data point and no new mystery.

⚠️ **One number worth watching: `2.json` carries 0.92 s of trailing still time, 5.8%.** That is below the 1.0 s threshold so it was not flagged, and it is not the [§30.1](FINDINGS.md) defect — the freeze fix is proven by the identical sample counts. It is Julien holding the arm still for about a second before pressing `w`. ⛔ **But `check_recordings.py`'s docstring claims "the gap between the two cases is wide in the measured data", and 0.92 against a 1.0 threshold is not wide.** The defect produced 1.8-4.4 s; a human pause before a keypress can reach 1 s. **The threshold is now known to be closer to the boundary than the comment says.** Not changed, because the right fix is not obvious and a false positive here costs only a re-record.

### 41.1 ✅⭐⭐ TEST B — THE WORKSPACE BOX IS CONFIRMED AS THE THING THAT STOPS HIM

```
[TELEOP  ] t=24.0s  hottest 35°C  jaw 34°C  ⚠️ 88Hz  q [0.2 1.45 2.04 -1.54 0.42 0.14]
           EE [0.219 0.029 0.475]  box 0.30/0.30m ⚠️ AT THE EDGE  ⚠️ SLOWED to 19% (near the reach limit)
```

✅ **`box 0.30/0.30m` is unambiguous: the box was fully used up at the moment the arm stopped.** [§37.5](FINDINGS.md)'s leading hypothesis is **confirmed**.

✅ **And the alternative is ruled out, by an absence:** no `⚠️ STUCK lead` warning appeared. That fires when the goal runs more than 80% of `max_lead_m` (0.04 m of 0.05) ahead of the achieved pose, which is what a joint limit or a stuck IK looks like. **So the IK was following fine. The box stopped him, nothing else.**

⭐⭐ **His complaint is now quantified, and he was right.** Julien, 2026-08-13: *"it stops moving in the direction I want it to move even though the arm hasn't even close to fully extended."* Measured in simulation from his own joint angles:

| | value |
|---|---|
| his tip position | `[0.219, 0.029, 0.475]`, **0.524 m** from the base |
| furthest tip position found (coarse grid over joints 1-4) | **0.736 m** |
| so he was stopped at | ⭐ **71% of the arm's reach**, with about 21 cm left |

⭐ **The simulation reproduces his tip position exactly** — the model gives `[0.219, 0.030, 0.475]` for his joint angles against his reported `[0.219, 0.029, 0.475]`. **So the kinematic model matches the real arm**, which is worth knowing independently: his sessions can be analysed offline from the joint angles on the status line. ⚠️ The 0.736 m pose is a mostly *upward* reach, so reach in a given direction is less than that; treat 0.736 as an upper bound rather than as the reach in every direction.

⚠️⭐ **A property of the box that matters for the fix: it is a CUBE, and the readout is a per-axis distance.** `off` is `max(abs(ee - home_ee))`, so `0.30/0.30` means one axis is at its wall. **The cube's corner is 0.30 × √3 = 0.52 m from the centre.** So the reachable region is three times deeper diagonally than along an axis, which is a strange shape to be constrained by and no help to a human trying to predict where the wall is.

### 41.2 ⛔⭐⭐ THE THROTTLE MESSAGE NAMES A CAUSE THE DATA DOES NOT SUPPORT

The same status line said **`⚠️ SLOWED to 19% (near the reach limit)`**. ⛔ **At his pose the arm was not near any reach limit.**

`CartesianTeleop._apply_speed_scale`'s own docstring records what "near the reach limit" looks like, measured in simulation, as the smallest singular value of the tip Jacobian (`sigma_min`, how much tip motion one radian of joint motion buys in the worst direction):

| sigma_min | what it means, per that docstring |
|---|---|
| 0.170 | middle of the workspace, comfortable |
| 0.121 | 0.25 m out |
| 0.048 | near full reach, joints already racing |
| 0.005 | stalled |

⭐⭐ **Measured at his exact joint angles: `sigma_min = 0.1713`.** A mid-workspace reference pose gives 0.1945. **His pose was as well-conditioned as the middle of the workspace.** So the throttle was firing, and the reason it printed is not the reason it fired.

⭐ **What the 19% actually implies:** the solver wanted about `1 / 0.19 = 5.3x` more joint speed than the 0.9 rad/s threshold, so roughly 4.7 rad/s. ⛔ **And a default-speed twist from his pose does not ask for that.** Simulated from his joint angles at the default 0.12 m/s, one axis at a time, the throttle settles at 100% on four of six directions and never below 60% on the other two. Driving further does eventually collapse it, because the arm moves into a worse configuration, and that took about 2.4 s of simulated driving.

⭐⭐ **The leading explanation, and it is not verifiable from the output: he had turned the linear speed up.** `-` and `+` change it by 1.25x per press from a default of 0.12 m/s. Simulated at his pose: 4x default gives 47%, 6x gives 33%, 8x gives 26%. **So somewhere around 8-10 presses reproduces 19%**, and pressing `+` repeatedly is the natural thing to do when the arm feels like it is refusing to move.

⛔⛔ **THE REAL DEFECT IS THAT NOBODY CAN TELL WHICH IT WAS. The status line shows the throttle percentage and not the speed setting it is throttling.** So `SLOWED to 19%` is unreadable: it means "the arm is struggling" and "you asked for eight times the default" equally well, and the parenthesis picks one of those and states it as fact.

⭐ **The fix is small and it belongs in the restructure**, which rewrites this part of `main()` anyway: show the linear speed setting and the effective speed, so the line reads something like `linear 0.96→0.18 m/s (19%)`. **And the parenthesis should say what was measured, not what it assumes.** Naming a cause that has not been measured is [§0](FINDINGS.md)'s defect class, in a message written to explain a symptom.

### 41.3 ⭐ SO WHAT SHOULD THE BOX BECOME — the three options, now with numbers

[§37.5](FINDINGS.md) listed three fixes. Option 1, showing the wall, is done and is what produced this section. The remaining choice, **and it is Julien's, because the box is a safety limit:**

| option | what it gives | what it costs |
|---|---|---|
| ⭐ **anchor the box to the base**, as a radius rather than a cube | The wall stops moving between sessions, so it becomes learnable. A radius also matches the shape of the real constraint, since the arm's reach is roughly a distance from the base. At 0.524 m he had 21 cm of reach left | Needs a radius chosen. **0.60 m would have let him continue and still stops well short of the 0.736 m limit** |
| keep the cube, make it live-adjustable | Cheapest, and he can widen it when a task needs it | The wall still moves every session, so it stays unpredictable |
| leave it and rely on the throttle | No work | ⛔ The throttle is a speed limit and not a position limit. It slows motion near bad configurations; it never stops the arm leaving the safe region |

⚠️ **Do not simply raise the cube.** A cube of ±0.30 already permits 0.52 m of diagonal travel while stopping axis travel at 0.30, so raising it widens an already-lopsided region. **A radius is the better shape and roughly the same amount of work.**

### 41.4 ✅ EVERYTHING ELSE IN THAT SESSION WAS HEALTHY

- **Temperatures**: hottest motor 38 °C, gripper 34 °C, against a 55 °C warning and a 65 °C stop. Yesterday's worst over a 337 s session was 43 °C.
- **The gripper frame check printed and worked on the real session start**: *"jaw limits [0.198, -5.052] verified against the jaws (shifted by -6.283 rad to match this session's frame); jaws normalise to 0.035"*. **That is the [§40.0](FINDINGS.md) mechanism reported at startup**, and 0.035 matches the 3.5% the ping reported.
- **The full shutdown worked**: `q` → `p` parked to `[-0.04 0. 0.01 0.02 -0.05 0.03]`, arrived **0.020 rad off**, then disabled all 7 motors and confirmed it.
- ✅ **Julien answered the LED question: "The lights are fine."** So both arms sat with normal indications while idle and powered, which is consistent with the vendor table in [§39.0](FINDINGS.md): steady red means disabled, and that is what a powered uncommanded arm shows. **The blinking red seen on 2026-08-13 was therefore a genuine fault and not a normal idle state.**
- **The axis map was not written**: *"unchanged — nothing was written"*, which is the guard from [§32.3](FINDINGS.md) doing its job.

---

## 42. ⛔⭐⭐ THE RESTRUCTURE BEGAN, AND ITS SECOND COMMIT FOUND A HOLE IN EVERY EXISTING CHECK — 2026-08-14, 12:00

> [ROADMAP §6.1](ROADMAP.md) step 1 moves one arm's state out of `main()`'s locals onto an `ArmSession`. It is landing as a series of commits that each leave the script runnable. Two commits are in, and the second one produced a defect that **neither the tests, nor the checker, nor a dry run** would have caught.

### 42.0 ⛔⭐⭐ A DRY RUN CANNOT VALIDATE THE PART OF `main()` THAT ONLY `--yes` REACHES

**What happened.** The rewriter turned `gripper_value = 0.0` into `arm.gripper_value = 0.0`. That line is an initialisation about forty lines above the `try` block, and **`arm` is constructed inside that block, after `build_robot()`.** So the line touches an object that does not exist yet and raises `UnboundLocalError`.

⛔ **Three nets, three holes:**

| net | why it missed this |
|---|---|
| 416 headless tests | none of them import `teleop_session.main()`; it needs a robot |
| `check_restructure.py`'s coherence pass | it checks that every name *resolves*, and `arm` **is** assigned in `main()`. Order was never checked |
| ⛔⭐ **the dry run** | **it returns before this code runs.** `--yes` is required before the SpaceMouse is opened and the state block is reached, so a dry run exits above the fault |

⭐⭐ **So the first thing to execute that line would have been a real session on the arm.** It would have failed *safely* — `UnboundLocalError` before `build_robot()` energises anything — and it would still have cost Julien a session and looked like the restructure had broken the arm.

✅ **Fixed, and the check now exists:** `check_restructure.py` finds the line where `arm` is constructed and refuses if any `arm.<field>` appears earlier. ⭐ **Proved by reintroducing the bug**: the checker reports `arm is built on line 750, but it is touched earlier, on line(s) [705]` and exits 1. **A guard that has not been seen to fail is not yet a guard** ([§39.4](FINDINGS.md), [§40.1](FINDINGS.md)).

⚠️⭐ **The general lesson, and it is bigger than this refactor: "the dry run passed" bounds less than it appears to.** The dry-run gate sits *early* in `main()` by design, because its whole point is to avoid opening devices. **Everything below that gate is unexercised by every automated check this repo has.** That covers the state block, the control loop, the park machinery and the shutdown path — which is most of the file. It is why [§0](FINDINGS.md)'s rule about hardware is stated the way it is, and it is worth knowing precisely rather than as a feeling.

### 42.1 ⛔ AND A SECOND TRAP: `nonlocal` NAMES ARE INVISIBLE TO A PARSER-DRIVEN REWRITE

The rewrite is driven by the parse tree rather than by text, because a text substitution would also rewrite the word inside comments — and `mode` appears **35 times in `main()`'s own comments** ([§36.3](FINDINGS.md)'s correction). ⛔ **But `nonlocal` names are not variable nodes.** They are plain strings in `ast.Nonlocal.names`, so the rewrite skipped three `nonlocal` statements that still named the moved locals.

⭐ **Python caught it** — `SyntaxError: no binding for nonlocal 'prev_q' found` — which is the good case. ⛔ **My verification did not**, because it searched `nonlocal` lines for `arm.` rather than for the bare moved names. **A check looking for the wrong pattern reports success.** Both the rewriter and the checker now handle declarations explicitly, and a declaration that loses all its names is removed rather than left empty.

⭐ **A pleasant side effect of the migration:** two `nonlocal` statements disappeared entirely. Mutating `arm.field` needs no declaration, so the nested functions get simpler as state moves out.

### 42.2 ⭐ WHERE THE SERIES STANDS, AND THE ORDERING CONSTRAINT THAT SHAPES IT

| commit | moved | `arm.<field>` accesses | left |
|---|---|---|---|
| `b52b72e` step 1a | `prev_q`, `guide_ref`, `home_ee` | 15 | 191 |
| step 1b | `gripper_value`, `stall_since` | 33 | **171** |

**Remaining, largest first:** `mode` 48 · `teleop` 19 · `park_ramp` 17 · `park_speed` 15 · `park_path` 13 · `park_s` 11 · `park_marks` 8 · `park_cmd` 7 · `park_best_err` 7 · `park_progress_t` 7 · `thermal` 6 · `park_leg_t` 5 · `park_target` 5 · `park_start_t` 3.

⛔⭐ **`mode` moves LAST, and this is a hard constraint rather than a preference.** `build_robot()` is called with `zero_gravity=(mode == "guide")`, and it runs *before* the robot exists — so before the `ArmSession` can exist. **The name with the most references is therefore the last one that can move**, which is the opposite of the order you would choose for comfort. ⚠️ The script will keep a local `mode` for the pre-construction decision even after the field moves; that is not duplication to be tidied away, it is the ordering made explicit.

⭐ **The park group (`park_*`, 91 references) is the next natural unit**, because `ArmSession` already implements `begin_path()` and `step_path()` against exactly those fields ([§36.2](FINDINGS.md)). ⚠️ It is also the group that moves 4.3 kg and that `q p d` and Ctrl-C depend on, so it gets its own commit and its own reading.

⚠️ **Nothing in this series has been on the arm.** [ROADMAP §6.1](ROADMAP.md) is explicit that the test is `--arms B` at N=1 once the series is complete, confirming it *feels identical*. Each commit is verified by: 416 headless tests, `check_restructure.py`, dry runs in all three start modes, and `teleop_sim.py` for the IK path.

---

## 43. ✅⭐⭐ THE WORKSPACE LIMIT IS NOW A SPHERE PLUS A FLOOR — AND SWAPPING IT FOUND TWO THINGS THE CUBE HAD BEEN DOING QUIETLY — 2026-08-14, 13:00

> Julien's decision, taken after [§41.1](FINDINGS.md) measured the old cube stopping him at 71% of the arm's reach. His words: *"Sounds good regarding the box."* ✅ **He also confirmed the first two restructure commits on hardware: "teleop feels identical."**

### 43.0 ✅ WHAT CHANGED

| | before | after |
|---|---|---|
| shape | ±0.30 m **cube** | **0.60 m sphere** around the base, plus a floor |
| anchored to | ⛔ wherever TELEOP was entered, so it moved every session | ⭐ the base, so it is in the same place every time |
| floor | accidental, a side effect of being a cube | ⭐ explicit, `0.05 m`, its own flag |
| flags | `--box` | `--reach`, `--floor` |
| readout | `box 0.28/0.30m` | `reach 0.52/0.60m`, plus a floor warning when close |
| the clamp | an untested inline block in `main()` | `teleop.clamp_to_workspace()` + `effective_limits()`, **13 tests** |

⛔ **`--box` is removed rather than aliased.** A flag that keeps working while its meaning changed underneath is worse than one that fails loudly, which is the same call `src/yam_can.py` made when `--arm arm1` became `--arm B`. `--box` is now an argparse error.

### 43.1 ⛔⭐⭐ THE CUBE WAS PROVIDING A FLOOR, AND A BARE SPHERE HAS NONE

**I recommended a radius. On checking the geometry, a radius alone is a safety regression, and I would have shipped one.**

A cube centred on a tip at `z = 0.475` bounded the tip above `z = 0.175`. Measured over a grid of the first four joints, **this arm can put its tip at `z = −0.377`**, which is below its own base, and its tip x as far back as `−0.641`. **A 0.60 m sphere permits all of that.**

⭐ **So the floor is not scope creep, it is replacing something that was there.** Working-contract rule 4: never continue past a hazard you have correctly identified. ⚠️ `0.05 m` is chosen and **not measured** — where the desk sits relative to the model's origin has deliberately never been measured ([ROADMAP §8.4](ROADMAP.md), his ruling). What *is* measured: every park pose on record puts the tip at `z ≥ 0.174` and within `0.433 m` of the base, so both limits clear every pose the arm rests in.

### 43.2 ⛔⭐⭐ A FIXED LIMIT CAN BE ENTERED FROM OUTSIDE, WHICH THE OLD ONE COULD NOT

The cube re-centred on the arm at TELEOP entry, so **the arm was always at the exact centre and the cube could never be entered from outside.** A fixed limit can be. Clamping to 0.60 m when the arm sits at 0.65 m would command it 5 cm inward **the instant TELEOP starts**, with nobody having asked.

✅ `effective_limits()` opens the limit far enough to include the starting pose, derived from `home_ee` so it needs no new state. ⭐ That also gives `home_ee` a continued job after the cube stopped using it. ⚠️ **Deliberately no shrinking back during a session:** a limit that moves mid-session is what was wrong with the cube, and trading one moving wall for another would be a poor exchange.

### 43.3 ⛔⭐⭐ AND THE WIDENING NEEDED A MARGIN, WHICH A TEST PROVED BY FAILING

**The first version widened the limit to *exactly* the starting distance.** A test then failed: commanding pure roll from the `FOLDED` pose moved the tool point **0.178 m**, where the bound is 0.02.

⭐ **The cause is a knife edge, not the arm.** `FOLDED` puts the tip at **0.610 m**, just outside the 0.60 sphere, so the limit was widened to exactly 0.610 and the tip sat *precisely on the wall*. **The clamp then fired on every cycle.** And a position clamp fighting the orientation task is already written down in `_limit_lead`'s own notes: *"the workspace box then re-clamps translation, which fights the orientation task — hence the oscillation."* My change moved that pose from never-clamped to always-clamped and activated it.

⚠️ **This repo has been cut by a knife edge before:** the park stall check passed at 0.020 and stalled at 0.021 against a 0.02 tolerance ([§26](FINDINGS.md)).

✅ **Fixed with `LIMIT_WIDEN_MARGIN = 0.05`, and the number is reasoned rather than tuned:** `max_lead_m` is 0.05, the distance the goal is *already* allowed to run ahead of the arm. **A limit closer than one lead-length sits inside the controller's own slack and will chatter by construction.** A test asserts the margin is at least `max_lead_m`, so the two cannot drift apart.

⭐⭐ **The order this happened in is the point.** I wrote a docstring describing a ratchet, implemented something that was not a ratchet, wrote the tests, and **the tests caught both** — the wrong implementation and then the knife edge. Neither would have been visible on the arm until a folded-pose rotation went wrong.

### 43.4 ⛔ A THIRD COPY OF THE OLD DESIGN WAS FOUND IN ITS OWN TEST FILE

`test_teleop_ik.py`'s `drive()` **reimplemented the teleop loop, including its own `WORKSPACE_BOX = 0.30` and its own cube clamp.** After the script changed, that simulation would have kept testing a loop shape the real code no longer has, and every test would have kept passing.

⭐ **That is exactly what happened to `ArmSession` for a day** ([§36.2](FINDINGS.md)): a copy of a design drifted while all 17 of its tests passed, because the tests asserted the superseded behaviour. ✅ `drive()` now imports the real `clamp_to_workspace` instead of imitating it, so there is one implementation. ⚠️ **Worth a habit: when changing behaviour, grep the test files for a second implementation of it, not only the callers.**

**416 → 430 headless tests.** ⚠️ **Not yet on the arm.** The reach limit only bites at 0.60 m and the floor at 0.05 m, so an ordinary session should feel unchanged; the visible difference is the status line reading `reach 0.52/0.60m` instead of `box 0.28/0.30m`.

---

## 44. ⛔⭐⭐ THE ARM FELL: A MOTOR STOPPED ANSWERING THE CAN BUS, AND BOTH ADAPTERS ARE NOW IN DFU — 2026-08-14, ~13:30

> Julien: *"The arm just fell down when it was at the edge on its left side… I had it at the edge in different positions for a while. It worked flawlessly, and then at some point, it just collapsed and stopped working."* Then, on the next attempt, both CAN adapters came up in their firmware bootloader and the motors are blinking red.

### 44.0 ⛔ WHAT ACTUALLY HAPPENED, IN ORDER

```
[TELEOP] t=49.0s  hottest 36°C  jaw 34°C  ⚠️ 89Hz  q [-1.59 2.33 1.81 1.45 -0.55 0.3]
         EE [0.014 -0.484 0.353]  reach 0.60/0.60m ⚠️ AT THE EDGE
ERROR:root:4th motor at DMChainCanInterface(channel=gsusb0) failed with info [5, 'DM4310']
DM Error in control loop: fail to communicate with the motor 5 on yam_real at can channel 0
```

1. **Motor 5 (wrist_roll) stopped replying on the CAN bus.** Not a motor fault code. A communication timeout.
2. ⭐ **It had already retried 15 times.** `set_control` calls `_send_message_get_response(..., max_retry=15)`, and each attempt waits 0.01 s for a reply then sleeps 0.001 s. **So the motor was silent for roughly 165 ms.** That is a sustained dropout, not one lost frame.
3. I2RT's control thread raised and exited.
4. ✅ **Our own detection worked exactly as designed** and printed *"the motor chain STOPPED… the arm is NOT being commanded. It will be sagging under gravity. Support it now if it is raised."* That guard exists because the loop once commanded a dead chain for 64 seconds ([§0](FINDINGS.md)).
5. ⚠️ **`motors confirmed disabled: []`** — the shutdown could not disable anything, because the bus was already gone. Expected, and it is why the arm sagged rather than being released deliberately.
6. On the next run, **both** CAN adapters enumerated as `0x0483:0xdf11 DFU in FS Mode`.

### 44.1 ⛔⭐⭐ I2RT'S AUTO-RECOVERY COULD NOT HAVE HELPED, FOR THREE SEPARATE REASONS

`dm_driver.py:584` has a recovery path. **All three of its conditions failed:**

| condition | reality |
|---|---|
| `except RuntimeError` | ⛔ the exception was an **`AssertionError`**, so the clause never ran |
| `"Motor error detected" in str(e)` | ⛔ the message is *"fail to communicate with the motor 5"* |
| `self.enable_auto_recovery` | ⛔ **defaults to `False`** everywhere, and nothing here sets it |

⭐⭐ **So the design is: a motor reporting a fault CODE can be recovered; a motor that stops ANSWERING is fatal.** The second case has no handler at all.

⛔⭐ **And a bare "retry harder" fix is not the answer, which is worth stating so nobody builds it.** The transport layer already retried for ~165 ms. Extending that would delay the death, not prevent it.

⛔⛔ **Do NOT turn `enable_auto_recovery` on as a response to this.** It would not have helped here, and it does something actively unwanted: it **cleans motor faults inside the control loop**, which is exactly the evidence-destroying behaviour [§39.1](FINDINGS.md) was written to stop. The rig now has a way to *read* a latched fault, and switching on a loop that erases them would throw that away.

### 44.2 ⭐⭐ DID MY CHANGES CAUSE THIS? MEASURED, NOT ASSERTED

**The workspace limit changed one hour before this run, so it is the first suspect and it was checked properly.**

✅ **The new clamp demands LESS joint motion than the old cube would have at that exact pose.** Simulated from his joint angles, 300 cycles, at the default speed:

| direction driven | new sphere clamp | old cube clamp |
|---|---|---|
| outward, away from the base | 0.78 rad/s demanded, no throttling | 1.00 rad/s demanded |
| sideways, tangentially | **0.24 rad/s**, no throttling | ⛔ **6.84 rad/s**, throttled to 7.4% |

⭐ **The radial pull-back is gentler than a per-axis clip**, because it corrects along the direction the arm is already extended rather than sliding it sideways. So the clamp itself did not work the arm harder.

⚠️⚠️ **BUT there is one real, unproven link, and it must not be buried: the new limit let him reach a much more heavily loaded pose.** Gravity torque, computed from the model:

| pose | worst joint torque | shoulder | tip distance |
|---|---|---|---|
| where the OLD cube stopped him, 11:00 | 4.48 Nm | **−3.47 Nm** | 0.524 m |
| ⛔ **where it FAILED, 13:30** | **11.60 Nm** | ⛔ **−11.60 Nm** | 0.599 m |
| parked | 6.59 Nm | 1.85 Nm | 0.210 m |

**The shoulder was carrying 3.3x more torque than at the pose the old cube allowed.** ⭐ And the worst gravity load anywhere in the sampled workspace is **15.06 Nm, which occurs at a tip distance of 0.601 m** — so the new 0.60 m wall sits right at the most heavily loaded band the arm can reach.

⚠️ **Context that cuts the other way, and it matters:** 11.60 Nm is 41% of the DM4340's 28 Nm encoding range, and **25% of sampled poses load some joint harder than that.** The arm holds its own 4.3 kg at any pose in GUIDE. ⛔ **And motor 5 is `wrist_roll`, carrying 0.21 Nm — the smallest load on the arm.** A shoulder-current story does not explain why *that* motor went quiet.

⭐ **So: more current was flowing, and the failure was in the least-loaded motor's communication. The link is plausible via supply sag or electrical noise on the bus, and it is not established.**

### 44.3 ⭐⭐ THE STRONGER EXPLANATION: A USB-LEVEL EVENT, NOT A MOTOR ONE

`check_rig.py --raw` after the failure, and **three things in it point the same way:**

```
bus 0 addr 1  0x05e3:0x0626  USB3.1 Hub
bus 0 addr 2  0x05e3:0x0610  USB2.1 Hub
bus 0 addr 3  0x291a:0x8355  USB BillBoard          SN23456789
bus 0 addr 4  0x0483:0xdf11  DFU in FS Mode         20593383594E
bus 0 addr 5  0x0483:0xdf11  DFU in FS Mode         2081337C594E
bus 1 addr 3  0x8086:0x0b5b  RealSense D405         (no serial reported)
bus 1 addr 9  0x8086:0x0b5b  RealSense D405         260323072846
```

1. ⭐⭐ **Both CANables sit on the same hub chain on bus 0, behind two cascaded hubs and a "USB BillBoard"** — a dock. **Both entering DFU together fits one event on that chain**, which is what [§28](FINDINGS.md) guessed on physical grounds and this is the first topology evidence for it.
2. ⛔⭐ **One D405 now reports NO serial**, where it read `255323071773` three hours earlier. **A USB device that stops returning its serial descriptor is in a partly-initialised state.**
3. **Every address shuffled again** — the SpaceMice and cameras all moved. Fifth observed instance, and harmless only because everything resolves by serial.

⭐⭐ **So the leading explanation is a power or reset event on the bus-0 hub chain**, which would take out the CAN link mid-cycle *and* leave both boards in their bootloader. **That is the fourth DFU occurrence** and [§35.5](FINDINGS.md) already recorded the cause as unexplained and certain to recur. ⚠️ It is still not proven; what is new is that the topology now supports it.

⭐ **The actionable consequence is hardware, not software:** [ROADMAP §8.1](ROADMAP.md) already lists *"a powered USB 3 hub with enough ports"* as a gap. **This is now the strongest argument for it, and for not sharing that chain with a dock.**

### 44.4 ⛔ WHY THE ARM SAGGING IS NOT FIXABLE IN SOFTWARE

Once the CAN link is gone, **nothing can command the motors to hold.** Every motor is set to enter damping mode 400 ms after its last command ([§36.0](FINDINGS.md)), so a raised arm descends against damping rather than falling freely. **That is the best available behaviour and there is no software change that improves it.** The only real mitigations are keeping the link up, and not leaving the arm extended and high when it does not need to be.

### 44.5 ✅ THE FLOOR WAS WRONG AND HE CAUGHT IT BEFORE IT RAN

Julien: *"I still need to check the bottom floor five centimeter thing that you said, which sounds problematic because then I can't really pick anything up from the table anymore."* ⭐ **He is right, and it shipped wrong.**

A floor **+0.05 m above the base plane** stops the tip 5 cm short of anything lying on the desk. **Picking objects off the desk is what the rig exists for.** A limit that forbids the task is worse than no limit, because it gets switched off.

✅ **Changed to −0.10 m**, and the reasoning is now explicit in `src/teleop.py`: **it bounds a gross downward excursion and it is NOT desk protection.** It cannot be desk protection, because the desk height relative to the model's origin has never been measured ([ROADMAP §8.4](ROADMAP.md), his own ruling). What −0.10 achieves:

- the tip can reach the desk even if the base plate is several cm thick;
- the tip still cannot reach the **−0.377 m** this arm is otherwise capable of;
- every park pose (z ≥ 0.174) clears it by more than 0.27 m.

⭐ **Three tests now pin his requirement**, so the floor cannot creep back above the base plane: one asserts the default is ≤ 0, one asserts it still bounds the gross excursion, one asserts every park pose clears it. ⭐ The status line warns below z = 0, which is roughly desk height, so it reads as *"you are down at the desk"* rather than as an alarm.

**430 → 433 headless tests.**

---

## 45. ✅⭐⭐ A BAD STOP NOW WRITES DOWN WHAT IT KNEW — 2026-08-14, 14:30

> Built because of [§44](FINDINGS.md). The arm fell, and **everything about the moment of failure was gone.** Recovering the gravity torques took a simulation of his joint angles, when the arm had measured them and thrown them away. [§35.5](FINDINGS.md) records that the underlying fault will recur, so the next occurrence should be cheap.

### 45.0 ⭐ WHAT IT RECORDS, AND WHY EACH FIELD IS THERE

`src/incident.py` writes one JSON file into `recordings/incidents/` when a session stops for any reason other than `q`. It runs **after** `shutdown_robot()`, so the motors are already off.

| field | why |
|---|---|
| `stop_reason`, `at`, `commit` | which code, when, and what it said. [§33.2](FINDINGS.md) is what happens without provenance |
| `commanded_joints`, `measured_joints`, `ee` | the pose, and the gap between what was asked and what was achieved |
| `reach_limit`, `floor_limit` | ⭐ so a failure at the wall is distinguishable from one anywhere else |
| `loop_hz` | [ROADMAP §8.2](ROADMAP.md) item 14 becomes a trend rather than anecdotes |
| `last_temperatures_c`, `hottest_seen_c` | thermal, ruled in or out immediately |
| ⭐⭐ `last_torques_nm` | **the field whose absence cost the most on 2026-08-14.** The arm measures torque every cycle and discarded it |
| ⭐⭐ `usb` | **every device with bus, address, ids and serial.** [§32](FINDINGS.md) has asked for this since 2026-08-13 and nobody had captured it *during* a failure. [§44.3](FINDINGS.md) had to reconstruct the topology afterwards to notice both adapters share a hub chain with a dock |
| `chain_alive` | whether the chain was already dead when this was written |

⚠️ **Every value is the LAST one the loop managed to read, not a fresh read.** A fresh read on a dead chain raises, and the last good reading is what describes the failure. `states` and `temps` survive the loop because Python keeps a function's locals.

### 45.1 ⛔⭐⭐ THE ONE HARD RULE, AND WHY THE GUARDS LOOK EXCESSIVE

**Nothing here may delay or prevent the motors being disabled.** A crash report that interferes with teardown is worse than no crash report on a rig with no emergency stop. Three layers, and each has a reason:

1. **Every field is gathered inside its own guard** (`_safe_fact`). On a dying chain, `get_joint_pos()` raises, reading a USB string descriptor on a claimed device raises, and `states` may be unbound if the loop never completed a cycle. **A missing field becomes a note in the file.**
2. **`write_incident()` never raises** and returns `None` if it could not write. Tested against an unwritable directory and an unserialisable value.
3. ⛔⭐ **The whole call site is wrapped too**, and this is the interesting one. **[§42.0](FINDINGS.md) established that a dry run returns long before this part of `main()`, and no headless test can reach it either, because it needs a real robot.** So this code path's *first* execution will be on the arm, during a failure. **A path that cannot be tested must not be able to add a second traceback on top of the one the operator is already reading.**

⭐ **11 tests, and the ones that matter are the ones trying to break it:** a field that raises, an unserialisable object, an unwritable directory, a same-second second incident, and a check that `recordings/` really is gitignored so an incident file cannot be committed by habit.

### 45.2 ⚠️ WHAT THIS DOES NOT DO

- ⛔ **It does not prevent anything.** It makes the next failure diagnosable. The fall itself is not fixable in software ([§44.4](FINDINGS.md)).
- ⛔ **It is not a substitute for `check_rig.py --raw`** at the moment a DFU fault is noticed. It captures the bus at *shutdown*, which may be after the event.
- ⚠️ **The file is evidence, not source.** `recordings/` is gitignored on purpose, and `describe()` tells the operator to paste it rather than commit it. An incident file carries a full pose.

**433 → 444 headless tests.**

---

## 46. ✅⭐⭐⭐ THE FAULT CODE IS `0xD` LOSS OF COMMUNICATION, ON ALL SEVEN MOTORS — THE CAUSE IS SETTLED — 2026-08-14, 15:00

> The first latched motor fault this repo has ever managed to read. [§39.1](FINDINGS.md)'s fix landed on its first real use, and the answer is unambiguous.

### 46.0 ✅⭐⭐⭐ WHAT THE MOTORS SAID

```
  ⛔ motor 1 (DM4340): LATCHED FAULT, left in place
       0xD loss of communication — LED: red, FLASHING
  … the same on motors 2, 3, 4, 5, 6 and 7
⛔ 7 motor(s) are holding a LATCHED FAULT, and it was NOT cleared
```

⭐⭐⭐ **All seven motors latched `0xD`, and that settles the cause.** A motor problem latches on *that* motor. **Seven motors independently reporting "I lost communication" means the bus went away from every motor's point of view**, which is the CAN adapter, not a motor. [§44.3](FINDINGS.md)'s hypothesis is now confirmed by the motors themselves.

⭐ **And it explains why the console blamed motor 5.** Motor 5 was simply the one being polled at the instant the link vanished. **There was never anything wrong with motor 5**, and a reading of that log which chases the wrist would have been a wasted day.

✅ **Everything my analysis in [§44](FINDINGS.md) said is confirmed, and one part of it is now over-cautious.** The 3.3x shoulder torque at the extended pose ([§44.2](FINDINGS.md)) was recorded as a *plausible unproven contributor*. **It can be demoted: a torque story predicts a fault on a loaded motor, and what happened was a communication fault on all seven simultaneously.** ⚠️ It is not fully excluded, because current draw can still couple noise onto a bus, but it is no longer the leading explanation of anything.

### 46.1 ⭐⭐ THE FIX THAT MADE THIS READABLE, AND WHAT IT COST TO NOT HAVE IT

⛔ **Before 2026-08-14 this information could not be obtained.** `ping_motors.py` called `motor_on()`, which loops `clean_error()` until the code reads normal, **with the root log level forced to ERROR so both messages naming the fault are suppressed** ([§39.1](FINDINGS.md)). Every attempt to look erased the answer and reported a healthy motor.

⭐⭐ **So the 2026-08-13 blinking lights are now explained retroactively too.** They will have been the same `0xD`, latched when the colleague's session dropped off the bus, and the 18:00 ping erased it. [§36.0](FINDINGS.md) concluded *"the lights are not reporting a motor error"* on the strength of a reading taken after the erasure. **The lights were reporting exactly that.**

⭐ **The dead `--attempt-error-clear` flag cost two days of not knowing.** That is what a guard wired to nothing is worth, stated as a number.

### 46.2 ⭐ HOW THE RIG CAME BACK, AND THE PROCEDURE IS NOW KNOWN

⛔ **Replugging USB was NOT enough.** Julien: *"I tried your command, which didn't work after I unplugged the USB device, but then I unplugged the arm from the power and replugged it and seems to work now."*

⭐⭐ **That is exactly right and it follows from the fault being in the motors.** The CANable is powered from USB, so a USB replug clears the adapter's DFU state. **The motors are powered from the wall, so a latched motor fault survives any amount of USB replugging** ([§37.6](FINDINGS.md) established that the blinking survived being unplugged, for the same reason). **Two separate power domains, two separate resets.**

**So the recovery procedure, in full:**

| symptom | what to cycle |
|---|---|
| `DFU in FS Mode` on an adapter | **USB.** Unplug the hub from the Mac, wait ten seconds, replug |
| motors blinking red, `0xD` latched | ⭐ **MAINS.** Unplug the arm's power and replug it. A USB replug cannot touch this |
| both at once, which is this case | **both, USB first** so `check_rig.py` can confirm the adapters before the motors are asked anything |

✅ **Confirmed working:** after the mains cycle, all seven motors read `err=0x1 (normal)`, 31-34 °C, jaws reconciling with the usual −2π shift at 3.6% open.

⚠️ **Note what this means for reading a fault: a mains cycle erases it.** So the order matters — **ping first, then power-cycle.** Had he power-cycled before pinging, `0xD` would have been lost again.

### 46.3 ⛔ MY AT-REST WARNING FIRED ON A HEALTHY ARM, AND THE RIGHT NUMBER ALREADY EXISTED

The second ping printed *"motor 2 is NOT at rest: −0.0171 rad/s = 7 quantisation steps. Something is moving or pushing the arm."* **Nothing was moving.** 0.0171 rad/s is about 1°/s.

⛔ **The threshold was 5 quantisation steps, which I picked by feel, and the measured number was already in this document.** [§33.1](FINDINGS.md) measured a **held** arm wobbling at **0.032-0.038 rad/s**, and `check_recordings.py` already uses **0.05 rad/s** for the same judgement. **Five steps on a DM4340 is 0.012 rad/s, a factor of three below the known floor, so it could only ever produce false alarms.**

✅ Fixed to `AT_REST_LIMIT = 0.05`, reusing the existing measurement. ⭐ **Same shape as [§36.3](FINDINGS.md): a threshold chosen by feel reads as measured because it has a number in it.** The fix is to look for the measurement that already exists before inventing one.

**444 → 450 headless tests.**

---

## 47. ✅⭐⭐ A CRASH NOW PARKS THE ARM INSTEAD OF DROPPING IT, AND THE FLOOR IS THE BASE PLANE — 2026-08-14, 15:30

### 47.0 ✅⭐⭐ THE SAFE STOP — his request, and exactly what it can and cannot do

> Julien: *"when the robot, um, is being moved and stuff, and then it crashes for some reason, it should always resort to… trying to do the safe crash if that's easy to implement for everything… so that we can go around the problem that it just deactivates as soon as it crashes. It should be, like, when I do control c."*

✅ **Done.** Ctrl-C already parked the arm to the session's starting pose and then disabled. **That behaviour now applies to every unplanned stop**: an exception in the loop, a thermal stop, a guard refusing, an IK failure. Before this, those ended with the arm holding until a human answered a menu.

⛔⭐⭐ **BE EXACT ABOUT THE LIMIT, BECAUSE THE FAILURE THAT PROMPTED THIS IS THE ONE CASE IT CANNOT HELP.** When the CAN link dies, the arm **cannot be commanded at all**, so no park is possible and it sags. That is precisely what happened: all seven motors latched `0xD loss of communication` ([§46.0](FINDINGS.md)), and `chain_alive()` was already false. **This feature would not have saved that session and nothing in software would have.** ⭐ It covers every *other* way a session ends badly, and those are the majority.

⚠️ **A thermal stop parks too, and that is deliberate rather than an oversight.** Holding keeps current in a hot motor indefinitely; disabling drops the arm. **Parking gets it to a supported pose and then removes current, which is better than both.**

⛔ **`q` deliberately still shows its menu.** Julien uses `q p d` and may want `g` instead, so a *planned* quit keeps the choice. Only unplanned stops park themselves.

⭐ **Six tests pin the rule** in `scripts/test_incident.py`, including one asserting the script still spells the condition the same way. ⚠️ **The branch itself cannot be executed by any headless test**, because it lives in `main()`'s shutdown path and needs a robot — the [§42.0](FINDINGS.md) problem again. So the rule is restated in the test file and a source check keeps the two from drifting apart, which is the [§36.2](FINDINGS.md) failure mode.

### 47.1 ✅ THE FLOOR IS 0.0, AFTER TWO CORRECTIONS FROM HIM THAT BRACKET THE ANSWER

| version | value | his objection |
|---|---|---|
| first | **+0.05 m** | *"then I can't really pick anything up from the table anymore"* |
| second | **−0.10 m** | *"ten centimeter below doesn't make any sense because then it's still gonna crash into the table"* |
| ⭐ now | **0.0 m** | *"maybe do, like, one millimeter above… or just do exactly on the base"* |

⭐⭐ **Both objections were right and together they define the answer.** Too high forbids the task. Too low permits driving into the desk. **The base plane is the only defensible value**, because the arm is bolted to the desk, so the desk is at or just below z = 0 and a tip at z = 0 is a tip touching the desk.

⚠️ **He means it to be tried:** *"we can test around with it later."* `--floor -0.005` gives a few mm if a flat object needs it. ⛔ **Do not raise it above 0 again**; that is the mistake the comment in `src/teleop.py` exists to prevent.

⭐ **A test that pinned the wrong thing was found by this change.** `test_every_park_pose_clears_the_default_floor` asked for 0.20 m of clearance, which passed at −0.10 and failed at 0.0, because the lowest park pose sits at z = 0.174. **The test was pinning the old floor value through a margin instead of pinning the property it cared about** — that a park is never obstructed. Now 0.15 m.

**⚠️ Neither change has been on the arm.** The safe stop only shows itself during a failure, which cannot be arranged on purpose, so it will be seen the next time something goes wrong. The floor is visible immediately: the status line reports the height above it whenever the tip is within 10 cm.

---

## 48. ✅⭐⭐ THE PARK GROUP MOVED, AND `ast.col_offset` IS A BYTE OFFSET — 2026-08-14, 16:00

### 48.0 ✅ WHERE THE RESTRUCTURE STANDS

**Third commit of [ROADMAP §6.1](ROADMAP.md) step 1.** The eleven park fields moved onto `ArmSession` in one unit, because the class already implements `begin_path()` and `step_path()` against exactly those fields ([§36.2](FINDINGS.md)).

| commit | moved | accesses | left |
|---|---|---|---|
| `b52b72e` 1a | `prev_q` · `guide_ref` · `home_ee` | 15 | 191 |
| `ca3befd` 1b | `gripper_value` · `stall_since` | 33 | 171 |
| **1c** | ⭐ **the whole park group**, 11 fields | **118** | ⭐ **76** |

**Remaining: `mode` 48 · `teleop` 20 · `thermal` 8.** ⛔ `mode` still moves last, because `build_robot()` reads it before the robot and therefore the `ArmSession` exists.

✅ **The constants were compared before the move rather than assumed**: `PARK_SPEED` is 0.40 and `PARK_RAMP` is 0.20 in both `teleop_session.py` and `arm_session.py`, along with `PARK_TOLERANCE`, `PARK_SETTLED`, `PARK_STALL_SECONDS`, `PARK_PROGRESS_EPS` and `MAX_CURSOR_LAG`. **All seven match**, so moving a field cannot silently change a value.

✅ **The ordering check earned its keep immediately.** All eleven park fields were initialised before `arm` is constructed, so the rewrite produced eleven `arm.park_* = …` lines that would each have raised `UnboundLocalError`. **The checker listed all eleven line numbers and refused.** ⭐ That is the [§42.0](FINDINGS.md) fault, three times over, caught mechanically instead of on the arm.

### 48.1 ⛔⭐⭐ `ast.col_offset` COUNTS BYTES, AND THIS REPO'S SOURCE IS FULL OF MULTI-BYTE CHARACTERS

**The rewrite is driven by the parse tree, because a text substitution would also rewrite the word inside comments** ([§36.3](FINDINGS.md)). ⛔ **But `ast` reports `col_offset` as a UTF-8 byte offset, not a character index**, and slicing a Python `str` by it is wrong on any line containing a multi-byte character.

This file is full of them. `⭐`, `⛔`, `⚠️`, `…` and `→` appear in comments and in printed strings. The line that failed:

```python
hint(f"  moving… {park_path.length - park_s:.2f} rad of path ")
```

`…` is **three bytes and one character**, so the reported offset for `park_s` landed two characters late and the rewrite would have produced corrupt source.

✅ **Fixed by slicing the line as bytes and decoding afterwards**, which is exactly what the offsets mean.

⭐⭐ **What actually saved this was an assertion, not care.** The tool asserts that the text at the reported offset *is* the name it expects, before writing anything. It failed on `expected 'park_s', got 'rk_s:.'`, wrote nothing, and left the tree valid. **Steps 1a and 1b had worked only because their lines happened to be pure ASCII ahead of the names** — so this was live on both earlier commits and simply never fired.

⚠️ **Carry this to any future offset-based edit in this repo:** the source is not ASCII, and it never will be, because the documentation conventions here use emoji deliberately. **Work in bytes or use a tokeniser.**

### 48.2 ⭐ A NOTE ON WHY THIS GROUP WAS SAFE TO MOVE IN ONE PIECE

The eleven fields are the state of one blended `JointPath` and its cursor. **They are read and written together by `begin_path()` and `step_path()`, which the class already implements**, so splitting them across commits would have left the script half-reading from the object and half from its own locals — a state neither the class nor the script models. ⚠️ **It is also the group that moves 4.3 kg and that `q p d` and Ctrl-C depend on**, so it got its own commit and its own reading rather than being bundled with anything else.

**450 headless tests, unchanged.** ⚠️ Not on the arm. The N=1 test comes once the series is complete.

### 48.3 ⛔⭐⭐ AND THE NEXT GROUP HAS A TRAP THAT STOPS THE MECHANICAL APPROACH: `thermal` WOULD BREAK THE DFU ERROR PATH

**Found before moving it, by asking where `thermal` is read.** It is read *after* the `finally` block, in the closing summary:

```python
    except Exception as exc:
        print(f"\n⛔ {type(exc).__name__}: {exc}")
    finally:
        ...
    print(f"\nhottest motor seen this session: {thermal.max_seen:.0f}°C")
```

⛔⭐ **That `except Exception` catches a failed `build_robot()`, prints the error, and falls through to the summary.** It is the path Julien sees whenever the adapters are in DFU, and his own output on 2026-08-14 shows it working:

```
⛔ RuntimeError: No candleLight CAN adapter found.
  …
hottest motor seen this session: 0°C
```

⛔⛔ **So if `thermal` becomes `arm.thermal`, that line raises `UnboundLocalError` whenever the build fails**, because `arm` is only created *after* `build_robot()` succeeds. **A clear "no adapter found" message would be replaced by a traceback, on the failure Julien hits most often.**

⭐ **The fix is a small design decision rather than a substitution, which is why this group stops the mechanical run:**

1. Declare `arm = None` before the `try`, and guard the summary. ⚠️ **But that breaks the ordering check**, which finds the construction point by looking for the first assignment to `arm`. It would then see line ~700 and stop catching real faults. **So the checker must first learn to find the `ArmSession(` call instead.**
2. Or keep a session-level `thermal` for the summary and give each arm its own for the guarding. ⚠️ Two guards for one question is what [§0](FINDINGS.md) warns about.
3. Or move the summary inside the `try`. ⚠️ It would then stop printing on a failed build, which is the one case it is most useful.

⭐⭐ **Recommendation: option 1, and improve the checker first.** For N arms the script will hold a *list* initialised empty before the `try` anyway, so `arm = None` is the shape the design is heading for. **Doing the checker first keeps the safety net ahead of the change it protects.**

⚠️ **`mode` (48 references) has its own version of the same problem** and it is already recorded: `build_robot()` reads it to decide `zero_gravity`, so the script must keep a local `mode` for the pre-construction decision even after the field moves. **Neither of the last two groups is a pure substitution.** Three of four were; these are not, and that is worth knowing before starting them.

---

## 49. ✅⭐ `q q` PARKS AND DISABLES IN ONE KEY — AND TWO PARTS OF HIS REQUEST ARE DELIBERATELY NOT BUILT — 2026-08-14, 16:45

> Julien, 2026-08-14: *"Adding g to the q menu sounds good. There should be an option that just combines park and disable. Maybe just pressing q should allow for park and disable, but park should allow for a normal park mode to zero or to, like, the standard position because then it would also allow for q p doing the base position and then going back to teleoperate and continuing, etcetera."*

### 49.0 ✅ WHAT WAS BUILT, AND ONE THING THAT ALREADY EXISTED

⭐ **`q` then `q` now parks and disables in one key.** It is the keyboard equivalent of Ctrl-C, which has done exactly this since 2026-08-12. Same motion, same guards, same interruptibility.

⛔ **It only releases the arm if the park actually arrived**, which is the rule the Ctrl-C path already follows: *"I could not reach the safe pose"* is exactly when a human should decide rather than a default. **A stalled or interrupted park leaves the arm holding and the menu open**, and says which of `arrived` / `stalled` / `stopped` / `dead` happened.

⚠️ **`g` was already in the quit menu**, and has been since 2026-08-11. Nothing needed adding. Recorded so nobody later reads his line as an unfinished request.

⭐ **One line of discoverability was added instead of a feature.** He described wanting *"q p doing the base position and then going back to teleoperate and continuing"*. **The plain `p` key already does that without quitting**: in a normal session `p` parks, the arm then holds, and `t` carries straight on. The quit menu and the help block now both say so.

### 49.1 ❓⭐⭐ TWO PARTS ARE NOT BUILT, AND BOTH ARE HIS TO DECIDE

**① Resuming a session out of the quit menu.** If what he wants is `q` → park → *back to driving*, that is a **structural change, not a key binding.** The control loop is `while running: …` and the quit menu sits *after* it, so "resume" means re-entering a loop that has already exited. That needs an outer loop around `main()`'s body.

⛔ **And it collides with the restructure that is half-done.** [ROADMAP §6.1](ROADMAP.md) step 1 is moving `main()`'s state onto `ArmSession`, 118 of 247 references in. **Changing the shape of the loop while its state is mid-migration would make both changes unreviewable.** ⭐ **Recommendation: do it after the restructure lands**, when `main()` is in its final shape. ⚠️ It may also be unnecessary, since plain `p` already parks without quitting.

**② A park target at "zero, or the standard position".** ⛔ **This would be the first park target that is not a MEASURED pose, and that is a real change rather than a detail.** Every park slot today comes from `s <digit>`, which saves where the arm physically is. [§37.3](FINDINGS.md) turned on exactly this distinction: a proposal to clamp park targets against the joint-limit margin was retracted *because* park targets are measured poses the arm has already held, so clamping them would refuse to return to a reachable pose.

⭐ **A computed "all joints zero" target is reachable and compact** — the model puts the tip at `[0.111, 0.000, 0.174]`, 0.206 m from the base, which is close in and 17 cm up. **So it is very likely fine.** ⚠️ *"Very likely fine"* is the wording that precedes the failures in [§0](FINDINGS.md), and driving 4.3 kg to a pose nobody has ever measured the arm in deserves his word first.

⭐ **The cheap alternative that needs no new machinery: he saves the standard pose once with `s 0`.** That makes it a measured pose, it becomes the base slot, and `q q` returns to it forever after. **That is one keypress and it changes nothing in the code.**

### 49.2 ⭐ THE MENU AS IT NOW READS

```
The arm is HOLDING its pose. It will not be released until you choose.
   q = PARK then DISABLE — the whole shutdown in one key
   p = PARK — drive back to the park pose, then it holds there
   g = go weightless so you can park it by hand
   d = disable now (⚠️ a raised arm will sag)
   ⭐ to park WITHOUT quitting, use p in the session itself, then t
```

⚠️ **Not on the arm yet.** `q q` runs the same `park_and_wait()` that `q p` and Ctrl-C already use, so the motion is proven; what is new is the key and the release-only-if-arrived decision.

**450 headless tests, unchanged.**

---

## 50. ✅⭐⭐⭐ STEP 1 OF THE BIMANUAL RESTRUCTURE IS COMPLETE — 247 OF 247 REFERENCES MOVED — 2026-08-14, 17:30

> [ROADMAP §6.1](ROADMAP.md) step 1: move one arm's state out of `main()`'s locals onto an `ArmSession`, so that N of them can run in one loop. **Landed as five commits, each leaving the script runnable.**

### 50.0 ✅ THE SERIES, AND WHAT EACH STEP COST

| step | moved | `arm.<field>` | left |
|---|---|---|---|
| 1a `b52b72e` | `prev_q` · `guide_ref` · `home_ee`, and the object is constructed | 15 | 191 |
| 1b `ca3befd` | `gripper_value` · `stall_since` | 33 | 171 |
| 1c `ffa036e` | the 11 park fields | 118 | 76 |
| 1d `27ba506` | `thermal` · `teleop` | 148 | 48 |
| ⭐ **1e** | **`mode`** | ⭐ **196** | ⭐ **0** |

⭐ **`main()` now holds no per-arm state.** What stays in the script is what [ROADMAP §6.1](ROADMAP.md) said should: building the robot, reading the SpaceMouse, key handling, and the session-level recorder and playback, which span every arm because ABC's format is two arms in one timeline ([§9.2](ROADMAP.md)).

⭐⭐ **Each step was verified by four things and none of them is the arm:** 450 headless tests, `check_restructure.py`, dry runs in all three start modes, and `teleop_sim.py` for the IK path. **The arm test is still owed** and it is one question: does `--arms B` at N=1 feel identical?

### 50.1 ⛔⭐⭐ `mode` WENT LAST, AND THE REASON IS A REAL CONSTRAINT

`build_robot()` is called with `zero_gravity=(start_mode == "guide")`, and it runs **before** the robot exists, so before the `ArmSession` that would hold the mode can exist either. ⭐ **The name with the most references, 48 of them, was therefore the last one that could move** — the opposite of the order anyone would pick for comfort.

✅ **Solved with a second name rather than a second assignment.** `start_mode` is a plain local, used exactly three times: set from `args.start_mode`, read by `build_robot()`, and handed to `arm.mode` immediately after construction. ⚠️ **Deliberately not one variable assigned twice**, which would leave the script and the object out of step for the lines in between.

### 50.2 ⛔⭐⭐⭐ THE ONE HAZARD IN THE WHOLE SERIES THAT WOULD HAVE BEEN SILENT

**`ArmSession.__init__` sets `self.mode = "hold"`.** That is the right default for a class that may be built before anyone has chosen a mode.

⛔⛔ **So if the script had not assigned `arm.mode = start_mode`, then `--start-mode guide` would build a WEIGHTLESS robot and run the loop believing it was in HOLD.** Nothing raises. The arm hangs from gravity compensation alone while the screen reads HOLD. ⭐ **That is the [§0](FINDINGS.md) defect class exactly, and GUIDE is the mode where a dynamics-model error becomes a falling arm** ([§11](FINDINGS.md)).

⭐ **Found by asking what the class's own default was, not by anything failing.** No test caught it, no dry run caught it, and the checker could not: every substitution was correct, and the fault would have been the *absence* of a line nobody had written. ⚠️ **This is the limit of a mechanical check: it verifies that what is there is right, never that something missing should be there.**

✅ **Two tests now pin it**: one asserts the class still defaults to `hold`, one asserts the script still contains the handover. Together they make the requirement visible rather than remembered.

### 50.3 ⭐ WHAT THE FIVE STEPS ACTUALLY BOUGHT, BEYOND THE REFACTOR

Every step found something that had nothing to do with moving a field:

| step | what it turned up |
|---|---|
| 1a | `nonlocal` names are invisible to a parser-driven rewrite ([§42.1](FINDINGS.md)) |
| 1b | ⛔ **a dry run returns before most of `main()`**, so three nets had holes ([§42.0](FINDINGS.md)) |
| 1c | ⛔ **`ast.col_offset` counts BYTES**, and this source is full of emoji ([§48.1](FINDINGS.md)) |
| 1d | the closing summary reads state on the *failed-build* path, so a naive move breaks the DFU message ([§48.3](FINDINGS.md)) |
| 1e | ⛔⭐ **the class's own default would have silently overridden `--start-mode`** |

⭐⭐ **Four of the five were found by a check refusing, and the fifth by reading the class.** None was found by the code failing. That is the argument for the series-of-commits-with-a-checker shape, stated as a tally rather than as a preference.

### 50.4 ⬜ WHAT COMES NEXT, IN ORDER

1. ⬜⭐⭐ **The hardware test, and it asks ONE question:** `uv run apps/teleop_session.py --yes --arm B --start-mode hold`, then drive normally. **Does it feel identical?** ⚠️ Try GUIDE and the gripper too, since [§47](FINDINGS.md) records that "teleop feels identical" only covered the fields TELEOP reads.
2. ⬜ **Step 2: the `a` selector** (B → G → BOTH) and per-arm status rows. Still one arm.
3. ⬜ **Step 3: `--arms B,G`.** ⚠️ First step that needs arm G, which a colleague borrows ([§35.6](FINDINGS.md)).
4. ⬜ Steps 4-6: GUIDE on two arms, then mirror mode, then the two-arm recorder.

⚠️ **`--arms` still does not exist.** Step 1 made the state *shape* right for N arms; it did not add the flag. That is step 2's job.

**450 → 452 headless tests.**

---

## 51. ✅✅⭐⭐⭐ THE RESTRUCTURE IS CONFIRMED ON THE ARM — STEP 1 IS FULLY CLOSED — 2026-08-14, 18:00

> Julien, after driving a session on arm B: *"Everything feels great. And as before, QQ works. Uh, all of the modes work."*

### 51.0 ✅ WHAT THAT CONFIRMS, AND IT IS THE WHOLE OF STEP 1

⭐⭐ **[ROADMAP §6.1](ROADMAP.md)'s N=1 test has passed.** It asked exactly one question — *does `--arms B` at N=1 feel identical?* — and the answer is yes, across **all modes**, which is what the earlier partial confirmation was missing.

| what he confirmed | what it covers |
|---|---|
| ⭐ *"all of the modes work"* | ⭐⭐ **the gap from [§47](FINDINGS.md) is closed.** The earlier *"teleop feels identical"* only exercised `prev_q` and `home_ee`. All modes means GUIDE (`guide_ref`), the gripper (`gripper_value`, `stall_since`), PARK (all 11 park fields), CONTROLS, and `mode` itself |
| ⭐ *"QQ works"* | the new one-key park-then-disable from [§49](FINDINGS.md), on hardware |
| *"Everything feels great"* | no behaviour change, which was the entire requirement of a 247-reference mechanical move |

✅ **So all five commits of step 1 are confirmed together**, along with the workspace sphere, the floor at the base plane, and the `q q` key. **Nothing in the 2026-08-14 body of work is now unverified on hardware**, except two things that can only show themselves during a failure: the safe stop ([§47.0](FINDINGS.md)) and the incident recorder ([§45](FINDINGS.md)).

⭐⭐ **The silent hazard in [§50.2](FINDINGS.md) is also confirmed as handled.** He drove *"all of the modes"*, which includes starting in GUIDE. Had the `arm.mode = start_mode` handover been missing, that session would have run weightless while reporting HOLD.

### 51.1 ⭐ WHAT THIS MEANS FOR THE SHAPE OF THE WORK

⭐⭐ **The single-arm system is now finished and verified end to end**, and it has been rebuilt on a class that takes N arms. **Everything from here is addition rather than repair.** The remaining bimanual steps are [ROADMAP §6.1](ROADMAP.md) 2 through 6, and **step 3 is the first that needs arm G**, which a colleague borrows.

⚠️ **`--arms` still does not exist.** Step 1 made the state *shape* right for N arms. **Adding the flag, the `a` selector and the per-arm status rows is step 2**, and it is now the next piece of engineering work in the whole project.

### 51.2 ⭐⭐ AND HE ASKED WHETHER THE SPEED DIAL WAS ALREADY WRITTEN DOWN. IT WAS.

> *"It would be great to have an option where maybe, like, the scroll of maybe the other mouse or something could be activated to continuously change the speeds. Then I already mentioned this at some point. Didn't I already tell you about this? If so, then just let me know where you wrote it down and when you're gonna plan on doing it."*

✅ **Yes, on 2026-08-13, and it was written up the same day** in [ROADMAP §7.6](ROADMAP.md) with his original words quoted, plus [ROADMAP §8.2](ROADMAP.md) item 13 as the tracked entry. ⭐ **That is the continuation system working as designed**: he raised an idea, it was recorded, and a day later the record answered the question instead of the idea being re-derived.

⛔⭐ **But re-reading it found that my own deferral reasoning was too strong, and it is now corrected in [ROADMAP §7.6](ROADMAP.md).** The note said the dial *"belongs after the two-arm work, or it will fight the assignment logic that already exists."* **Only one arm is driven today**, arm G is usually unplugged, so the second puck is free right now. **The real cost is designing a third puck role twice, which is much smaller than a conflict.**

⭐ **Recommendation on record: the scrubbing version could be built before step 2 if he wants it**, because it concerns playback rather than driving, so it is the least entangled of the three uses he ranked. ⚠️ His call, and both orders are defensible.

**452 headless tests. Nothing pushed (working-contract rule 9).**

---

## 52. ✅⭐⭐⭐ STEP 2 IS BUILT — `--arms`, THE `a` SELECTOR, ONE ROW PER ARM — AND THE TESTED PARK IS NOT THE ONE THAT RUNS — 2026-08-14, evening

> [ROADMAP §6.1](ROADMAP.md) step 2, landed as three commits: `9aae3a8` the flag, `2a2dab8` the selector, `8557e76` the rows. **486 headless tests, `check_restructure.py` green, nothing pushed.** ⬜ **Not yet seen on the arm** — the bench test is one question and it is in [HANDOFF](HANDOFF.md).

### 52.0 ✅ WHAT IS NOW BUILT, AND WHAT IT DELIBERATELY REFUSES TO DO

| piece | state |
|---|---|
| `--arms B` / `--arms B,G` parse, `--arm B` unchanged | ✅ `src/arm_session.py::parse_arms`, 11 tests |
| `a` cycles which arm the MODE keys aim at, B → G → BOTH | ✅ `ArmSelector`, 5 tests |
| one status row per arm, session facts on the first row only | ✅ `StatusLine.set_rows` + `status_row()`, 18 tests |
| ⛔ **two arms actually running** | ❌ **refused, on purpose** — see 52.1 |

⛔⭐ **`--arms B,G` ERRORS OUT AND SAYS WHY, and that refusal is the safety content of the whole step.** Step 1 moved one arm's *state* onto an object, so the shape takes N arms. Everything around it is still single-arm: **one SpaceMouse is opened, one axis map is loaded, one robot is built, one park pose is read.** So two arms would drive B, never build G, and print a plan naming both. ⚠️ **The hazard is not a flag that does nothing — it is a flag that reads as two arms being under control while the second is not being commanded at all**, and an uncommanded arm that someone has raised sags ([§11](FINDINGS.md)). That is working-contract rule 4: never warn-and-continue on a hazard you have correctly identified.

⭐ **A subprocess test pins the refusal**, because a dry run is the only way a headless test can reach `main()` at all. ⚠️ **When step 2's remaining plumbing lands, that test has to be deliberately deleted** — which is the point: the refusal can neither outlive its reason nor quietly vanish before it.

### 52.1 ⛔⭐⭐⭐ THE FINDING OF THE STEP: STEP 1 MOVED THE STATE AND NOT THE BEHAVIOUR, SO THE TESTED PARK IS THE ONE THAT DOES NOT RUN

**Checked, not assumed** — `rg 'arm\.(step_path|begin_path|enter_hold|enter_teleop|enter_guide|resync|clamp_gripper|read_thermal|gripper_stall_release)\(' scripts/teleop_session.py` returns **nothing**. The script calls exactly one method on the class, `arm.alive()`, once, in the incident record.

⛔ **So `ArmSession` today is two things at once:** the live home of 21 fields of per-arm state, which the script reads and writes on every cycle — and **eleven methods that nothing outside their own tests has ever executed.** `step_path()` implements the park; the park that actually drives the arm is the `mode == "park"` branch in `main()`. Both exist, both are maintained, and **the tested one is inert.**

⚠️⚠️ **WHY THIS IS WORSE THAN ORDINARY DUPLICATION, AND IT IS A [§0](FINDINGS.md) DEFECT IN THE DOCUMENTATION LAYER.** A reader who opens `scripts/test_arm_session.py`, sees 45 passing tests covering the park cursor, the waypoint marks, the stall verdict and the gripper release, and concludes *the park is proven* would be **wrong about the code that moves 4.3 kg** — and nothing in the file says so. The claim is true of a copy. This is the same shape as a measurement whose instrument is not recorded beside it ([§36.3](FINDINGS.md)): the number is real, the thing it describes is not what the reader thinks.

⭐ **What the class's own docstring said until this session:** *"STATUS, 2026-08-13: built, unit-tested, brought up to date, and STILL NOT wired into `teleop_session.py`."* True when written, wrong the next morning when step 1 landed. It now says what is wired and what is not, and points at `check_restructure.py` — **a script that recomputes the answer, rather than a sentence that has to be maintained** ([§33.3](FINDINGS.md)).

⭐⭐ **THE DECISION, WITH THE REASONING, so it is not re-litigated:** the remaining plumbing **parameterises the script's own closures by arm** rather than switching the script over to the class's methods. Two arms then run on code Julien has already driven, with no behaviour change to confirm. **Collapsing the two implementations is the right end state and it is a separate, behaviour-changing commit** that costs a bench session — tracked in [ROADMAP §8.2](ROADMAP.md). ⛔ **The one trap found while checking equivalence:** the script does `arm.mode = "park"` and *then* calls its own `enter_hold()`, which does not touch the mode; the class's `enter_hold()` sets `mode = "hold"`. A naive substitution at that one site would leave a park running in HOLD. Every site needs that check, which is exactly why it is not a mechanical change.

### 52.2 ⭐⭐ THE STATUS ROW COULD ONLY EVER BE EXECUTED ON THE ARM, AND NOW IT CANNOT BE THE THING THAT KILLS A SESSION

The heartbeat row was **sixty lines inside the 100 Hz loop**. No test could reach it, `--yes` is required to get near it, and it runs one second into a session — so a formatting slip in it would present as *the session dying for no visible reason*, with the motors live.

⭐ It is now `status_row()` at module level with **13 tests** against a fake arm, and they pin the three things the row exists to show — each of which was once a defect found by *looking at this line*:

| the row shows | the defect it exists for |
|---|---|
| `??°C ⚠️BLIND`, never a number | a failed read became 0 °C and disarmed the thermal stop ([§24.1](FINDINGS.md)) |
| drift from where GUIDE started | 33 s of an arm sinking while the readout said 35 °C ([§11](FINDINGS.md)) |
| how much reach is left | the workspace wall was invisible ([§41.1](FINDINGS.md)) |

Plus the two that would kill a session rather than mislead: a six-motor arm (`--no-gripper`) has no jaw temperature and must not `IndexError`, and **the row must never command anything.**

⭐ **The transferable form, and it sharpens the rule this repo already had.** *The class decides, the script narrates* was written to make decisions testable. **It said nothing about the narration, and the narration is what a human reads to decide whether to hit the power switch.** A display block inside a control loop is untestable code in the most-read part of the program.

### 52.3 ⚠️ TWO MORE INSTANCES OF THE STALENESS PATTERN, BOTH IN `COMMANDS.md`, BOTH FIXED

[§33.3](FINDINGS.md) counted six; these are the eighth and ninth, and they were found by reading the file to add one row to it:

1. **It advertised `--box 0.4`**, removed on 2026-08-14 when the workspace became a sphere plus a floor ([§43](FINDINGS.md)). The flag now errors, deliberately, so the document was recommending a command that fails.
2. **It said `ö`/`ä` change the gripper step**, wrong since 2026-08-13. Those keys mean the ease ramp everywhere now, and the gripper step became `--gripper-step` precisely because one key meaning two things pushed the step to its 0.200 ceiling by accident ([§30](FINDINGS.md)).

⭐ **Both are the "cache with no invalidation" pattern in its cheapest form:** a document that lists flags will go stale every time a flag changes, and nothing checks it. **The remedy that would actually work is mechanical** — a check that every flag named in `COMMANDS.md` exists in the script's parser, and that every flag in the parser is named in `COMMANDS.md`. Not built; logged in [ROADMAP §8.2](ROADMAP.md).

### 52.4 ⚠️ A COUNT THAT WENT DOWN WHILE THINGS WERE ADDED, AND WHY IT IS NOT A REGRESSION

`check_restructure.py` reported **196** `arm.<field>` accesses after step 1 and **189** after step 2c, while two new fields were being added. **Nothing was lost.** The status block now reads its arm through the loop variable `one`, so those reads are `one.<field>` and the checker — which counts accesses through the name `arm` specifically — cannot see them.

⛔ **So that number is a coherence check, not a progress metric**, and comparing it across commits invites exactly the wrong conclusion. The check that matters is the one above it: *none of the 21 moved names survives as a local*.

### 52.5 ⬜ WHAT REMAINS BEFORE TWO ARMS CAN RUN, in the order I would do it

Each is a commit, each leaves the script runnable, and **none of them needs the arm**:

1. ✅ **DONE, `b33ff82`.** The per-arm helpers take the arm they act on: `resync` · `enter_hold` · `enter_teleop` · `enter_guide` · `begin_path` · `park_plan_line`, about thirty call sites. ⚠️ `clamp_gripper` takes no arm on purpose — the band is a property of the gripper hardware, identical on both arms, and it is passed to `park_target_from` as a one-argument callable.
2. ⬜ Per-arm axis map and control frame — **88 sites, so split it in two** (`control_frame` first, then `axis_map`), the way step 1 was split. The store already supports per-arm maps (`--fork-map`, tested). ⛔ **The trap is the same one `mode` hit** ([§50.1](FINDINGS.md)): the plan print, the pre-build `map_store.for_arm()` call, the `finally` block and the closing summary all run where `arm` may be `None`, so they need a session-level `start_frame` beside `arm.frame` — **a second name, not one variable assigned twice.**
3. ⬜ Per-arm park pose and slots (`park_slots(data, name)` is already keyed by arm).
4. ⬜ Per-arm puck: one `pick_device_by_wiggle(exclude=…)` call per arm, which is built and tested (`scripts/test_puck_assignment.py`).
5. ⬜ Per-arm CONTROLS and button state: `last_active_axis` · `last_active_value` · `last_input_kind` · `learn_button` · `buttons_prev`.
6. ⬜ `for one in arms:` around the thermal read, the stall guard and the mode action; mode keys apply to `selection.names()`.
7. ⬜ The shutdown and consent flow over N arms — ⛔ **a fault on one arm stops both** (ROADMAP §6's ruling), and the park-then-disable path has to run for every arm.
8. ⬜ Build N robots, drop the refusal, delete the test that pins it, and refuse `--start-mode guide` when N>1.

⚠️ **Then, and only then, step 3 needs arm G**, which a colleague borrows ([§35.6](FINDINGS.md)). **That is the next thing that needs Julien for a physical action.**

### 52.6 ⭐⭐ A NEW MECHANICAL CHECK, AND IT WAS FALSIFIED BEFORE IT WAS TRUSTED

Turning six closures into functions that take an arm created about thirty call sites where **Python cannot see a wrong argument count until the line runs** — and every one of those lines is inside the control loop with the motors live.

⛔ **None of the existing nets covers arity.** `compile()` accepts it. Check 4 asks only whether a *name* resolves. **A dry run returns before the loop** ([§42.0](FINDINGS.md)). So a mis-called helper's first execution would have been on the arm, mid-session.

✅ **`check_restructure.py` check 5** finds every function inside `main()` whose first parameter is named `one`, then verifies every call to it passes at least one positional argument and a legal count. It reports how many helpers it found, so a rename makes it **go quiet rather than wrong**.

⭐⭐ **And it was tested by breaking the file, not by reading it.** Two patched copies — one call with the arm removed, one with an argument too many — and it refused both, naming the line. ⚠️ **This is the [§0](FINDINGS.md) discipline applied to a checker instead of to the code:** a green verdict from a check nobody has ever seen fail is not evidence. It is the same rule as *prefer a test that could falsify the claim* (working-contract rule 5), aimed one level up.

### 52.7 ⛔ A FIELD THAT HAD BEEN LYING SINCE `v` WAS FIRST PRESSED, AND ONLY THE COLLAPSE WOULD HAVE FOUND IT

`ArmSession.frame` is set at construction from `--frame` and **was never updated when `v` cycled the control frame.** The session moved to `tool`; the object still said `world`.

⭐ **Nothing reads it today, which is exactly why it was invisible** — the script builds its `CartesianTeleop` from its own `control_frame` local. ⛔ **But `ArmSession.enter_teleop()` builds one from `self.frame`**, so the collapse in [§8.2 item 23](ROADMAP.md) would have quietly put the arm back in the frame the session started in — a wrong axis mapping, arriving as *the puck drives the wrong way after pressing `v`*, with no exception anywhere.

⚠️ **The general form, and it is worth more than the one-line fix:** a field that nothing reads cannot be wrong yet, and it is not therefore harmless. **It is a loaded trap for whoever wires it up.** Unwired state has no feedback path — the same reason this whole class went stale in an hour while unwired ([§52.1](FINDINGS.md)).

**486 headless tests. `check_restructure.py` green with five checks. Nothing pushed (working-contract rule 9).**

---

## 53. ✅⭐⭐⭐ STEP 2 IS CONFIRMED ON THE ARM, THE PER-CYCLE LOOP IS N-ARM, AND FOUR MECHANICAL REWRITES EACH INTRODUCED A DEFECT — 2026-08-14, late evening

> Eight plumbing commits (`b33ff82` … `fbb6945`) after step 2's three. **486 headless tests, `check_restructure.py` green with seven checks, nothing pushed.** ⬜ **None of the eight has been on the arm** — the bench test is at the top of [HANDOFF](HANDOFF.md) and is one question.

### 53.0 ✅✅ WHAT JULIEN CONFIRMED, AND IT CLOSES STEP 2's OWN TEST

> *"Everything seemed to work as it should."* — driving TELEOP, a recording started and discarded, `q q` parking to 0.027 rad and disabling.

⭐ **His paste carries the three things step 2 changed, all correct:**

| what appeared | what it confirms |
|---|---|
| `[B TELEOP  ] t=  30.0s  ⚠️ 88Hz  hottest 39°C  jaw 35°C …` | the per-arm row, with the arm's name in the label |
| `⚠️ 88Hz` sitting beside the clock rather than after the temperatures | the session facts moved into the row's lead, as designed ([§52.2](FINDINGS.md)) |
| `⏹ RECORDED 6.1s, 532 samples … recording discarded` | the recorder is untouched by the restructure |

⭐ **Also visible and worth noting: 88 Hz.** The loop-rate warning fires below 92, and this is the fourth session in a row reading 83-88 rather than 100 ([§31.1](FINDINGS.md), tracked as [ROADMAP §8.2](ROADMAP.md) item 14). It is not new and it is not from step 2.

⭐⭐ **AND ARM G IS AVAILABLE.** Julien, this session: *"the g is free. You can use it. We can do both together and continue with the next features if sensible."* ⚠️ **That removes the wait, not the work** — `--arms B,G` still refuses, and three pieces have to land before it can start (§53.6).

### 53.1 ⛔⭐⭐⭐ A MECHANICAL REWRITE PUT `frame=arm.frame` ON THE LINE THAT CREATES `arm`, AND THE CHECKER SAID IT WAS FINE

Moving the control frame onto the arm rewrote 39 sites by region. One of them was the construction line itself:

```python
arm = ArmSession(robot, name=arm_names[0], frame=arm.frame, …)
```

⛔ **`arm` is `None` there** — it is the `None` declared above the `try` — so that line raises `AttributeError` **after `build_robot()` has already enabled the motors**, and the `finally` block then disables them. **On a raised arm that is a sag.** A dry run cannot reach it: `--yes` is required, and the build is the line before.

⛔⛔ **`check_restructure.py` printed *"✓ `arm` is built on line 948 and nothing touches it earlier"* and was RIGHT.** Its ordering test was `lineno < built_at`, and the bad read was ON `built_at`. ⭐ **The check now covers the construction statement itself, bounded by the call's `end_lineno`** rather than its first line — because `arm = ArmSession(\n … arm.frame …)` puts the read on a continuation line, where a `<= built_at` test would miss it again.

⭐⭐ **Falsified before being trusted, in both positions**: one patched copy with the read on the call line, one with it on a continuation line. It refused both, by line number.

⚠️ **The transferable form: an off-by-one in a guard is invisible while the guard is green.** This one had been green for five commits and was never wrong before, because nothing had ever read `arm` on that exact line.

### 53.2 ⛔⭐⭐ `break` STOPPED MEANING WHAT IT SAID, BECAUSE A `for` APPEARED AROUND IT

The liveness check and the thermal stop both ended in `stop_reason = …; break`, breaking the **`while`**. Wrapping them in `for one in arms:` silently turned that into a break of the **`for`**.

⛔ **Consequence, had it shipped: a dead chain or a thermal stop would be recorded and the cycle would carry on commanding every arm**, with the stop taking effect only when some later check happened to fire. That is [§0](FINDINGS.md)'s defect class again — no exception, a plausible-looking session, and a guard that has quietly stopped guarding.

✅ Both now record into `stop_reason` inside the loop and act on it immediately after: `if stop_reason: break`, twice.

⭐ **Generalise it: adding a loop around existing code changes the meaning of every `break`, `continue` and bare `return` inside it.** None of them is flagged by any checker here, and the code keeps running.

### 53.3 ⛔⭐⭐ THIRTEEN COMMANDS SURVIVED A MECHANICAL `arm.` → `one.` PASS, BECAUSE THEY DID NOT SAY `arm`

The mode-action block (324 lines: TELEOP, CONTROLS, playback, PARK) was converted by rewriting `arm.<field>` to `one.<field>`. **Thirteen lines still said bare `robot`**, and every one of them was a command or a read on the robot handle:

```python
one.robot.command_joint_pos(full)   # was: robot.command_joint_pos(full)
```

⛔ **With two arms, every one of those would have driven arm B while the loop believed it was acting on G.** Found by grepping the converted block for the bare name, not by the substitution and not by any test.

⭐ **The rule this gives: a rename is exact about what it matches and silent about what it does not.** `arm.` and `robot` were two names for state belonging to the same arm; only one of them contained the word being rewritten.

### 53.4 ⚠️⭐ THE SAME PASS REWROTE PROSE, WHICH IS TRAP 1 IN THE CHECKER'S OWN HEADER

Renaming `park` → `base_pose` inside `park_and_wait` rewrote its docstring, six comments and one message the operator reads: *"park stopped."* became *"base_pose stopped."*, and *"the interleaved park (mode == 'park')"* became *"the interleaved base_pose (mode == 'base_pose')"*. **Two lines in that function name the parameter; ten were prose.**

⚠️ **`check_restructure.py` would have been perfectly happy** — it parses, and prose is invisible to it. **Reading the diff caught it.** That is exactly trap 1 in the checker's own docstring, written because `mode` appears 35 times in `main()`'s comments, and it is why [§36.3](FINDINGS.md)'s published count was 35% too high.

### 53.5 ⛔⭐⭐⭐ THE SHARED AXIS MAP IS ONE OBJECT FOR BOTH ARMS, SO A "PER ARM" EDIT WOULD APPLY TWICE

**Checked in the source rather than assumed.** `AxisMapStore.for_arm()` ends with `m = frames[frame]; return m` — **the same `AxisMap` instance** whenever the scope is SHARED, which is the default and what the rig runs today.

⛔ **So "apply this edit to every selected arm" is wrong for map edits.** With BOTH selected and a shared map, pressing `x` would flip the X motion **twice** — back to where it started — while printing two messages each claiming a flip. `f` (reverse) and the button swap have the same shape. Nothing raises, the map file ends up unchanged, and the operator has watched two confirmations go by.

⭐⭐ **THE DECISION, and it is simpler than deduplicating by object identity: a map edit always applies to exactly ONE arm — the first selected — and the scope decides whether that reaches the other arm.** Two independent reasons, and either alone would settle it:

1. **A map edit comes from one physical gesture on one puck.** `f` and `1`-`6` act on *"the control you just used"*, which is a memory of one hand on one device ([§52.1](FINDINGS.md)'s `last_active_axis`). There is no meaning to applying it to an arm whose puck was not touched.
2. **The scope already answers "does this reach both arms?"** and it is printed in the plan and again at exit: *"SHARED — edits here affect BOTH arms"*. A shared map edited once IS both arms edited.

⚠️ **What this does NOT change:** mode keys stay aimed at every selected arm, because `g` on two arms is a real and deliberate operation (8.6 kg weightless, which is why it is aimed at all).

### 53.6 ⬜⭐⭐ WHAT IS LEFT BEFORE `--arms B,G` CAN START — three commits, and the design is settled here

⭐ **The per-cycle loop is done.** Liveness, temperatures, the stall guard, the mode action and the status rows all run per arm, and every per-arm piece of state lives on `ArmSession` — **33 fields**, proven by `check_restructure.py`. What remains is the keyboard, the shutdown and the build.

**1. ⬜ The key dispatch aims at the selection.** The classification, decided:

| keys | apply to | why |
|---|---|---|
| `g` `t` `h` | every selected arm | a mode change is the thing `a` exists to aim ([§52](FINDINGS.md)) |
| `m` CONTROLS | ⛔ **refuse when BOTH is selected** | it is a wizard that edits ONE map from ONE wiggle; ask for `a` first |
| `x` `y` `z` `1` `2` `3` `f` `u` `0` `b` `v` | ⭐ **the FIRST selected arm only** | §53.5 — a shared map is one object, so twice is worse than once |
| `s` `p` and their digits | every selected arm, each to its OWN slot | the poses are per arm and keyed by arm in the file |
| `o` `c` gripper | every selected arm | a command, and with BOTH selected two grippers is what the operator asked for |
| `-` `+` `,` `.` `ö` `ä` `e` | park speed and ease ramp: every selected arm. Linear/angular speed, corners, ease profile, `r`: session-wide | speed and ramp live on the arm; the rest are one session setting and the plan line shows one value |
| `w` `l` `q` `?` | session | the recorder spans arms, and quitting is not per arm |
| ⛔ `a` | the selector itself | refuses while any arm is in CONTROLS ([§52](FINDINGS.md)) |

**2. ⬜ The shutdown and the consent flow over N arms.** `park_and_wait()` is single-arm and blocking. ⭐ **Extend it to a list and advance every arm per cycle** rather than parking them one after another: sequential parking doubles the shutdown and makes *"any key stops it"* stop only one arm. ⛔ **A fault on one arm still stops all** ([§53.2](FINDINGS.md) applies here too — the loop's `break`s). ⭐⭐ **And it has NO tests today**, while being the most safety-relevant path in the file: it is what Ctrl-C, `q q` and every unplanned stop run. It is importable, so a fake robot can test arrival, stall, a key press, a dead chain, and one arm arriving before the other.

**3. ⬜ Build N robots and drop the refusal.** In one commit, with three refusals added in the same breath:

- ⛔ **`--start-mode guide` refused when N>1.** ROADMAP §6's ruling: two arms going weightless on a first run is the worst possible first run.
- ⛔ **`w` and `l` refused when N>1**, until the two-arm recorder exists. ⚠️ **This is not tidiness: `Trajectory` holds one arm's joints, and the playback branch reads a single session cursor** — so two arms in replay would be driven from the same slice. ABC wants 14 states in ONE timeline ([§9.2](ROADMAP.md)), which is the recorder's job ([ROADMAP §8.2](ROADMAP.md) item 7).
- ⭐ **Delete `test_two_arms_are_refused_rather_than_half_driven`** in the same commit. It was written to be deleted deliberately ([§52.1](FINDINGS.md)); leaving it would fail, and *changing* it to expect success would quietly discard the reason it existed.

⚠️ **Then step 3 is his: `--arms B,G --start-mode hold`, desk clear, gripper enabled.** Arm G is free as of this session.

### 53.7 ⬜ THE BENCH TEST OWED FOR THE EIGHT PLUMBING COMMITS, and why it is worth two minutes

**Nothing in them changes behaviour at N=1**, so the test is the same single question as step 2's: does one arm still feel identical? ⛔ **Ask it before the next block lands**, not after. Eight mechanical commits can be attributed to a commit if something is wrong; eleven, with the keyboard rewritten in the middle, cannot — and three of the eight introduced a real defect that was caught by reading rather than by a test ([§53.1](FINDINGS.md), [§53.3](FINDINGS.md), [§53.4](FINDINGS.md)).

```bash
uv run apps/teleop_session.py --yes --arms B --start-mode hold
```

⭐ **Drive it, press `m` and leave it, park with `p 0`, and quit with `q q`.** The only visible differences from the last session are three lines of wording: `axis map B  :` and `park pose B :` in the plan, and `axis map B:` in the closing summary.

**486 headless tests. Nothing pushed (working-contract rule 9).**

---

## 54. ✅✅⭐⭐⭐ STEP 2 IS COMPLETE — TWO ARMS BUILD, AND THE NEXT THING NEEDED IS JULIEN AT THE BENCH — 2026-08-14, night

> Five more commits (`e3a5c71` … `66a563e`). **498 headless tests, `check_restructure.py` green with eight checks, nothing pushed.** ⬜ **`--arms B,G` has never run on the hardware** — that run is step 3 and it is his ([§54.7](FINDINGS.md)).

### 54.0 ✅ WHAT NOW WORKS, AND WHAT DELIBERATELY DOES NOT

⭐ **`--arms B,G --start-mode hold` builds two arms and drives them from one loop.** Everything below `ArmSession` is per arm: the robot, the puck, the axis map, the control frame, the base pose and slots, the CONTROLS memory, the temperatures, the last chain read, and this cycle's puck deflection. **34 fields**, proven by the checker rather than claimed.

| deliberately refused | why |
|---|---|
| ⛔ `--start-mode guide` with two arms | ROADMAP §6's ruling: two arms weightless on a first run is the worst possible first run. `g` reaches the same state, but only after the operator selected BOTH and pressed a key at a rig they are watching |
| ⛔ `w` and `l` with two arms connected | `Trajectory` holds ONE arm's joints and the playback cursor is one session clock. ABC wants 14 states in one timeline ([§9.2](ROADMAP.md)), so this is the recorder's job ([ROADMAP §8.2](ROADMAP.md) item 7), not a small extension |
| ⛔ `m` CONTROLS with BOTH selected | it edits one map from one wiggle. It says to press `a` first |

### 54.1 ⛔⭐⭐⭐ THE LOOP VARIABLE LEAKED, AND AT N=1 THAT WORKS PERFECTLY

**Python leaves a `for` variable bound after the loop ends.** So `one.robot` written *below* a `for one in arms:` block still runs, using whichever arm the loop finished on. ⭐⭐ **With one arm that is always the right arm.** It works, every test passes, and it proves nothing.

⛔ **How it got in:** a `robot` → `one.robot` rewrite was bounded by line numbers, and the start line it searched for matched the FIRST `for one in arms:` in the file — the liveness loop, not the mode loop it was written for. It ran across the session-level blocks in between, and three lines came out reading a leaked variable:

| site | what it would have done at N=2 |
|---|---|
| the recording sampler | recorded arm G's joints while the label said the session's first arm |
| the button-driven gripper | moved arm G's jaws when arm B's puck button was held |
| the save-pose handler | saved arm G's pose under arm B's name |

✅ **`check_restructure.py` check 6 now refuses any use of `one` outside a region that binds it** — a `for` loop (Name *or* tuple target), a function parameter, a comprehension generator, or a `lambda one=one:`. It found exactly those three.

⭐ **Two of them were fixed by naming the arm out loud** (`rec_arm = arms[0]`, `arm`). **The third was fixed structurally, and it needed to be anyway: the puck block is now per arm.** It read `arm.reader` once, outside any loop, so with two arms it would have read one hand and handed that deflection to both arms.

⛔⭐ **AND I NEARLY DID IT AGAIN TWO COMMITS LATER.** The CONTROLS readout began with `arm = wizard`, which repoints the session's own `arm` for the REST of the loop, including the incident record and the teardown. Same shape, same silence at N=1.

⭐⭐ **So check 6 now watches `arm` too**, with a sharper rule: since the build became `for name in arm_names:`, `arm` holds the LAST arm built once the loop ends, so **every read of it after that loop is about "an arm" where the code means "the session"**. Six such reads existed. All six are now loops over `arms`, including the closing temperature report, which prints per arm.

### 54.2 ⚠️ CONTROLS OWNS THE WHOLE LIVE BLOCK, so the other arm's row is not painted during it

While CONTROLS is open its readout replaces the status rows, because it repaints continuously rather than once a second: the operator is watching the arm and the readout together to attribute a motion to a gesture.

⚠️ **At N=2 that means the other arm's row disappears while the wizard is open.** It is deliberate for now, and bounded: `m` refuses when two arms are selected, so the wizard is always a conversation with one named arm. ⭐ **If it turns out to matter on the bench, the fix is small** — paint the other arms' rows above the wizard's line instead of replacing them, which `StatusLine.set_rows()` already supports.

### 54.3 ⭐⭐ WHICH KEYS AIM, AND WHICH DO NOT — as built

Three names, computed once per keypress: `aimed` (every selected arm), `edit_arm` (the first selected arm), `wizard` (the arm inside CONTROLS, if any).

| keys | act on | why |
|---|---|---|
| `g` `t` `h` | every selected arm | a mode change is what `a` exists to aim |
| `s` `p` + digits | every selected arm, each to ITS OWN slot | `s 1` with BOTH selected records two poses under one digit, which is what a two-arm waypoint is |
| `o` `c` | every selected arm in TELEOP | they are commands, and BOTH means both grippers |
| `-` `+` `ö` `ä` in a park | every selected arm's speed and ease ramp | those live on the arm |
| `x` `y` `z` `1` `2` `3` `f` `u` `0` `b` `v` `m` | ⭐ **the FIRST selected arm only** | [§53.5](FINDINGS.md): a SHARED map is ONE object, so twice would flip a motion back to where it started |
| `w` `l` `q` `?` `e` `r` `,` `.` | the session | the recorder spans arms; the ease profile, corner blending and both speeds are one setting |

⚠️ **One consequence worth knowing at the bench: `p 1 2` with BOTH selected starts a run on each arm at the same moment**, each through its own slots. A slot empty on one arm is skipped for that arm only and never cancels the other's run.

### 54.4 ✅⭐⭐ THE SHUTDOWN PARK IS N-ARM AND FINALLY HAS TESTS — 12 of them

`park_and_wait()` became `park_arms()`, taking the list and advancing **every arm on every cycle**.

⭐ **Sequential parking would also have worked and would have been worse**, for two reasons that have nothing to do with speed: *"any key stops it"* would stop only the arm currently moving, and the second arm would hold a pose for the whole of the first arm's park with nobody watching it.

⚠️ **A dead chain is skipped, not fatal.** That arm cannot be commanded and is already sagging; the live arm can still be parked, which beats leaving it holding and far beats disabling it. The dead one is named loudly. ⭐ **The worst outcome wins** — dead, then stalled, then stopped, then arrived — because the caller releases the motors only on `arrived`.

⭐⭐ **This path had NO tests in either form**, while being what Ctrl-C, `q q`, a thermal cut-out and any loop exception run, with the arm raised when it starts and released when it finishes. It was verified only by Julien pressing Ctrl-C on live hardware.

⛔⭐ **AND ONE OF THE NEW TESTS COULD NOT FAIL FOR THE REASON IT CLAIMED.** It asserted that two arms received a similar NUMBER of commands, which sequential parking satisfies too: two arms a similar distance from their targets need a similar number of cycles either way. **That is working-contract rule 5 exactly** — evidence that cannot distinguish the claim from its opposite. It now logs the ORDER of commands across both arms, and I ran it against a deliberately sequential implementation to watch it refuse before trusting it.

### 54.5 ⭐ THE BUILD LOOP, AND THE ONE THING THAT MADE IT MORE THAN A `for`

⛔ **`build_robot()` stays in the script, visible.** It energises motors and is the most dangerous call in the project. With two arms it runs twice, and **the second build starts while the first arm is already holding under power** — so an arm is appended to `arms` the moment it is constructed, and a session whose second build fails still disables the first arm on the way out.

⛔ **The teardown disables every arm, each wrapped on its own.** An unwrapped loop whose first `shutdown_robot()` raised would leave the second arm's motors **energised and unattended**, which is the worst possible end to a teardown. If one cannot be confirmed disabled, it says so and says to cut the mains.

### 54.6 ⭐⭐ THE CHECKER HAS EIGHT CHECKS NOW, AND EVERY ONE OF THEM CAUGHT SOMETHING REAL

`uv run checks/check_restructure.py` — **run it after every commit in this series.**

| check | what it caught, in this repo, for real |
|---|---|
| 1 no moved name survives as a local | the original 247-reference move |
| 2 fields are read through `arm` or `one` | had to widen when the per-cycle work became a loop, or it called three correctly-moved fields "dropped" |
| 3 nothing reads `arm` at or before construction | ⛔ `frame=arm.frame` ON the construction line ([§53.1](FINDINGS.md)) |
| 4 every name resolves | ⛔ a region replacement that deleted `status_row()` along with the function it replaced |
| 5 per-arm helpers are called with an arm | arity, which `compile()`, check 4 and a dry run are all blind to |
| 6 no leaked loop variable | ⛔ three leaked `one` uses, then six stale `arm` reads ([§54.1](FINDINGS.md)) |
| 7 retired names stay retired | `control_frame` and `park`, deleted rather than moved |
| 8 progress | the series has a visible finish line |

⭐⭐ **Checks 3, 5 and 6 were each falsified before being trusted** — patched copies of the file, until the check refused by line number. ⚠️ **A green verdict from a check nobody has watched fail is not evidence.**

### 54.7 ⬜⭐⭐⭐ STEP 3 IS JULIEN'S, AND IT IS THE FIRST TIME TWO ARMS HAVE EVER RUN

⛔ **Everything below needs a human at the rig.** Working-contract rule 1: anything that sends a setpoint is his.

**Prerequisites, all checked and true as of 2026-08-14 night, with nothing energised:**

- ✅ **Both CAN adapters are on the bus**, out of DFU (`check_rig.py`).
- ✅ **Both arms have gripper calibration** in `config/gripper_limits.json` — B and G, so `--arms B,G` can run with the gripper enabled. ⚠️ Arm G's saved range reconciles by a −2π shift, which `build_robot()` verifies at startup and **refuses loudly** if it cannot. That refusal is a pass, not a failure: it happens before any control loop starts.
- ✅ **Two SpaceMice are attached.** The session asks for each puck by wiggle, in `--arms` order: arm B's first, then arm G's, with the first one excluded.
- ✅ **Both D405 cameras are attached** (serials `255323071773` and `260323072846`), which is not needed for this run and is recorded because the count has changed twice.

```bash
uv run checks/check_rig.py && uv run apps/ping_motors.py --arm B --yes && uv run apps/ping_motors.py --arm G --yes
```

✅⭐ **THAT PING WAS RUN BY THE AGENT ON 2026-08-14 AT NIGHT, and all 14 motors answered:** no arm holding a fault, error clearing OFF so a latched fault would have been named rather than erased, temperatures **32-36 °C** against a 55 °C warning, every motor at rest.

⭐⭐ **And it produced one fact worth having before the run: the two arms need OPPOSITE gripper shifts tonight.** Arm B reconciles with **−2π** (closed +0.198 → open −5.052) and arm G with **+2π** (closed +6.425 → open +1.197). ⚠️ **Both are handled automatically** by `build_robot()`'s reconciliation, and this is exactly what [§40](FINDINGS.md) established: **the ±2π shift is a property of the session, not of the arm.** Do not write either direction into a config file; run the ping.

⚠️ **One thing he will see and should not read as a fault: arm B's jaws are 3.6% open**, so the ping warns that almost no closing travel is left. The script says it itself — *"Harmless, and it looks like a fault if unexpected."*

**Then the run itself. Desk clear, gripper enabled, hand near the mains:**

```bash
uv run apps/teleop_session.py --yes --arms B,G --start-mode hold
```

⭐ **What to expect, in order:** wiggle arm B's puck, then arm G's · two build lines, one per arm · **two status rows**, arm B on top with the clock beside it · both arms HOLDING.

⭐⭐ **The five things worth trying, cheapest and safest first:**

1. **Read the two rows.** Each should show its own temperatures and its own joint angles.
2. **`a`** — it should say `SELECTED: G`, then `BOTH`, then back to `B`.
3. **`t` with only B selected** — arm B enters TELEOP and arm G stays HOLDING. Drive B with its puck. ⛔ **Then check the thing this whole design turns on: does arm G's puck move arm G?** Driving is never aimed, so it should, in whatever mode G is in.
4. **`a` to BOTH, then `t`** — both arms in TELEOP, each following its own hand.
5. **`q` then `q`** — both arms park at the same time, then disable. ⚠️ **This is the path with 12 new tests and zero hardware runs.** Watch that both arms move together rather than one after the other.

⛔ **What CANNOT work yet, so it is not a defect:** `w` and `l` refuse with two arms connected, `--start-mode guide` refuses, and `m` refuses while BOTH is selected. ⚠️ **And GUIDE on two arms is 8.6 kg going weightless** — ROADMAP §6.1 step 4 puts it last on purpose. Do it with one arm selected first, if at all.

**498 headless tests. Nothing pushed (working-contract rule 9).**

---

## 55. ✅✅⭐⭐⭐ TWO ARMS RAN ON THE HARDWARE, AND HIS TWO QUESTIONS ABOUT THE REFUSALS ARE ANSWERED HERE — 2026-08-14, night

> Julien, after the first two-arm session: *"Wow. Everything seems to work, and it seems quite good."* ⭐ **This is the first time two YAM arms have ever been driven from one process in this project.**

### 55.0 ✅ WHAT HIS RUN PROVED, LINE BY LINE FROM HIS OWN PASTE

Every one of these was read out of his terminal output rather than assumed:

| what he did | what it proves |
|---|---|
| `--arms B,G` with no `--start-mode` → refused | the guide refusal fires, and its message names the fix |
| `--arms B,G --start-mode hold` → plan with **two ARM lines, two axis maps, two park poses** | the plan is per arm, and each arm read its own entry from `config/park_pose.json` |
| wiggled arm B's puck, then *"one unassigned puck left — using it for G"* | `exclude=` works: the second puck could not be the first one ([§54.1](FINDINGS.md)) |
| two build lines, **B shifted −2π and G shifted +2π**, jaws normalising to 0.036 and 0.636 | two robots built in sequence, and the gripper reconciliation handled opposite shifts on the same rig in one session |
| `⭐ MODE: B HOLD · G HOLD` | the startup banner is per arm |
| `a` five times: G → BOTH → B → G → BOTH | `ArmSelector` cycles as designed and never lands on BOTH first |
| `t` on BOTH, then `g` on B alone, `t` on B, `g` on G alone, `t` on BOTH | **mode keys aim.** GUIDE was entered on each arm separately, which is ROADMAP §6.1 step 4 done the careful way |
| two status rows, arm B carrying the clock and `⚠️ 91Hz`, each arm its own temperatures, `q`, `EE` and `reach` | one row per arm, session facts on the first row only ([§52.2](FINDINGS.md)) |
| `q` → *"Every arm is HOLDING its pose"*, then `q` → **both `PARKING` lines, then both `PARKED` (0.019 rad)**, then both disabled | ⭐⭐ **`park_arms()` on hardware: two arms parked TOGETHER**, not one after the other, and its 12 tests describe what actually happened |
| per-arm closing report: temperatures and axis maps for B and G, both *"unchanged"* | the teardown is per arm and the map store wrote nothing it should not |

⭐⭐ **AND A FREE MEASUREMENT WORTH MORE THAN IT LOOKS: the loop ran at 91 Hz with TWO arms.** One arm has been reading **83-88 Hz** for four sessions ([§31.1](FINDINGS.md), [ROADMAP §8.2](ROADMAP.md) item 14). **Two arms did not make it slower.** ⚠️ Read that as one sample, not a law — but it is evidence against "CAN traffic is what costs the loop its 100 Hz", because doubling the traffic did not cost anything. **The two-arm control budget is fine**, which is the thing this run had to establish.

### 55.1 ⭐⭐ HIS QUESTION 1: WHY DO RECORDING AND PLAYBACK REFUSE WITH TWO ARMS?

**Because a recording holds ONE arm's joint angles, so with two arms it would silently save half a demonstration.**

⛔ **The concrete mechanism, checked in the code rather than reasoned about:**

- `src/recording.py::Trajectory` stores one sample as one list of joint positions, and the sampler calls `take.append(t, rec_arm.robot.get_joint_pos())` — **one arm, `arms[0]`**.
- So hand-guiding both arms through a task and pressing `w` produces a file containing arm B's 7 joints and metadata saying `"arm": "B"`. **The file is not wrong about itself.** It is simply half of what the operator just demonstrated, and nothing on screen would say so.
- Playback is worse in kind: `replay` is a single session-level cursor, and only the arm that parked to the recording's start pose enters `replay` mode. So `l` would drive arm B through B's recording while arm G sat wherever it was. **A two-arm demonstration would replay as a one-arm motion.**

⭐ **Why the fix is not a small extension:** [ROADMAP §9.2](ROADMAP.md) — `amazon-far/abc`, the target training format, wants **14 states and 14 actions per timestep, both arms in ONE timeline**. So the recorder has to grow from "one arm's angles" to "every arm's angles, in order, with the arm list in the metadata", and playback has to drive N arms from one cursor. That is [ROADMAP §8.2](ROADMAP.md) item 7, and it is the next real feature.

⚠️ **Why refuse rather than let it record one arm:** a dataset that mislabels how a demonstration was produced is worse than one that omits it ([ROADMAP §6.6](ROADMAP.md) on provenance). The refusal is one line and it names the reason on screen; a quietly half-captured episode would be found at training time, weeks later.

### 55.2 ⭐⭐ HIS QUESTION 2: WHY CAN HE NOT START IN GUIDE MODE?

**Because `--start-mode guide` makes the arms weightless before anything is on screen, and with two arms that is 8.6 kg going limp at once.**

⛔ **What GUIDE actually is:** `zero_gravity_mode` sets the position gain to **zero**, so the computed gravity compensation is the ONLY thing holding the arm up. There is no position term to absorb a modelling error. **Any shortfall in the model is an unopposed torque**, which is how the arm fell on 2026-08-10 with `--no-gripper` making the model 0.695 kg light and the elbow 39% short ([§11](FINDINGS.md)).

⭐ **Why pressing `g` later is different, and it is not about trust:** by then the arm is already built and holding, the operator has chosen which arm with `a`, and they are watching. `--start-mode guide` happens during startup, while the plan is still printing and the operator's attention is on the terminal. **Same end state, completely different amount of warning.**

⚠️ **He CAN still have it**, and he did in his run: select one arm, press `g`. ROADMAP §6.1 step 4 asks for GUIDE on two arms LAST, and his session did the careful version — GUIDE on B alone, then GUIDE on G alone.

### 55.3 ⚠️ HIS QUESTION 3: WHAT ELSE COULD BE WEIRD? — the five things worth knowing

1. ⭐ **`m` (CONTROLS) refuses while BOTH is selected**, and says to press `a` first. It edits one arm's map from one puck wiggle.
2. ⛔ **Map edits go to the FIRST selected arm only, even with BOTH selected** — `x` `y` `z` `1` `2` `3` `f` `u` `0` `b` `v`. `AxisMapStore.for_arm()` returns the SAME object to both arms while the scope is SHARED, so applying an edit per arm would flip a motion twice and print two confirmations ([§53.5](FINDINGS.md)).
3. ⚠️ **The map scope IS shared today**, which his plan printed twice: *"SHARED — edits here affect BOTH arms"*. Editing B's map changes G's. `--fork-map` gives each arm its own copy.
4. ⚠️ **CONTROLS takes over the whole live block**, so the other arm's status row disappears while the wizard is open ([§54.2](FINDINGS.md)). It is deliberate and easy to change if it annoys him.
5. ⚠️ **Arm B's jaws sit 3.6% open**, so `ping_motors.py` warns there is almost no closing travel left. The script says itself that this is harmless and looks like a fault if unexpected.

### 55.4 ✅⭐⭐ MIRROR MODE IS BUILT, and it is the feature that was waiting for two arms

**`i` engages it: the selected arm leads, the other follows it joint for joint.** Julien's idea, 2026-08-11: *"be able to move one of the arms in the guide mode and have the second arm just mirror the exact movements with zero latency."*

⭐ **`src/mirror.py` has existed since 2026-08-11 with 14 tests and NO script to use it**, because it needed exactly the two-arm process step 2 just built. [HANDOFF §5.5](HANDOFF.md)'s task list has had it as item 1 the whole time.

⛔ **It asks twice, like `l`.** Engaging starts a motion on the follower while the operator's hands and eyes are on the leader. The plan line quotes the gap it will close, the speed (0.30 rad/s), and says to hold the leader still until the row reads FOLLOWING.

⭐ **What the two-stage engagement is for:** the arms are never in the same pose when you start, so commanding the leader's angles straight across would make the follower **jump** the gap at whatever the rate limiter allows. `MirrorLink` ramps at a bounded speed until the gap is under 0.05 rad, then tracks — with **one rate limit applied in both states**, because an earlier version handed over by assigning the target directly and that handover was itself a 5 rad/s jump.

⚠️ **What the script adds on top of the tested class**, each for a reason from this repo's own history:

- the jaws go through `clamp_gripper`, never straight from the leader ([§4](FINDINGS.md): motor 7 cooked three times)
- `prev_q` is kept in step, so switching the follower to TELEOP afterwards does not snap it back ([§9](FINDINGS.md))
- the follower goes under position control before anything is commanded, or it would ignore every command while the row showed it tracking
- leaving MIRROR by ANY route drops the link, in one place rather than a cancel in each of `g`/`t`/`h`/`i`

⛔ **`MIRROR_SIGNS` is still a geometric PREDICTION and has never run on hardware.** It only matters for `--mirror mirror`, for arms that FACE each other. The default `copy` is right for arms side by side, which is how they stand.

⬜ **Never run on the arm.** It is ROADMAP §6.1 step 5 and it needs Julien.

### 55.5 ⚠️ TWO MORE STALE BLOCKS IN `COMMANDS.md`, both found by reading it to add one row

The tenth and eleventh instances of [§33.3](FINDINGS.md), and both were **hours** old rather than days:

1. Its header said *"`--arms B,G` refuses today and says why"*, written that afternoon and false by the evening.
2. Its calibration comment said *"as of 2026-08-10 `config/gripper_limits.json` holds B only, which is why G refuses to start with the gripper enabled"*. **Both arms are calibrated**, which is why his two-arm run kept the gripper.

⭐ **The second one had a real cost shape:** it would have told a reader to run `calibrate_gripper.py --arm G`, and that routine **drives the jaws into both mechanical stops**. A stale sentence that recommends a motion is worse than one that merely misinforms. ⭐ **Both now carry the correcting note**, and item 24 of [ROADMAP §8.2](ROADMAP.md) is the mechanical check that would have caught the first.

### 55.6 ⬜ WHAT IS NEXT, in the order I would do it

1. ⬜⭐⭐ **MIRROR on the arm** ([ROADMAP §6.1](ROADMAP.md) step 5). Ten minutes, and it is the first two-arm *feature* rather than plumbing. Procedure: [§55.7](FINDINGS.md).
2. ⬜⭐⭐ **The two-arm recorder** ([ROADMAP §8.2](ROADMAP.md) item 7, and the thing his question 1 is about). `Trajectory` grows to N arms, the sampler reads every arm, playback drives every arm from one cursor, old one-arm files keep playing. **This removes the `w`/`l` refusal** and is the last thing between the rig and collecting real demonstrations.
3. ⬜ **Collapse the two park implementations** ([ROADMAP §8.2](ROADMAP.md) item 23). `ArmSession.step_path()` and its tests describe a park that never runs.
4. ⬜ **The throttle message that names an unmeasured cause** ([ROADMAP §8.2](ROADMAP.md) item 21).
5. ⬜ **A private git remote of his own** ([ROADMAP §8.2](ROADMAP.md) item 7 in [HANDOFF §5.5](HANDOFF.md)), which needs his GitHub account.

### 55.7 ⬜⭐ THE MIRROR RUN, when he wants it — about ten minutes

⚠️ **Arms clear of each other and of anything on the desk.** The follower will move on its own.

```bash
uv run apps/teleop_session.py --yes --arms B,G --start-mode hold
```

1. **`a` until the row shows `SELECTED: B`.** That arm leads.
2. **`i`.** It prints the gap it will close and waits. Read it, then press **Enter**.
3. **Watch arm G's row**: it should read `ALIGNING — x.xxx rad to close`, then `FOLLOWING (copy)`. ⚠️ **Hold arm B still until it says FOLLOWING.**
4. **Now `g` on arm B** (it is still the selected arm) and hand-guide it slowly. Arm G should reproduce the movement.
5. **`i`** to stop, or `h` aimed at G. Then `q` `q` as usual.

⛔ **What to watch for, and what each would mean:** arm G moving in the wrong direction means `copy` is wrong for how they stand, so try `--mirror mirror` · arm G stopping with `MIRROR STOPPED` means it fell more than 0.35 rad behind, which means it is blocked, at a joint limit, or faulted · arm G never reaching FOLLOWING means arm B kept moving during ALIGNING.

**502 headless tests. Nothing pushed (working-contract rule 9).**

---

## 56. ✅⭐⭐⭐ MIRROR RAN, ITS STOP MESSAGE WAS WRONG, AND THE TWO-ARM RECORDER IS BUILT — 2026-08-14, late night

> Julien, after the first mirror session: *"works really well in general … I was able to control B in teleoperate in guide mode, that worked well. G followed."* ⭐ **Two arms, one hand, on the hardware.**

### 56.0 ✅ WHAT HIS MIRROR RUN PROVED, from his own output

| what happened | what it proves |
|---|---|
| `⭐ MIRROR: arm B LEADS, arm G FOLLOWS (copy)` with a **0.08 rad** gap quoted | `pick_pair` aimed it at the selected arm, and the plan measured the real gap before he committed |
| `▶ MIRROR engaged` then TELEOP on B, then GUIDE on B, and G tracked through both | ⭐⭐ **The leader's mode is independent of the link.** He drove B with the puck AND hand-guided it, and the follower copied both |
| his final rows: B `q [-0.58 1.15 0.37 0.7 -0.65 -0.11]`, G `q [-0.58 1.17 0.38 0.7 -0.66 0.27]` | ⭐ **Five of six joints agreed within 0.02 rad.** The copy is accurate; only joint 6 was out, by 0.38 |
| he engaged a second time from a **0.20 rad** gap after putting both arms in GUIDE | `enter_hold(follower)` before engaging takes the follower out of zero-gravity, so the commands reach it |
| `q q` parked both arms to 0.020 rad and disabled both | the shutdown is unaffected by the link |

⭐⭐ **He also put BOTH arms in GUIDE at once** (`⭐ MODE: GUIDE on B+G`), which is 8.6 kg weightless and is ROADMAP §6.1 **step 4**, done.

### 56.1 ⛔⭐⭐⭐ THE STOP MESSAGE NAMED THREE CAUSES AND THE REAL ONE WAS A FOURTH

His run stopped twice: *"follower fell 0.370 rad behind the leader (limit 0.35). It is blocked, at a joint limit, or faulted."*

⛔ **None of the three was true.** The arithmetic is in his own paste: five joints agreed within 0.02 rad while **joint 6, the gripper twist, was 0.38 rad apart**. He was hand-guiding the leader's wrist, which is the easiest joint to move by hand, **faster than the follower is allowed to move**. At a 1.0 rad/s follow limit, 0.38 rad of lag is about 0.4 s of faster-than-limit motion.

⚠️ **This is the same defect as [§41.2](FINDINGS.md)**, where the speed throttle blamed the reach limit at a manipulability of 0.1713 that its own docstring calls comfortable. **A message that lists possible causes without measuring any of them will name the wrong one**, and it costs the reader a hypothesis they then have to disprove.

✅ **`MirrorLink` now measures the leader's per-joint speed as it goes.** On a stop it reports:

- which joint opened the gap, by index and by name (`gripper_twist`)
- how fast the leader was moving THAT joint
- which of the two explanations the numbers support: faster than the follow limit means *"could not keep up"*; inside the limit means *"blocked, at a joint limit, or faulted"*

⭐ **And two things his run showed were missing:** the row now warns past 70% of the gap limit, so there is notice before it trips, and the stop says *"press i then Enter to engage it again"*.

### 56.2 ⭐⭐ HIS QUESTION: IS THE SPEED LIMIT REAL? — no, and here is the dial

**It is a software limit, and the motors are nowhere near theirs.** `SafeRobot` clamps every command from every mode to **1.0 rad/s per joint**, below all control logic ([§37.0](FINDINGS.md)). Julien's own hand-guided recordings reach **2.4 to 3.7 rad/s** ([§37.2](FINDINGS.md)), so the hardware does those speeds and only this code refuses to ask for them.

✅ **`--max-speed` now exposes it**, the same way `--reach` and `--floor` expose the workspace limits. The default is unchanged at 1.0, the plan line prints the value, and it flags the line when it is above the default.

⛔ **Raising it stays his decision** ([§37.2](FINDINGS.md)'s standing rule), and the recommended path has not changed: **1.0 → 1.5, then 2.0, one step at a time, watching the `⚠️ STUCK lead` warning rather than temperature.** What cooks these motors is holding still against a stop, not moving; the hottest reading in a 337-second session was 43 °C against a 55 °C warning.

⭐ **The mirror follow speed now READS the follower's own cap** instead of repeating `1.0`. A hardcoded copy would have silently become the binding limit the moment he passed `--max-speed 1.5`, and the mirror would have stayed slow with nothing on screen to explain it.

⚠️ **One thing to expect when he does raise it:** the gap limit (0.35 rad) is unchanged, so a faster leader can still open a gap faster than the follower closes it. If mirror keeps stopping at a higher `--max-speed`, the next dial is `DEFAULT_MAX_GAP` in `src/mirror.py`, and that one trades faithfulness for tolerance.

### 56.3 ✅⭐⭐ THE TWO-ARM RECORDER IS BUILT, so `w` and `l` no longer refuse

**A recording is now every arm's joints concatenated in `--arms` order.** That is exactly ABC's shape — 14 states per timestep, two arms in ONE timeline ([ROADMAP §9.2](ROADMAP.md)) — so the internal format and the target format have the same layout and no conversion can silently reorder them.

⭐ **`src/recording.py::Layout` owns the mapping** (which slice is which arm, which indices count towards lag) with 7 tests including a save/load round trip written to a real file.

⛔ **Four decisions inside it, each with a failure it prevents:**

1. **The arm order is metadata, not a convention.** A recording made with `--arms B,G` and replayed in a `--arms G,B` session would drive each arm with the other's joints. `l` refuses when an arm named in the file is absent, and the measured vector for the lag check is built in the RECORDING's order.
2. **Old recordings still play.** `Layout.from_meta` reads files carrying a single `arm` field, which is all six of Julien's existing recordings — verified by running `check_recordings.py` over them, where they now show an `arms` column reading `B`.
3. **Every arm must arrive before any arm plays.** Each parks a different distance to its own slice of the start pose. Starting the clock on the first arrival would leave the second arm still parking while the recording ran.
4. **The lag check takes INDICES, not a prefix count.** With one arm the gripper was last, so "the first six joints" skipped it. With two arms arm B's gripper sits at index 6, in the middle.

⭐⭐ **A free combination worth knowing: pressing `w` while mirroring records BOTH arms.** One hand produces a two-arm demonstration, and `method` records `B:guide+G:mirror` so the dataset says how it was made. ⚠️ Whether it is *good* training data is a separate question — the follower is a rate-limited copy, so it lags slightly and never shows independent two-hand coordination. Tracked as [ROADMAP §8.2](ROADMAP.md) item 26.

### 56.4 ⛔⭐⭐ NOTHING IN THIS PROJECT KNOWS WHERE THE OTHER ARM IS

**Every limit here is per arm and relative to that arm's own base:** the 0.60 m reach sphere, the floor, the joint margins. **No code anywhere knows a second arm exists in the same space.**

⚠️ **Until MIRROR that was almost harmless**, because every motion had a hand on the arm or a puck under it. **MIRROR is the first mode where an arm moves with nobody's hand on it**, and the arms stand side by side, so a leader reaching across can drive the follower into it.

⭐ **Today the only guard is the operator, and the mirror plan line now says so in as many words.** ⭐ **What a real fix looks like:** both arms are already MuJoCo models, because that is how `mink` solves the IK, so putting both in one scene and asking for the minimum distance between their bodies is a per-cycle quantity. Same shape as the workspace clamp: measure, then refuse. Logged as [ROADMAP §8.2](ROADMAP.md) item 25, and it wants his decision about how close is too close.

### 56.5 ⚠️ FOUR MORE DEFECTS I INTRODUCED IN THIS BLOCK, and what caught each

| defect | caught by |
|---|---|
| the playback moved out of the per-arm loop and broke the if/elif chain, so PARK fell outside the loop | **check 6**, with 35 leaked uses of `one` |
| the `w` handler read a layout variable the sampler assigns LATER in the same cycle, so the first `w` of a session would raise `NameError` with motors live | **reading the diff**; no test or dry run reaches that line |
| a comment claimed the mirror align speed came from `src/mirror.py` while the number was duplicated beside it | **reading it back** after writing it |
| a new test asserted a constraint an existing test already covered, and my change made the existing test's docstring false | **grepping the test file before adding to it** |

⭐ **The `NameError` one is worth generalising: a variable read in the KEY DISPATCH must be assigned before the dispatch, not after.** Section 3 runs before section 3.4 every cycle, including the first. It is now a function, so no ordering can bite.

### 56.6 ⛔ THE GRIPPER FRAME CAN NO LONGER BE CHOSEN BY LIST ORDER

His two runs an hour apart loaded arm G's jaw limits **in different frames**: `[6.425, 1.197]` (shifted +2π) and then `[0.142, −5.086]` (unshifted). ⭐ **Both were correct** — the jaws physically moved between the runs, which is [§40](FINDINGS.md)'s point that the ±2π shift is a property of the session rather than of the arm.

⚠️ **Checking it properly turned up a latent hazard.** Each candidate shift accepts raw positions in a window of `travel + 2·margin`, and the candidates sit 2π apart. **If the jaws' travel ever exceeded 5.683 rad, two windows would overlap and both shifts would "fit"** — and the code took the first. Picking a jaw SCALE by list order is what commanded the gripper 2.6 rad past its stop and cooked motor 7.

✅ **Both reconcilers now refuse when more than one shift fits.** Measured today: arm B's travel is **5.250 rad** and arm G's is **5.228**, so there is about **0.43 rad of headroom** and a scan of every raw position from −10 to +10 rad finds exactly one match everywhere. **The guard is dormant**, which is precisely why it had to be code rather than a comment.

### 56.7 ⬜ WHAT IS NEXT

1. ⬜⭐⭐ **Record and play back a two-arm demonstration on the hardware.** Everything is built and none of it has run. The procedure is [§56.8](FINDINGS.md).
2. ⬜⭐ **`--max-speed 1.5`, one step, watching STUCK lead** — his decision, and it is what makes mirror and playback feel right.
3. ⬜ **Collapse the two park implementations** ([ROADMAP §8.2](ROADMAP.md) item 23).
4. ⬜ **A collision model** ([ROADMAP §8.2](ROADMAP.md) item 25), which needs his decision on the margin.
5. ⬜ **The throttle message that names an unmeasured cause** ([ROADMAP §8.2](ROADMAP.md) item 21) — the same defect class as §56.1, one mode over.

### 56.8 ⬜⭐⭐ THE TWO-ARM RECORDING RUN — about ten minutes

```bash
uv run apps/teleop_session.py --yes --arms B,G --start-mode hold
```

1. **`a` to B, `i`, Enter** — mirror engaged, arm G following.
2. **`g`** (still aimed at B) and hand-guide arm B through a short movement.
3. **`w`** to start recording. The banner should say **`RECORDING B GUIDE · G MIRROR (14 joints per sample)`**.
4. Move for five to ten seconds, then **`w`** again and **`7`** to save it (slot 7 is empty; 1-6 hold one-arm files).
5. **`h`** aimed at B to stop guiding, then **`i`** to drop the mirror.
6. **`l` then `7`.** The plan appears. **Enter.** ⭐ Both arms should park to their own start pose, print *"waiting for …"* if one arrives first, and then play together.
7. **`q` `q`.**

⭐ **What proves it worked:** `uv run checks/check_recordings.py` shows `7.json` with **`arms` = `B,G`**, and the playback drove both arms.

**512 headless tests. Nothing pushed (working-contract rule 9).**

---

## 57. ⛔⭐⭐⭐ SIX DEFECTS FROM HIS THIRD TWO-ARM SESSION, AND THE SPEED ANSWER IS A LAYER NOBODY HAD LOOKED AT — 2026-08-15, small hours

> Julien: *"works really well in general"*, then two specific complaints that were both right and both pointed at real defects. **519 headless tests, eight checks green, nothing pushed.**

### 57.0 ✅ WHAT WORKED, from his output

Recording two arms works: `⏺ RECORDING B TELEOP · G TELEOP (14 joints per sample)`, then `⏹ RECORDED 4.9s, 440 samples`, then saved. ⭐ **The 14-wide sample, the per-arm modes in the label and the save all did what they were built to do on the first try.** Both arms parked to their own slice of the start pose, one at 0.60 rad of travel and the other at 1.05.

⭐ **Also confirmed:** MIRROR with the leader in TELEOP and then in GUIDE, `--max-speed 5` accepted and survived (44-48 °C against a 55 °C warning), and the loop at 88-90 Hz with two arms.

### 57.1 ⛔⭐⭐⭐ THE PLAYBACK CANCELLED ITSELF, AND HIS LOG HELD THE PROOF ONE LINE APART

```
     arm B is at the start pose; waiting for G.
  ⚠️  playback cancelled — it never reached the start pose.
```

**The "leaving PARK abandons the run" block fires for any arm whose mode is no longer `park` while `park_path` is still set.** An ARRIVAL sets the mode to `hold` and left the path in place, so the next cycle treated a **completed** park as an abandoned one and cancelled the pending playback the park existed to reach.

⚠️⚠️ **IT COULD NOT SHOW WITH ONE ARM.** The handover happened in the same cycle as the arrival, so nothing was pending when that block ran. With two arms the first arrival WAITS for the second, so the pending playback was still there to cancel. **That is the third defect in two days whose signature is "correct at N=1 by construction"** ([§54.1](FINDINGS.md), [§56.5](FINDINGS.md)).

✅ **Fixed twice over, deliberately.** The arrival clears its own path, and the cancel is gated on the **measured** remaining path (`left > PARK_TOLERANCE`). ⭐ The second half is the one that cannot rot: a future exit that forgets to tidy up still cannot cancel a playback whose park actually finished.

### 57.2 ⛔⭐⭐ A STALE READ IN THE PLAYBACK, AND IT WOULD HAVE CORRUPTED THE ONE MEASUREMENT THAT MATTERS

When the playback moved out of the per-arm loop ([§56](FINDINGS.md)) it kept feeding `q` to the tracking log. `q` had been that arm's measured pose. **It is still a bound local of `main()` from other branches** — the park, the save handler — so it did not raise.

⛔ **It silently handed `TrackingLog` a 7-element snapshot taken at the end of the park, against 14-element targets**, and `observe()` quietly compared the first seven joints. That table is the only measurement anyone has of what the arm can physically follow ([§37.1](FINDINGS.md)), and it would have been wrong with nothing on screen to say so.

⭐ **Found by scanning the moved block for every name it READS but never WRITES**, which left only `all` and `range`. ⚠️ **No checker can see this class today:** `q` resolves, so check 4 is satisfied, and it is not a loop variable, so check 6 is not either.

### 57.3 ⛔⭐⭐⭐ THE SPEED ANSWER: `SafeRobot` CLIPS THE COMMAND TO 0.25 rad FROM THE MEASURED POSE

**Julien:** *"The max speed increasing didn't work as well as I was hoping. The robot was never blocked by anything. It just, like, didn't kind of catch up at high speeds."* **He was right on both counts, and the reason is a layer below anything the speed discussion had touched.**

`SafeRobot.command_joint_pos` applies **two** limits, and only the first was ever discussed:

1. a rate limit: the command may not move more than `max_speed · dt` per call — this is what `--max-speed` raises;
2. ⛔ **a following-error limit: the command may never be more than `max_lag` = 0.25 rad from the MEASURED position.**

⭐⭐ **So the mirror's gap can never be closed by more speed.** The gap it measures is `leader − follower_measured`, which is `(leader − command) + (command − follower_measured)`, and the second term is clipped at 0.25. **With a 0.35 rad trip limit, the leader only has to get 0.10 rad ahead of the follower's command.** Past a certain leader speed that is unavoidable, whatever `max_speed` says.

⚠️ **AND THE OTHER HALF OF WHY IT FELT LIKE NOTHING HAPPENED: `--max-speed` never touched teleop at all.** TELEOP clamps its own per-cycle joint change to `MAX_JOINT_STEP` = 0.015 rad, which is **1.5 rad/s** and sits below `--max-speed`. He raised the ceiling to 5 and teleop stayed exactly as fast as before.

✅ **Three changes, none of which moves a default:**

| flag | what it is |
|---|---|
| `--max-speed` | the `SafeRobot` rate cap (existed since earlier tonight) |
| ⭐ `--teleop-speed` | the per-cycle IK clamp, **the number that actually binds teleop, a park and a playback** |
| ⭐ `--mirror-gap` | how far the follower may fall behind before the link stops. **A tolerance, not a speed** |

⭐ **And the plan line now prints every layer and which one wins:** `joint speed : teleop 2.50 · planned 2.50 · mirror 5.00 rad/s`, with a second line naming the 0.25 rad following-error limit. [§37.0](FINDINGS.md) cost four days to the same invisibility; naming the number was the fix then, and showing which one binds is the fix now.

⭐⭐ **RECOMMENDATION ON RECORD, and it is his call as always** ([§37.2](FINDINGS.md)): `--max-speed 2 --teleop-speed 2`, which doubles teleop and matches the raise-in-steps plan already written down. For MIRROR add `--mirror-gap 0.6`, because the follower's lag at speed is physics rather than a fault. ⛔ **Watch the `⚠️ STUCK lead` warning, not temperature** — 44-48 °C in his `--max-speed 5` run against a 55 °C warning.

### 57.4 ⭐⭐ THE MIRROR STOP NOW HAS THREE MEASURED CAUSES, because two were not enough

At `--max-speed 1.5` it correctly said *"the leader moved it at 2.21 rad/s and the follower may only move at 1.50"*. ⛔ **At `--max-speed 5` it said *"blocked, at a joint limit, or faulted"* and nothing was blocked.** The follower was tracking as hard as it could.

✅ `MirrorLink` measures the **follower's** speed as well as the leader's, giving three causes decided in order of certainty:

| cause | test | what it means |
|---|---|---|
| `stuck` | the follower barely moved (< 0.05 rad/s) | blocked, at a joint limit, or faulted |
| `follow_limit` | the leader was faster than the follow allowance | the SOFTWARE limit binds — raise `--max-speed` |
| `tracking` | the follower moved nearly as fast as asked and still lost ground | ⭐ **the ARM is the limit** — more speed will not help |

⭐ The script adds the hardware's own evidence: `SafeRobot`'s clip count **for that link**, which is direct proof the command was being held back. And the advice differs per cause, including saying plainly when more speed will not help.

⛔ **The message was also TRUNCATED in his run**, losing the half that named the cause. `StatusLine.say()` fits each line to the terminal while a live block is up. The reason and the detail are separate fields now, one per line, with a test asserting both stay short.

### 57.5 ⛔⭐⭐ I DISARMED A GUARD BY RENAMING A LABEL, AND CAUGHT IT THE SAME DAY

`check_recordings.py::label_verdict` flags a recording whose label says HOLD while the arm moved at hand-guiding speed. **Its rule was `method == "live:hold"`, an exact match on a label FORMAT.**

Making the recorder multi-arm changed the label to `live:B:hold`, or `live:B:guide+G:mirror`. ⛔ **So the check that caught `3.json` would never have fired again.** Nothing failed. It parses the label now — strip `live:`, split on `+`, drop any `ARM:` prefix — and a test pins seven cases across both formats.

⚠️ **The general form: a guard that matches a string is coupled to a format, and a format change is exactly the kind of edit nobody thinks of as touching safety.**

### 57.6 ⛔ HE OVERWROTE `1.json`, WHICH IS THE FIFTH TIME A SLOT HAS BEEN LOST

His two-arm recording saved to slot 1 and replaced a hand-guided one-arm take from 2026-08-13. Twice before, an overwrite destroyed the only copy of a measurement ([§33.2](FINDINGS.md), [§34.7](FINDINGS.md)), and once more three hours later. **Every time, the prompt said nothing about what was in the slot.**

✅ **An occupied slot now names what it holds and asks for the same digit again**, the shape `l` and `p` already use. `src/recording.py::describe_slot` reads the FILE rather than the name, and reports an unreadable slot as unreadable, because that matters most to whoever is about to replace it.

### 57.7 ⛔⭐⭐ A DOCSTRING DESCRIBED A FEATURE THAT WAS NEVER WRITTEN

`src/mirror.py`'s header has said since 2026-08-11 that *"alignment reports its progress and gives up rather than chasing forever, exactly like PARK's stall detector."* **The gap check only ever ran in the `following` state.** A leader that kept moving during ALIGNING had the follower chasing it indefinitely.

⚠️ Julien never hit it, because the plan line tells him to hold the leader still and he did. ⭐⭐ **The lesson is about writing, not about mirrors: a design note written in the present tense reads afterwards as a description of the code.** This one sat three days inside the file whose whole argument is that untested behaviour fails on first contact.

✅ Implemented, with the patience PARK uses, and the header now says both what it does and that it was false until today.

### 57.8 ⛔ A PARK COULD HIJACK A PENDING PLAYBACK

`l` parks each arm to the recording's start pose and hands over on arrival, because **the first command of a playback is the only dangerous one**. Pressing `p 0` during that park replaced the path with a park to the BASE pose, and the arrival still handed over. **The recording would have begun from a pose it was never taught.**

✅ `begin_path` cancels a pending playback unless it is the playback's own call. ⭐ Found by auditing rather than by hitting it: it needs `p` pressed inside the two or three seconds the start-pose park takes.

### 57.9 ⚠️ THREE OF MY OWN TEST SCENARIOS IN A ROW DID NOT MODEL WHAT THEY CLAIMED

1. Two arms parking "together" was checked by comparing command COUNTS, which sequential parking also satisfies ([§56](FINDINGS.md)).
2. A follower "at its physical limit" closed a third of the gap per cycle, which keeps up easily; the link never tripped.
3. An alignment "chasing a moving leader" had the follower 1.0 rad ABOVE the leader and then moved the leader UP, so the leader closed the gap itself.

⭐⭐ **All three passed while proving nothing, and all three were caught by printing the per-cycle trace.** That is now the rule, written into the tests themselves: **a dynamics scenario built from intuition needs its trace read once before its assertion is trusted.** It is working-contract rule 5 applied to the test's own setup rather than to the claim.

### 57.10 ⬜ WHAT IS NEXT

1. ⬜⭐⭐ **Record and play back two arms end to end.** The cancel bug stopped him at the last step, and it is fixed. Procedure: [§57.11](FINDINGS.md).
2. ⬜⭐ **Try `--max-speed 2 --teleop-speed 2`** and see whether teleop feels different, which is the first time that number has ever moved.
3. ⬜ **Collapse the two park implementations** ([ROADMAP §8.2](ROADMAP.md) item 23).
4. ⬜ **A collision model** ([ROADMAP §8.2](ROADMAP.md) item 25) — needs his decision on the margin.
5. ⬜ **The throttle message that names an unmeasured cause** ([ROADMAP §8.2](ROADMAP.md) item 21). It is the last of the four guessing messages; three were fixed tonight.

### 57.11 ⬜⭐⭐ THE RUN THAT IS OWED — the two-arm recording, end to end

```bash
uv run apps/teleop_session.py --yes --arms B,G --start-mode hold --max-speed 2 --teleop-speed 2 --mirror-gap 0.6
```

1. **`a` to BOTH, `t`** — drive both arms with both pucks for a few seconds.
2. **`w`**, move both arms, **`w`**, then **`7`**. ⭐ Slot 7 is empty; an occupied slot now tells you what is in it and asks again.
3. **`l` then `7`, then Enter.** Both arms park to their own start pose. ⭐ **Watch for *"waiting for G"* followed by the playback actually starting** — that is the fixed bug.
4. **`q` `q`.**

⭐ **Proof afterwards:** `uv run checks/check_recordings.py` shows `7.json` with `arms` = `B,G`, and the playback printed a per-joint table naming rows `B base_yaw`, `G base_yaw` and so on.

**519 headless tests. Nothing pushed (working-contract rule 9).**

---

## 58. ⭐⭐⭐ THE HANDOFF AT THE END OF 2026-08-15: MIRROR WORKS, THE RECORDING PLAYBACK IS UNTESTED, AND HERE IS EVERYTHING TO DO WITHOUT HARDWARE — small hours

> ⛔ **Written because the chat ran out of context, not because the work stopped.** Julien had to leave and the arms will be disconnected. **This section is the one a contextless agent should read after [HANDOFF](HANDOFF.md)'s entry block.** 519 headless tests, eight checks green, nothing pushed.

### 58.0 ✅ WHAT HIS LAST SESSION PROVED — `--max-speed 2 --teleop-speed 2 --mirror-gap 0.6`

**Julien: *"the mirror worked much better this time. I think the mirror gap made a big difference because it allows for a bit of adjustment and, like, fall behind and stuff."*** ⭐ **So `--mirror-gap` was the right dial, and the reasoning behind it was right: the follower's lag at speed is physics, and the tolerance is what has to move.**

⭐⭐ **AND THE THREE-CAUSE DIAGNOSIS PROVED ITSELF ON HARDWARE, both ways, in one session:**

| what he did | what the message said | correct? |
|---|---|---|
| guided the leader at 1.86 rad/s with a 2.00 allowance | *"the follower managed 1.36, inside its 2.00 allowance — so the ARM itself could not track that fast, not the software"*, plus *"SafeRobot held the command back on 58 cycles"* | ✅ and it also said *"More `--max-speed` will NOT help"* |
| squeezed the wrist at **6.32 rad/s** | *"the leader moved it at 6.32 rad/s and the follower may only move at 2.00, so it could not keep up"*, 188 clipped cycles | ✅ the software limit really did bind there |

⭐ **6.32 rad/s is a new measurement**: the fastest hand motion this rig has ever recorded, well above the 2.4-3.7 rad/s of [§37.2](FINDINGS.md). ⚠️ It was a wrist twist, which is the easiest joint to move by hand.

✅ **`i` at the prompt switching copy ↔ mirror worked**, and he used it several times including engaging in `mirror` once. ✅ **`i` turning the link off worked.** ✅ Both arms parked and disabled cleanly every time. Temperatures stayed at 43-44 °C.

### 58.1 ⛔ WHAT IS STILL UNTESTED, AND IT IS THE ONE THING THAT MATTERS MOST

**He never got to the recording playback.** He recorded two arms successfully on 2026-08-14 (`14 joints per sample`, saved to slot 1), and the playback cancelled itself because of the bug fixed in [§57.1](FINDINGS.md). **That fix has never run.**

⬜ **So the first hardware job, whenever the arms are back, is [§57.11](FINDINGS.md)'s procedure.** The single thing to watch: both arms park to their own start pose, one prints *"waiting for G"*, and **the playback then actually starts**.

### 58.2 ⬜⭐⭐ THE JAW BLOCK — the fix his gripper complaint really needs

**What happened:** in MIRROR the follower's jaw command is the leader's *measured* jaw position, re-sent every cycle. He squeezed the leader's jaws while the follower was already holding an object, so the follower was commanded to close further than the object allows. The stall guard released the jaws, the next cycle commanded them back, and it repeated every 0.4 s.

✅ **Fixed for now: the message is rationed** (once, then at most every five seconds with a count, and the repeat explains the MIRROR case).

⬜ **What it still needs, and why I did not guess at it:** remember the release point and refuse to command tighter until the leader opens past it. ⛔ **That needs the SIGN of "tighter" in raw joint terms**, and the raw jaw frame flips by ±2π between sessions ([§40](FINDINGS.md), [§56.6](FINDINGS.md)) — B loads at `[0.198, −5.052]` and G at `[0.142, −5.086]`, so "closed" is the larger number in both, but the normalised and raw directions are not the same thing and a wrong sign would command the jaws HARDER onto the object. ⭐ **Establish it with `calibrate_gripper.py`'s own output or a two-line probe before writing the clamp.** Tracked as [ROADMAP §8.2](ROADMAP.md) item 29.

### 58.3 ⭐⭐ EVERY FLAG THAT CONTROLS SPEED, AND WHICH ONE BINDS WHAT

⛔ **Four limits sit on top of each other and only the lowest binds.** This has cost real time twice ([§37.0](FINDINGS.md), [§57.3](FINDINGS.md)), so here is the whole set in one place:

| flag | default | what it limits | who it binds |
|---|---|---|---|
| `--max-speed` | 1.0 | `SafeRobot`'s rate cap on the command | everything, but only when it is the lowest |
| `--teleop-speed` | 1.5 | the per-cycle IK step, a park, a playback ceiling | ⭐ **TELEOP** and planned moves |
| `--max-lag` | 0.25 | how far the command may lead the MEASURED pose | ⭐⭐ **tracking, and therefore MIRROR** |
| `--mirror-gap` | 0.35 | how far the follower may fall behind before stopping | MIRROR's patience, not its speed |

⭐ **The plan line prints all of them and flags the ones you raised.** ⚠️ `--max-lag` is also a torque limit, because the motor's push is `kp × (command − measured)`. Raising it makes the arm catch up harder AND hit harder. It is the one lever with no measurement behind it yet: **watch `limited_cycles` in a mirror run at 0.25 first**, which his last session put at 58 and 188 for two runs.

### 58.4 ⬜⭐⭐⭐ WHAT CAN BE DONE WITH NO HARDWARE AT ALL — in the order I would do it

⭐ **All of this is real work and none of it needs an arm.** Everything here is checkable with `uv run checks/check_restructure.py`, the 519 headless tests, and `--yes`-less dry runs.

1. ⬜⭐⭐ **Collapse the two park implementations** ([ROADMAP §8.2](ROADMAP.md) item 23). `ArmSession.step_path()` and its tests describe a park that **never runs**; the live one is the `mode == "park"` branch in the script. A reader who sees those tests pass is wrong about the code that moves 4.3 kg. ⛔ The trap is written down: the script does `arm.mode = "park"` and then calls its own `enter_hold()`, which leaves the mode alone, while the class's `enter_hold()` sets it to `hold` ([§52.1](FINDINGS.md)). Needs one bench pass over all modes afterwards.
2. ⏳⭐⭐ **A simulation harness — HALF BUILT 2026-08-15, see [§59.0](FINDINGS.md). `src/fake_arm.py` exists; the session cannot yet be told to use it.** ⭐ **The pieces already exist:** `scripts/teleop_sim.py` drives the IK with no arm, `mink` and MuJoCo are already dependencies (that is how the IK solves), and `scripts/test_park_arms.py` and `scripts/test_status_row.py` show the shape — a fake robot with `get_joint_pos` / `command_joint_pos` / `num_dofs` / `motor_chain.running`. ⛔ **What is missing is a fake that behaves like the real thing over TIME**: a first-order lag on each joint so a command is followed rather than teleported, plus `SafeRobot`'s two limits, so the loop can be run end to end without hardware. **That would have caught three of this week's defects** — the double-advanced cursor, the playback cancel, and the stale `q`. ⚠️ It cannot replace hardware for feel, gravity compensation or thermal behaviour, and saying so is part of building it.
3. ⬜ **The throttle message that names an unmeasured cause** ([ROADMAP §8.2](ROADMAP.md) item 21) — the last of four guessing messages; three were fixed on 2026-08-14/15 and this one has the pattern to copy.
4. ⬜ **A collision model** ([ROADMAP §8.2](ROADMAP.md) item 25). Both arms are already MuJoCo models, so the minimum distance between their bodies is computable per cycle. ⛔ Needs Julien's decision on the margin, so build the measurement first and report it before refusing anything.
5. ⬜ **Show each arm's control frame on its status row** ([ROADMAP §8.2](ROADMAP.md) item 28). Small, and `v` aims at one arm so two arms can differ.
6. ✅ **A mechanical check that `COMMANDS.md`'s flag list matches the parser — DONE 2026-08-15, `scripts/check_flags.py`, see [§59.1](FINDINGS.md)** ([ROADMAP §8.2](ROADMAP.md) item 24). That file has now gone stale **four** times in two days, and one stale line recommended a command that drives the jaws into both stops.
7. ⬜ **The cameras.** Both D405s are attached and the identification problem is unsolved: two identical cameras support the same picture sizes, so the trick used elsewhere cannot tell them apart, and macOS's USB order is not OpenCV's index order ([§22](FINDINGS.md), [§34.5](FINDINGS.md)). ⭐ **The wiggle method is the answer** ([§28.6](FINDINGS.md)) and it needs no arm: open each index, ask a human which window moved, remember the serial. ⚠️ `librealsense` works only with `sudo` on macOS, so keep streaming on the OpenCV path ([§28](FINDINGS.md)).
8. ⬜ **The MCAP export in ABC's schema** is still deferred by Julien pending his friend's spec ([ROADMAP §8.2](ROADMAP.md) item 7). ⭐ **Our own recordings are already the right SHAPE** — every arm's joints in one timeline — so that work becomes a serialisation rather than a re-collection ([§56.3](FINDINGS.md)).

### 58.44 ✅⭐⭐ THE DRY RUN NEEDS NO HARDWARE — verified 2026-08-15 with both arms UNPLUGGED

⭐⭐ **Omit `--yes` and the whole session runs with nothing connected.** Confirmed after the arms were disconnected:

```bash
uv run apps/teleop_session.py --arms B,G --start-mode hold
```

printed the full plan, the per-arm build, the CONTROLS block and `DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.`, then exited **0**.

⛔⭐⭐ **THIS SECTION SAID "exit 2 is the dry-run code, NOT a failure" AND THAT WAS WRONG. Corrected 2026-08-15, an hour after writing it.** The exit codes are:

| exit | means |
|---|---|
| **0** | ✅ a dry run that ran to the end |
| **2** | ⛔ **argparse rejected the arguments.** `--arms Z` (no such arm) and `--nonsense` both give 2 |

⛔⭐⭐ **HOW THE WRONG VERSION GOT WRITTEN, because the mechanism will catch the next agent too.** The evidence was a **zsh** loop:

```zsh
for f in "--arms B" "--arms B,G"; do uv run apps/teleop_session.py $f; echo $?; done
```

⛔ **zsh does NOT word-split an unquoted parameter expansion, unlike bash.** So `$f` arrived as **one** argument containing a space, argparse said `unrecognized arguments: --arms B`, and every iteration exited 2. **I then read that 2 off a run whose output I had discarded with `>/dev/null`, and explained it as a feature.** ⚠️ Use `${=f}` in zsh, or pass real words, or check the output rather than only the code.

⛔⭐ **This is [§0](FINDINGS.md)'s pattern applied to a TEST HARNESS rather than to the stack**: a plausible, confident, wrong answer with no exception raised. **The harness that checks the code needs the same distrust as the code.**

⭐ **What this buys, and it is more than it looks:** every flag combination, `parse_arms`, the `ArmSelector`, the mirror pairing, the recording-slot layout, the park plan and every printed line can be exercised **on any machine, with no arms, no CAN adapter and no cameras**. ⛔ **What it does NOT touch is the 100 Hz loop itself** — no cycle ever runs, so nothing about tracking, `SafeRobot` clipping, mirror engagement or playback timing is tested by it. Those need [§58.4](FINDINGS.md)'s simulation harness or the real rig.

⚠️ **Use this before asking Julien for a hardware run.** Several defects this session ([§53](FINDINGS.md), [§56](FINDINGS.md)) were startup-path defects that a dry run would have caught for free.

### 58.45 ✅⭐⭐ THE INCIDENT RECORDER IS VERIFIED — five files exist and one has now been READ

⛔ **[HANDOFF](HANDOFF.md)'s entry block has said since 2026-08-14 that the incident recorder *"can only prove itself during a failure, so it is not confirmed and cannot be"*. That is now half wrong and the half matters.**

✅ **It has fired five times** — `recordings/incidents/` holds files from 17:07, 18:03, 18:06, 18:14 and 18:22 on 2026-08-14, 3.7 to 4.8 KB each, written on Ctrl-C stops and thermal warnings. ⭐ **Nobody had opened one until now.** `2026-08-14T18-14-16` contains, for **both** arms:

- `mode`, `measured_joints` (7 each), `last_torques_nm`, `last_temperatures_c` per joint, `ee`
- the session's `reach_limit`, `floor_limit`, `loop_hz`, the git `commit`, and the whole `usb` bus

⭐⭐ **So the field whose absence cost the most on 2026-08-14 is there**: arm B's joint 3 was carrying **5.9 Nm** and arm G's **7.7 Nm** at the moment that session stopped. Recovering those on the day of the fall took a simulation of joint angles the arm had measured and discarded ([§45](FINDINGS.md)).

⛔⭐ **AND ONE FIELD IS USELESS AS WRITTEN: `chain_alive` is ALWAYS `false`.** The incident block runs *after* `shutdown_robot()`, deliberately, so the motors are off before anything is attempted — which means the chain has already been stopped by the time the field is read. **A field that reports the same value in every file is not a measurement**, and this one would be read as "the chain was dead when it stopped", which is exactly the distinction that mattered on 2026-08-14 ([§46.0](FINDINGS.md): a dead CAN link means no park is possible).

⬜ **The fix is two lines and it is [ROADMAP §8.2](ROADMAP.md) item 31:** capture the liveness of each arm into a local **before** `shutdown_robot()` and record that instead. ⚠️ Not done here because it was found with almost no context left, and a rushed edit to the teardown path is the wrong trade.

### 58.5 ⚠️ THE STANDING RULES A FRESH AGENT WILL BREAK FIRST

1. ⛔ **The agent never runs anything that sends a setpoint.** Scripts that enable motors and send nothing are yours (`check_rig.py`, `ping_motors.py`, `identify_arm.py`, `check_arms_match.py`). Anything that commands a position is Julien's. [HANDOFF §4](HANDOFF.md) rule 1.
2. ⛔ **Nothing is pushed until he says so.** [HANDOFF §4](HANDOFF.md) rule 9.
3. ⭐ **Run a session until you need him, then say what you need.** [HANDOFF §4](HANDOFF.md) rule 11, and the four things that count as needing him.
4. ⛔ **`uv run checks/check_restructure.py` after every commit.** Eight checks, and every one of them has caught something real ([§54.6](FINDINGS.md)).
5. ⚠️ **A mechanical rewrite needs its region and its prose checked, not just its substitutions.** Five defects this week came from region-bounded edits ([§53](FINDINGS.md), [§54.1](FINDINGS.md), [§57.2](FINDINGS.md)).
6. ⚠️ **A test scenario built from intuition needs its trace printed once before its assertion is trusted.** Three of mine in a row proved nothing ([§57.9](FINDINGS.md)).
7. ⛔ **"Correct at N=1 by construction" is this week's defect signature.** Three separate bugs worked perfectly with one arm and would have been wrong with two ([§54.1](FINDINGS.md), [§56.5](FINDINGS.md), [§57.1](FINDINGS.md)). **With one arm connected, a passing test proves less than it looks like.**

**519 headless tests. Nothing pushed (working-contract rule 9).**


## 59 ⭐⭐ 2026-08-15, SECOND SESSION — THE SIMULATOR EXISTS, AND THE DOCS ARE NOW CHECKED BY MACHINE

⭐ **Both items were built with the arms connected and untouched**, on purpose: Julien was running the owed two-arm playback on hardware, so everything here is new files only. Nothing in `scripts/teleop_session.py` was edited.

### 59.0 ✅⭐⭐⭐ A SIMULATED ARM THAT LAGS — `src/fake_arm.py`, [ROADMAP §8.2](ROADMAP.md) item 30

⛔⭐⭐ **THE POINT IS THE LAG, AND IT IS WHY THE EXISTING FAKES COULD NOT HAVE CAUGHT THIS WEEK'S BUGS.** `scripts/test_park_arms.py` and `scripts/test_status_row.py` carry a fake robot whose `command_joint_pos` **sets the measured position**. That is the right fake for what they test, and it is a fake in which **following error cannot exist** — so every defect living in the gap between *commanded* and *measured* is unreachable by construction. Three defects reached the hardware this week for that reason: the double-advanced cursor, the playback that cancelled itself, and the stale `q` feeding 7 elements against 14 targets.

⭐⭐ **THE LAG IS MEASURED, NOT INVENTED, AND THAT IS THE WHOLE CLAIM TO TRUST.** [ROADMAP §8.2](ROADMAP.md) item 11 closed on 2026-08-13 with a law fitted on three playbacks and then tested on held-out data:

> following error ≈ **0.04 to 0.10 rad + 0.033 s × speed**

Both halves are modelled as the different physical things they are, and the constants then fall out rather than being tuned:

| the law's term | what it physically is | in `fake_arm.py` |
|---|---|---|
| `0.033 s × speed` | a first-order lag, the joint chasing its target | `tau = 0.033` s |
| `0.04 to 0.10 rad` | static friction, a joint not moving for a small error | `deadband = 0.05` rad |

⭐ **Why that combination reproduces the law exactly.** Following a ramp at speed `v` the joint settles where its movement per step equals `v·dt`, so `(|gap| − deadband)·(1 − e^(−dt/tau)) = v·dt`, which for a small step is `|gap| = deadband + v·tau`. **A constant plus a term proportional to speed** — the shape the measurement found. `scripts/test_fake_arm.py` drives it at four speeds and asserts every one lands inside the measured band.

⭐ **It wraps the REAL `SafeRobot`**, not a reimplementation of its two limits, because a copy of a safety limit tests the copy. It can also **block a joint**, **kill the CAN chain** and **make the thermal read fail** — three failure paths that until now needed hardware, an obstruction, or an unplugged cable.

⛔⭐⭐ **WHAT IT CANNOT DO, and this belongs in every summary of it.** No gravity, so the real arm's droop under its own 4.3 kg and every gravity-compensation question are outside it. No dynamics, so no inertia and no coupling between joints. The thermal model is a **shape** with an uncalibrated constant, never a number to compare with a motor. And nothing about **feel** — whether teleop is pleasant, whether mirror tracks well enough to be useful, whether a speed is safe in the room. ⭐ **A simulator nobody has compared against reality is worse than none**, because the bugs it "clears" get shipped.

⬜⭐ **WHAT IS STILL MISSING, AND IT IS THE HALF THAT PAYS OFF:** `scripts/teleop_session.py` cannot yet be told to use it. A `--sim` flag that swaps `build_robot` for `build_fake_robot` is what turns this from a tested module into the thing that runs the whole loop with no arm. ⛔ **Deliberately not done in this session** — it edits the session script, and Julien was driving the real arms with it at the time.

### 59.1 ✅⭐⭐ THE DOCUMENTED COMMANDS ARE NOW CHECKED BY MACHINE — `scripts/check_flags.py`, [ROADMAP §8.2](ROADMAP.md) item 24

⭐ It reads every `uv run scripts/X.py …` line in `docs/` — **79 of them across five files** — pulls `X.py`'s real `argparse` declarations out with the `ast` module, and reports a flag the parser does not have, a value outside its `choices`, a value that will not parse as the declared `type`, and a value handed to an on/off flag. It also lists flags **no document mentions at all**, which is how `--max-lag` stayed invisible while Julien was asking why the arm could not keep up.

⛔⭐⭐ **THE THREE DEFECTS IT FOUND IN ITSELF ARE MORE INSTRUCTIVE THAN THE TOOL.** It came up green on the real docs the first time it ran, and `scripts/falsify_check_flags.py` — ten cases, seven breaks that must be reported and three correct commands that must be left alone — is what stopped that green being believed.

| what went wrong | why | the general lesson |
|---|---|---|
| **nine FALSE positives** | the pattern read `A && B --arm B --yes` as one command and blamed B's flags on A | ⚠️ **noise gets a checker ignored, which leaves the repo worse than no checker** — a green-looking thing nobody reads |
| **two silent MISSES** (`--arm Q`, `--arms B,Q`) | `choices=sorted(ARM_SERIALS)` and `ARM_SERIALS` is a dict literal in `src/yam_can.py`, so the choices never resolved and the check was **skipped without saying so** | ⛔⭐ **a check that cannot resolve its data must not pass quietly** — the same shape as a thermal guard treating an unreadable temperature as a safe one |
| **a regression I created while fixing a real false positive** | `--arm <B\|G>` is a legitimate placeholder; the rule added to allow it also counted any ALL-CAPS word as one, and **arm names are single capitals**, so `--arm Q` silently stopped being checked | ⛔⭐⭐ **a rule added to remove a false positive can quietly create a false negative, and only a falsification run makes it visible — before and after both look green** |

⭐ **The falsification count is the instrument**: it went 7 → 6 on that last one, and nothing else would have shown it.

### 59.3 ✅⭐⭐ THE TWO-ARM DISTANCE IS MEASURABLE NOW, AND ONE TAPE READING MAY CLOSE THE WHOLE QUESTION — [ROADMAP §8.2](ROADMAP.md) item 25

⭐⭐ **THE CHEAP ANSWER FIRST, BECAUSE IT MAY END THE DISCUSSION.** Each arm is already confined to a sphere of `REACH_LIMIT` = **0.60 m** about its own base, by a limit Julien chose on 2026-08-14. **Two spheres of radius 0.60 m cannot intersect if their centres are more than 1.20 m apart.** So:

> ⭐ **If the two bases are more than 1.20 m apart, a collision is geometrically impossible while the reach limit is enforced, and no new limit is needed at all.**

⛔ **With one exception that matters: GUIDE mode.** Hand-guiding is not subject to the reach limit, because nothing in software can stop a hand. So the clearance above covers TELEOP, MIRROR's follower and playback — **not** two arms being hand-guided toward each other.

⛔⭐⭐ **AND THE ONE NUMBER NEEDED IS NOT IN THIS REPO.** Nothing records where the two bases are relative to each other: not a document, not a config file, not a model. It cannot be computed, derived or inferred from anything the software can see — **it is a tape-measure reading**, which is why `collision.BasePose` has no default and every function requires one. ⚠️ A plausible default would be an unmeasured number quietly deciding whether two 4.3 kg arms may occupy the same space.

⭐ **The per-pose measurement** takes link positions from the same MuJoCo model and the same `mink.Configuration` the IK already uses, then gives each body the bounding-sphere radius **the model itself declares** (`geom_rbound`). ⚠️ **Bounding spheres are bigger than the parts inside them, so every figure UNDER-reports clearance** — read it as "at least this much", never as "the gap". For a safety measurement that is the correct direction to be wrong in.

⛔ **Four things it cannot see, all of which make it optimistic:** the jaws are posed shut whatever they really are; **anything the arms are HOLDING does not exist**, and a two-arm handover is exactly when they are closest; the desk, mounts and camera cables are absent; and it is a **snapshot**, so two arms 5 cm apart and closing fast read identically to two arms 5 cm apart and stopped.

⛔ **It refuses nothing, on purpose** — [§58.4](FINDINGS.md) item 4's ruling. This repo already carries one limit Julien never chose: the ±0.30 m cube that stopped him at 71% of the arm's reach for days ([§41.1](FINDINGS.md)).

⭐⭐ **AND THE ARM IS NOT SHAPED THE WAY I ASSUMED, which a red test taught me.** Three of the 14 tests failed on the first run and **all three were my assumptions, not the code**:

| what I assumed | what the model measures |
|---|---|
| joint index 1 is a shoulder that reaches sideways | it lifts the tip **163 mm vertically** and moves it **3 mm** horizontally. Index 0 is the base yaw; **index 2 is the elbow and the biggest horizontal mover, 260 mm at 1 rad** |
| two arms facing AWAY have more room than two facing each other | ⛔ **the opposite**, 0.265 m against 0.377 m. At rest, **`link3` sits 24 cm out the BACK of the arm with a 197 mm bounding radius**, so turning an arm around swings that rear link toward its neighbour |

⚠️ **The lesson is about testing rather than geometry:** I asserted an intuition about a machine I had not measured, and a red test briefly looked like a defect in the code. The tests now pin what actually matters — that the joint angles and the yaw are not ignored — and leave *which* arrangement is roomier to be measured per bench.

### 59.2 ⚠️ WHAT THE RIG READ AT THE START OF THIS SESSION, so the next agent can compare

✅ **Both arms healthy and cold**, 2026-08-15: all 14 motors answered register reads, `ping_motors` reported **no latched fault on either arm** with error clearing OFF (so a real reading, not an erased one), and every motor sat at **27-30 °C**. Every joint at rest within 3 quantisation steps of the zero code.

⚠️ **Only ONE D405 camera was attached**, serial `255323071773`, where 2026-08-14 night had two. ⭐ **This is the fourth different camera count in three days.** Ask `check_rig.py`, never a document — including this paragraph.

⭐ **The jaw frames differed between the arms in the same session**: B reconciled with a **−2π** shift and G with **none**. Both are correct; the shift is a property of the session, not the arm ([§33](FINDINGS.md)).


## 60 ⭐⭐⭐ 2026-08-17 — THE TWO-ARM PLAYBACK IS CONFIRMED ON HARDWARE, AND `--sim` NOW RUNS THE LOOP WITHOUT ANY

### 60.0 ✅✅✅⭐⭐⭐ THE OWED RUN IS DONE. THE LAST UNCONFIRMED FEATURE IN THE PROJECT WORKS

⭐⭐ **Julien ran it on 2026-08-17 at ~12:12 and it worked end to end.** This closes the item that had sat open since 2026-08-14, when the playback cancelled itself because one arm reached its start pose before the other ([§57.1](FINDINGS.md)). **The fix had never run until now.**

⭐ **The exact sequence from his log, because this is the evidence:**

```
⭐ MODE: PARK → arm B's start pose in recording 7, 0.22 rad of travel at 0.40 rad/s
⭐ MODE: PARK → arm G's start pose in recording 7, 0.16 rad of travel at 0.40 rad/s
⭐ PARK reached in 1.2s (0.019 rad off) → HOLD
     arm G is at the start pose; waiting for B.          ← ⭐⭐ THE FIX, WORKING
⭐ PARK reached in 1.8s, 0.4s of that settling (0.020 rad off) → HOLD
▶  PLAYING 5.2s of recorded movement on B+G at 1.00x
⭐ PLAYBACK finished in 5.2s → HOLD
```

⭐ **G had less distance to cover (0.16 rad against 0.22) so it arrived first, waited, and the playback then started.** That is precisely the case that used to abort.

✅ **Everything else in that session, checked line by line:**

| what | reading | verdict |
|---|---|---|
| two-arm recording | 5.2 s, **468 samples**, 14 joints per sample | ✅ and 468 appears in **both** the stop and the save message, so the [§34.0](FINDINGS.md) double-count fix is still holding |
| loop rate | **90 Hz** (468 ÷ 5.2), with the `⚠️ 90Hz` warning showing | ⚠️ still under 100, but **up from the 83-84 Hz of 2026-08-13**. [ROADMAP §8.2](ROADMAP.md) item 14 stays open |
| playback tracking | worst lag **0.076 rad**, against a 0.15 hold threshold and a 0.25 `max_lag` | ✅ comfortable. **0.0 s spent waiting for the arm to catch up**, so the replay was faithful |
| `q q` shutdown | both arms parked (0.020 and 0.022 rad off), **all 14 motors confirmed disabled** | ✅ |
| temperature | B peaked 36 °C, G 38 °C, jaws 33 and 32 | ✅ far below the 55 °C warning |
| jaw frames | B reconciled **−2π**, G with **none**, in the same session | ✅ expected; the shift is a property of the session ([§33](FINDINGS.md)) |
| speed flags | `--max-speed 2 --teleop-speed 2` raised, `--max-lag` left at **0.25** | ⚠️ so [ROADMAP §8.2](ROADMAP.md) item 27, raising the real ceiling, is **still untried** |

⚠️ **What that run did NOT test:** the overwrite guard (slot 7 was empty, so it never fired), MIRROR (not used), and `--max-lag` above 0.25.

### 60.1 ⛔⭐⭐ AND READING THAT LOG FOUND A DEFECT: THE TRACKING TABLE WAS LYING ABOUT WHICH ARM EACH ROW WAS

⛔ **His table printed six named rows and six rows labelled just `joint`, for a recording of fourteen joints.** Two rows were absent and nothing said so.

| fault | cause | why nothing caught it |
|---|---|---|
| ⛔ **arm G's rows had no name** | the print did `YAM_JOINTS.get(flat_index + 1)`, and `YAM_JOINTS` holds keys **1-7**. Arm G's flat indices 7-13 became keys 8-14, missed, and fell back to `"joint"` | ⭐ **at N=1 the flat index and the per-arm index are the same number.** The "correct at N=1 by construction" signature again |
| ⛔ **no row said which arm it was** | there was no arm prefix at all, so the six named rows read as *"the arm"* when they were only arm B | a plausible-looking table nobody had reason to distrust |
| ⚠️ **12 rows for 14 joints, silently** | rows with a top speed under 0.01 rad/s are skipped, which is right — a joint that never moved says nothing about tracking | ⚠️ **but saying nothing about skipping them is not.** Both absent rows were the grippers, which he never touched. A reader counting rows would conclude the recorder had lost two joints |

⛔⭐ **THE CORRECT CODE ALREADY EXISTED EIGHT LINES BELOW THE WRONG CODE.** The saved tracking JSON built per-arm names, with a comment explaining exactly why that was necessary. **So the printed table and the saved file had been disagreeing.** Two copies of one expression is what let them drift, so it is now one function, `flat_joint_names`, with `tracking_table` for the rows — both module-level and pure, the same reason `status_row` and `park_arms` were extracted.

### 60.2 ✅⭐⭐⭐ `--sim` RUNS THE ENTIRE LOOP WITH NOTHING ATTACHED, AND DRIVING IT FOUND FOUR MORE DEFECTS

⭐ **Proven by running it through a pseudo-terminal, not by reasoning about it.** A simulated two-arm session built both arms, entered TELEOP, recorded 14 joints, saved, parked both arms with *"waiting for B"*, **replayed his REAL recording 7**, printed a full tracking table with every row naming its arm, then parked and disabled all seven motors on each arm. **18 of 18 checks.**

⭐⭐ **THAT RUN IS ALSO A CROSS-CHECK OF THE SIMULATOR AGAINST HARDWARE, on the same recording:**

| | worst lag range |
|---|---|
| the **real** arms, his 12:12 run | **0.029 to 0.076 rad** |
| the **simulated** arms, same recording 7 | **0.056 to 0.069 rad** |

⭐ Same ballpark, from constants fitted four days earlier on different data ([§59.0](FINDINGS.md)). ⚠️ The sim's range is **narrower** because it has no gravity and no per-pose load, so it cannot produce the very low lags a lightly-loaded real joint achieves. **That is the expected shape of the difference, which is worth more than agreement would be.**

⛔⭐⭐ **THE FOUR DEFECTS, AND NOT ONE WAS FINDABLE BY READING:**

1. ⛔ **The missing-joints note was TRUNCATED by the screen painter.** It listed every unmoved joint; in a playback where nothing moved that was fourteen names on one line, cut off as `B base_yaw, B shoulder_pit…`. **A note whose entire job is to say which joints are missing, cut off before it says so, is worse than no note, because it looks answered.** Now two names plus a count, and the length is asserted in a test.
2. ⛔⭐⭐ **A SIMULATED RECORDING LANDED IN `recordings/` CLAIMING TO BE REAL** — `"method": "live:B:teleop+G:teleop"` — beside his six real demonstrations, which are destined to become training data ([ROADMAP §9.2](ROADMAP.md)). **A fake take that reads as real is the worst thing this project could leave lying about.** Now stamped **two independent ways** (a `sim:` prefix and `simulated: true`) and written to `recordings/sim/`, which is gitignored. ⚠️ The offending file was deleted.
3. ⛔ **The prefix is computed in TWO places and the second hardcoded `"live:"`**, silently undoing the stamp inside the very fix for it. ⭐ **Third time in one session that two copies of one expression drifted** (the tracking-table names, the flag checker's choices, this). **The separate `simulated` field exists for exactly this reason: two independent marks, so one being overwritten cannot make a fake take look real.**
4. ⛔ **The folder split first applied to READS as well**, which quietly removed one of the best uses of a simulator: **replaying a real take against simulated arms before committing it to 4.3 kg.** Sim recordings win when both exist; real ones stay playable.

⭐ **The fake also grew `motor_list` and `motor_interface`, so `shutdown_robot`'s park-then-disable runs end to end without hardware.** ⚠️ That is the most safety-critical code in the project and had only ever been tested by Julien pressing Ctrl-C on a live arm. One of the two new tests pins that **no motor is disabled while the chain is still running**, which is the ordering its docstring exists to defend.

### 60.3 ⛔⭐⭐ HIS COLLISION RULING: THE BASES ARE ~0.70 m APART, SO THE ARMS **CAN** REACH EACH OTHER, AND HE IS AVOIDING IT BY HAND

⭐ **Julien, 2026-08-17:** *"maybe the arms are, like, seventy centimeters apart. One arm can easily touch the other arm when it's, like, standing still. But that's not a huge problem because I can just, like, make sure when I do the features and stuff that I avoid collisions manually… we'll most likely move them closer together rather than further away."*

⛔⭐⭐ **SO THE CHEAP ESCAPE DOES NOT APPLY.** [§59.3](FINDINGS.md) showed that beyond **1.20 m** of base separation the existing 0.60 m reach limit makes a collision geometrically impossible. **0.70 m is well inside that, and he intends to move them CLOSER.** So:

| | |
|---|---|
| ⛔ **the reach limit does not protect the arms from each other** | and it never will at this spacing |
| ⭐ **his decision: manual avoidance**, by choosing what the features do | recorded as a decision, not a gap |
| ⛔ **therefore no automatic refusal is added** | which is what [§58.4](FINDINGS.md) item 4 already ruled: the margin is his |
| ⬜⭐ **what IS worth offering: a WARNING, not a refusal** | the distance is computable every cycle, so the status row could show it and go loud when it closes. **A warning constrains nothing and removes the "I forgot" failure**, which is the one manual avoidance is exposed to |

⚠️⭐⭐ **THE MEASUREMENT AT HIS SPACING, AND I GOT IT WRONG THE FIRST TIME.** I wrote "about 21 cm" here, having read the wrong row of my own tool's output. `uv run checks/check_collision.py --separation 0.70` actually says:

| pose | conservative clearance | closest parts |
|---|---|---|
| ⛔ **both at rest, all joints zero** | **2.5 cm** | `tip_left` ↔ `link3` |
| both elbows out | 6.9 cm | `link2` ↔ `link3` |
| one elbow out, one at rest | 6.9 cm | `link2` ↔ `link3` |
| both yawed toward each other | 9.6 cm | `tip_left` ↔ `link3` |
| both yawed AND elbows out | 21.2 cm | `link2` ↔ `link3` ← **the row I misread** |

⭐ **The worst case is the arms doing NOTHING**, which confirms his own words exactly: *"one arm can easily touch the other arm when it's standing still."*

⚠️⚠️ **BUT DO NOT PANIC AT 2.5 cm, AND THIS IS WHERE THE ESTIMATE'S LOOSENESS MATTERS.** The closest pair is a gripper tip against `link3`, and `link3`'s declared bounding sphere is **0.197 m** around a link that is nowhere near 40 cm thick. So those two spheres nearly touching means their *centres* are about 30 cm apart, and **the real metal gap is much larger than 2.5 cm.** ⛔ The number is a floor, never the gap ([§59.3](FINDINGS.md)).

⭐⭐ **THE ACTIONABLE CONSEQUENCE IS UNCHANGED AND STRONGER: bounding spheres are too coarse to give him a usable margin at 0.70 m.** A warning built on this figure would cry wolf constantly at rest. ⬜ **So a status-row warning needs the finer geometry first** — MuJoCo's own `mj_geomDistance` against the actual meshes rather than one sphere per body. That is the real prerequisite, and it is hardware-free work.

### 60.4 ⚠️⭐ THE CAMERAS ARE BLOCKED ON A macOS PERMISSION DIALOG, WHICH IS HIS TO CLICK

⛔ **`camera_view.py --list` on 2026-08-17 could not open a single camera:** `OpenCV: not authorized to capture video (status 0)`. macOS lists three devices (MacBook Air, **one** D405, Julien's iPhone) and OpenCV opened none of them.

⭐ **So step zero for all camera work is Julien granting camera access to whichever terminal runs it, then re-running the command.** Nothing in software can do this, and no camera item can proceed until it is done.

⚠️ **Only ONE D405 is attached** (`255323071773`). **The fifth different camera count in four days.** ⛔ **Ask `check_rig.py`, never a document, including this one.**

⭐ **What each camera item needs, so the second camera is only fetched when it is actually required:**

| [ROADMAP §8.2](ROADMAP.md) item | needs the second D405? | notes |
|---|---|---|
| **5** telling two identical D405s apart | ⛔ **YES, both attached** | the entire problem is distinguishing two identical devices; with one there is nothing to distinguish |
| **16** reading 848x480 as 16-bit depth | ✅ **no, one is enough** | ⭐⭐ **the most valuable camera item.** If that mode is the depth stream, **depth works with plain OpenCV and no SDK** — an open goal since [§8](FINDINGS.md). ⛔ **CHECK THE PIXEL FORMAT FIRST**: forcing it into an 8-bit photograph throws the depth away ([§31.2](FINDINGS.md)) |
| **6** several cameras with timestamps | ⚠️ **partly** | the machinery can be built and tested with one, but *"they line up"* cannot be proven with one. **Images must line up with joint data or the dataset is unusable** |


## 61 ⭐⭐⭐ 2026-08-17, LATE — THE FLAGS BECOME SAVED DEFAULTS, AND THE MIRROR MESSAGE WAS SENDING HIM AFTER THE WRONG FLAG

### 61.0 ⛔⭐⭐⭐ THE MIRROR STOP MESSAGE NAMED THE WRONG FLAG, AND IT COST HIM THREE SESSIONS

⛔⭐⭐ **This is the most expensive defect of the day and it was pure wording.** The `follow_limit` branch of the MIRROR stop message said, in full:

> `⭐ That allowance is --max-speed, now 2.0 rad/s. Raise it one step.`

⛔ **The stop is not triggered by `--max-speed`. It is triggered by the gap passing `--mirror-gap`**, which sits at a built-in **0.35** and which he had not set in either of those two runs. The message even printed the number — *"the follower fell 0.350 rad behind on joint 5 (limit 0.35)"* — **and never said which flag sets it.** A limit's value without its flag name is unusable.

⭐ **What he actually did, across three sessions on 2026-08-17**, chasing that advice:

| run | `--max-lag` | outcome |
|---|---|---|
| 1 | 0.25 (default) | mirror stopped at 0.350 rad on joint 5 |
| 2 | **0.4** | mirror stopped at **0.350** rad on joint 5 |
| 3 | **1.0** | mirror stopped at **0.362** rad on joint 5 |

⛔⭐⭐ **Raising `--max-lag` fourfold changed the stopping point by 12 thousandths of a radian, because it was never the binding limit.** ⭐ **And he had already found the answer himself two days earlier**: the 2026-08-15 run that he described as working *"much better"* used `--mirror-gap 0.6`.

⭐ **The message now names both routes, tolerance first**, with a concrete `--max-speed` computed from the leader speed it measured, and states plainly that `--max-lag` does not affect this stop. ⚠️ **Third time a speed-layer confusion has cost him a session** ([§58.3](FINDINGS.md) is the table that exists because of the first two).

⭐⭐ **AND THE REAL ANSWER TO HIS QUESTION** — *"I still don't know how to increase the max speed to a point where it's just really fast"*. He was moving the leader arm **by hand at 2.21 and 2.59 rad/s** while the follower was capped at 2.00. ⭐ **A follower cannot track a hand unless its cap exceeds what a hand does**, and [§37.2](FINDINGS.md) measured his own hand-guided recordings at **2.4 to 3.7 rad/s**. ⛔ **So `--max-speed 2` was always going to lose.** The value to try is **4**, with `--mirror-gap 0.6`. ⚠️ Both are safety limits and both are his to set.

### 61.1 ✅⭐⭐⭐ THE FLAGS ARE SAVED DEFAULTS NOW — `src/settings.py`, `--save-defaults`

⭐ **His request:** *"all of these flags should be default options that can be changed in some controls mode and then should be saved so that I don't always have to run with all of the flags, and I can change the default mode."*

⭐⭐ **Three layers, and the ORDER is the whole design:**

> built-in constant  →  `config/session_defaults.json`  →  command-line flag

⛔ **A deliberately typed flag always wins over the file.** That is a safety property rather than a convenience: a setting someone chose for one run must never be silently overridden by something on disk.

⭐ **To write them, run once with the flags wanted and add `--save-defaults`.** It saves the **effective** values, so what lands in the file is what the session just ran with. ⚠️ It works on a dry run too, which is the safest way to use it: nothing is energised and the file is still written.

⛔⭐⭐ **THREE SETTINGS CAN NEVER BE SAVED, and each absence is a decision:**

| flag | why not |
|---|---|
| `--yes` | ⛔ **energising the motors must be a conscious act on every single run.** A saved `yes: true` turns a dry run into a live one for anyone who has forgotten the file exists, and the dry run is this project's main safety habit |
| `--arms` | ⚠️ which arms are plugged in changes between sessions, and arm G is shared with a colleague. A stale value would try to build an arm that is not there |
| `--sim` | ⛔ a saved `sim: true` means either a session that looks normal and drives nothing, or somebody believing a simulated run was real |

⭐⭐ **FOUR OF THE SAVEABLE SETTINGS ARE SAFETY LIMITS** (`max_speed`, `max_lag`, `reach`, `floor`), so the plan prints which values came from the file **and flags any that are LOOSER than the built-in constant**. ⛔ **A flag is visible in the shell history and on screen; a saved default is not.** Without that line a session could run at four times the built-in speed limit with nothing explaining why.

⚠️ **The floor is the one limit where SMALLER is looser** — it is how far down the tip may go — so the direction is declared per setting rather than assumed. A rule that only understood "bigger is looser" would miss the one setting that lets the arm drive into the desk.

⛔ **Wrong types are refused, never coerced.** `float("fast")` raises and **`bool("false")` is `True`**, so a string `"false"` coerced to a bool would switch a setting ON while reading as off. Unknown keys are dropped **and reported**, because a misspelled setting that looks saved and does nothing is the silent failure this file could otherwise introduce.

⛔⭐ **THE FILE IS GITIGNORED, AND THAT IS A SAFETY DECISION.** Tracking it would let a `git pull` change how fast the arm may move on a machine with arms attached, with nobody typing a flag or reading a diff. ⚠️ The cost is that his preferred speeds do not travel to another clone. **That is the right way round: the settings are cheap to re-save and a surprise speed limit is not.**

⚠️⭐ **I created a `config/session_defaults.json` while testing and then DELETED it.** It held `max_speed: 4.0`, which is **my** number derived from [§37.2](FINDINGS.md) rather than his choice. ⛔ **Safety limits are Julien's to set**, so leaving a file behind that quietly quadrupled one would have broken the rule the whole feature is built to respect.

### 61.2 ✅⭐⭐ THE SAVE PROMPT WAS A DEAD END, AND HE FOUND IT THE FIRST TIME THE GUARD EVER FIRED

⭐ **Julien:** *"when I pressed save seven, then the guard came up. But then when I pressed a different number, I wanted to save it on, it's still discarded. So we should maybe find a way to save something that was accidentally saved on somewhere where we don't wanna overwrite."*

⛔ **He was exactly right.** The rule was *"the SAME digit confirms, anything else discards"*, so once the guard fired the only two exits were:

1. ⛔ **overwrite the take you were trying to protect**, or
2. ⛔ **lose the recording you just made.**

⭐⭐ **The obvious third thing a person wants — put it somewhere else — was the one thing it would not do.** ⚠️ And the asymmetry matters: an overwrite destroys old work, a discard destroys new work, and **a hand-guided recording cannot be re-taken identically**. Both outcomes must be deliberate acts.

⭐ **The rule now, in `save_slot_action`:**

| keypress | outcome |
|---|---|
| a digit on a **free** slot | ✅ saves |
| a digit on an **occupied** slot | ⚠️ asks |
| the **same** digit again | ✅ replaces |
| ⭐ a **different** digit while asking | **re-aims at that slot** (and asks again if it is also busy) |
| anything **not a digit** | ⛔ discards, naming what was kept |

⚠️ **Enter and space DISCARD here, and that is deliberate but worth knowing:** they CONFIRM in the `l` and `p` prompts, so a hand trained on those will reach for Enter. Here it must destroy the new take rather than the old one, because the old one is the thing already on disk.

⭐ It was extracted from `main()` into a pure function purely so it could be tested at all. **12 tests, and three of them fail against the old rule** — which is what makes them worth having.

### 61.3 ⛔⭐⭐ THE CAMERAS WORK FOR HIM AND CANNOT WORK FOR THE AGENT. THIS IS PERMANENT

✅ **Camera access is granted and the identification works.** His 2026-08-17 run:

```
    0  RealSense D405 (depth)        1280x720       99    0.1s  colour
    1  MacBook Air Camera            1920x1080       3    0.0s  colour
    2  Julien's iPhone Camera        1920x1080       1    0.2s  colour
```

⭐ All three identified **by measurement**, each asked for a mode only it offers.

⛔⭐⭐ **AND THE SAME COMMAND RUN BY THE AGENT FAILS: `OpenCV: not authorized to capture video (status 0)` on every index.** macOS grants camera access **per parent application**, and an agent's shell has a different TCC identity from his terminal. ⛔ **The agent cannot grant it**: it is a system privacy setting, it needs a dialog nobody can click on its behalf, and changing system settings is outside what an agent may do.

⭐⭐ **SO THE DIVISION OF LABOUR ON CAMERAS IS FIXED FROM NOW ON: the agent writes camera tooling and Julien runs it.** ⚠️ A fresh agent should not spend time diagnosing this as a bug in the code, and should not ask him to "just re-run it" expecting a different result.

⭐ **[ROADMAP §8.2](ROADMAP.md) item 16's tooling ALREADY EXISTS** and nobody had noticed: `camera_view.py --probe` sweeps resolutions and codecs and reports the real FOURCC pixel format for each. So the depth question needs one command from him rather than new code:

```bash
uv run apps/camera_view.py --camera d405 --probe
```

⚠️⭐ **AND THERE IS A CONTRADICTION WORTH SETTLING WITH IT.** The device names itself *"Depth Camera 405"* and `camera_view.py` warns that *"macOS exposes only this camera's DEPTH stream over plain UVC, so expect a depth/infrared picture rather than colour"*. ⛔ **But his measurement says `colour` at 1280x720.** Either the warning is stale, or a depth frame delivered as three channels reads as colour to the brightness check. **The probe's FOURCC is what settles it, and the answer decides whether depth is available with no SDK at all** ([§8](FINDINGS.md), [§31.2](FINDINGS.md)).


## 62 ⭐⭐⭐ 2026-08-17, EVENING — HIS SPEED SESSIONS ANSWERED FOUR QUESTIONS, AND THE SIMULATOR CAUGHT A CRASH 616 TESTS MISSED

### 62.0 ✅⭐⭐⭐ MIRROR WORKS AT SPEED, AND THE ANSWER WAS `--mirror-gap 2`

⭐⭐ **Four runs on 2026-08-17, and the fourth worked.** The sequence is the finding:

| run | flags | result |
|---|---|---|
| 1 | `--max-speed 4 --mirror-gap 0.6 --max-lag 0.4` | ⛔ stopped, gap **0.636** on joint 6, leader at **5.66 rad/s** |
| 2 | `--max-speed 10 --mirror-gap 0.6 --max-lag 1` | ⛔ stopped, gap **0.640** on joint 6, leader at **6.83**, follower **managed 2.64** |
| 3 | `--max-speed 10 --mirror-gap 2 --max-lag 1` | ✅ **`FOLLOWING (copy) — tracking 0.012 rad behind`**, through 83.8° of hand-guided drift |
| 4 | same, plus `--teleop-speed 10` | ✅ used for CONTROLS and PARK work |

⭐⭐ **THE PHYSICAL LIMIT IS JOINT 6, `gripper_twist`, AT ABOUT 2.6 rad/s.** Run 2 is the measurement: the follower was *allowed* 10 rad/s and **managed 2.64**, so the message correctly switched from *"the software is the limit"* to *"the ARM itself could not track that fast"*. ⛔ **More `--max-speed` cannot help past that**, and the three-cause diagnosis said so on the run where it became true.

⭐ **So the working recipe for hand-guided mirror is a WIDE TOLERANCE, not more speed:** `--mirror-gap 2`. The follower runs ~0.012 rad behind in normal tracking and only needs the headroom for the moments a hand outruns joint 6.

⚠️⭐ **AND HIS HAND IS FASTER THAN THIS REPO BELIEVED.** [§37.2](FINDINGS.md) put hand-guided motion at **2.4-3.7 rad/s**, measured in August from three playbacks. On 2026-08-17 he was measured at **5.66 and 6.83 rad/s**. ⛔ **My own mirror message printed the stale 2.4-3.7 range in the same breath as a live 5.66 reading**, which is how a message discredits itself. **It quotes the measurement now and cites no remembered range.**

### 62.1 ⛔⭐⭐⭐ THE SIMULATOR CAUGHT A CRASH THAT 616 UNIT TESTS COULD NOT

⛔ **My own save-prompt change read `replace_slot`, a local assigned ONLY inside the overwrite-guard branch.** So the **first save of any session**, with the guard never having fired, raised `UnboundLocalError` and took the session down.

⭐⭐ **`scripts/test_save_slot.py` has 12 tests of that exact decision and every one passed.** They call the pure function directly and hand it that argument. **The defect was in the CALL SITE.**

> ⛔⭐⭐ **THE LESSON, and it generalises past this repo: extracting a decision into a pure function and testing it does not test the code that calls it.** The extraction was right — it made the rules testable at all — and it moved the risk rather than removing it.

⭐ A `--sim` run found it in seconds, which is exactly the gap `--sim` was built to close. **So the driver is now `scripts/drive_sim_session.py` rather than a scratch file**, and it is the only thing in the repo that runs the 3000-line loop end to end: two arms built, the selector, TELEOP on both, a 14-joint recording, the save prompt, replaying a REAL recording on simulated arms, the staged park with one arm waiting, the tracking table, and `q q` disabling all 14 motors. **22 checks.**

⚠️ **The safe stop did its job**: both arms parked and all 14 motors were confirmed disabled after the traceback. The recording would have been lost.

### 62.2 ⛔⭐⭐ THE LIVE SPEED KEYS HAD NO CEILING AT ALL

⛔ His CONTROLS readout on 2026-08-17: **`lin 19.852 m/s  rot 954°/s`**. The default is 0.12 m/s, so that is **165x**, reachable in about 23 presses of a key that repeats when held.

⭐ `+` did `linear_scale *= 1.25` with nothing above it, at **three** sites, and both angular sites were the same. ⚠️ **Every other live adjustment in the file already had a bound** — park speed clamps both ways, the gripper step has a 0.200 ceiling — so these were the exception rather than the rule.

⭐ Backstops now at **2.0 m/s** and **12 rad/s**, deliberately generous so they can never be the reason a setting will not go where he wants. ⛔ A commanded speed that high does not move the arm that fast, because `SafeRobot` and the reach limit still bind. **What it does is make the IK target jump the whole workspace in one cycle, so the arm slams to the boundary at whatever `--max-speed` allows.**

### 62.3 ✅⭐⭐⭐ THE GRIPPER STALL NOW LATCHES — [ROADMAP §8.2](ROADMAP.md) item 29, CLOSED

⛔ **The release always worked and was undone on the next cycle.** His log, and the numbers are the whole diagnosis:

```
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.152
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.151
⚠️  ARM G GRIPPER STALLED (14 times now) (+1.03 Nm, not moving) — released to 0.150
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.147
```

⭐ Each release backed the command off to the measured jaw position. Each next cycle MIRROR copied the leader's jaw straight back over it. **A one-cycle correction against a source that re-commands at 90 Hz can only nibble.** ⛔ Pushing hard while not moving is the worst thermal case there is — full current, no motion, no cooling — and **motor 7 has been cooked three times** by this shape of problem.

⭐ **`ArmSession.hold_jaw` latches at the stalled position:**

| asked for | result |
|---|---|
| nothing latched | ✅ obeyed |
| **further CLOSED** than the block | ⛔ held at the block, so it stops pushing **but keeps the grip** |
| **more OPEN** than the block | ✅ obeyed, **and the latch clears** |

⚠️ **Opening always clears it**, which matters more than it looks: the object may have been put down, or the leader's hand may have opened. Anything moving away from the obstruction is evidence it is no longer being pushed into, and a latch needing an explicit reset would eventually be why the jaws refused to work for a reason nobody could see.

⛔ Wired into **both** MIRROR and TELEOP, because his log shows the stall firing four more times in TELEOP — holding a puck button re-commands the jaws every cycle exactly as MIRROR does.

⚠️⭐ **`teleop_session.py` ALREADY CARRIED A COMMENT DESCRIBING THIS EXACTLY** — *"in MIRROR the follower's jaw command is the leader's measured jaw, re-sent every cycle, so squeezing the leader while the follower holds an object fires this every 0.4 s indefinitely"*. **So the diagnosis existed for days and the fix did not.** Same pattern as the tracking-table names in [§60.1](FINDINGS.md): the right answer written down beside the wrong behaviour.

### 62.4 ✅⭐⭐⭐ A LIVE SETTINGS SCREEN ON `n`, AND HIS OBJECTION TO MY WORRY WAS CORRECT

⭐ I built the saving half in [§61.1](FINDINGS.md) and **asked** whether a live editor was wise, worrying that a value written to disk could be one he never typed. His answer:

> *"How would the live key change be one that I didn't type myself? That does not seem to make any sense to me if I'm in a control mode changing the key. And mainly, this is for saving which default value I wanna change, because if I change controls, normally, they always save in the live program."*

⭐⭐ **He is right on both counts and my worry was confused.** A keypress he makes IS him typing the value, and **the axis map is the precedent**: edited live with keys, written to `config/spacemouse_map.json`. ⚠️ Worth recording as a judgement error on my side: I invented a risk from the mechanism (a key rather than a flag) instead of from the outcome (who chose the number).

⭐ `n` shows the six limits, marks which one `-`/`+` moves, and prints the **built-in value beside any that differ** so a pushed safety limit is visible. `1-6` picks · `-`/`+` changes · `0` reverts to the session's start · `s` saves · `t`/`g`/`h` leaves.

⛔⭐ **`max_speed` and `max_lag` are pushed onto the live `SafeRobot` objects, so a change binds 4.3 kg on the very next cycle**, and the screen says that in those words.

⭐ **Speeds step by a RATIO** (0.1 → 0.125 and 8 → 10 are the same felt step; a fixed increment cannot be both). ⚠️ **The floor steps ADDITIVELY and its lower bound is negative** — it crosses zero, its default *is* zero so a ratio could never move it, and he uses `--floor -0.005` for a flat object on the desk.

### 62.5 ⛔⭐⭐ THE CAMERA DEPTH QUESTION CANNOT BE ANSWERED THIS WAY. `--probe` RETURNS NO PIXEL FORMAT AT ALL

⭐ He ran it, which is the right division of labour now ([§61.3](FINDINGS.md)). The result:

```
requested                actual       codec   measured fps
1920x1080 MJPG           1280x720     ÿÿÿÿ    30.0
1280x720  as-is          1280x720     ÿÿÿÿ    30.0
640x480   MJPG           640x480      ÿÿÿÿ    30.0
424x240   as-is          424x240      ÿÿÿÿ    30.0
```

⛔⭐⭐ **`ÿÿÿÿ` IN EVERY ROW MEANS THE PIXEL FORMAT IS UNREADABLE.** It is `CAP_PROP_FOURCC` returning **-1**, and `scripts/camera_view.py` already carries a comment saying that property is not readable through macOS's AVFoundation backend. **So [ROADMAP §8.2](ROADMAP.md) item 16 cannot be answered by asking OpenCV**, and no amount of re-running will change it.

⚠️⭐ **TWO MORE THINGS THAT CORRECT ITEM 16'S OWN PREMISE:**

1. ⛔ **848x480 was never swept.** The probe's resolution list is a fixed set in the script (1920x1080, 1280x720, 960x540, 640x480, 424x240). **The mode item 16 is about was not among them**, so nothing has tested it.
2. ⚠️ **Every larger request collapses to 1280x720**, and only 640x480 and 424x240 come back as asked. So the camera exposes three real sizes through this path, and `424x240` is the one unique to the D405.

⭐⭐ **WHAT WOULD ACTUALLY SETTLE IT: look at the PIXEL DATA rather than asking for a label.** A 16-bit depth frame packed into 8-bit channels has statistics no photograph has — one channel varying smoothly while another varies wildly, or all three channels identical. ⛔ **That needs a tool that captures one frame per size and reports per-channel statistics**, and it is [ROADMAP §8.2](ROADMAP.md) item 36. ⚠️ The agent can write it and **cannot run it**.

⭐ **The MJPG column is its own small finding:** requesting MJPG changes nothing at any size, so macOS is ignoring the codec request entirely and **resolution is the only lever**, exactly as the script's closing note says.


## 63 ⛔⭐⭐⭐ 2026-08-17, LATE — ITEM 16 IS ANSWERED **NO**, AND THE JAW LATCH WAS BEING CLEARED BY NOISE

### 63.0 ⛔⭐⭐⭐ THE D405 IS A PLAIN COLOUR CAMERA OVER UVC. [ROADMAP §8.2](ROADMAP.md) ITEM 16 IS CLOSED, ANSWERED NO

⭐⭐ **A definitive negative, from Julien's 2026-08-17 run of `scripts/probe_camera_pixels.py`:**

```
     848x480 → 848x480    uint8, 3 channel(s), zeros  0.00%   → ORDINARY PHOTOGRAPH
    1280x720 → 1280x720   uint8, 3 channel(s), zeros  0.00%   → ORDINARY PHOTOGRAPH
     640x480 → 640x480    uint8, 3 channel(s), zeros  0.00%   → ORDINARY PHOTOGRAPH
     424x240 → 424x240    uint8, 3 channel(s), zeros  0.00%   → ORDINARY PHOTOGRAPH
```

⭐⭐ **AND 848x480 DID EXIST AFTER ALL.** It came back at exactly 848x480, so the mode item 16 was written about is real; the old `--probe` simply never asked for it. **It is a colour photograph like the rest.**

⛔ **All four signatures are absent in every mode**: no spike of exactly-zero pixels (0.00%, where depth is full of holes), three channels that differ from each other rather than being identical, and no rough-beside-smooth split. ⭐ **So there is no depth in what macOS hands us over plain UVC**, and the hope recorded since [§8](FINDINGS.md) is settled.

⭐ **WHAT THIS BUYS EVEN THOUGH IT IS A NO**, and it is why negative results are worth writing down at all: **nobody needs to spend a day on `librealsense` and `sudo` on macOS** ([§28](FINDINGS.md)) hoping for a free depth stream. The D405 is a colour camera for this stack, and the depth capability needs the SDK or it needs nothing.

⚠️ **The one caveat, stated because the tool states it:** these are statistics, not a format read. A frame with 0.00% zeros and three differing channels is very hard to produce from depth data, but the claim is "behaves exactly like a photograph in four independent ways", not "the buffer is 8-bit BGR".

### 63.05 ⚠️⭐ I HAVE NOW BROKEN A LINK TWICE BY CITING A FILE FROM MEMORY

⚠️ Writing §63.0 I referenced `[CLAUDE.md](../CLAUDE.md)` for the "log negative results with equal weight" rule. **That rule lives in the *Mind Understanding* repo and this repo has no `CLAUDE.md` at all.** Two days earlier I added `[SETUP.md](SETUP.md)` to the roadmap row that complains about broken links ([§59.1](FINDINGS.md)).

⭐ **Both were caught by `check_links.py` and neither by me**, and both have the same cause: **citing a path I remember rather than one I checked.** ⚠️ The remedy is already written down in that same roadmap row — run `check_links.py` before the commit, not after — and it works. **What it does not do is stop the mistake happening**, so a third instance should be read as evidence that a cross-repo reference needs looking up every time.

### 63.1 ⛔⭐⭐⭐ THE JAW LATCH WAS BEING CLEARED BY SENSOR NOISE, AND I HAD WRITTEN A TEST SAYING THAT WAS CORRECT

⛔ His log, one session after the latch was built:

```
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.117
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.098
⚠️  ARM G GRIPPER STALLED (+1.03 Nm, not moving) — released to 0.104
```

⭐ **Three, where the latch should have held after the first.** The clearing rule released on **any** value above the block, and a jaw position read off a motor jitters by thousandths every cycle. The leader in MIRROR is a hand-held arm being squeezed, so its measured jaw never sits still. **One sample a hair above the block disarmed the whole mechanism**, the next cycle pushed again, and the stall recurred — which is exactly what the latch was built to end.

⭐ There is now a **3% margin** (`JAW_CLEAR_MARGIN`). A deliberate open clears it in one press, since the gripper step is 0.02 and the puck-button rate is faster than that. Jitter never will.

⛔⭐⭐ **WORSE THAN THE BUG: I WROTE A TEST ASSERTING THE WRONG BEHAVIOUR AND DEFENDED IT IN ITS OWN DOCSTRING.** `test_the_tiniest_opening_still_counts` said *"deliberately a strict inequality rather than a tolerance"*, reasoning that a tolerance would let a command a hair **below** the block through on every cycle.

> ⛔⭐⭐ **That was the wrong risk to weigh.** A hair below the block is **harmless** — the jaws are still not pushing at the block. A hair **above** it disarms everything. ⚠️ **A test can encode a mistake as confidently as code can, and a docstring arguing for it makes the mistake harder to see rather than easier.**

⚠️⭐ **AND I CANNOT YET PROVE WHICH FAILURE HIS LOG SHOWS.** Three deliberate squeezes onto an object and one latch cleared twice by noise produce **the same three lines**. ⭐ So the session now prints *"arm G jaws opened past the block at 0.117 — free to close again"* whenever a latch lets go. **A latch that silently comes and goes is indistinguishable from one that never worked**, and the next run will not leave that ambiguity.

### 63.2 ⭐⭐ THREE SETTINGS-SCREEN FIXES, ALL FROM WATCHING HIM USE IT ONCE

⭐ The screen worked on the first try. ⚠️ **Everything wrong with it was about what a person naturally presses**, which is the part no test predicts:

| what he did | what happened | now |
|---|---|---|
| ⭐ pressed **up and down arrows**, twice | `(key '\x1b[B' does nothing here)` — a raw escape sequence echoed at him | ✅ **arrows move the selection.** Unknown escapes are NAMED, never echoed: *"a UI that prints its own escape codes looks broken even when it is not"* |
| ⭐ pressed **`n`** inside the screen | told it does nothing | ✅ **`n` closes it**, the way `i` toggles mirror. Pressing the key that opened something is the obvious way to shut it |
| ⚠️ pressed about **13 keys** | **13 full fifteen-line screens** | ✅ **one line per change** (`▸ max_lag 0.250 → 0.312`), the full block only on `?` and after `0`. The rest of the session already worked this way |

⛔ **The reprint volume was the worst of the three, because it is not cosmetic.** He reads sessions back from the scrollback, which is how five of this week's defects were found. **Thirteen copies of a settings screen buries everything else the session did.**

⭐ **What DID work first time**, from the same log: `reach` raised live from 0.600 to 0.938 and back to 0.750, and the status row then read `reach 0.36/0.75m`, so a live safety-limit change reached the running loop exactly as intended.

### 63.3 ⚠️⭐ TWO PENDING PROMPTS CAN BE OPEN AT ONCE, AND NOTHING SAYS SO

⚠️ His log shows the gripper-button learning (`b`) and the SETTINGS screen (`n`) **both armed at the same time**:

```
⭐ LEARNING THE GRIPPER BUTTONS for arm B.
   Press the puck button you want for OPEN …
  ⭐ SETTINGS — the speed and safety limits, live.       ← n pressed while b was waiting
     ...
  ⚠️  that is already the other button — press the OTHER one   ← b consumed a puck press
```

⭐ **They coexist because they read different devices**: `b` waits on a PUCK button while SETTINGS reads the KEYBOARD, so neither blocks the other. ⚠️ **Nothing on screen said the button-learn was still waiting**, and a puck press he made for one purpose was consumed by the other.

⛔ **Not fixed, and it is [ROADMAP §8.2](ROADMAP.md) item 38.** It is genuinely harmless today — nothing moves and no setting is corrupted — but two invisible modal states is the shape that produces a "why did that do nothing?" session later. ⚠️ **The fix needs a decision about which should win**, and that is Julien's call rather than mine to guess.


## 64 ⭐⭐⭐ 2026-08-17, NIGHT — WHY THE MIRRORED ARM SITS IN A 2 cm SPHERE, AND WHY IT KEEPS STOPPING ON THE WRIST

### 64.0 ⭐⭐⭐ HIS QUESTION ANSWERED: THE FOLLOWER IS SHORT BECAUSE A POSITION-CONTROLLED ARM ALWAYS IS, AND NOTHING WAS READING IT BACK

⭐ **Julien, 2026-08-17:** *"I can move the mirrored robot about in a maybe two centimetre diameter sphere around the position it should actually be at… when I try to pick up something from the table, sometimes my guiding robot is already moving into the table whilst my mirror robot isn't even far enough down to pick up the object. Why is it not millimetre perfect? Is that a software problem or a limitation from the motors?"*

⭐⭐ **BOTH, AND THE SPLIT IS EXACT.**

⛔ **The motor half.** The follower is position-controlled: each motor pushes toward its commanded angle with a force proportional to how far away it is. So it settles where that force balances gravity and friction, which is **always short of the command**. That residual is the constant term in this repo's own measured law ([ROADMAP §8.2](ROADMAP.md) item 11): **0.04 to 0.10 rad of error even at zero speed**. Stiffness is a motor and gain property, and no command changes it.

⛔⭐⭐ **The software half, and it is the fixable one: NOTHING EVER NOTICED.** `mirror.follower_target()` copies the leader's measured angles, and the command converges to exactly that. So the follower ends up at `leader − droop` **forever**, and no part of the loop ever read the difference back. ⭐ **But the difference is measured every single cycle.** That is what makes it correctable above the SDK.

⭐⭐ **HIS 2 cm IS EXACTLY WHAT THE NUMBERS PREDICT.** His status row reported `tracking 0.012 rad behind` and `0.024 rad behind`. Measured on the shipped model at the extended pose his log shows him reaching with:

| joint error | worst tip displacement |
|---|---|
| 0.012 rad | **5.6 mm** |
| 0.024 rad | **11.3 mm** |

⭐ So a ±1 cm sphere, which is the 2 cm diameter he described. **His impression was a measurement.**

✅ **THE FIX: `--mirror-catchup`.** Accumulate the remaining error into a small bias and aim slightly past the leader until the follower actually arrives. That is integral action, and a standing offset under constant load is precisely what integral action is for.

⛔⭐⭐ **FOUR GUARDS, and each one prevents a specific way this could misbehave:**

| guard | what it prevents |
|---|---|
| ⭐ **`following` state only** | during ALIGNING the gap is large by design and the bias would wind to its clamp instantly |
| ⭐ **only while the leader is SLOW** (under 0.25 rad/s) | a moving leader's error is partly honest lag; integrating that would push the follower **past** the leader every time it stopped. ⭐ And lining a gripper up with something on the table is slow by nature, so it helps exactly when it matters |
| ⛔⛔ **a hard 0.06 rad clamp** | a BLOCKED follower never closes its error, so an unclamped integral grows forever and the arm **lurches** when the block clears. Same family as the stale cached variable that snapped this arm on 2026-08-10 |
| ⛔ **reset on every (re)engage** | carrying control state across an engagement is that same class of bug |

⭐ **A fifth guard came from a test.** The leader's speed estimate is smoothed, so at the *start* of a fast motion it still reads slow and a sliver of bias accumulates (measured: 0.0011 rad, about 0.5 mm). ⛔ **A frozen bias never gives that back**, so every slow-to-fast transition would add a little more and a long session could creep to the clamp for a reason nobody chose. **So a fast leader now DECAYS the bias rather than freezing it**, which makes the term self-correcting.

⛔ **DEFAULT OFF.** It changes what a 4.3 kg arm does, so it is opt-in: `--mirror-catchup 3`, or setting 7 on the `n` screen where it reaches a **running** mirror so he can watch the follower close onto the leader while holding it still.

⚠️ **I made a unit error writing its status line** and caught it before committing: it read `worst_bias() * 1000` and called the result "mm-equivalent", which is wrong because 0.024 rad is 11 mm rather than 24, and the radians-to-millimetres conversion depends on the joint and the pose. **A fabricated unit in a status row is worse than no figure**, because it reads as a measurement. It reports radians.

### 64.1 ⛔⭐⭐⭐ THE MIRROR GAP WAS ONE THRESHOLD FOR SIX VERY DIFFERENT JOINTS, AND THAT IS WHY HE KEPT RAISING IT

⭐ **Every mirror stop in his logs was on joint 5 (`wrist_roll`) or joint 6 (`gripper_twist`)** — the two joints that barely move the tip. Measured on the shipped model across **four poses taken from his own logs**:

| joint | | tip metres per radian | danger |
|---|---|---|---|
| 1 | `base_yaw` | 0.333 | ⛔ high |
| 2 | `shoulder_pitch` | 0.390 | ⛔ high |
| 3 | `elbow_pitch` | **0.418** | ⛔ **the worst** |
| 4 | `forearm_pitch` | 0.169 | ⚠️ half |
| 5 | `wrist_roll` | 0.100 | ⭐ a quarter |
| 6 | `gripper_twist` | **0.051** | ⭐ **an eighth** |

⛔⭐⭐ **THE GRIPPER TWIST IS 6.6× LESS DANGEROUS THAN THE ELBOW AND WAS HELD TO THE SAME NUMBER.** So the only way to tolerate a flicked wrist was to raise the threshold for the shoulder too. **He reached `--mirror-gap 1.335` doing exactly that, and at the elbow's 0.418 m/rad that allows 56 cm of tip error** — on a limit whose entire purpose is noticing that the arm has gone somewhere wrong.

✅ **The threshold is per joint now**, scaled as `1 / sensitivity` normalised to the worst joint and **capped at 4×**: `(1.26, 1.07, 1.00, 2.48, 4.00, 4.00)`.

⭐ **The effect on his actual stops:** joint 5 falling 0.364 rad behind now passes **at the default 0.35 gap**, and joint 6 falling 1.369 behind passes too. ⛔ **The same 0.364 rad on the elbow still stops it.**

⚠️⚠️ **THE CAP IS DELIBERATE AND THE MEASUREMENT DOES NOT JUSTIFY IT.** Pure tip-displacement scaling would give joint 6 about 6.6×. **Tip position is the right basis for a DANGER limit and the wrong basis for task accuracy**: 1.4 rad on the gripper twist is the gripper rotated **80° from where it should be**, which ruins a grasp while barely moving the tip. So the wrist gets more rope than the shoulder, and less than the arithmetic alone would allow.

⭐ **The stop message names the joint closest to its OWN limit** rather than the one with the largest raw gap. Those differ once the thresholds differ, and naming the wrong joint sends him after the wrong flag. It also prints the multiplier, so `its limit is 1.40 = 0.35 × 4.00` can be reconciled with what `--mirror-gap` says.

### 64.2 ⛔⭐⭐ THE SETTINGS KEYS PRODUCED NUMBERS NOBODY WOULD CHOOSE

⛔ Multiplying by 1.25 per press was defensible reasoning (0.1 → 0.125 and 8 → 10 are the same *felt* step) and the result was unusable. Holding `+` gave him:

```
1.000 → 1.250 → 1.562 → 1.953 → 2.441 → 3.052 → 3.815 → 4.768 → 5.960 → 7.451 → 9.313
```

⛔ **He can never get back to 2, or 4, or 10.** Every value he had been running by flag was unreachable by key. ⚠️ And those numbers **leaked into messages**: `limit 1.33514404296875`, `its 1.4901161193847656 rad following-error limit`, `--max-speed 16 (now 7.45058)`.

✅ **The keys walk a ladder of round numbers now** — `1, 1.5, 2, 3, 4, 6, 8, 10, 15, 20` for the speeds, with its own ladder per setting. A value set by flag that sits between rungs moves to the nearest rung in the direction pressed, so nothing is ever stuck.

⭐ **The seventeen-digit floats were a symptom rather than a second bug**, and every limit printed to a person is formatted now as well, because a flag can still set any value.

### 64.3 ✅ WHAT HIS LOG SHOWS THAT IS ALREADY FIXED

⚠️ **His session predates two commits**, so a fresh reader should not chase these:

1. ⚠️ **The settings screen reprinted all fifteen lines on every keypress** — thirteen copies in one session. ✅ Fixed in the same session as [§63.2](FINDINGS.md): one line per change, the full screen only on `?` and after `0`.
2. ⚠️ **Arrow keys printed `(key '\x1b[B' does nothing here)`**. ✅ They move the selection now.

⭐ **What worked first time in his log and is worth recording as confirmed:** `reach` raised live from 0.600 to 0.938 and back to 0.750, with the status row then reading `reach 0.36/0.75m`. **A live safety-limit change reached the running loop exactly as intended.** And the new linear-speed ceiling held at `linear speed 2.000 m/s (ceiling)` instead of the 19.852 m/s it reached before ([§62.2](FINDINGS.md)).

⚠️⭐ **He also ended that session with `max_speed 20`, `teleop_speed 20` and `max_lag 3.0`, all at their ceilings.** So the "deliberately generous" backstops were all reached inside one session of pressing `+`. ⛔ **They are doing their job as a backstop against a held key and they are NOT out of reach**, which is worth knowing before treating any of them as a safety margin.


## 65 ⭐⭐⭐ 2026-08-17, LATE NIGHT — THE FOUR SPEED LIMITS EXPLAINED, AND TWO OF MY OWN CHANGES WERE WRONG

### 65.0 ⭐⭐⭐ THE CANONICAL ANSWER: FOUR LIMITS IN SERIES, AND THE SMALLEST ONE BINDS

⭐ **Julien asked what `max_lag` actually does**, and this table is the answer to that and to every future version of the same question. **They are four different quantities in three different units**, which is why "raise the speed" has repeatedly failed to do anything.

| flag | units | what it bounds | where it acts |
|---|---|---|---|
| `--linear-scale` | **m/s** | how fast a full puck push asks the **TIP** to move | before the IK |
| `--teleop-speed` | **rad/s** | how far the IK's answer may move any **ONE JOINT** per cycle | after the IK |
| `--max-speed` | **rad/s** | the same per-joint cap again, **below all control logic** | inside `SafeRobot` |
| `--max-lag` | **rad** | ⭐ how far the **COMMAND** may run ahead of where the arm actually **IS** | inside `SafeRobot` |

⭐⭐ **`max_lag` IS NOT A SPEED, AND THAT IS THE WHOLE CONFUSION.** Every cycle, `SafeRobot` pulls the command back to `measured ± max_lag`. So:

> ⭐ **Reaching a far target becomes a RATCHET.** Ask for 1.01 rad while the arm sits at 0.12 with `max_lag` 0.25, and the command sent is **0.37**. The arm moves toward it, reaches say 0.20, and the command becomes 0.45. It walks forward at whatever speed the arm can actually manage.
>
> ⛔ **A BLOCKED joint therefore never arrives, and that is the point.** The command can never be more than 0.25 rad past a joint that will not move, which bounds how hard the motor pushes. It is a torque limit expressed as a distance.

⚠️ **Which is why raising `max_lag` did nothing for his mirror stops** ([§62.0](FINDINGS.md)): those stops are triggered by `--mirror-gap`, a different limit in a different file.

### 65.1 ⛔⭐⭐⭐ MY 2.0 m/s LINEAR CEILING WAS A REGRESSION, AND HE CAUGHT IT

⭐ **Julien, 2026-08-17:** *"the linear limit is at two, the ceiling, and then the max speed was set way higher, but that didn't make any difference to the teleop speed. I was limited by the two before. And before, I was able to go much faster."*

⛔ **He is exactly right and the arithmetic is simple.** At 2.0 m/s of tip demand with a 0.4 m lever, the joints only need about **5 rad/s**. So `--teleop-speed 15` and `--max-speed 20` were never reached and raising them could not do anything. **The cartesian limit was the binding one, and I had capped it at a sixth of what he had been using** — he reached 19.852 m/s before my cap existed ([§62.2](FINDINGS.md)).

⚠️⭐ **THE LESSON ABOUT MY OWN REASONING.** I justified 2.0 as *"~17x the default and far past anything useful for hand-scale work, so it can never get in his way"*. ⛔ **I compared it to the DEFAULT instead of to what he had actually been running**, which was in the very log I was reading. **A backstop sized against the default value rather than against observed use is a limit dressed as a backstop.**

✅ Raised to **15.0 m/s**, chosen so it can never bind: `teleop_speed`'s own ceiling is 20 rad/s, which at a 0.6 m lever is about **12 m/s** of tip speed. So it stops a held key running away and stops being a speed limit.

⭐ **And `linear_scale` is setting 8 on the `n` screen now**, because he asked why it was not there: *"if it's limited in the linear normal mode, then why would it not allow me to change it in the control panel area?"* **It is one of the four limits in series, and leaving it off that screen made it the invisible one.** Both the screen and the main-loop `-`/`+` keys now go through one function, so the two paths cannot step differently.

### 65.2 ⛔⭐⭐⭐ THE PER-JOINT GAP SCALING GAVE A **STALLED** JOINT 2.48× LONGER TO BE PUSHED

⛔ **A safety consequence of yesterday's change that I did not think through.** His log:

```
⛔ MIRROR STOPPED — the follower fell 0.869 rad behind on joint 4
   (its limit is 0.87 = 0.35 × 2.48), forearm_pitch
     the follower barely moved that joint (0.01 rad/s), so it is blocked…
     ⚠️ SafeRobot held the command back on 1115 cycle(s)
```

⭐ **1115 clipped cycles is about twelve seconds** of a motor pushing at its full `max_lag` allowance against a joint moving **0.01 rad/s**. The session's hottest readings were **45 and 46 °C**, the highest this project has recorded.

⛔⭐⭐ **THE SCALING IS RIGHT FOR LAGGING AND WRONG FOR STALLED, AND THOSE ARE OPPOSITE CASES.** Multiplying joint 4's tolerance by 2.48 is correct when it is merely behind. It also gave a joint that was **not moving at all** 2.48× longer before anything stopped it. ⭐ **The code already told the two apart** by the follower's measured speed, which is what made the fix small: **lagging keeps the generous scaled limit, stalled stops on the tight unscaled one.**

⚠️ **Tolerating lag was never meant to mean tolerating a stall**, and the message now says which limit fired, because *"0.42 behind, limit 0.87"* reads as a bug.

⚠️⭐ **A SIDE EFFECT: `--mirror-catchup` NEVER GOT A FAIR TEST.** That session had a genuinely stuck joint 4 the whole time, so the correction had no chance to show itself. ⬜ **It is still unproven** ([ROADMAP §8.2](ROADMAP.md) item 39).

### 65.3 ⛔⭐⭐ THE REPEATED GRIPPER MESSAGE WAS THE **DETECTOR**, AND THE LATCH WAS WORKING ALL ALONG

⭐ His log settles it, and the direction of the numbers is the proof:

```
ARM B GRIPPER STALLED — released to 0.304, 0.311, 0.314, 0.315, 0.315, 0.315, 0.315, 0.316
  ⭐ arm B jaws opened past the block at 0.316 — free to close again.
```

⭐⭐ **Eight messages, every value creeping OPEN by a thousandth.** Arm G did the same four times. **The latch was doing its job throughout.**

⛔ **The detector fires on high torque with no movement, and holding the jaws at the block is exactly that.** A position controller sitting at its target against an object still produces torque and still is not moving. So the condition stayed true for as long as the grip was held, and every cycle re-reported a stall that had already been handled.

⛔⭐ **AND RE-LATCHING WAS INDEPENDENTLY WRONG.** Each detection moved the block to the new measured position, so **a jaw slowly relaxing dragged the block open with it** and the protection loosened by itself. That is the mechanism behind the creeping numbers.

✅ **While a block is latched, the detector now says nothing and does nothing.** The latch is the response; re-detecting it is noise that also weakens it.

⭐ **Three passes were needed to get this right, and each found a different layer:** the release was undone every cycle ([§62.3](FINDINGS.md)), then the latch was cleared by sensor noise ([§63.1](FINDINGS.md)), and now the detector was re-firing against a working latch. ⚠️ **All three looked identical from the log** — a repeating stall message — which is why each one needed his numbers rather than reasoning.

### 65.4 ⚠️ TWO SMALLER THINGS HIS LOG SHOWS

1. ⚠️ **He pressed `-`/`+` on `mirror_catchup` about 33 times, hunting.** 3 → 5 → 8 → 12 → 8 → 5 → 3 → 2 → 1 → 0 → 1 → … up to the ceiling and back down twice. ⛔ **The `n` screen covers the status row, so the one thing that would tell him whether it was working was hidden while he tuned it.** ⬜ Not fixed; it is [ROADMAP §8.2](ROADMAP.md) item 43.
2. ⚠️ **`i` inside the SETTINGS screen says it does nothing.** He pressed it wanting to re-engage the mirror. ⭐ Reasonable as written, and worth reconsidering alongside item 43.


## 66 ⭐⭐⭐ 2026-08-17, FINAL SESSION OF THE DAY — THE PARK SPASM, THE LATENCY ANSWER, AND A FALSE ALARM IN THE EXIT SUMMARY

### 66.0 ✅⛔⭐⭐⭐ THE PARK SPASM: `park_arms` NEVER RESYNCED `SafeRobot`, SO THE FIRST COMMAND JERKED TOWARD THE ARM'S OLD POSE

⭐ **Julien:** *"in quit mode I put the arms in guide mode and moved an arm a bit further. Then I wanted to park it with p, and it quickly spasmed for, like, a tenth of a second, for seemingly no reason."*

⛔⭐⭐ **THE REASON, and it is the 2026-08-10 snap in miniature.** `SafeRobot` is stateful: its rate limiter walks from `_last_cmd`, the last position it ever sent. GUIDE commands **no positions** (kp = 0), so after hand-guiding, `_last_cmd` still holds the pose from BEFORE his hand moved the arm. The first park command was then pulled toward that stale pose and clipped to `measured ± max_lag` — **a jerk of up to 0.25 rad toward where the arm USED to be**, lasting the few cycles `_last_cmd` needs to converge. A tenth of a second, exactly as he described.

⭐ **`SafeRobot.resync()` exists precisely for this** and its docstring says *"call this on EVERY mode transition."* Every in-session transition does. ⛔ **`park_arms` is a mode transition that lives OUTSIDE the mode system** — it runs from the quit menu and from Ctrl-C — which is why it was the one place that missed it. Same root shape as the two park implementations diverging ([ROADMAP §8.2](ROADMAP.md) item 23): motion code outside the mode system does not inherit the mode system's hygiene.

✅ Fixed: `park_arms` resyncs each arm before its first command, with a test pinning that the resync comes **before** command zero. ⬜ Unconfirmed on hardware; the reproduction is exactly his sequence (`q` · `g` · move an arm by hand · `p`).

### 66.1 ⭐⭐⭐ THE LATENCY ANSWER: THE SOFTWARE ADDS ~20 ms, THE PHYSICS ADDS THE REST, AND "NEAR-ZERO AT MAX SPEED" NEEDS A DIFFERENT CONTROL MODE

⭐ **His question:** *"the signal is a hundred hertz, so why is it lagging behind so badly at high speeds? Shouldn't it be instant, with incredibly low latency? Is that a limit of how slow our system is? Is there a way to have near-zero latency at max speeds, or is that not possible?"*

⭐⭐ **THE CHAIN, WITH MEASURED NUMBERS.** From his hand to the arm's motion:

| stage | adds | measured or known |
|---|---|---|
| SpaceMouse USB report | ≤ ~8 ms | HID report rate |
| our loop picks it up | ≤ ~11 ms | loop measured at **89-90 Hz** |
| IK solve + command | < 1 ms | inside the same cycle |
| I2RT's 250 Hz control thread | ≤ ~4 ms | its own rate |
| CAN frame to the motor | ~1 ms | 1 Mbit/s bus |
| **software total** | **~15-25 ms** | ⭐ **this is NOT what he is feeling** |
| ⛔ **the arm physically tracking** | **~33 ms × speed, PLUS 0.04-0.10 rad droop** | [ROADMAP §8.2](ROADMAP.md) item 11, fitted then verified on held-out data |

⛔⭐⭐ **THE DOMINANT LAG IS PHYSICS, AND IT IS STRUCTURAL TO POSITION CONTROL.** The motors run a PD position controller: torque is produced **in proportion to position error**. An arm that is moving therefore MUST be lagging — no error, no torque, no motion. That is where the measured `0.033 s × speed` comes from, and no flag in this repo touches it. ⚠️ On top of it, `max_lag` caps the command at 0.25 rad ahead, so at high demanded speed the command rides that clip and the arm moves at whatever its torque can deliver.

⭐ **MIRROR adds one more loop hop** (~11 ms: the leader's measured pose becomes the follower's command next cycle) **plus a second copy of the physical lag**, because the follower is itself a PD-controlled arm. So leader-to-follower is roughly: leader physics + 11 ms + follower physics. At 2 rad/s that is ~0.07 rad per arm of speed-proportional lag alone, which matches every FOLLOWING reading he has seen.

⭐⭐ **CAN IT BE MADE NEAR-INSTANT? Partly, and the honest ranking is:**

1. ⭐⭐ **Velocity feedforward — the real one.** The DM motors' MIT-mode command carries `(kp, kd, position, velocity, torque)` and we send **velocity = 0** today, so the motor must generate all its torque from position error. Sending the target's velocity too lets torque flow **before** error builds. This attacks the 33 ms term itself. ⬜ [ROADMAP §8.2](ROADMAP.md) item 44 — needs I2RT-interface work and careful bring-up.
2. ⚠️ **Higher kp (stiffness)** — shrinks both the droop and the speed lag, risks oscillation, and item 17's original premise was refuted. Hardware experiment, his call.
3. ⭐ **Loop rate 90 → 200 Hz** — saves ~5 ms of the ~20 software ms. Real but small ([ROADMAP §8.2](ROADMAP.md) item 14 is the open "why 90 not 100" question).
4. ⛔ **What cannot work:** any amount of raising `--max-speed`/`--teleop-speed` past where the physics binds. The four-limit table ([§65.0](FINDINGS.md)) says which limit binds; once it is the arm itself, flags are exhausted.

⚠️ **Set the expectation honestly: a 4.3 kg arm with finite torque can never track a hand with literally zero lag.** What is achievable with feedforward and tuning is lag that stops being *felt* — tens of milliseconds and a droop too small to see — which is also what the teleoperation rigs this project imitates settle for.

### 66.2 ⚠️⭐ THE SCARY EXIT LINE WAS A FALSE ALARM, AND THE COMPARISON IT PRINTS IS APPLES TO ORANGES

⚠️ His session ended with `axis map G: DOWN←z+ LEFT←x− FWD←y− PAN←yaw+ TILT←roll− ROLL←pitch−` / `was: X←y+ Y←x+ …`, which reads as G's map having been scrambled and saved.

✅ **Verified safe on disk, two ways.** The saved world map is byte-identical to what the session started with, and a fresh dry run loads **identical shared world maps for both arms**. ⛔ Nothing needs restoring.

⭐ **What the line actually was:** G was left in the **camera** frame, so the exit summary printed G's camera-frame row (`DOWN/LEFT/FWD/PAN/TILT` are the camera frame's motion names) while its `was:` line prints the start-of-session copy **in world labels**. **Different frames of the same store, shown as before/after.** The `.prev` diff confirms the only world change on disk predates this session. ⬜ The summary comparing across frames is [ROADMAP §8.2](ROADMAP.md) item 45 — cosmetic, but it cost real verification time tonight and will cost it again.

### 66.3 ⭐ WHERE THE WHOLE PROJECT STANDS — the completion list, in one place

✅ **Confirmed working on hardware:** two arms from one script · GUIDE/TELEOP/HOLD per arm · saved poses and sequences · two-arm recording AND playback · MIRROR at speed (`--mirror-gap 2`, 0.012 rad tracking) · the gripper stall latch · live settings (`n`) with save · the four-limit chain explained in the plan · `q q` shutdown parking both arms and disabling all 14 motors.

⬜⭐ **Built and still needing ONE hardware confirmation each:** `--mirror-catchup` (never had a fair test, [§65.2](FINDINGS.md)) · the park-spasm resync ([§66.0](FINDINGS.md)) · the stalled-joint tight limit · the latched-gripper silence.

⬜⭐⭐ **The real remaining work, in rough order of value:**
1. **Cameras**: identify two D405s (needs the second one attached, wiggle method) · timestamped multi-camera capture aligned with joint data ([ROADMAP §8.2](ROADMAP.md) items 5, 6). ⛔ Agent writes, Julien runs — permanent ([§61.3](FINDINGS.md)).
2. **The MCAP export** in ABC's schema, deferred pending his friend's spec (item 7). Our recordings are already the right shape.
3. **Velocity feedforward** for real latency (item 44, new).
4. **Labels while driving, noise per waypoint, mixed runs** (items 8, 9, 12) — the data-collection features.
5. **Finer collision geometry** before any proximity warning (item 35).
6. Housekeeping: `chain_alive` in incidents (31), the loop-rate question (14), the two-prompt overlap (38), the settings-screen visibility (43), the exit-summary frames (45).

### 66.4 ⭐⭐ THE TEAM-HANDOVER PLAN LIVES IN [ROADMAP §10](ROADMAP.md)

⭐ His ask: *"we need to fully clean up the repo and have a clear plan of what the repo should look like when my team wants to recreate it."* The plan is written as a proposal for his ratification, **not executed** — a restructure is exactly the kind of consequential, hard-to-reverse change that gets ratified first, and doing it at the end of a nearly-full context would be doing it carelessly.

## 67 ⭐⭐⭐ 2026-08-18 — HIS RULING RESCOPES THE PROJECT: A FINISHED WALKTHROUGH FOR A FROM-SCRATCH REBUILD. PLUS THREE BENCH RESULTS

### 67.0 ⭐⭐⭐ THE RULING, and it filters every open list in this repo

His words, 2026-08-18: *"whatever we're doing here is only supposed to be a finished walkthrough of where this should go, and then my team and I will rebuild everything from scratch. But for that to work, we need a fully complete and finished version of the interface… I don't need to have anything measured out because it's not finished yet… make sure that they're all only tasks we need to do to finish all of the features and finish the whole setup so that we can then consolidate everything and build a full plan of how to build this kind of thing, what is necessary, all of the problems to run into."*

⭐⭐ **What that means, concretely:**

1. **This repo is the reference implementation.** Its job is to prove every feature once and to carry the findings. The team rebuilds from scratch against the consolidation plan, and **the plan is the deliverable** ([ROADMAP §8.5](ROADMAP.md), [§10.4](ROADMAP.md)).
2. **In scope:** everything needed to finish the features and the setup — the camera chain (items 5–8), the jaw-pause completion (items 3, 10), the code-truthfulness fixes (items 23, 21, 28, 31, 43, 45), and the consolidation plan itself.
3. **Out of scope, by this ruling:** anything that measures or hardens THIS physical bench as if it were final — the desk height (item 22), the scaling-limit verification (item 18), the powered hub as a *task* (item 20 becomes a plan note: a problem the rebuild team must expect), and finer collision geometry (item 35 — his standing ruling is manual avoidance, so the proximity warning it enables is not a feature of this rig; the rebuild plan carries it instead).
4. ⭐ [ROADMAP §8.4](ROADMAP.md)'s earlier ruling said the same thing from the hardware side (*"we will still move everything around… some type of guiding system that we can then work towards when we reimplement the whole thing from scratch"*). **This ruling extends that filter from the hardware measurements to the whole task list.**

⚠️ [ROADMAP §8.2](ROADMAP.md) stays the single tracked list. Out-of-scope rows are **marked, never deleted** — "we decided not to, and why" is exactly the kind of information the plan must carry.

⭐ **On cameras, his operating rule for the walkthrough:** *"try everything you can do with one"* D405, and if something is *"unnecessarily hard work to do with one that you could just do with two much easier"*, say so and he gets the second one. The C920 is available as the scene camera on request. So: build the capture chain against one D405 + the C920; item 5 (telling two identical D405s apart) genuinely needs the second D405 attached and waits for it.

### 67.1 ✅⏸ MIRROR-CATCHUP RAN, AND HIS VERDICT IS THAT THE FEATURE IS WEAK — closes [§65.2](FINDINGS.md)'s wait, item 39

His report, 2026-08-18: the higher the catchup number, the more jittery the follower. At 20 it *"starts to move into directions and then it starts to shake"*. At low values (*"two or something"*) the follower is *"still kind of just moving around slowly"*. His summary: *"the feature isn't that great… it doesn't really work that well as far as I understand."*

⭐ **Both behaviours are what the mechanism predicts, so the feature is working as designed and the design is the limit.** The catchup term is an integrator: it accumulates the leader-minus-follower gap into a bias added to the command. At high gain the bias overshoots, the overshoot becomes new gap with the opposite sign, and the loop shakes. At low gain it slowly integrates droop AND sensor noise, which reads as aimless drift. An integrator aimed at a moving, noisy target does both of these things.

⛔ **The real fix for the droop it was built to cancel is velocity feedforward** ([§66.1](FINDINGS.md), [ROADMAP §8.2](ROADMAP.md) item 44): attack the cause (a PD position controller makes no torque without error) rather than integrate the symptom.

✅ **His ruling on the ceiling, same report: the 20 stays.** *"It could stay that way"* — it is a backstop, not a working value. Item 42 is answered by the same words: no ceiling is lowered.

**Item 39 closes as: ran, works poorly, default stays OFF, superseded by item 44 as the real path.**

### 67.2 ✅ THE JAW BLOCK IS CONFIRMED ON HARDWARE — the stall fires once

His report, 2026-08-18: mirror with an object in the follower's grip *"worked out fine"*, and *"the block only shows once. That should be fine."* That is [§63.1](FINDINGS.md)'s fix (the 3% clearing margin) doing its job on the arm, and it also confirms the message rationing from [§58.2](FINDINGS.md) in the same run. **Item 29's hardware confirmation is done.**

⚠️ One phrase in his report could not be parsed from the transcript — *"it's just that the camp would like to go slower"* — recorded here verbatim so it is not lost. Asked in chat; if it was a real observation (the arm should move slower while gripping?), it gets its own row.

### 67.3 ⚠️ THE RIG AS READ ON 2026-08-18 — a dated reading; run `check_rig.py`

Both CAN adapters on the bus running firmware, no DFU. Two SpaceMice. ⛔ **ONE camera: the D405 `255323071773`. No C920 and no second D405 on the bus at reading time** — although he reports having plugged "the other cameras" in at some earlier point, so the bench has changed again since whatever session that was. The capture work (item 6) needs the C920 replugged; item 5 additionally needs the second D405.

### 67.4 ⬜ WHAT REMAINS OWED AT THE BENCH, complete as of 2026-08-18

1. ⬜ **The park-spasm resync, 30 seconds** ([§66.0](FINDINGS.md)): start a session, `q` · `g` · move an arm by hand · `p`. Before the fix the arm jerked for ~0.1 s toward its pre-guide pose; now the park should start smoothly from where the arm actually is.
2. ⬜ **Replug the C920** (and the second D405 whenever convenient) so the camera chain can be built and run. ⛔ Camera *commands* are permanently his to run — macOS grants camera access per parent app and an agent shell can never have it ([§61.3](FINDINGS.md)).
3. ⬜ **Velocity feedforward, gently** ([§67.9](FINDINGS.md), added later on 2026-08-18): a normal session with `--vel-ff 0.25`, drive TELEOP slowly, watch for buzz or overshoot (`n` · `9` · `-` backs it off live). If it feels right, raise it and watch the FOLLOWING numbers shrink.

Everything else on the owed list is now confirmed: catchup ([§67.1](FINDINGS.md)), the jaw block ([§67.2](FINDINGS.md)), the two-arm playback ([§60.0](FINDINGS.md)).

### 67.5 ⛔⭐⭐ TWO TEST FILES WERE SITTING RED AND NOBODY COULD SEE IT, because there is no single runner — and one of them was a REAL regression

⛔ **Found 2026-08-18 by running all 26 test files in one loop, which nothing does routinely.** Each `scripts/test_*.py` carries its own `main()`; `pytest` is not even installed in this environment. So "the tests pass" has only ever meant "the files someone happened to run passed" — and two files were failing:

1. ⚠️ **`test_incident.py` 17/18 — a stale source-pin, red for days.** It asserted the literal string `chain_alive(robot) and (interrupted or unplanned)`; the N-arm rewrite changed the gate to `live = [one for one in arms if one.alive()]` days ago. The *safety property survived* (the park is still gated on liveness, now per-arm, which is better); only the pin went stale. ✅ Updated to pin the new shape.
2. ⛔⭐⭐ **`test_settings.py` 35/38 — a REAL regression in the ladder rework ([§64.2](FINDINGS.md)), and the tests CAUGHT it, and it shipped anyway.** `adjust()`'s fallback snapped a value with no rung left in the pressed direction to the FAR end of the ladder. A value outside the ladder's ends is legal (flags and saved defaults): **`+` on `floor` at 0.5 DROPPED it to 0.1, and `-` on `max_speed` at 0.1 RAISED it to 0.25** — a press moving a safety limit the way the operator did not ask, on a rig with no e-stop. ✅ Fixed: no rung left in the pressed direction → the value stays exactly where it is, and `at_bound` already tells the operator why. Two new tests pin it, plus one pinning that his flag values (1, 1.5, 2, 4, 10) stay on the ladder.

⭐⭐ **The transferable lesson sharpens [§59.1](FINDINGS.md)'s:** there, a checker was silently disarmed and only a falsification *count* showed it. Here, working tests fired and **nothing collected the shot** — a suite that is never run as a whole is a fixture with no counter. **[ROADMAP §10.5](ROADMAP.md) step 2 (one runner over all test files) stops being a tidiness item and becomes defect-backed.** ✅ **Built the next session: `uv run checks/run_tests.py`** ([§70.4](FINDINGS.md)) — the old one-loop sweep (`for f in tests/test_*.py; do uv run "$f" | tail -1; done`) is superseded.

### 67.6 ✅ ITEM 31 IS BUILT: LIVENESS IS CAPTURED BEFORE THE MOTORS ARE DISABLED

✅ The teardown now captures each arm's `alive()` into `alive_at_teardown` **before** the `shutdown_robot()` loop, and the incident file records it as **`chain_alive_at_teardown`**. ⭐ The key is *renamed on purpose*: every old incident file's `chain_alive` was the meaningless post-shutdown read ([§58.45](FINDINGS.md)), and a reader must be able to tell the two apart by name alone. A source-ordering test pins capture-before-shutdown and forbids the old read from coming back. ⚠️ Like everything on the teardown path, its first real execution will be during a failure; the sim drive (25/25) exercises the surrounding path.

### 67.7 ⭐⭐⭐ HIS SECOND 2026-08-18 MESSAGE: THE RESTRUCTURE IS RATIFIED, EVERYTHING GETS BUILT IN THE PROTOTYPE, AND MARIUS'S MISSING FILE ARRIVED

⭐⭐ **Ruling 1 — the restructure HAPPENS, overruling the agent's skip-recommendation.** His words: *"I think the file organization might be important because we want to have a really smartly built repo… it would be great if your code is already that, and we can actually use and look at it, because it's already well organized."* So [ROADMAP §10](ROADMAP.md) is ratified as work, not just as a description. ⭐ **And it has a new input:** *"look at some of the other branches or commits from some of my colleagues… explore those branches… find the understanding of how our stuff looks and what our current plans are, so you can integrate those plans into the plans that you do yourself. You don't have to do everything as we're doing it. You can come up with more intelligent ideas."* The colleagues' work lives in the public `Hohnik/LaRobot` repo (where `julien/yam-teleop-wip` was pushed once). ⬜ Explore it before executing §10.

⭐⭐ **Ruling 2 — the general principle, given via the speed-dial example:** *"I would build everything in this prototype so that we know how to do it, and then we can explain how to do it in the rebuild plan."* **So when a feature hesitates between "build here" and "plan-note", the default is BUILD.** Item 13 (the second SpaceMouse as a speed dial) moves into scope — his note: it *"should be quite easy… should be made intelligently in some way."* ⚠️ This principle does NOT reverse his explicit earlier rulings (collision stays manual avoidance, nothing measured on this bench).

⭐ **Ruling 3 — velocity feedforward (item 44) is a GO**: *"try to build everything else relevant, especially regarding the motor speed."* Build it off by default; he tests slowly.

⭐ **Ruling 4 — the two-prompt collision (item 38) is the agent's to settle sensibly** (*"I don't know what you mean by the puck prompt, but just do that sensibly as well"*). Decision: the newest prompt cancels the older one, with a printed line saying so.

✅ **Marius's `Setup-Anleitung.md` arrived and item 32 is CLOSED.** The file [docs/Setup-Plan.md](Setup-Plan.md) line 284 has pointed at since the beginning existed on Marius's side and never entered the repo. Julien sent it 2026-08-18 via WhatsApp with: *"an original file regarding how we might wanna go about the setup… not particularly important, and we didn't follow loads of the steps… just so you know what was planned in the beginning."* Marius's own caveat (2026-08-07): *"das ist zeug was ich vorbereitet hatte… hat bestimmt noch überarbeitungsbedarf an stellen wie z.B. ZMQ o.ä."* ✅ Copied in **byte-identical** (`cmp`-verified) as [docs/Setup-Anleitung.md](Setup-Anleitung.md); the repo is at **991/991 links resolving** for the first time. ⭐ **Why it matters more than "not particularly important" suggests:** it is the team's original plan written down — phases with gates, the exact MCAP topic schema and encoding contract, the two-branch training strategy (small/local diagnosis vs large/external SFT), and a 12-mistakes table. **The consolidation plan ([ROADMAP §8.5](ROADMAP.md)) should be written against it: what was planned, what this repo actually did, and why the deviations happened.** ⚠️ Treat as a historical document: v2, Stand 2026-08-06, partly not followed (e.g. it assumes Ubuntu/SocketCAN; this repo drives CAN from macOS via `gs_usb`).

### 67.8 ⛔⭐⭐ JULIEN SPEAKS HIS MESSAGES, AND THE TEXT ARRIVES THROUGH A MEDIOCRE SPEECH-TO-TEXT MODEL — a working-contract fact

His explanation, 2026-08-18, after being asked about an unparseable phrase: *"I'm currently using the voice command input feature… my vocal speaking gets first transferred to text by probably some mediocre speech to text model, and then you get whatever the speech to text model understood."* ⛔ **He does not see the transcription himself** (*"I don't even know what that sentence is supposed to mean"*).

⭐⭐ **The rule he asked for:** whenever a phrase reads garbled, incoherent, or has weird words in weird places, **quote it back with its surrounding context and ask** — he can usually reconstruct what he meant from where it happened. Never silently guess a reading and act on it; a mis-heard word about a safety limit or an arm name is exactly how a wrong instruction becomes motion. Now working-contract rule 12 in [HANDOFF §4](HANDOFF.md).

### 67.9 ✅⭐⭐⭐ VELOCITY FEEDFORWARD IS BUILT — item 44, his "especially" of 2026-08-18. UNRUN ON HARDWARE

✅ **What exists now:** `SafeRobot.vel_ff` (0.0 = OFF = exactly the old behaviour), the `--vel-ff` flag, **setting 9 on the `n` screen** (ladder 0 · 0.25 · 0.5 · 0.75 · 1.0, savable), and a plan line naming it whenever it is on. At `vel_ff > 0` each command goes out via I2RT's `command_joint_state` carrying `vel_ff ×` **the rate-limited command's own derivative** as the velocity setpoint. [COMMANDS.md](COMMANDS.md) has the operator's version.

⭐⭐ **The three design decisions, each load-bearing:**
1. **The setpoint is the derivative of the LIMITED command, never the caller's raw target** — so `|vel| ≤ max_speed × vel_ff` *by construction*, and the feedforward can never ask for a speed the rate limiter just refused. No new safety surface.
2. ⛔ **The jaw (index 6) never gets feedforward.** On a jaw squeezing an object the extra torque pushes harder into it, which is how motor 7 was cooked three times.
3. **A wrapped robot without `command_joint_state` falls back to position-only and warns exactly once** — a set gain that silently does nothing is the fails-by-lying pattern ([§0](FINDINGS.md)).

⭐ **No vendor patching:** I2RT's `MotorChainRobot.command_joint_state({"pos", "vel"})` existed all along, one function below the `command_joint_pos` this stack always called; the velocity remap in its `JointMapper` is linear, so a zero stays a zero through the jaw's normalised map. **`FakeArm` grew `command_joint_state` too: it RECORDS the setpoints (so tests assert the plumbing) and deliberately does not model the tracking benefit** — that constant must come from the arm, not from imagination ([§33.3](FINDINGS.md)).

✅ **Verified:** `scripts/test_vel_ff.py`, 8 tests (off-is-identical · exact first-cycle setpoint · jaw exclusion · limiter bound over 50 runaway cycles · gain clamp above 1 · fallback warns once · the setting's ladder reaches both ends · positions identical with ff on/off). Full sweep 694/694 across 27 files, and `drive_sim_session.py --vel-ff 0.5` runs the whole loop **25/25 with feedforward on** (the driver now passes extra flags through to the session). ⭐ Mirror benefits twice, since leader-to-follower stacks two copies of the physical lag ([§66.1](FINDINGS.md)).

⬜ **The hardware run it owes, low speed first:** `--vel-ff 0.25`, drive TELEOP gently, watch for buzz or overshoot (`n` · `9` · `-` backs it off live), then raise toward 1.0 and compare the FOLLOWING readings against the `0.033 s × speed` law — the speed-proportional term should visibly shrink while the 0.04-0.10 rad droop stays. ✅ **Ran the same evening — [§67.10](FINDINGS.md).**

### 67.10 ✅⭐⭐⭐ HIS EVENING BENCH SESSION, 2026-08-18: THE PARK SPASM IS CONFIRMED FIXED, FEEDFORWARD RAN AND "FEELS DIFFERENT", AND HE WANTS IT EXAGGERATED

✅⭐ **The park-spasm fix is CONFIRMED ON HARDWARE.** His words: *"I also did the park test, and that worked smoothly."* His pasted log shows the exact reproduction from [§66.0](FINDINGS.md): quit menu → `g` (both arms weightless, parked by hand) → `p` — and both arms parked cleanly, twice, 0.010-0.020 rad off. **That was the last unconfirmed motion fix.**

⭐⭐ **Feedforward ran on both arms at 0.25, TELEOP, 161 s, and his first verdict is real-but-unclear:** *"The feedforward feels different. I don't know if it feels great yet… it kind of seems like the controls sometimes are not moving together, but maybe that's just because now they're actually more direct, and so they're less smooth. If so, then that's great."*

⭐ **A mechanism that fits his "not moving together", recorded as a hypothesis and NOT as fact:** with feedforward each joint's lag shrinks in proportion to its own commanded speed, so joints on different speed profiles stop sharing the uniform lag that used to smooth a blended motion. If that is what he felt, it is the feature working. ⚠️ Unconfirmed; the tracking data of a with/without comparison at the same task would settle it.

✅⭐⭐ **His ask, implemented the same evening: *"exaggerate the numbers so that I can actually see what's happening… increase the numbers so that I can have a higher max value."*** The ceiling rose 1.0 → **3.0** (`yam_robot.VEL_FF_CEILING`, synced to the editor's bounds by a test): **1.0 stays the physically-motivated rung** (exactly the command's own speed), everything above is **labelled exaggeration** — the motor is told the target moves faster than it does, so overshoot is expected, and the plan line says so whenever the gain is above 1. The ladder is now 0 · 0.25 · 0.5 · 0.75 · 1 · 1.5 · 2 · 3. The position command stays rate-limited and lag-clipped underneath at any gain.

✅⭐ **The settings ladder proved itself on hardware in the same log**: he walked `vel_ff` 0.25 → 1.0 → 0 → 1.0 by key, every step a round rung, with "at the ceiling" / "at the floor" said at the ends — and the wrong-direction regression fixed in [§67.5](FINDINGS.md) never fired.

✅⭐⭐ **Two frictions visible in his paste, both fixed the same evening:**
1. ⛔ **He pressed `q` INSIDE the settings screen twice, wanting to quit, and got "(does nothing here)" both times.** Now `q` closes the screen and hands over to the session's own quit flow (which holds every arm and asks — nothing is released by the keypress). Item 38's ruling pattern applied: his intent was unambiguous.
2. ⭐ **He tuned `vel_ff` blind** — the screen covered the status row, the exact item 43 complaint from the 33-press `mirror_catchup` night ([§65.4](FINDINGS.md)). **Now every `-`/`+` press prints each arm's live status row under the change**, mirror note included, so the effect of a setting is visible per press. **Item 43 is closed.**

⚠️ **Also in his message: the C920 comes later** — *"I'll plug in the Logitech later so that you can have access to that when you need it."* Camera capture stays queued behind that.

⬜ **What feedforward still owes: a verdict he can see.** Next bench run: same gentle movement at `--vel-ff 0` and then at 2 or 3, watching the FOLLOWING numbers — exaggeration exists precisely so the difference stops being subtle.

### 67.11 ✅⭐ THREE MORE TRUTHFULNESS ITEMS CLOSED THE SAME EVENING — 38, 21, 28

1. ✅ **Item 38, the two-prompt overlap, his ruling "do that sensibly": the newest prompt cancels the older one, said out loud.** The structural fact that makes one check enough: button learning (`b`) can only be ARMED while no keyboard prompt is open, because prompt handlers consume every key including `b`. So whenever both are armed, the keyboard prompt is the newer one — a per-cycle check where the puck buttons are read cancels the learning with a printed line.
2. ✅ **Item 21, the SLOWED message: it prints the measured pair now, never a guessed cause.** `SLOWED to 19% (joints asked for 7.9 rad/s, cap 1.5)`. The throttle's real trigger is the IK asking for more joint speed than the cap; "near the reach limit" was one possible cause asserted as fact and once wrong on a comfortable pose ([§41.2](FINDINGS.md)). Reading the pair: spikes only when extended = a singular pose; high everywhere = the linear setting outruns the joints. `CartesianTeleop` stores `requested_rate` for exactly this line, and a test forbids a guessed cause from returning.
3. ✅ **Item 28, the control frame on the row:** `[B TELEOP/w]` · `/t` · `/c` for world · tool · camera, padded to CONTROLS' 8 columns so nothing misaligns. `v` aims at one arm, so two arms can be driven in different frames at once, and until now nothing on screen said which was which.

✅ Verified: 696 checks across 27 files all green, the sim drive 25/25 (its script now also walks `vel_ff` live on the settings screen).

### 67.12 ⭐⭐ HIS RULING: NO SECOND D405 — the walkthrough's camera set is ONE D405 plus the C920

His words, 2026-08-18: *"I will not get the second camera, so just work without it for now. Let me know if you desperately need it for something."*

⭐ **What that rescopes, checked against every camera item:**

1. ⛔ **Item 5 (telling two identical D405s apart) leaves the walkthrough** — the problem cannot exist with one D405, because a D405 and a C920 are different models with different names. **It becomes a consolidation-plan note for the rebuild**, which per [Setup-Anleitung.md](Setup-Anleitung.md) B3 wants 2× D405 on the wrists: the two D405s DO carry distinct USB serials readable with no root ([§34.5](FINDINGS.md) — the docs once assumed otherwise), and the unsolved half is mapping a serial to an OpenCV index. The runtime fallback that always works: capture from each index and have the operator cover one lens.
2. ✅ **Item 6 (timestamped multi-camera capture) stays fully buildable**: two cameras (D405 + C920) exercise every multi-camera problem except identical-device identity — alignment to joint data, differing latencies, differing frame rates. The capture tooling gets written against LaRobot's dual-timestamp `Frame` shape ([ROADMAP §10.6](ROADMAP.md)) so the team lifts it unchanged.
3. ⚠️ **Nothing is desperately blocked.** The one thing a second D405 would enable is developing the serial→index mapping against real duplicated hardware; the plan carries it as the rebuild's first camera task instead.

⚠️ The C920 is also still to be replugged (his word: *"later"*); the capture tooling can be WRITTEN before it arrives, since the agent can never run cameras anyway ([§61.3](FINDINGS.md)) — he runs one command when it is in.

### 67.13 ✅⭐⭐ THE PUCK SCRUB IS BUILT — item 13, his idea, top-ranked version. UNRUN ON HARDWARE

✅ **What exists:** at the play prompt (`l` then a slot), **`j`** starts the recording as a PUCK SCRUB instead of a fixed-speed run. Push forward to play, pull back to rewind, let go to freeze. Either puck works (largest deflection wins — during playback nobody's hand is driving, so whichever hand is free is the deadman). `h` or `t` ends it; the end message reports where the cursor stood. [COMMANDS.md](COMMANDS.md) documents it beside `l`.

⭐⭐ **The design decisions, each from [ROADMAP §7.6](ROADMAP.md):**
1. **A mode entered on purpose (`j`), never the default** — §7.6's caution: a long unattended playback must not need a held hand; this mode exists FOR the hand.
2. **The spring centre is the deadman**: `scrub_rate` has its own deadband on top of the reader's hardware deadzone, so a released puck always means a frozen cursor and a holding arm.
3. **Backwards is safe by the same argument forwards is**: every pose at every cursor value is one a hand physically put the arm in ([§57.1](FINDINGS.md)'s park-to-start flow still runs first, unchanged).
4. **The pace caps at 1.5× in both directions** — the same ceiling a plain playback may reach, so scrubbing can never ask for a speed `l` could not, and SafeRobot still binds underneath.
5. **The lag hold works in both directions**: an arm `MAX_CURSOR_LAG` behind freezes the cursor exactly as in a normal playback.
6. ⭐ **The "third puck role" worry dissolved**: the dial is not a device role but a *playback behaviour* reading the already-assigned pucks, which are idle during replay. No assignment logic changed at all.

✅ **Verified:** `scripts/test_scrub.py` 6/6 (deadman freeze · forward/backward symmetry · linear-past-deadband capped rate · clamps at both ends, never finishes · bidirectional lag hold · grippers excludable from the lag check), full sweep **702 checks across 28 files**, sim drive 25/25, `check_flags` ✓. ⬜ **Owes one hardware feel-run**: `l` a recording, `j`, scrub it both ways. ✅ **Ran the same evening — [§68](FINDINGS.md).**

## 68 ⭐⭐⭐ 2026-08-18, EVENING — FIVE RUNS READ LINE BY LINE: THE SCRUB AND THE MIRROR VARIANT CONFIRMED, AN UNPLUG DROPPED THE ARMS, AND THE FEEDFORWARD QUESTION ANSWERED

> ⭐ His instruction: *"deeply check through… the entire history of the runs I gave you so that you know what happened and that you can find any problems, especially UX or usability."* Every block below traces to a line in his pasted logs.

### 68.0 ✅⭐⭐ WHAT THE RUNS CONFIRMED, each for the first time on hardware

1. ✅⭐⭐ **The PUCK SCRUB works** — his words: *"the scrub works. [I was] able to go forward and backwards"* (the transcript's "Unable" is the speech-to-text; the logs show the scrub running). Two-arm scrub on B+G (slot 8) and one-arm on B (slot 9), park-to-start each time.
2. ✅⭐⭐ **The MIRROR variant (`mirror`, for arms FACING each other) ran for the first time** — until now `MIRROR_SIGNS` was *"a geometric prediction"*. His 286 s session: `FOLLOWING (mirror) — tracking 0.016 rad behind`, with B at `q[0] = −0.39` and G at `q[0] = +0.39` — the sign flip, live, on hardware. He also switched copy↔mirror at the prompt, both directions of leadership.
3. ✅ **Item 43's fix proved itself**: the settings rows under every `-`/`+` press are in his logs, mid-scrub and mid-teleop, showing REPLAY/TELEOP/HOLD state per press.
4. ✅ **The overwrite guard, the park-arrival handover, the frame-named exit summary, and the failed-build path** (a transient `USBError` at build exited cleanly, retry worked) all appear in the logs doing their jobs.

### 68.1 ✅⭐⭐⭐ HIS FEEDFORWARD QUESTION, ANSWERED — the mechanism fits every one of his observations

His report: at gain 3 the arm *"moves as long as I'm holding the mouse, but then basically pushes back into the original position after I let go… like it's controlling it into the other direction."* At gains below 1, *"some latent movement"* after release. At exactly 1, *"basically doesn't change the direction at all."* At 0, *"the control continues, kind of smoothed out."* He asked: *"what exactly is the goal here?… does that make sense?"*

⭐⭐ **It makes complete sense, and each level is the physics doing exactly what the number says:**

- **Gain 0:** the arm always trails the frozen command; on release the position term keeps pulling it the rest of the way — the smooth continuation he felt.
- **Gain < 1:** same trailing, but the velocity setpoint cuts to zero in one cycle on release, so the motor brakes while the position term still pulls — the "latent movement", shorter and less smooth than at 0.
- **Gain = 1:** the arm tracks the command almost exactly (that is the point), so at release there is almost no leftover gap in either direction — *"doesn't change direction at all"*. **1 is the physically correct value; his observation is the confirmation.**
- ⛔ **Gain > 1: the motor is told the target moves FASTER than it does, so the arm overshoots the rate-limited command.** While he holds: the position term pulls back against the setpoint's push at every crossing — **the forward jitter**. On release: the arm sits PAST where the command stopped, and the position term pulls it back — **the "controlling it back the other direction" he saw. It was never aiming backwards; it was returning from overshoot.**

✅⭐⭐ **Mitigation built the same evening, both halves in `SafeRobot` ([§68.3](FINDINGS.md)-adjacent code, tests in `test_vel_ff.py`):**
1. ⛔ **A joint already past its command gets ZERO push, immediately** — the crossing stops the drive instead of fighting through it. At gains ≤ 1 the gate almost never engages, so the physically-exact setting is unchanged.
2. ⭐ **The setpoint is smoothed over ~2 cycles** — the 90 Hz stepped derivative no longer arrives as jerk, and a release decays the push over a few cycles instead of cutting it (the sub-1 "latent movement"). ⚠️ The smoothing constant is a tuning choice, not a measurement — verify on the arm. `resync()` clears the smoothed state, so no mode change inherits it (the park-spasm family, one layer down). ⬜ **Owes a re-run at gain 2-3: the jitter and the pull-back should both be visibly smaller.**

### 68.2 ⛔⭐⭐⭐ AN UNPLUGGED SPACEMOUSE DROPPED THE ARMS — fixed: a dead puck now parks gracefully

⛔ **What his log shows:** mid-TELEOP, `⛔ OSError: read error`, then straight to `motors confirmed disabled` — **no park**. The per-cycle `reader.read()` raised, the exception skipped the whole post-loop safe-stop, and the `finally` disabled every motor wherever the arms stood. His words: *"everything just deactivates and the arms just fall down."*

✅ **The fix:** the per-cycle read is guarded. A dead puck reads as **centred** (zero deflection — the same honest stand-in `--sim` uses) and sets a stop reason that routes through the **SAFE STOP**: every live arm parks, then the motors disable. His ruling implemented exactly: *"a graceful quit is probably good when something gets unplugged."* A source-pin test in `test_incident.py` keeps the guard in place.

### 68.3 ✅⭐ THE `l`-DURING-PLAYBACK OVERLAP — his mid-scrub find, fixed

⛔ **His report:** pressing `l` while a scrub ran opened the PLAY prompt **on top of** the running playback — the menu and the motion both live, the item-38 family again — and the *"playing…"* line kept refreshing underneath, reading as stuck. ✅ **Fixed:** `l` refuses while any playback runs (*"press h or t to stop it, then l"*) — stopping a playback stays an explicit act, never a side effect of asking for the next one. ✅ And the held-cursor line during a scrub now reads **"scrubbing… at Xs of Ys"** — a position, never a countdown, because a scrub goes both ways.

### 68.4 ✅⭐ THE TIME-LAPSE DIAL — his ask, built

His ask: set the scrub's top speed *"before… or while I start the scrub… so that I could scrub through it in time-lapse speed."* ✅ `--scrub-max` (default 1.5, savable) presets it; **`-`/`+` during a scrub walk a ladder up to 8×**, printed per press; the start banner names the current pace. ⭐ **Safe by construction:** a fast cursor is held back by the lag hold, so only the CLOCK is fast — the arm's speed stays bounded by `SafeRobot` and the recording's own motion.

### 68.5 ⭐⭐ HIS RULINGS: NO SECOND SPACEMOUSE EITHER, and what that parks

- **The second SpaceMouse is gone** (a friend is testing with it): *"continue as much as possible without the second mouse."* The **teleop speed-dial role for a second puck is OUT** (*"they didn't want the other space mouse feature to be integrated"*) — noted for the architecture plan; the scrub already covers the dial idea with ONE puck, since the pucks are idle during playback.
- ⛔ **Two-arm sessions REFUSED with one puck** — his log: *"✗ no unassigned SpaceMouse left for G."* That blocked MIRROR and two-arm playback entirely, although the follower and replay arms never need a puck. ✅ **Built the same evening (item 47): when every attached puck is already assigned, the remaining arm joins with the zero-deflection `StillPuck`** and a plain note — HOLD/GUIDE/playback/scrub/MIRROR-follower all work, only its own TELEOP is inert. ⚠️ Gated deliberately: a free-but-unmoved puck still aborts, so a wanted assignment never silently becomes a dead TELEOP. ⬜ One bench run confirms MIRROR with one puck.
- One D405, no C920, no second D405, no second puck: the walkthrough's hardware is now exactly **what is on the desk**, and every "would need more hardware" thread lives in the consolidation plan instead.

### 68.6 ⛔⭐⭐⭐ FEEDFORWARD ABOVE 1 IS A MEASURED DEAD END — his second verdict, the stepping mechanism, and why no fix exists up there

⛔ **His verdict on the mitigated version, 2026-08-18 late:** *"still jittering back and forth, and therefore not really that usable… basically moving in steps now… between five and ten steps per second… kind of feels like the motor's vibrating"* — and **it stepped during PARKS**, a motion that *"knows exactly where it's going."* His ruling: *"either we find a way in which this works and makes the controls nicer and more exact, or it's just not that usable."*

⭐⭐ **The stepping rate is the diagnosis.** The past-the-command gate is an on/off switch: push (arm behind) → overshoot (gain > 1 guarantees it) → gate cuts to zero → the arm falls back behind → the gate reopens and the push rebuilds through the 2-cycle filter → overshoot again. That relaxation cycle at 90 Hz with a 2-cycle filter and real motor dynamics lands at a few Hz — **his 5-10 steps per second, felt from the outside.** The gate turned continuous jitter into bang-bang stepping; it moved the symptom, never removed it.

⭐⭐⭐ **Why nothing can fix gain > 1, stated once so nobody retunes at this wall again:** above 1 the velocity setpoint CONTRADICTS the position command by construction — the motor is told to move faster than the trajectory it is simultaneously told to hold. Every smoothing, gating or fading scheme is then a choice of HOW the contradiction discharges: continuous jitter (raw), stepping (gated), or sluggishness (faded until the effective gain is ~1 again). **A "fixed" gain 3 is gain 1 wearing a costume.** The exaggeration range did its actual job — it made the effect feelable and produced the overshoot understanding ([§68.1](FINDINGS.md)) — and is now retired on his either/or.

✅ **Applied:** `VEL_FF_CEILING` is **1.0** again (values above clamp, and the plan says so when it clamps), the ladder is `0 · 0.25 · 0.5 · 0.75 · 1`, and the gate + smoothing STAY — at gains ≤ 1 the gate only catches transient overshoot and the smoothing still removes derivative noise and release-cut, which is what they were good for. ⭐ **The setting he confirmed as precise is the ceiling: 1.** Parks under gain ≤ 1 track BETTER, never worse — the stepping he watched was the >1 range.

⭐ **If more responsiveness is ever wanted, the legitimate lever is the servo gains (`kp`/`kd`), not more feedforward** — with item 17's standing caveats: stiffer joints hit harder, and the measured speed benefit was refuted once already. Carried into the consolidation plan as a rebuild note.

### 68.7 ⚠️ SMALL OBSERVATIONS FROM THE LOGS, kept so they are not re-derived

- In the vel_ff-3 scrub, lag reached **0.876 rad** and the cursor held for a long stretch — consistent with the overshoot oscillation fighting the tracking; worth re-checking after the §68.1 mitigation.
- His 40 s single-arm run walked `vel_ff` 0→1.5 while driving: the settings rows show `q` changing between presses — he tunes while moving, which is exactly what the live rows are for.
- The Ctrl-C during the vel_ff-3 scrub wrote the **first incident file carrying `chain_alive_at_teardown`** ([§67.6](FINDINGS.md)).
- ⚠️ At `vel_ff 2` in TELEOP his wrist hit `q[5] = −2.02` with a persistent `STUCK lead 0cm/14°` — the rotation lead pinned while the wrist sat far from centre. Not diagnosed; if it recurs after the mitigation, it earns its own item.

### 68.8 ⭐⭐⭐ 2026-08-18, LATE NIGHT — ONE PUCK NOW FOLLOWS THE SELECTION (his design), AND BOTH SCRUB COMPLAINTS DIAGNOSED

✅ **Item 47 confirmed on hardware**: *"the two arms with one puck works. I can only control B"* — which was the built behaviour, and his next sentence designed its successor.

✅⭐⭐ **THE SHARED PUCK FOLLOWS THE SELECTION — his design, built the same night.** His words: *"it would maybe be smart if we only have one SpaceMouse that we can switch the arm when we use `a`… I can control G if I press that I'm on mode G… and both arms at the same time"* — and his own answer on how BOTH should work: *"maybe they can just, from their initial position, be controlled by the one mouse"*, no mirror needed. So, in a multi-arm session with EXACTLY one real puck: `a` aims the puck as well as the mode keys (B → G → BOTH; BOTH drives both arms at once, each from its own pose); unaimed arms read a centred puck and hold; the selection message says the puck moved with it. ⛔ **The shared reader is read ONCE per cycle** — two arms draining one HID queue would split the event stream between them (the §54.1 family). The unplug guard covers the shared path too. With one puck per arm, nothing changes. Source-pinned in `test_puck_assignment.py`. ⬜ Owes a feel-run: `a` through B → G → BOTH in TELEOP.

⭐⭐ **Slow scrub "small steps, maybe two per second" — DIAGNOSED: the arm's own static friction, not the code.** `pose_at` interpolates between samples (checked — no sampling steps in the target), and the cursor advances smoothly every cycle. What steps is the ARM: at a very slow commanded pace, the position error grows slowly, the joint sticks until the error builds enough torque to break friction, slips forward, and sticks again — classic stick-slip, and the measured 0.04-0.10 rad friction deadband ([ROADMAP §8.2](ROADMAP.md) item 11) is exactly the constant that predicts it at a few Hz. ⚠️ **The same stepping exists in ultra-slow parks; it is the price of very slow motion on friction-heavy joints under position control.** A little `vel_ff` (≤ 1) may soften it (the kd term helps break friction before error builds) — worth one try, never promised.

⭐⭐ **"No way that was eight times" — CORRECT, and it is the lag hold doing its job.** The dial sets how fast the CLOCK may run; the lag hold freezes the clock whenever the arm falls 0.15 rad behind. His slot-9 recording was taught at ~0.85-1.02 rad/s, and `max_speed` caps the arm at 1.0 rad/s — so on moving stretches the achievable pace is roughly **1x, whatever the dial says**; 8x only materialises on near-still stretches. **That is the safety design, and the failure was that nothing SAID so.** ✅ The scrubbing hint now reports the measured pace once per second — *"scrubbing… at 3.2s of 9.8s, effective 0.94x of the recording"* — and, only when the hold actually bit in that window, adds *"the ARM binds, raise max_speed (n, 1) to scrub faster"*. ⭐ The real lever for faster skimming IS `max_speed`, which is his to raise on the `n` screen.

### 68.9 ✅⭐⭐⭐ THE PARK MERGE IS CODE-COMPLETE — item 23, all four audit groups, and the audit caught two things substitution would have missed

✅ **What changed, group by group ([ROADMAP §6.2](ROADMAP.md) executed as written, one commit each):**
1. **`enter_hold` → the class, 16 sites.** Two were the §52.1 trap and were REORDERED, never substituted: the park seed in `begin_path` and the mirror engage both want a different mode after the command-seed, so the mode is written after the call. The other 14 collapse or substitute cleanly.
2. **`enter_guide` → the class, 3 sites.** The class RETURNS the "staying in HOLD (NOT weightless)" warning; every caller prints it — the warning that once explained a falling arm ([§11](FINDINGS.md)) cannot be dropped.
3. **`enter_teleop` → the class, 4 sites**, with `make_teleop` injected (the factory is why the class is testable). The CONTROLS `m` site is the trap's third instance, reordered (`enter_teleop` then `mode="map"`).
4. ⭐⭐⭐ **The park itself: `ArmSession.begin_path`/`step_path` — the code with 48 tests — finally moves the arm.** The script's 165-line park branch became a 107-line narrator of the returned `ParkStep`: the class decides, the script narrates. The `e` key pushes the easing onto every arm, so mid-park easing changes still bite.

⭐⭐ **What the audit caught that blind substitution would have shipped:**
- **The class's settle phase was two lessons behind the script** (both learned after the class was written on 2026-08-13): it stopped commanding when the cursor ended — which, with velocity feedforward on, freezes a stale nonzero setpoint at the motor — and it never credited error improvement during settle, so a slowly-closing arm would be declared blocked at the stall timeout. Both merged into `step_path`, with tests, including one pinning that a TERMINAL verdict stops the commands.
- **`check_restructure`'s "moved but never read through arm" rule went stale BY DESIGN**: five fields (`park_cmd`, `park_best_err`, `park_progress_t`, `park_leg_t`, `park_start_t`) graduated to class-internal. The checker gained the TIGHTER rule 2b — any script access to them is a fault, because it would re-open the §52.1 split — and the rule was **falsified before being trusted** ([§59.1](FINDINGS.md)'s lesson): a planted `one.park_cmd` read produced "1 fault, do not commit", and the clean file passes.

⭐ **Deliberately NOT merged: `park_arms`** (the quit/Ctrl-C/auto-park). Both halves of §52.1's complaint are false for it — it is tested (14 tests) AND it is the code that runs — and it survives teardown states the in-session park never sees.

✅ **Verified:** 714 checks across 28 files, `drive_sim_session` 25/25 plain AND with `--vel-ff 0.5` (records, replays, parks and tears down through the new path), `check_restructure` coherent, 1066/1066 links. ⬜ **Owes the item-23 bench pass: every mode once on the arm — GUIDE, TELEOP, CONTROLS, a `p 1 2 3` run, a playback, `q q`.** Nothing builds on the merge until that has run (his rule 11 checkpoint). The jaw pause (items 3, 10) is the first thing waiting behind it.

## 69 ⭐⭐⭐ 2026-08-18, LATE NIGHT — THE BENCH PASS CONFIRMS THE PARK MERGE, ONE ZOMBIE FOUND AND FIXED, AND THE 2 cm IS THE FRICTION FLOOR

### 69.0 ✅✅⭐⭐⭐ THE ITEM-23 BENCH PASS RAN, AND THE MERGE IS CONFIRMED ON HARDWARE

His verdict: *"everything really feels exactly as before. It seems really good."* His four sessions exercised, on the merged code: every mode entry (HOLD/TELEOP/GUIDE on one arm, on aimed arms, on BOTH) · two recordings saved with the overwrite guard · a three-waypoint run **`p 4 5 6` with per-leg timings (3.8 s → 1.5 s → 3.6 s) and the settle note** — the merged park narrating `ParkStep` on real hardware · a playback to completion with the full per-joint tracking table · a scrub · CONTROLS opened and left · `q q` and Ctrl-C shutdowns, all parks 0.019-0.033 rad. **Item 23 is CLOSED. The jaw pause (items 3, 10) and the §10 restructure are unblocked.**

✅ **Also confirmed in the same runs:** the puckless arm joining (§68.5) · the shared puck following the selection, including BOTH driving both arms from one mouse (§68.8) · MIRROR working with one puck · the mirror stop message naming both remedies with live numbers · the new SLOWED/frame/scrub readouts. ⚠️ Two transient `USBError [Errno 19]` at build (the §35.5 DFU family — retry worked both times; the powered-hub note in the plan stands).

### 69.1 ⛔✅⭐⭐ THE ZOMBIE REPLAY ARM — found by his pass, fixed the same hour

⛔ **What his log shows:** during a two-arm scrub he pressed `m` (CONTROLS). B left replay; **arm G stayed `[G REPLAY]` for the rest of the session** — cursor frozen (the advance needs ALL replay arms in the mode), no message, no exit. Mode keys re-mode only the AIMED arm, and the cleanup waited for NO arm to be in replay, so the zombie satisfied neither side. ✅ **Fixed: ANY arm leaving replay ends the playback for EVERY replay arm** — the released arms go to HOLD through the class, the message says so, and a source-pin forbids the old wait-for-everyone condition from returning. A playback is one thing; it ends as one thing.

### 69.2 ⭐⭐⭐ THE 2 cm MIRROR OFFSET IS THE STATIC-FRICTION FLOOR — the naive story refuted by the vendor's own code, and the honest option list

⭐ **His question:** the arms position exactly under the mouse, so why does the mirror follower sit ~2 cm off? *"It really shouldn't be an issue."*

⛔ **The obvious story is WRONG, and the vendor's code proves it.** The obvious story: a PD position controller must keep a standing error to hold against gravity (error = gravity ÷ kp), so send gravity as a feedforward torque. **But `motor_chain_robot.py` line 366 already does that**: `motor_torques = joint_commands.torques + g * gravity_comp_factor + friction_comp` — gravity compensation is added to EVERY command, position mode included. The droop survives it.

⭐⭐ **What the offset actually is: static friction (stiction).** A joint does not move until `kp × error` exceeds its breakaway friction, so every stop leaves each joint up to `friction ÷ kp` short, in whatever direction it was last moving. The numbers close: item 11's measured constant droop is **0.037 rad (stiff joints) to 0.080 rad (soft)**, and §64.1's measured tip-metres-per-radian (elbow 0.418 m/rad) turns 0.037 rad into **~15 mm at the tip** — his 2 cm sphere, from the measured constants, with no new hypothesis. ⚠️ The vendor's `friction_comp` is velocity-based, so it is zero at standstill — it never helps break the LAST bit of stiction.

⭐⭐ **Why teleop feels exact anyway: HIS EYES close the loop.** In teleop he pushes until the ARM is where he wants — the command quietly sits beyond it, absorbing the friction band. In mirror, nobody closes the loop: the follower's command IS the leader's pose, and the friction band stands exposed. The same floor shows everywhere nobody compensates: every park settles *"0.019-0.033 rad off — as close as the arm holds itself under load"*.

⭐ **The options, ranked honestly:**
1. **Raise `kp`** — the one real lever: error = friction ÷ kp, so double stiffness halves the floor. It is item 17 with its standing caveats (stiffer joints hit harder; the SPEED argument for it was refuted once). A bench decision, his.
2. ⛔ **Integral action is measured out** — that WAS `mirror_catchup`, and his verdict stands: it integrates noise and wanders (§67.1).
3. ⛔ **Dither** (a tiny oscillating torque to keep joints unstuck) trades the offset for a permanent buzz — the §68.6 stepping, deliberately.
4. **Accept it as the repeatability floor and STATE it**: ~0.02-0.08 rad per joint, ~1-2 cm at the tip, a property of PD control on friction-heavy geared motors. The rebuild plan carries it as a hardware characteristic, next to the kp option.

**Recommendation: 4 now, 1 if a use-case needs sub-centimetre following — and nothing in the data-collection goal does: a demonstration's value is the trajectory, and ±1 cm of follower offset is far below the noise a learned policy tolerates.**

### 69.3 ✅⭐ ALL THREE CAMERAS ARE ON THE BUS — the camera chain is fully unblocked

`check_rig.py`, 2026-08-18 late (dated reading): **two D405s** (`260323072846` new, `255323071773` the arm-B one, distinct serials, no root needed) **and the C920**. So item 5 (telling identical D405s apart) is buildable again after all — his earlier no-second-camera ruling was overtaken by him plugging it in — and item 6 (timestamped capture) has its full hardware. ⛔ The permanent constraint stands: **the agent can never run a camera** (macOS per-app permission, §61.3) — the tooling gets written headless with synthetic tests, and he runs one command.

## 70 ⭐⭐⭐ 2026-08-18, THE SESSION AFTER THE BENCH PASS — THE JAW PAUSE IS BUILT (items 3 + 10): A RUN STOPS WHERE ONLY THE JAWS MOVE, WAITS BY MEASUREMENT, AND REPORTS THE GRAB

### 70.0 ✅⭐⭐ WHAT EXISTS NOW, and the design held up from §6.6.2 unchanged

✅ `ArmSession.begin_path` runs `plan_gripper_stops` over the resolved run: one blended `JointPath` per segment, and between segments the arm HOLDS at the split waypoint while the jaws are commanded and **waited for**. The wait is **measured, never timed** — the jaws must be still for `JAW_SETTLE_SECONDS` (0.35 s) after at least `JAW_MIN_WAIT` (0.25 s), and a jaw stalled on an object is still, so a successful grab resumes the run by the same rule as an empty close. `JAW_TIMEOUT_SECONDS` (3.0 s) bounds a jaw that never settles: the run says so and continues rather than gating for ever — the ROADMAP §6.6.2 requirement, verbatim. On the resume cycle `check_grasp` grades a closing leg (item 10 — *"it needs the pause in item 3 first"* — wired exactly there) and the script narrates: *"✋ holding something — the jaws stopped 0.200 of the stroke short of closed"* or *"∅ the jaws closed onto themselves."* The plan line counts the stops **before Enter** (*"RUN 2 → 3 · ⏸ 1 jaw stop"*), and the MODE line says *"pausing 1× for the jaws"* so a pause can never read as a stall.

⭐ A run with no jaws-only leg produces exactly one segment and is **behaviourally identical to before** — pinned by test. A leg that moves the arm AND the jaws together stays unsplit with a warning naming it (§6.6.2's rule: guessing wrong on 4.3 kg is worse than saying so) — and that advice is **suppressed for a playback's drive-to-start** (`mixed_leg_advice=False`), where nobody can act on it.

### 70.1 ⛔⭐⭐ TWO REAL DEFECTS FIXED IN PASSING, both of the fails-by-lying family

1. ⛔⭐ **The park commanded the saved jaw value RAW, every cycle, ignoring the stall latch.** A park whose pose closes the jaws onto an object would push into it at 90 Hz for the rest of the run — the stall guard latched a block (`block_jaw_at`) and `step_path` never looked at it. That is §58.2's shape one layer down, on the code path where motor 7 was cooked three times. ✅ Every park command now goes through `_command_park`, which routes the jaw through `hold_jaw`: the latch holds (keep the grip, stop the push) and a deliberate OPEN still clears it. Pinned by `test_a_latched_jaw_block_is_honoured_by_the_park` — whose first version asserted the wrong scenario and failed honestly: it opened the jaws mid-run, and opening clears the latch **by design**, which the test now documents instead of fighting.
2. ⛔⭐⭐ **`err` and `lag` counted the jaw, so a SUCCESSFUL grab would have read as a BLOCKED park.** A held object parks the jaw an object-width from its command for the whole rest of the run; the cursor's lag hold would freeze (the jaw "lag" never closes) and the arrival verdict could never fire. Both are **arm-only** now (`_arm_err`), the same by-index gripper exclusion the playback already used for the same reason. `test_err_and_lag_ignore_a_held_object` pins the lift-after-grab case end to end.

### 70.2 ⭐ THE CHECKER GRADUATION, falsified before trusted — and the abandon block was under-counting

`park_s` left the script entirely (its last read was `left = park_path.length - park_s` in the abandon block, which had to become `abandon_path()` anyway — a queued run's dropped distance spans segments the script cannot see, so the inline arithmetic would have UNDER-REPORTED what was cancelled). `check_restructure` moved it to the CLASS-INTERNAL rule with the six new jaw-pause fields, and the tightened rule was **falsified before being trusted** (§59.1's discipline): a planted `one.park_s + len(one.park_queue)` read produced *"2 fault(s). Do not commit"*, and the clean file passes.

### 70.3 ✅ VERIFIED, and what is still owed

✅ **702 checks across 29 files** (15 new in `scripts/test_jaw_pause.py`: the split, the hold, the measured wait — a slower jaw is waited on longer, with the settle window's arithmetic checked — the grab/empty/opening/timeout grasp verdicts, the latch, the arm-only arrival, abandon counting the queue, preview count == run count, 6-joint poses never split, a jaws-only FIRST leg pausing immediately). ✅ `drive_sim_session.py` 25/25 plain AND with `--vel-ff 0.5`. ✅ `check_restructure` coherent, `check_flags` ✓, 1060/1060 links. ⚠️ The fake jaw follows at a rate and stops at an object; it does not model torque, so the interaction between the pause and the script-level stall guard (which needs `eff`/`vel` from a chain read) runs on hardware only.

⬜ **The bench pass it owes (~3 minutes, one arm):** with an object on the desk — `s 1` above it open · `s 2` at it open · close by button · `s 3` · open · `s 4` lifted-pose closed? — simplest real sequence: save `1` above-open, `2` at-open, `3` at-CLOSED (only the jaws differ from 2), `4` lifted-closed, then `p 1 2 3 4 Enter Enter`. Watch for: the plan line saying *"⏸ 1 jaw stop"* · the arm stopping exactly at waypoint 2's pose · *"⏸ 3: only the jaws move…"* · the grab verdict line · the lift completing with the object held and the park arriving despite the jaw gap. ⛔ If the jaws close on nothing, the *"∅ closed onto themselves"* line is the pass, not a failure.

### 70.4 ✅⭐⭐ ONE RUNNER OVER EVERY TEST FILE — [ROADMAP §10.5](ROADMAP.md) step 2, the §67.5 remedy, falsified before trusted

✅ **`uv run checks/run_tests.py`** runs all 29 `test_*.py` files (4 processes wide), prints one line per file and a **TOTAL: 702/702 checks across 29 files**. A file fails on any of THREE independent signals: nonzero exit, no `N/M passed` count line (a crash prints a traceback and no count), or N < M — because §67.5's red files each showed a different one. ⭐ **The TOTAL is the point**: it is the §59.1 catch-counter applied to the suite itself, so a silently disarmed check shows as the number dropping while everything stays green, and the runner says so in its own output.

⭐ **`scripts/falsify_run_tests.py` proves the runner can see a failure** — four fixtures (passing · failing assert · crash before any count · a LIAR that prints `3/3 passed` and exits nonzero), and the runner must catch exactly the right three. 6/6.

⛔ **A stale claim corrected in passing:** [ROADMAP §10.5](ROADMAP.md) step 2 said *"pytest already available"* while [§67.5](FINDINGS.md) had measured the opposite (`No module named 'pytest'`) — both sat in the repo at once, the §33.3 pattern across two files. Re-measured now: pytest is NOT installed. The runner is deliberately zero-dependency; choosing pytest belongs with the step-3/4 package restructure, next to the team's own pytest layout in LaRobot ([ROADMAP §10.6](ROADMAP.md)).

### 70.5 ✅⭐⭐⭐ THE JAW PAUSE IS CONFIRMED ON HARDWARE — his run, the grasp check told the truth about a missed grab, and the miss is the friction floor

✅ **His bench pass, 2026-08-18 evening, ran the §70.3 procedure twice** (once at 0.40 rad/s, once at 0.98): the plan line counted the stop (*"pausing 1× for the jaws"*), the run blended `1 → 2`, held at the split, printed *"⏸ 3: only the jaws move — going to 0.17"*, resumed after a measured 0.6 s, and finished at slot 4 (0.030 and 0.020 rad off). His verdict: *"It pretty much worked. There were nearly no issues."* **Items 3 and 10 are closed on hardware.**

⭐⭐ **The grasp check's first real answer was a TRUE NEGATIVE, which is the whole point of item 10.** The run printed *"∅ the jaws closed onto themselves — nothing gripped"* — and the Lego piece had indeed been missed, by his estimate a millimetre to half a centimetre short (⚠️ his message says both; speech-to-text, rule 12 — either reading leads to the same conclusions below). A failed grab that announces itself is a bad demonstration that never enters a dataset silently ([ROADMAP §6.6](ROADMAP.md)).

⭐⭐ **Why it missed, decomposed — and what was done about the one software part:**

1. **The friction floor** ([§69.2](FINDINGS.md)): a replayed waypoint settles 0.02-0.04 rad short (his own runs: 0.039, 0.030, 0.020), which is roughly 8-16 mm at the tip. A millimetre-scale grab sits INSIDE that floor, so waypoint replay alone cannot guarantee it. The levers are unchanged: kp (item 17, his call) or accept-and-state.
2. ✅ **The software's own contribution is fixed**: the jaws used to start the instant the cursor finished, while the arm still crept. **The settle-gate now holds the jaw command until the arm is within 0.02 rad or has stopped improving for 0.5 s**, and the pause line prints the settled offset (*"arm settled 0.021 rad off"*) — so the NEXT missed grab is diagnosable at a glance: offset at the floor = the pose was taught off; offset well above = the arm never got there.
3. ⭐ **The dataset-grade answer was already designed and this miss is its evidence: the composite run** ([ROADMAP §6.6.1](ROADMAP.md), item 12) — approach legs as waypoints (tolerant, noise-friendly), the grab leg as a hand-taught recording (millimetre-precise because his hand closed the loop). His 2026-08-12 finding that teleop feels exact BECAUSE his eyes close the loop ([§69.2](FINDINGS.md)) is the same physics from the other side.

⚠️ **One narration item from his log, fixed:** the mixed-leg warning printed with a double period (*"grip something.."*) — the string carried its own period and the script adds one. The warning itself fired correctly (his single-slot parks moved arm and jaws together, which is exactly what it describes).

### 70.6 ⭐⭐ THE HARDWARE-WINDOW CAPTURE, 2026-08-18 evening — everything the agent could read before the rig went out of reach, taken because he was leaving

> ⚠️ Dated readings, like every hardware number in this file. Their value is that the rig is about to be UNREACHABLE (he took the Mac home), so these are the ground truth the hardware-free work gets written against until the next bench day.

1. ⭐⭐ **The full USB inventory with all three cameras attached** (`check_rig.py --raw`): both CANables (`2081337C594E5018` = B, `20593383594E5018` = G, bus 0, running firmware) · **two D405s with distinct serials** (`255323071773` at bus 1 addr 3, `260323072846` at bus 1 addr 10) · the C920 (bus 1 addr 8) · **ONE SpaceMouse** (the second is with his friend, [§68.5](FINDINGS.md)).
2. ⛔⭐⭐ **NEW FACT, and it changes item 6's design: the C920 reports an EMPTY USB serial** — the same defect the SpaceMice have ([§0](FINDINGS.md) #5). So "select cameras by serial" can never cover the C920; a capture config that keys every camera by serial is wrong BY MODEL. The C920's stable identity today is model name (only one C920 exists on this rig) — the plan should say so, and the D405s stay serial-keyed.
3. ⭐ **The cameras' USB locationIDs** (the port-path number macOS assigns; stable per physical port, readable with no camera permission): D405 `255323071773` → **19005440** (0x01220000) · D405 `260323072846` → **18939904** (0x01210000) · C920 → **18092032** (0x01141000). ⭐ **Why captured: this is the raw material for the serial→OpenCV-index problem (item 5).** A hypothesis to TEST, not a fact: AVFoundation device uniqueIDs on macOS often embed the locationID, so if OpenCV's index order can be joined to uniqueIDs, serial→locationID→uniqueID→index closes the chain with no root and no lens-covering. Nobody has verified the middle join on this rig.
4. ✅ **Motor health, both arms** (`ping_motors`, error clearing OFF): 14/14 online, every error `0x1 (normal)`, 31-33 °C, everything at rest. Arm B's jaws 3.6% open with the usual −2π shift; **no latched fault anywhere**, as a real reading rather than a cleared one.
5. ✅ **The register diff** (`check_arms_match.py`): every readable register identical on B and G apart from per-unit calibration, **and nothing has moved since the 2026-08-14 baseline** (commit `2f70f6e`). The standing caveat stands: scaling limits are not readable ([§38.1](FINDINGS.md)).

⭐ **What this closes for the hardware-free stretch:** the camera-chain tooling (item 6) can be written against real serials, real locationIDs and the real C920-has-no-serial constraint; the consolidation plan gets a verified rig inventory; and the §10 restructure can proceed knowing the rig it describes was healthy when last read.

### 70.7 ✅⭐⭐⭐ THE PACKAGE RESTRUCTURE LANDED — [ROADMAP §10.5](ROADMAP.md) step 3: `src/yam/`, importable, no path hacks, suite total unchanged

✅ **What exists now:** `src/` is the package `yam`, installed editable by `uv sync` via uv's own build backend (`uv_build` — chosen deliberately: it ships inside uv 0.9.22, so a fresh `uv sync` needs NO network, which matters on a laptop that leaves the building). 17 modules moved by `git mv` (history preserved) into the [ROADMAP §10.2](ROADMAP.md) layout with [§10.6](ROADMAP.md)'s domain folders: `yam.robot` (was `yam_robot`) · `yam.can` · `yam.session` (was `arm_session`) · `yam.teleop` · `yam.motion`/`mirror`/`recording`/`settings`/`collision`/`incident`/`provenance` · `yam.inputs.{spacemouse,spacemouse_live,keyboard,axis_map}` · `yam.fake.arm` · `yam.ui.screen`. **All 52 `sys.path.insert` headers are gone** (the vendored-i2rt insert in `yam.can` stays; the SDK is not a package). `cameras/` is deliberately NOT created empty — item 6's code creates it when it exists.

⭐⭐ **The near-miss, and the runner caught it in one command:** the first mechanical rewrite matched imports only at line start, and this repo lazy-imports inside functions — 12 files carried indented `from mirror import …` sites the regex never saw. `run_tests.py` (built one session earlier) surfaced all six failing files in one line: **612/635, 6 files FAILING** — where the old habit of running files one at a time would have found them one bench-surprise at a time. The second pass matched any indentation and the total returned to exactly **705/705**, which is the catch-counter doing its real job: same number as before the move = nothing silently disarmed.

⭐⭐ **The one genuinely dangerous mechanical detail: five modules each computed `REPO = Path(__file__).parent.parent`.** Moving a file one (or two) levels deeper silently re-anchors every config path it resolves — gripper calibration, MuJoCo models, the i2rt path, the incidents dir — with no exception anywhere, the [§0](FINDINGS.md) shape applied to the filesystem. **`REPO_ROOT` is now defined ONCE in `yam/__init__.py` and imported**, so it cannot drift per file again.

⭐ **Doc convention set here:** live docs (README, COMMANDS, ROADMAP's active sections) had their `src/…` paths remapped; **dated FINDINGS entries keep the paths of their day** — rewriting history to match the present tree would falsify the record, and the old→new mapping is [ROADMAP §10.2](ROADMAP.md)'s own table.

✅ **Verified, all on the moved tree:** `run_tests.py` **705/705 across 29 files** · `drive_sim_session` 25/25 plain AND `--vel-ff 0.5` · `check_restructure` coherent · `check_flags` ✓ (including the two re-pathed `uv run src/yam/inputs/spacemouse_live.py` doc lines) · 1108/1108 links · all three falsifiers (`run_tests` 6/6, `check_flags` clean, `fake_arm` PASS). ⚠️ What no checker can say, as always: that the FEEL is unchanged — but no behaviour was edited, only imports and locations, and the sim drive runs the identical loop.

⬜ **What remains of [ROADMAP §10.5](ROADMAP.md): step 4** (sort `scripts/` into `apps/` · `checks/` · `tests/`, update `check_flags` + docs same commit) · **step 5** (README as the day-one door — ⚠️ wants his answer on ONE language) · **step 6** (the one-page hardware bring-up checklist).

### 70.8 ✅⭐⭐⭐ STEP 4 LANDED — apps/ · checks/ · tests/ — AND THE FALSIFIER CAUGHT A CHECKER GOING GREEN-WHILE-BLIND, live, mid-restructure

✅ **The split ([ROADMAP §10.5](ROADMAP.md) step 4, 2026-08-19):** all 59 scripts moved by `git mv` — 29 `test_*.py` → `tests/` · the 7 `check_*.py`, 3 `falsify_*.py`, `run_tests.py` and `drive_sim_session.py` → `checks/` · the 18 runnable tools (teleop_session, camera_view, calibrate_gripper, ping_motors, the probes, …) → `apps/`. `scripts/` no longer exists. Both pre-analysed traps resolved as planned: tests OF an app or check script import it from its directory via two explicit path lines (the library `yam` needs none), and every documented COMMAND line was re-pathed everywhere, dated entries included, while prose keeps its day's paths.

⭐⭐⭐ **The §59.1 scenario happened FOR REAL, and the falsifier caught it.** `check_flags`' command-extraction regex hardcoded `uv run (scripts/…)`. After the move it matched nothing, so the checker printed its usual green *"every documented command's flags exist"* while validating **zero** commands. Nothing in its own output differed. **`falsify_check_flags` failed 6 of 10 cases** ("wanted reported, got left alone") — the catch-count dropping is the ONLY thing that saw it. The pattern now names the real directories (with a comment saying why the alternation is load-bearing), and the checker reports **106 command lines against 30 parsers** — up from 79, because the widened pattern also sees the `src/yam` diagnostic and lines the old pattern never covered. ⭐ The lesson, now with a live specimen: **a checker's green run is a claim about the inputs it can still see**, and only a falsification count notices when that set silently becomes empty.

✅ **Verified, all from the new paths:** `checks/run_tests.py` **705/705 across 29 files** (total unchanged through the second move in two days) · sim drive 25/25 plain and `--vel-ff 0.5` · `check_restructure` coherent · `check_flags` 106/30 ✓ · 1120/1120 links · all three falsifiers pass.

⭐ **His language ruling, 2026-08-19, unblocks step 5:** *"both German and English are completely fine, you can just write everything in English, but we are all fluent in both."* So: everything new in English, the German documents stay as they are, and the README rewrite has no open questions left.

### 70.9 ✅⭐⭐ THE CAMERA CAPTURE CHAIN IS BUILT (item 6) — LaRobot-shaped frames, one shared reader, and the bandwidth question turned into one command

✅ **What exists ([ROADMAP §8.2](ROADMAP.md) item 6's four design decisions, executed):** `src/yam/cameras/` — `Frame` (field-aligned with the team's LaRobot record so the rebuild lifts capture code unchanged: `camera_name · sequence · camera_timestamp_ns · host_timestamp_ns · rgb · depth`), the hardware-confirmed `FrameGrabber` **moved verbatim out of the viewer** with the app importing it back (one copy, the §52.1 rule; the only additions are a store-time host stamp and `newest_stamped()` — every call the viewer already made is byte-identical), and `CaptureSet`, which samples N cameras at the control loop's own moments without ever blocking and keeps per-camera fresh/duplicate/gap accounting. `apps/capture_probe.py` composes it with the viewer's confirmed identification machinery.

⭐⭐ **The two honesty decisions, because they are what make the dataset trustworthy later:**
1. **`camera_timestamp_ns` is `None`, always, on this stack** — OpenCV's AVFoundation backend cannot report a device-side capture time (the same backend that cannot report FOURCC, [§63.0](FINDINGS.md)'s family). A fabricated device stamp would be the [§0](FINDINGS.md) pattern applied to time; the dataset must know the camera did not tell us. The host stamp is taken when the reader thread STORES the frame, under the same lock — the closest observable to the exposure this backend offers.
2. **A repeated frame keeps its `sequence`**, so a 30 fps camera sampled at 90 Hz shows ~2 duplicates per frame to any consumer instead of silently padding a dataset — the recorder (item 7) will dedupe or keep them knowingly.

✅ **Verified headless:** `tests/test_capture.py`, 8 tests driving the REAL classes with fakes — honest fields · none-before-first-frame · repeat-keeps-sequence · a slow camera never holds up a fast one · the gap math · stop-stops-everything · the real thread with a scripted blocking capture (8 frames, stamps, clean join, the confirmed 2-tuple API unchanged) · `fourcc_name`. Full suite **713/713 across 30 files**, `check_flags` 108/31, 1142/1142 links, sim drive 25/25, viewer tests 58/58 on the shared grabber.

⬜ **What only the bench can say, one command each:**
1. `uv run apps/capture_probe.py --cameras d405,c920 --seconds 10` — per camera: achieved fps, fresh ratio, worst blind gap. ⭐ **That run IS the two-camera bandwidth measurement** [§34.5](FINDINGS.md) has warned about since 2026-08-13: exhaustion shows as a low fps and long gaps, never as an error. `--save` files the JSON + one PNG per camera under `recordings/cameras/`.
2. `uv run apps/camera_view.py --list` — now prints each camera's AVFoundation **uniqueID**, which is the one-glance test of item 5's open hypothesis: if a D405's uniqueID embeds its USB locationID ([§70.6](FINDINGS.md): `0x01220000` / `0x01210000`), serial→index closes with no root and no lens-covering.

### 70.10 ✅⭐ LABELS WHILE DRIVING — item 8's keypress half, built into the recording file itself

✅ **`k` while recording toggles BAD↔good from that moment** (his idea from [ROADMAP §6.6](ROADMAP.md): mark the stretches where a demonstration went wrong, so they never train a model as if they were good). The model is deliberately minimal: a recording starts implicitly `good`, each mark holds until the next, and `Trajectory.label_spans()` merges the presses into `(start, end, label)` stretches — the shape the dataset export (item 7) will consume. `bad_seconds()` is the one-number summary, printed at stop and shown per slot by `check_recordings.py`.

⭐ **Two decisions worth keeping:** ① marks live inside the recording's `meta`, so the FILE FORMAT is unchanged — every recording saved before labels existed loads and reads as all-good, which is what it always was; ② labels are DATA, never control — a bad stretch plays back exactly like a good one, because the labels' consumer is the export, and a playback that silently skipped stretches would be a different (and surprising) feature.

⚠️ **One honest miss while testing it:** the old-file fixture GUESSED the file shape (nested sample lists) instead of matching `to_dict()`'s real flat rows, and failed against its own library — the claim-names-a-method rule ([§36.3](FINDINGS.md)'s family) applied to a fixture. Matched to the real shape, with a comment saying so.

✅ Verified: 4 new tests in `tests/test_recording.py` (69/69 there), full suite **717/717 across 30 files**, sim drive 25/25, `check_restructure` coherent. ⬜ Owes one bench feel-run: record, `k` twice, read the stop summary and the listing.

### 70.11 ⬜⭐ THE BENCH LIST FOR HIS NEXT VISIT — four items, about five minutes, and what each one answers

> Written 2026-08-19 while the rig is out of reach, so tomorrow starts with a list instead of an archaeology dig. **Plug in: both arms (CAN adapters + mains power), BOTH D405s and the C920, the one SpaceMouse — and put a graspable object (the Lego piece) on the desk.**

1. **`uv run apps/capture_probe.py --cameras d405,c920 --seconds 10 --save`** (~30 s) — the two-camera bandwidth measurement ([§70.9](FINDINGS.md)): per camera the achieved fps, fresh ratio, worst blind gap. ⚠️ With two D405s attached, `d405` may refuse as ambiguous — then `--indices` per the listing. Add the second D405 in a second run if the first is clean: three cameras on one tree is the real dataset load.
2. **`uv run apps/camera_view.py --list`** (~30 s, one glance) — the new uniqueID lines against [§70.6](FINDINGS.md)'s locationIDs (`0x01220000` / `0x01210000`): if a D405's uniqueID embeds its locationID, item 5 (telling identical cameras apart) closes with no root and no lens-covering.
3. **The grab re-run** (~2 min): re-run the §70.3 sequence on the Lego piece. ⭐ **The pause line now prints `arm settled X rad off`** — at the friction floor (~0.02-0.04) the POSE was taught off, so re-save the at-object waypoints a few millimetres further and it grips; well above the floor means the arm never got there, which is a different conversation. The grasp verdict line says whether it held.
4. **One labelled recording** (~1 min): `w` · move · `k` · move · `k` · move · `w`, save it, then `uv run checks/check_recordings.py` — the stop summary and the listing should both show the bad stretch ([§70.10](FINDINGS.md)).

⚠️ Standing, unchanged: the repo has NO remote of Julien's own (229 commits on one Mac — needs his GitHub account, ~5 minutes with him present), and the two old API keys from `AutonomousMAS/.env` are still unrotated (Mind Understanding's `state/NOW.md` §4 item 2, open since 2026-08-06).

### 70.12 ✅⭐⭐⭐ COMPOSITE RUNS ARE BUILT (item 12, phase 2) — poses park, takes play, one queue of confirmed handovers, 29/29 in the simulator on the first try

✅ **What exists:** typing `p 1 w8 1 Enter Enter` runs three legs — park to pose 1, PLAY recording 8, park to pose 1 again. Inside the `p` prompt, `w` arms "the next digit names a RECORDING"; the plan line reads `RUN 1 → ▶8 → 1` and lists the takes; consecutive poses group into ONE blended park exactly like a plain run (jaw pauses included, so a pose leg can still grab). **This is his §6.6.1 idea executed**: precision where the task needs it (the taught take), variation where it tolerates it (the planned poses) — and the missed-grab evidence from [§70.5](FINDINGS.md) was its motivating case.

⭐⭐ **Why it went in clean — the two structural choices from [ROADMAP §6.6.1a](ROADMAP.md):**
1. **The runner is a queue advanced ONLY in arrival/completion branches** (the [§57.1](FINDINGS.md) rule at composite scale): a pose leg completes when EVERY awaited arm's park arrives; a take leg completes when its playback finishes; one `abandon_composite()` is called from every exit — mode keys, a blocked park, a blocked or abandoned playback, and any operator-started park (`begin_path` grew `for_composite` beside `for_replay`).
2. **The take leg IS the confirmed `l` flow**, not a copy of it: the prompt's validation and park-to-start moved into shared closures (`load_take`, `park_to_take_start`, `start_take`) that both callers use. Every take is validated at Enter, BEFORE any motion, so a bad slot refuses the whole run instead of stranding it halfway.

⭐ **The §6.6.1a traps, resolved:** ① handled as designed (arrival-only advancement, one abandon). ② **dissolved on inspection** — during a composite, `replay_pending` is only ever set by the composite's OWN take leg, so the replay-cancel guard never misfires and needed no composite exception; the `for_composite` flag exists for the composite-abandon rule instead. ③ **dissolved** — `w` during a composite stays allowed on purpose: recording DURING a replayed leg is the §6.6 collection method itself, not a conflict. ④ handled: pose legs park the arms aimed at Enter; each take drives its file's own arms.

✅ **Verified:** the sim drive grew the three-leg composite scenario and passed **29/29 on the first run** — announced with its leg count, the take parked to the recording's start, the queue narrated between legs, the completion line, both arms parked after. Suite 717/717 across 30 files, `--vel-ff 0.5` variant 29/29, `check_restructure` coherent, 1166/1166 links. The (b) pin held: pose-only sequences take the identical code path (the composite branch only diverts sequences containing takes) and the original 25 sim checks passed unchanged.

⬜ **What only the bench can say:** one composite grab — save poses around the object, record the grab-take by hand, then `p <pose> w<take> <pose> Enter Enter`. The seam rule means the arm parks to the take's own first pose before playing, so no leg can jump by construction. ⏸ Phase 3 (teaching a leg MID-run) stays unbuilt until phase 2 has met the arm ([ROADMAP §6.6.1a](ROADMAP.md)).

### 70.13 ✅⭐⭐ THE EPISODE EXPORT IS BUILT (item 7) — the C3 contract as written, refusals where a silent default would poison a dataset, and both open halves named in its own output

✅ **What exists:** `yam/episode.py` + `apps/export_episode.py`. One saved recording becomes one MCAP episode in [Setup-Anleitung.md](Setup-Anleitung.md) C3's exact shape: the eight state/action topics (`/left-arm-state` (6) … `/right-ee-action` (1)), every stream synchronous on the **33,333,333 ns tick**, joint-space values. Labels ([§70.10](FINDINGS.md)), full recording provenance, the arm mapping and the action policy ride in one `/episode-meta` JSON message — C3's own advice (*"Zusätzlich alles mitloggen … in eigenen Topics"*). Ran end to end on his real slot-8 recording: 190 ticks, 6.3 s.

⭐⭐ **The three honesty decisions, each where a silent default would have been poison:**
1. ⛔ **`--left`/`--right` are REQUIRED, never defaulted.** The contract's sides are physical bench positions; nothing in a recording can derive them; a wrong default would mirror every episode and raise nothing. The mapping is auditable in the episode's own metadata.
2. **The action policy is named IN the episode**: a hand-taught demo holds measured positions only (GUIDE has no command stream — the hand is the controller), so the exported action at tick k is the state at tick k+1, the standard demonstrated-action construction. A training run can read what it is getting.
3. **What is missing is said on every run**: no camera topics (frames are not wired into the recorder yet), and the output is *"the contract as written"* until the Anleitung's own **C4 mini-sample gate** has verified it against ABC's loader — the encoding detail C3 does not specify lives in ABC's `export_mcap.py`, which this repo does not have.

⭐ **The test discipline paid again, in the other direction:** every test writes a real file and READS IT BACK with the MCAP decoder (asserting on writer inputs would measure the wrong instrument, [§36.3](FINDINGS.md)) — and the one red test was the TEST's error, not the code's: it asserted the last action equals the last state, which only holds when the duration is an exact tick multiple. The corrected invariant (the last action drives to the recording's true final pose) is now pinned with a comment naming the mistake.

✅ **Verified:** `tests/test_episode.py` 8/8 (exact names/dims · exact tick spacing on every stream · next-tick actions · mapping decides sides, never file order · metadata carries mapping/policy/labels · single-arm refused · wrong mapping refused · the camera warning). Refusals also proven at the app layer (same-arm mapping, missing slot). ⬜ **Open, both named above: wiring `yam.cameras` into the recorder (needs the session's camera flags + a bench run), and the team's C4 gate.**

### 70.14 ✅⭐⭐⭐ THE CONSOLIDATION PLAN IS DRAFTED — [docs/PLAN.md](PLAN.md), the deliverable his ruling named, awaiting his read

✅ **What it is:** the rebuild plan his 2026-08-18 ruling made the endgame ([§67.0](FINDINGS.md)), drafted while every feature it describes actually exists. Its shape: the phase map (the team's own [Setup-Anleitung.md](Setup-Anleitung.md) A→E, each phase answered with what the walkthrough proved, where reality disagreed, and the first trap) · nine assignable work packages with lift-vs-rebuild pointers · the decisions made-with-evidence and open-with-owner · the method chapter (how this stack fails and the seven defences that worked) · the measured dead ends nobody should rebuild. **It points, it never copies** — every claim carries its FINDINGS/ROADMAP reference, and all 1245 repo links resolve with it in place.

⭐ **Drafted deliberately with its three open inputs NAMED INSIDE IT** (the task, the model confirmation, one camera-integrated collection run) — [ROADMAP §8.5](ROADMAP.md) predicted exactly these, and his ruling turned them from blockers into open-decision sections. ⬜ **What turns the draft into the deliverable: his read and ratification** — it is HIS plan to his team, and no agent ratifies that.

⚠️ **The one §8.5 caveat honoured rather than argued away:** §8.5 wanted "one complete run of the pipeline, even a bad one" before the plan. The joints-side pipeline HAS run end to end (record → label → composite → export, on his real slot-8 recording); the camera-integrated run has not, and the plan says so where it matters (WP6/WP7 and the C4 gate) instead of hiding it.

### 70.15 ✅⭐⭐⭐ ITEM 5 IS ANSWERED WITHOUT COVERING A LENS — the uniqueID hypothesis CONFIRMED, by the agent, and a permission belief corrected

✅ **The measurement (2026-08-19, all three cameras attached):** AVFoundation's `uniqueID` for a USB camera is **one 64-bit number in hex: locationID in the high 32 bits, then VID (16), then PID (16)**. Verified digit for digit against [§70.6](FINDINGS.md)'s ioreg capture on every camera: D405 `255323071773` → `0x122000080860b5b` (= `0x01220000`·`8086`·`0b5b`) · D405 `260323072846` → `0x121000080860b5b` · C920 → `0x1141000046d08e5`. **So the chain closes: USB serial → (ioreg, no root) → locationID → uniqueID → the AVFoundation device — two IDENTICAL D405s told apart with no lens-covering.** `yam/cameras/identity.py` implements it pure (`tests/test_camera_identity.py`, fixture cut from the real dump, 5/5), and it re-derives per session from live ioreg, so a replug into the same port changes nothing and a port change re-joins automatically.

⭐⭐ **A belief this corrects: camera ENUMERATION needs no macOS permission — only CAPTURING does.** The agent ran the AVFoundation device listing itself, from its own shell, cameras untouched. Every earlier "the agent can never run cameras" statement stands for capture and fell for enumeration; the identification question was agent-answerable all along, and nobody had split the two operations.

⚠️ **What is still one step from perfect:** OpenCV exposes no uniqueID, so uniqueID → OpenCV index still needs a measurement — the mode probe separates MODELS, and two same-model cameras need one physical confirmation (cover a lens) after which `camera_view`'s existing hint file (already keyed by uniqueID) pins it. With this chain, that confirmation is needed at most once per port arrangement, and the hint's identity survives replugs. On the rebuild's Ubuntu stack the whole question dissolves: librealsense reads serials directly.

### 70.16 ⚠️ THE RIG AS RE-READ ON 2026-08-19, and what the sweep found

Both CAN adapters on the bus running firmware, no DFU · **one SpaceMouse** (as he said; nothing currently built needs the second — it returns only when simultaneous two-hand teleop demos are wanted) · both D405s and the C920, same serials and ports as [§70.6](FINDINGS.md). ⛔ **No motor replies on either arm** (`online motors: []`, both) — the adapters run from USB, the motors from the wall, so the first suspect by this repo's own table is that **the arms' mains power is not on yet**. Nothing was retried past one ping per arm; mains is a physical action and is his. ⚠️ The register diff (`check_arms_match`) is queued behind mains for the same reason.

### 70.17 ⬜⭐⭐ THE CONSOLIDATED LIST OF EVERYTHING THAT IS HIS — ⚠️ SUPERSEDED by [§71.3](FINDINGS.md) after his bench pass ran items 1-3 on 2026-08-19

> Everything agent-runnable from the reconnected rig has been run ([§70.15](FINDINGS.md), [§70.16](FINDINGS.md)). What follows is his, in the order that unblocks the most. The second SpaceMouse is NOT needed — nothing built requires it; it returns only when simultaneous two-hand teleop demos are wanted.

**At the bench, ~15 minutes total:**

1. ⛔ **Turn on the arms' MAINS power** — both adapters answer, no motor does ([§70.16](FINDINGS.md)); motors run from the wall.
2. **Then the health pair:** `uv run apps/ping_motors.py --arm B --yes` · same for G · `uv run checks/check_arms_match.py --yes` (the register diff against the 2026-08-14 baseline).
3. **Cameras, two commands:** `uv run apps/camera_view.py --list` (the identification measurement; note which INDEX each camera got), then `uv run apps/capture_probe.py --indices <the three indices> --seconds 10 --save` — the multi-camera bandwidth measurement ([§70.9](FINDINGS.md)). ⚠️ `--cameras d405` will refuse as ambiguous with two D405s attached; indices are the way today.
4. **The grab re-run** on the Lego piece (the [§70.3](FINDINGS.md) sequence): the pause line now prints *"arm settled X rad off"* — at the friction floor (~0.02-0.04 rad) the pose was taught short, so re-save the two at-object waypoints a few millimetres further and it grips.
5. **One labelled recording:** `w` · move · `k` · move · `k` · `w` · save — then `uv run checks/check_recordings.py` shows the bad stretch ([§70.10](FINDINGS.md)).
6. **One composite grab:** save poses around the object, record the grab as a take, then `p <pose> w<take> <pose>` Enter Enter ([§70.12](FINDINGS.md)).

**Decisions, minutes each:** the noise bound ([ROADMAP §8.2](ROADMAP.md) item 9 carries the lean) · which arm stands on the bench's LEFT (the episode exporter requires it) · ⭐ **read [docs/PLAN.md](PLAN.md)** — his ratification turns the draft into the team's deliverable.

**From the team, whenever:** ABC's `export_mcap.py` or the `abc_minimal` repo (to match the episode encoding byte for byte), else the C4 mini-sample gate adjudicates.

**Standing, unchanged:** a private remote for this repo (needs his GitHub, ~5 minutes together) · the two old API keys from `AutonomousMAS/.env` (Mind Understanding `state/NOW.md` §4 item 2, open since 2026-08-06).

## 71 ⭐⭐⭐ 2026-08-19 — THE ARMS ARE BACK ON MAINS AND HIS PASS RAN CLEAN, ONE MEASUREMENT WAS LOST TO ARGUMENT FORMAT, AND ITEM 48 IS BUILT

### 71.0 ✅⭐⭐ HIS BENCH PASS: every motor healthy, the register diff byte-identical to the baseline, and the camera indices measured

✅ **Mains is on and both arms answer** — closes [§70.16](FINDINGS.md)'s blocker, exactly as diagnosed (adapters ran from USB, motors from the wall). His pings: **all 14 motors**, temperatures **26-28 °C** against the 55 °C warning, every motor at rest (worst velocity 3 quantisation steps from zero), **no latched fault on either arm with error clearing OFF** — so that is a real reading, not an erased one ([§39.1](FINDINGS.md)'s fix doing its job). Jaws: B reconciles with the usual −2π shift and sits 3.6% open; G needs no shift and sits 2.7% open. ⚠️ Both "only N% of closing travel left" warnings are the harmless nearly-closed case the tool itself names — the jaws were simply left closed.

✅ **The register diff is clean**: 140 reads, only `inertia` and `flux` differ (per-unit measured data, as established in [§38](FINDINGS.md)), and **every register on every motor reads exactly what the 2026-08-14 baseline recorded** — nothing has been written to any motor's flash in five days of sessions.

✅⭐ **The camera indices are measured for today's port arrangement**: C920 = **0**, MacBook = **3**, iPhone = **4** — and indices **1 and 2 are the two D405s BY ELIMINATION** (macOS lists five devices, five indices exist, three are identified by their unique modes, and both leftovers delivered 1280x720 colour). ⭐ **The [§70.6](FINDINGS.md) glance is CLOSED**: his `--list` output printed the uniqueID lines and they match [§70.15](FINDINGS.md)'s arithmetic digit for digit (`0x122000080860b5b` / `0x121000080860b5b` / `0x1141000046d08e5`). ⬜ **The one remaining unknown is which D405 serial is index 1 and which is 2** — and the probe's `--save` PNGs answer it by VIEWPOINT (one camera rides arm G), no lens-covering needed. [§71.3](FINDINGS.md) item 1 carries it.

### 71.1 ⛔⭐ THE BANDWIDTH MEASUREMENT WAS LOST TO ARGUMENT FORMAT — three refused spellings in a row, and the defect was the doc AND the parser together

⛔ **What happened, verbatim from his terminal**: `--indices 0, 1, 2` → `error: unrecognized arguments: 1, 2` · `--indices 0 1 2` → the same · `--indices <0, 1, 2>` → the shell ate the brackets. The flag took ONE comma-joined string (`0,1,2`), [§70.17](FINDINGS.md) item 3 said only *"--indices <the three indices>"*, and he dictates commands through speech-to-text — so every spelling he would naturally produce was refused, and the bench minute was spent on argparse instead of on the measurement.

✅ **Fixed at the parser, not the docs**: `--indices` and `--cameras` now take space- AND comma-separated forms alike (`yam/cameras/specs.py::flatten_tokens`, tested with his three spellings verbatim), and a non-numeric token is refused BY NAME. Proven by running all three of his exact command lines from the agent shell: each now parses and proceeds to the open step, which only his terminal can pass ([§61.3](FINDINGS.md)).

⭐ **The meta-lesson, one sentence**: a command handed to Julien must be PRE-FILLED VERBATIM — `<the three indices>` was a placeholder in exactly the position where a copy-paste command belongs, and his global working-contract rule 8 (pre-filled command blocks) already said so.

⛔ **Fixed in passing, found by reading**: `resolve_camera()` can hand back an ALREADY-OPEN capture (kept open to save a reopen), and `capture_probe.py` discarded that handle without releasing it — the configured reopen would then find the device busy. His owed `--cameras` run would have hit it; `--indices` never took that path.

### 71.2 ✅⭐⭐⭐ ITEM 48 IS BUILT — camera frames ride recordings, and a recording becomes an episode WITH images. The last named gap in the walkthrough is code now

✅ **What exists, exactly the [ROADMAP §8.2](ROADMAP.md) item 48 design ①-④**: a `--cameras` session flag (specs: `c920` by measurement · a raw index · `d405:<serial>` through the [§70.15](FINDINGS.md) identity chain + the hint file, refusing with the establishment instruction when no hint exists) · while `w` records, the loop samples `CaptureSet` and one writer thread per camera JPEG-writes `recordings/frames/<slot>/<camera>/<seq>.jpg` plus an `index.json` of `(seq, host_stamp_ns)` (`yam/cameras/writer.py`) · the recording's meta names the directory, the clock epoch and the per-camera counts, and `check_recordings.py` RE-COUNTS the frames from disk and flags orphaned directories · `yam/episode.py` joins frames to the 30 Hz ticks by nearest stamp and writes C3's `/top-camera` `/left-wrist-camera` `/right-wrist-camera`, with the role mapping REQUIRED exactly like the arm sides (`--top`/`--left-wrist`/`--right-wrist` on `export_episode.py`).

⭐⭐ **The traps, handled as named**: ① the writer never blocks the loop — hand-off by reference, drop-OLDEST on backpressure with a count that lands in the index, the meta, the stop line and the export warnings (a silent drop is the [§0](FINDINGS.md) pattern; dropping newest would pair current joints with stale pixels). ② `--sim` refuses `--cameras` as a pure tested function. ③ teardown flushes every index before the summary — the writer thread's loop shape IS the flush (it exits only on stop-AND-empty). Beyond the design: frames freeze on the SAME line the sampler stops ([§30.1](FINDINGS.md)'s rule extended to images), a discarded/aborted/quit take's frames are DELETED on every path, and a frameless save CLEARS the slot's stale frames so old images can never sit beside a new recording.

⭐ **The clock fact that makes the join exact, measured on this Mac**: `time.perf_counter` (the loop and sample clock) and `time.monotonic_ns` (the frame stamps) are the SAME clock (`mach_absolute_time()`), so `take_mono0`, stamped on the `w` keypress line, puts frames and samples on one axis with no cross-clock arithmetic. `yam/episode.py::nearest_frame_per_tick` is pure and tested against a non-zero epoch, so an absolute-time bug cannot pass.

✅ **Verified**: suite **743/743 across 33 files** — up from 730/730, and the 13 reconcile exactly (8 `test_frame_writer.py` + 5 new episode tests, including a REAL-JPEG round trip, deterministic drop-oldest arithmetic under a blocked encoder, and read-back of camera topics from a real MCAP file). `drive_sim_session` 29/29 unchanged (the frameless path is the sim path, by ②). `check_flags` green over the updated docs. `check_recordings` runs clean over the real 9 recordings.

⬜⛔ **What only the bench can say (design ⑤, unchanged)**: the encode+write cost of 3 cameras at ~30 fps against the 90 Hz loop (the writer reports drops honestly, the loop-rate warning already exists), whether the D405s deliver beside the C920 on one USB tree ([§34.5](FINDINGS.md)), and the first real camera-carrying take. **Then the pipeline is complete to C4's doorstep**: record with frames → label → export with roles → the team's loader adjudicates the encoding.

### 71.3 ⬜⭐⭐ THE HIS-LIST — ⚠️ SUPERSEDED by [§71.7](FINDINGS.md) after his second bench pass ran items 1, 2 and 4 the same morning

**At the bench, ~15 minutes total:**

1. ⭐ **The bandwidth measurement, now unblocked**: `uv run apps/capture_probe.py --indices 0 1 2 --seconds 10 --save` (today's measured indices: 0 = C920, 1 and 2 = the two D405s). It answers [§70.9](FINDINGS.md)'s three-camera question, and the two saved 1280x720 PNGs show which D405 is which by viewpoint — **say which index shows the arm-G view**, and the agent pins `config/camera_index_hint.json` so `--cameras d405:<serial>` works from then on.
2. **The first camera-carrying take**: `uv run apps/teleop_session.py --yes --arms B,G --cameras c920 --start-mode hold` · `w` · move · `w` · save to a slot · then `uv run checks/check_recordings.py` shows the 📷 line and `uv run apps/export_episode.py --slot <n> --left <arm> --right <arm> --top c920` writes the first episode WITH images. (All three cameras once step 1 has pinned the D405 hints.)
3. **The grab re-run** on the Lego piece ([§70.3](FINDINGS.md) sequence): the pause line now prints *"arm settled X rad off"* — at the friction floor (~0.02-0.04 rad) the pose was taught short, so re-save the two at-object waypoints a few millimetres further and it grips.
4. **One labelled recording:** `w` · move · `k` · move · `k` · `w` · save — then `uv run checks/check_recordings.py` shows the bad stretch ([§70.10](FINDINGS.md)).
5. **One composite grab:** save poses around the object, record the grab as a take, then `p <pose> w<take> <pose>` Enter Enter ([§70.12](FINDINGS.md)).

**Decisions, minutes each:** the noise bound ([ROADMAP §8.2](ROADMAP.md) item 9 carries the lean) · which arm stands on the bench's LEFT (the episode exporter requires it) · ⭐ **read [docs/PLAN.md](PLAN.md)** — his ratification turns the draft into the team's deliverable.

**From the team, whenever:** ABC's `export_mcap.py` or the `abc_minimal` repo (to match the episode encoding byte for byte — now including the camera-topic encoding), else the C4 mini-sample gate adjudicates.

**Standing, unchanged:** a private remote for this repo (needs his GitHub, ~5 minutes together) · the two old API keys from `AutonomousMAS/.env` (Mind Understanding `state/NOW.md` §4 item 2, open since 2026-08-06).

### 71.4 ✅✅⭐⭐⭐ HIS SECOND PASS, SAME MORNING: THE BANDWIDTH QUESTION ANSWERED, ITEM 48 CONFIRMED ON HARDWARE TWICE, AND THE FIRST EPISODE WITH IMAGES EXISTS

✅⭐⭐ **The bandwidth question ([§70.9](FINDINGS.md), item 6) is ANSWERED: all three cameras deliver 30.0 fps at 1280x720 simultaneously.** His `capture_probe --indices 0 1 2 --seconds 10 --save`: every camera 30.0 fps captured, mean gap 33.3-33.5 ms (the 30 fps period), worst gap 66 ms (one skipped frame, once), fresh ~40% of the 90 Hz samples (a 30 fps source can refresh at most ~1/3 of them). **The USB tree carries the full three-camera set with no bandwidth exhaustion at this resolution.** Report + one PNG per camera: `recordings/cameras/2026-08-19_103849_*`.

✅✅⭐⭐ **Camera frames INTO recordings ran on the arms twice, first try each** ([§71.2](FINDINGS.md) built it earlier the same day):
1. Slot 1: 5.1 s / 455 samples in TELEOP, **153 frames** (= 30 fps for 5.1 s), zero drops, saved as `1.json + frames/1/`.
2. Slot 2: 8.7 s / 777 samples **with `k` labels riding the same take** (4.9 s marked bad — his-list item on labels is closed too), **262 frames**, zero drops.
3. `check_recordings` showed both 📷 lines counted from disk, and `export_episode --slot 1 --left G --right B --top c920` wrote **the first episode WITH images**: 154 ticks, `/top-camera`, plus the honest warning that two wrist topics are missing. ⚠️ **The `left=G right=B` mapping is HIS ENTRY from that command and not yet confirmed as the standing bench layout** — it decides mirroring for every episode, so it needs his one-word confirmation before collection in anger.

⭐ **The loop-rate half of design point ⑤ is answered for one camera: the cost is invisible.** The camera sessions held 89-90 Hz; a no-camera session the same morning ran 83 Hz. The three-camera cost inside a session is the one number still open, and the capture side of it is now known-good.

⚠️ **The grab runs: the pause fired all three times, settle offsets 0.020-0.022 rad (exactly the [§69.2](FINDINGS.md) friction floor), and the jaws closed onto themselves — nothing gripped, three times.** Whether an object was present is his to say: if these were mechanics tests, everything behaved; if real attempts, the [§70.5](FINDINGS.md) advice stands (re-save the two at-object waypoints a few millimetres further into the piece).

⚠️ **A visible non-lever, so nobody pulls it again: he raised the park speed 0.98 → 1.50 rad/s and the run went 7.6 s → 6.5 s.** The `SafeRobot` cap of 1.0 rad/s binds below everything ([§65.0](FINDINGS.md)); raising the park speed past it buys nothing. The lever that would actually speed parks up is `max_speed` in the `n` settings screen, and it is a safety limit, his to raise.

### 71.5 ✅⛔⭐⭐⭐ WHICH D405 IS WHICH INDEX — PINNED FROM HIS ANSWER, AND THE HINT FILE WAS ALREADY POISONED IN EXACTLY THE WAY THE NEW GUARD NOW CATCHES

✅ **His viewpoint answer (2026-08-19): index 1 is the arm-G view, index 2 is the arm-B view.** Joined with the serial↔arm attribution (the G-mounted D405 is `260323072846` — his own bench word of 2026-08-14, [§34.5](FINDINGS.md); `255323071773` is the original arm-B camera) and the measured serial→uniqueID chain ([§70.15](FINDINGS.md)), re-verified against live ioreg before writing: `0x121000080860b5b → 1` and `0x122000080860b5b → 2` now sit in `config/camera_index_hint.json`. The full chain dry-runs from the agent shell: `d405:2603 → 260323072846 → 0x121… → index 1`.

⛔⭐⭐ **THE FIND: the hint file already carried BOTH D405 uniqueIDs → index 0 — and index 0 is the C920.** Stale rows from some earlier identification pass, harmless until yesterday, lethal after: the new `--cameras d405:<serial>` path trusts the hint (twins allow no verification of WHICH), so a session would have opened the WEBCAM under a wrist camera's name, recorded it, exported it, and nothing would have raised. The classic [§0](FINDINGS.md) shape, one config file away.

✅ **Two defences, both landed:**
1. The entries are corrected from his physical confirmation (above).
2. **The session now MODEL-CHECKS every hint-resolved index before trusting it** (`model_discriminating_mode` in `camera_view.py`): the opened device must answer a mode only the claimed camera's MODEL offers — `424x240` for the D405 on this rig, a mode the C920 does not have and the pixel probe already proved the real D405 answers over OpenCV ([§63.0](FINDINGS.md)'s run captured it). A stale hint now refuses with the re-establishment instruction instead of recording the wrong camera. Tested with the fixture twins (`tests/test_camera_render.py`).

⚠️ **The one case no software question can catch, named so nobody thinks it is covered: the two D405s' USB cables swapped between their same two ports.** uniqueIDs follow PORTS, so a swap silently exchanges the G-view and B-view names. It is physical, it needs a deliberate act at the hub, and the first glance at any recording's frames shows it.

⭐ **Serial prefixes work now, because he dictates**: `d405:2553` and `d405:2603` resolve (any unique prefix; ambiguity refuses and names the candidates). The recorded camera NAME always carries the FULL serial (`d405-260323072846`), so dataset names do not depend on how much of the serial was typed.

### 71.6 ✅⭐ THE SLOT PROMPT SHOWS THE WHOLE SHELF NOW — his save cost ELEVEN keypresses because every slot was occupied and the prompt revealed them one digit at a time

⛔ **What the log shows**: after his first take, he pressed 6·5·4·7·8·9·1·2·3·4·1·1 — nine one-slot warnings read one at a time before he found a slot he was willing to replace. Every fact those warnings revealed was known to `describe_slot` before the first digit.

✅ **Fix**: the save prompt prints one line per occupied slot (duration · arms · method · date) plus which slots are free, once, up front (`yam/recording.py::slot_overview`, both freeze sites, tested). The per-digit replace confirmation stays — it is the [§33.2](FINDINGS.md) overwrite guard, and the overview removes the hunting, never the guard.

### 71.7 ⬜⭐⭐ THE HIS-LIST — ⚠️ SUPERSEDED by [§72.5](FINDINGS.md) after his midday run closed the three-camera take and ran the composite on hardware

**At the bench:**

1. ⭐ **The three-camera take** (the wrist hints are pinned now): `uv run apps/teleop_session.py --yes --arms B,G --cameras c920,d405:2603,d405:2553 --start-mode hold` · `w` · move · `w` · save. `checks/check_recordings.py` should show three 📷 counts; the export then takes all three roles: `uv run apps/export_episode.py --slot <n> --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773` ⚠️ with the wrist flags matching whichever arm really stands on which side — see the decision below. This run is also the three-camera loop-rate number ([§71.4](FINDINGS.md)).
2. **One composite grab:** save poses around the object, record the grab as a take, then `p <pose> w<take> <pose>` Enter Enter ([§70.12](FINDINGS.md)).
3. **The grab with the object**, if the [§71.4](FINDINGS.md) runs were mechanics tests: re-save the two at-object waypoints a few millimetres further in; the settle print bounds the software's share.

**Decisions, minutes each:** ⭐ **confirm the bench sides** — his export used `left=G right=B` and every episode inherits it · the noise bound ([ROADMAP §8.2](ROADMAP.md) item 9 carries the lean) · **read [docs/PLAN.md](PLAN.md)** — ratification turns the draft into the team's deliverable.

**From the team, whenever:** ABC's `export_mcap.py` or the `abc_minimal` repo, else the C4 mini-sample gate adjudicates.

**Standing, unchanged:** a private remote for this repo · the two old API keys from `AutonomousMAS/.env` (Mind Understanding `state/NOW.md` §4 item 2).

## 72 ⭐⭐⭐ 2026-08-19, MIDDAY RUN — THE THREE-CAMERA EPISODE EXISTS, THE COMPOSITE RAN ON HARDWARE AND EXPOSED A REAL HANDOVER BUG, AND HIS GRIP QUESTION IS ANSWERED WITH THE BUG IN HAND

### 72.0 ✅✅⭐⭐ WHAT HIS RUN PROVED — the walkthrough's collection loop is complete on hardware

1. ✅⭐⭐ **The three-camera take ran first try**: slot 3, 7.3 s, 646 samples, **219 + 218 + 219 frames at ~30 fps each, zero drops** — and 646/7.3 = **88.5 Hz, so THREE writer threads cost the loop nothing measurable** (one camera ran 89-90 Hz, no camera ran 83 Hz the same morning). Design point ⑤ of item 48 is now fully answered on hardware.
2. ✅⭐⭐ **The full three-role export ran**: `recordings/episodes/3.mcap`, 218 ticks, `/top-camera` + `/left-wrist-camera` + `/right-wrist-camera` — **the first complete C3-shaped episode this project has produced.** Only the C4 gate (ABC's loader) remains between this and Gate C.
3. ✅⭐ **The composite ran on the real arms** — four take legs and pose legs in one 7-leg queue, tracking tables per playback, and the abandon machinery fired correctly when a leg was abandoned (2 queued legs dropped, counted). ⛔ It also exposed a real defect — [§72.1](FINDINGS.md).
4. ✅ The serial-prefix specs and the model check worked on the bench exactly as written (`model-checked at 424x240` on both D405s), the slot overview showed the whole shelf and the save took two keypresses (eleven yesterday), and the gripper stall latch fired twice in teleop and released correctly.
5. ⛔⭐ **The placeholder pattern bit AGAIN**: he typed `--slot <3>` with literal angle brackets because [§71.7](FINDINGS.md)'s command carried `--slot <n>`. Two instances in two days ([§71.1](FINDINGS.md) was the first) make it a rule: **a command handed to Julien contains only typeable text — a real example value, never a bracketed placeholder.** [§72.5](FINDINGS.md) is written that way.

### 72.1 ⛔⭐⭐⭐ THE COMPOSITE HANDOVER BUG — a playback began with arm B 1.28 rad AWAY from the recording's start, and the sim had been showing it invisibly all along

⛔ **What his log shows, line by line**: after the pose leg (waypoints 1→2→3→1), the take leg for recording 2 announced B's park (1.29 rad of travel) — and in the SAME instant printed *"arm B is at the start pose; waiting for G"*. G's 0.00 rad park arrived a cycle later, the playback started, and B was dragged 1.28 rad to catch up (worst lag 1.294 rad, clock held 1.2 s), while B's still-running park was cancelled as abandoned and the composite dropped its remaining legs.

⭐⭐ **The mechanism, confirmed in the code**: an arrival event advanced the composite queue, the queue armed the take leg *inside that same event*, and the arrival then fell through into the ready-check and was credited as "this arm reached the recording's start" — when what had actually arrived was the POSE leg's park at waypoint 1. Only pose-leg→take-leg transitions hit it, which is why the first three take legs (playback→take, no arrival in flight) ran clean.

⛔⭐⭐ **The sim had reproduced it from day one and the checks were blind**: `recordings/sim/last_drive.log` shows the same instant false credit (B announced at the start pose with 1.44 rad of travel, ONE arrival before PLAYING) — and the 29 checks still passed, because they asserted the take-park was *announced*, never that every arm *arrived* before playing. The [§0](FINDINGS.md) pattern in a checker: green while validating the wrong thing. ⚠️ The abandon cascade is timing-dependent, which is why two morning sim runs passed and the midday one failed "composite completed" — the same underlying defect, flickering.

✅⭐⭐ **Fixed with two independent defences, falsified before trusted:**
1. **Every park carries its PURPOSE** (`replay` · `composite` · `operator`), stamped when it begins and taken at its arrival — the ready-credit now requires the arrived park to have been the playback's own park-to-start, so no other leg's arrival can ever be mistaken for it.
2. **The gate MEASURES before playing**: when the bookkeeping says every arm is ready, each replay arm's ARM joints (jaws excluded — a jaw holding an object sits off on purpose) are checked against the recording's start pose, and anything beyond 0.25 rad (`REPLAY_START_TOLERANCE` — parks arrive within ~0.05, settle within ~0.05 more) refuses the playback loudly instead of dragging the arm.
3. The sim driver gained the checks that would have caught this a week ago: every replay arm must ARRIVE (two "PARK reached" between the take-park announcements and PLAYING) and the start-pose guard must never fire in a healthy run. **Falsified: 29/31 on the pre-fix code, 31/31 after.**

⚠️ **Safety accounting, honestly**: the arm was never in free flight — `max_lag` clamped every command to 0.25 rad ahead of the measured pose, so the "jump" was a bounded ratchet-drag, exactly what that layer exists for. What the bug DID poison is data and grips: a playback that starts off-pose grips off-pose.

### 72.2 ✅⭐ HIS GRIP QUESTION, answered with the audit he asked for — what changed, what did not, and what the two-of-three grips mean

His observation: the playback grips used to be very consistent, and this run gripped twice with only one really good. **The audit:**

1. ⛔ **Yes — something built since then made one playback worse: [§72.1](FINDINGS.md).** The composite is new since his consistent era, and a playback that starts 1.28 rad off-pose grips wherever it happens to pass. Found, fixed, double-guarded. This is the one concrete regression the history supports.
2. ✅ **The playback machinery itself is unchanged** since the consistent era (`replay_step` untouched; the park merge was his own "feels exactly as before" pass, [§69.0](FINDINGS.md)). The only jaw-path change is the settle-gate, and it was read end to end today: it keeps commanding the SAME split pose while it waits ([session.py's step_path]), so it can delay a grab by ~half a second but cannot move where the jaws close.
3. ⭐ **His own hypothesis stands as the rest of the answer: the recording's quality.** The consistent grips replayed a HAND-GUIDED take; today's grips replayed teleop-driven takes recorded while working the cameras. Same playback fidelity, different taught path.
4. ⚠️ **The floor under everything stays [§69.2](FINDINGS.md)**: every arrival settles 0.02-0.04 rad short (up to ~1.5 cm at the elbow), direction not repeatable, so millimetre grabs sit AT the noise floor — that is why "not millimetre perfect" is the honest expectation and kp (item 17) is the one real lever. ⭐ A small observation from his own logs, worth one line: settle offsets track approach speed (0.020-0.022 rad at 0.40 rad/s park speed; 0.033-0.039 at 1.50) — teaching AND grabbing at the same, lower speed is free consistency.

### 72.3 ✅⭐ RECORDING A PARK RUN — it has always worked, the one key he tried was taken by the composite syntax, and now the prompt says so

His ask: record the waypoint run itself, so the park feature produces demonstrations. **The flow exists**: press `w` BEFORE `p` — the sampler records through every mode including PARK (mode switches mid-take are hardware-proven, slot 9 carries `teleop+guide`), cameras and labels ride along, and the take's `modes` field says the arm was in PARK. What he pressed instead was `w` INSIDE the park prompt, which by design means "the next digit names a take leg" (the composite syntax) — so the park prompt now states both meanings, and [COMMANDS.md](COMMANDS.md)'s dataset section documents the collection loop: teach waypoints once, then `w` · `p 1 2 3` Enter · `w` · save, with the scene reset between runs. ⭐ **This IS the automated collection lane**: one taught sequence, many recorded episodes with variation coming from the scene.

### 72.4 ⭐⭐ THE TEAM IS BUILDING NOW — their main re-read 2026-08-19, and one live contract risk found

Two PRs landed on `Hohnik/LaRobot` main since the 2026-08-18 exploration, the second the same morning as this session: a **working MuJoCo simulation** of the ABC `put_bottles` scene (30 Hz ticks, top/left/right cameras — ABC's own `camera_keys`) and a `docs/ARCHITECTURE.mmd` declaring their plan: an `Input` interface (Keyboard · Spacemouse · **Policy**) and a `Robot` interface (Real · Sim) — the same seams [PLAN.md](PLAN.md) names. Their `inputs/mcap_recording.py` is an **empty file**, so this walkthrough's episode exporter is the only running C3 implementation anywhere. ⛔ **The one live risk: their `Observation` carries the gripper in METRES and our episodes carry it NORMALISED 0..1, and C3 names the ee dim without a unit** — flagged in [PLAN.md](PLAN.md) §4; ABC's `export_mcap.py` adjudicates at C4. Full re-read: [ROADMAP §10.6](ROADMAP.md). ⭐ Also written this session, at his ask for levelled documents: **[ARCHITECTURE.md](ARCHITECTURE.md)** — the layer between the plan and the code (the five shapes, the five base ideas, the module map), now in the README's read order. His ruling on the doc system, recorded: the PLAN's pointer style is right, FINDINGS' linking is right, and more detail belongs in MORE documents at DIFFERENT levels, not in longer ones.

### 72.5 ⬜⭐⭐ THE HIS-LIST, current — supersedes [§71.7](FINDINGS.md); every command below is typeable verbatim

**At the bench, when he wants:**

1. ⭐ **One automated collected episode — the full loop in one run** (also re-proves the [§72.1](FINDINGS.md) fix on hardware): start `uv run apps/teleop_session.py --yes --arms B,G --cameras c920,d405:2603,d405:2553 --start-mode hold`, teach or keep waypoints, then `w` · `p 1 2 3` Enter · `w` · save to slot 4 · `uv run apps/export_episode.py --slot 4 --left G --right B --top c920 --left-wrist d405-260323072846 --right-wrist d405-255323071773`. A composite variant (`p 1 w3 1` Enter Enter) also re-runs the fixed handover with real distances.
2. **Grip consistency, if he wants it better**: teach and run at one LOW speed (the settle offsets in his own logs are half as large at 0.4 rad/s as at 1.5), re-teach the grab from a hand-guided take rather than teleop, and the remaining ~1 cm is the [§69.2](FINDINGS.md) friction floor — kp (item 17) is the one lever past it, his call.

**Decisions, minutes each:** ⭐ **confirm the bench sides in one word** — both his exports used `left=G right=B`, every episode inherits it · the noise bound ([ROADMAP §8.2](ROADMAP.md) item 9) · **ratify [docs/PLAN.md](PLAN.md)** (he has read it: *"the plan in general looks good"* — one word makes it the deliverable).

**For the colleagues (his call on timing):** the private remote (~5 minutes together, his GitHub) is the best vehicle — it ships [PLAN.md](PLAN.md), [ARCHITECTURE.md](ARCHITECTURE.md) and the whole evidence base in one link; a chat-drafted message to the team exists in the session transcript of 2026-08-19.

**From the team, whenever:** ABC's `export_mcap.py` or the `abc_minimal` repo (episode encoding byte for byte, camera topics included, and the gripper-unit question — [§72.4](FINDINGS.md)).

**Standing:** the two old API keys from `AutonomousMAS/.env` (Mind Understanding `state/NOW.md` §4 item 2).
