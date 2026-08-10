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
| `arm1` | `2081337C594E5018` | everything up to ~11:00 was this one |
| `arm2` | `20593383594E5018` | plugged in mid-session; **its adapter enumerated FIRST** |

⛔ **Never select the adapter by index.** `chain_channel('arm1')` returned `gsusb1` at 10:58 and `gsusb0` at
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
- **Measured limits (arm1, 2026-08-10): `+0.0704 … −5.0528`, usable stroke 5.123 rad = 78% of declared.**
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
  a stop is stall torque: full current, no motion, no cooling. Commands are clamped to **[0.15, 0.85]**.

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
PARK mode does exactly that at 0.25 rad/s.
⚠️ What it *cannot* know is what is now in the way, which is why it moves slowly and any key aborts it.

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
- **SpaceMouse axis directions are wrong/unintuitive.** Not yet mapped to Julien's expectation; the session
  script lets him flip x/y/z live and saves the result to `config/spacemouse_map.json`.
- **Two SpaceMice are now connected, and the ambiguity is SOLVED — by asking the hardware.** Both report an
  empty serial, so select-by-serial does not transfer from the CAN adapters. They differ only in USB
  `port_numbers` — `(1,3)` and `(1,4)` — which hidapi does not expose, and which tell a human nothing about
  which puck is under which hand. So `pick_device_by_wiggle()` opens all of them and uses **whichever one the
  operator moves.** Unambiguous, needs no config, survives replugging into any port, and costs five seconds.
- No git remote. Everything exists only on this Mac.
