# Archived robot source explanations

Verbatim passages from `216fda0`, moved September 16, 2026. These preserve
historical reasoning and may contain superseded claims. Current contracts live
in src/yam/robot.py; see [CLEANUP](../CLEANUP.md) for current evidence.

## MODULE at source lines 1–25

```text
"""Build and safely tear down a whole-arm YAM robot on macOS.

Everything that talks to all seven motors goes through here, so the startup and
shutdown rules live in exactly one place.

TWO PROBLEMS THIS SOLVES, both found on real hardware 2026-08-10.

**1. The gripper re-calibrated on every single startup.**
`linear_4310.yml` has `gripper_limits: null`, so `get_yam_robot()` runs
`detect_gripper_limits` each time it is constructed. That routine applies a
constant 0.5 Nm and waits for the position to *stop changing* — i.e. it drives
the jaws into each hard stop and holds them there. Julien: *"they move really
quickly and quite hard… they seem to crash into the ends and then seem to try to
push further."* He is right, and the fix is not to soften a routine that runs
forever — it is to run it **once**, gently, and cache the answer. Supplying
`gripper_limits_override` is what turns the auto-detection off
(`get_robot.py:223-225`).

**2. Teardown raced itself.** `MotorChainRobot.close()` prints *"Robot closed
with all torques set to zero"*, but it only calls `motor_chain.close()`, which
shuts the bus **without disabling the motors** — while the chain's own 250 Hz
control thread is still mid-transaction. The observed result was an exception
storm from a dying thread and no certainty that anything was disabled.
Order matters: **stop the control thread → disable the motors → close the bus.**
"""
```

## comment at source lines 45–59

```text
#: ⭐⭐ THE CEILING ON EVERY COMMANDED JOINT SPEED, and the single most consequential
#: number in this file. `SafeRobot` clamps every command from every mode to this, BELOW all
#: control logic — so `MAX_PLANNED_JOINT_SPEED` (1.5), `park_speed` (up to 1.5) and any
#: playback multiplier are all above it and never bind
#: ([FINDINGS §37.0](../docs/FINDINGS.md)).
#:
#: ⚠️ IT IS A SOFTWARE LIMIT, NOT THE HARDWARE'S. Julien's own hand-guided recordings reach
#: **2.4 to 3.7 rad/s**, so the motors do those speeds; only this refuses to command them
#: ([§37.2](../docs/FINDINGS.md)).
#:
#: ⛔ RAISING IT IS JULIEN'S DECISION, one step at a time, watching `SafeRobot.max_lag`
#: (0.25 rad; the worst ever measured is 0.181) rather than temperature — what cooks these
#: motors is holding still against a stop, not moving. `teleop_session.py --max-speed 1.5`
#: raises it for one session without changing this default, the same way `--reach` and
#: `--floor` expose the workspace limits.
```

## comment at source lines 62–74

```text
#: ⛔⭐⭐ HOW FAR THE COMMAND MAY RUN AHEAD OF THE MEASURED POSE. **This is the limit that
#: actually caps tracking**, and until 2026-08-15 nothing had a name for it and no flag
#: reached it ([FINDINGS §57.3](../docs/FINDINGS.md)).
#:
#: ⭐ A MIRROR follower cannot be pulled closer by more `max_speed`: the gap it measures is
#: `(leader − command) + (command − measured)`, and this clips the second term. So with a
#: 0.35 rad trip limit the leader only has to get 0.10 rad ahead of the command.
#:
#: ⚠️ IT IS ALSO A TORQUE LIMIT, which is why raising it is a real decision rather than a
#: tuning knob. The motor's push is `kp × (command − measured)`, so a bigger allowance means
#: a harder push to catch up. `SafeRobot`'s own docstring already says the pair of limits are
#: deliberately loose for this reason: squeeze the following error too hard and the arm cannot
#: overcome its own friction, and loosen it and the arm hits harder when it meets something.
```

## comment at source lines 77–89

```text
#: ⭐ The hard cap on the velocity-feedforward gain (item 44). 1.0 sends exactly the
#: rate-limited command's own speed, which is the physically-motivated value — and the one
#: Julien felt as precise (*"one basically doesn't change the direction at all"*).
#: ⛔⛔ THE CAP WAS 3.0 FOR ONE DAY AND THAT RANGE IS A MEASURED DEAD END — do not raise it
#: again without reading [FINDINGS §68.6](../docs/FINDINGS.md). Above 1.0 the velocity
#: setpoint CONTRADICTS the position command by construction: the motor is told to move
#: faster than the trajectory it is also told to hold. Raw, that was continuous jitter and
#: a visible spring-back at release; with the past-the-command gate it became 5-10 Hz
#: stepping ("like the motor's vibrating"), including during parks. Two bench sessions,
#: same verdict: *"just not that usable."* Any fix that smooths gain 3 converges to gain-1
#: behaviour, so 1.0 IS the fix. More responsiveness, if ever wanted, is a kp/kd tuning
#: question ([ROADMAP §8.2](../docs/ROADMAP.md) item 17's caveats), never more gain here.
#: ⛔ Kept in mechanical sync with `settings.LIVE_BOUNDS["vel_ff"]` by a test.
```

## comment at source lines 99–126

```text
# ⛔⭐ THE MASS THAT MADE THE ARM FALL, 2026-08-10.
#
# `GripperType.NO_GRIPPER` does not merely leave motor 7 unenabled — it swaps the
# *dynamics model* used for gravity compensation. The bare arm XML gives its
# terminal `gripper` body `mass="1e-6"` (yam.xml:38), and the real mass is merged
# in from the gripper XML, which NO_GRIPPER replaces with a stub. Summing
# linear_4310.xml's inertials: body 0.553219 + two fingers at 0.0710042 =
# **0.695 kg**, at the very end of the arm.
#
# Measured consequence, in simulation, at the saved park pose:
#
#     gravity torque WITH gripper    [-0.00, -4.81, 6.34, 1.34, -0.07, -0.00] Nm
#     gravity torque WITHOUT gripper [-0.00, -2.67, 3.88, 0.49, -0.00,  0.00] Nm
#     joint 3 (elbow_pitch) short by  +2.47 Nm  =  39% of what it needs
#
# In `zero_gravity_mode=True` the constructor sets **kp = 0** and commands zero
# torque (motor_chain_robot.py:241), so `motor_torques = gravity_comp` alone
# (:366). There is no position term to take up a shortfall — the missing 2.47 Nm
# is an unopposed torque pulling the elbow down, and the arm folds forward. That
# is exactly what happened: GUIDE mode was entered with --no-gripper and the arm
# sank while the status line reported a calm 35 °C for 33 seconds.
#
# Passing `ee_mass` restores it: worst residual falls 2.465 -> 0.188 Nm (3% of the
# elbow's requirement), verified in simulation.
# ⚠️ `ee_inertia` is NOT usable — the SDK writes an `ipos` attribute that MuJoCo
# rejects ("Schema violation: unrecognized attribute: 'ipos'", should be `pos`).
# That is a bug in the vendored tree, so only the mass can be corrected, and the
# 0.188 Nm residual is the centre-of-mass offset we cannot express.
```

## frame_correct_gripper_limits at source lines 158–194

```text
    """Express saved jaw limits in the frame `get_yam_robot()` will actually use.

    ⭐ THIS IS THE REAL FIX. An earlier version compared the saved limits against
    the RAW motor position and, finding them consistent, passed them through
    unchanged — which was wrong, because the robot does not use the raw position.

    Two shifts are involved and they are easy to conflate:

    **(a) The calibration frame.** `calibrate_gripper.py` records limits through a
    bare `DMChainCanInterface`, which applies no wrap correction. If the jaws were
    somewhere different when it ran, the recorded numbers can be a whole 2π out.

    **(b) The runtime frame.** `get_yam_robot()` adds ±2π to `motor_offset` at
    every construction, chosen from the motor's momentary position
    (`get_robot.py:268-274`), and every reported position is then `raw − offset`.
    **The limits are NOT given the same treatment**, so unless we apply it here,
    positions and limits live in frames 2π apart.

    Worked example, measured 2026-08-10:

        jaws raw                6.3235
        6.3235 > π  ⇒ runtime reports  6.3235 − 2π = 0.0403
        saved limits            [6.481, 1.231]        (un-shifted)
        normalised = (0.0403 − 6.481) / (1.231 − 6.481) = 1.227   ← outside [0,1]

    A normalised position outside [0,1] is clipped back to the nearest limit by
    `motor_chain_robot.py:390`, which commands the motor into a stop it is already
    past. That is what cooked motor 7 three times.

    With the runtime shift applied the limits become [0.198, −5.052] and the same
    position normalises to **0.030** — comfortably inside. As a cross-check, that
    range is what the very first calibration of the day measured independently:
    [0.0704, −5.0528].

    Returns None only if no ±2π placement brackets the jaws at all, which means
    the jaws really are outside their measured travel and a re-calibration is due.
    """
```

## comment at source lines 199–214

```text
    # (a) put the recorded range in the same wrap frame as the raw reading
    #
    # ⛔⭐⭐ IT REFUSES WHEN MORE THAN ONE SHIFT FITS, rather than taking the first.
    #
    # Each candidate shift accepts raw positions in `[lo + k − margin, hi + k + margin]`,
    # a window of `travel + 2·margin`. The candidates are 2π apart. **So if the jaws'
    # measured travel ever exceeds `2π − 2·margin` (5.683 rad at margin 0.3), two windows
    # overlap and both shifts "fit" a position in the overlap.** Picking the first would be
    # picking a jaw SCALE by list order, and a wrong scale is what commanded the gripper
    # 2.6 rad past its stop and cooked motor 7 (see `reconcile_gripper_limits` below).
    #
    # ⚠️ MEASURED 2026-08-14: arm B's travel is 5.250 rad and arm G's is 5.228, so each has
    # about 0.43 rad of headroom and a scan of every raw position from −10 to +10 rad finds
    # exactly ONE matching shift everywhere. **The guard is dormant today**, which is why it
    # has to be a refusal rather than a comment: a wider re-calibration would silently
    # re-introduce the choice.
```

## reconcile_gripper_limits at source lines 229–255

```text
    """Shift saved jaw limits into the frame the jaws are actually in, or give up.

    ⛔ THIS IS THE FIX FOR THE WORST BUG OF 2026-08-10, and the mechanism is worth
    understanding because it will recur anywhere raw motor positions are cached.

    `get_yam_robot()` applies a **±2π wrap correction at every construction**,
    chosen from wherever the motor happens to be sitting at that instant
    (`get_robot.py:268-274`). `calibrate_gripper.py` builds a `DMChainCanInterface`
    directly and gets **no** such correction. So the limits are written in one
    coordinate frame and read in another, and whether they happen to agree depends
    on the jaws' position when each ran.

    When they disagree, the consequence is not a wrong number — it is a cooked
    motor. `motor_chain_robot.py:390` force-clips every gripper command into
    `[min(limits), max(limits)]` **regardless of where the jaws are**. With the
    jaws at −1.380 and the range at [+1.231, +6.481], the gripper was commanded
    2.6 rad away and held there against a mechanical stop. 43 °C → 65 °C in five
    seconds.

    So: try the saved range shifted by 0, +2π and −2π. If one brackets the
    measured position, that is the same physical range expressed in this session's
    frame — return it, and nothing needs to move. If none does, the limits are
    genuinely stale and **None** is returned so the caller re-measures.

    ⚠️ Never "warn and continue" here. That is precisely what was done, and it is
    what burned the motor.
    """
```

## advance_park_command at source lines 268–297

```text
    """One cycle of a park trajectory: move the COMMAND toward the target.

    ⛔⭐ THE BUG THIS FIXES, and it matters because it is completely invisible.

    PARK used to command `measured + clip(target - measured, ±step)` — it
    re-anchored to where the arm actually was, every single cycle. The commanded
    position was therefore never more than **one step** ahead of reality:
    `0.4 rad/s × 0.01 s = 0.004 rad`, about **0.23°**.

    A position controller makes torque from the *error* between command and
    measurement. Capping that error at 0.23° caps the torque at `kp × 0.004`, which
    does not overcome static friction plus 4.3 kg of arm. So the arm does not move;
    because it does not move the measurement does not change; and because the
    measurement does not change the next cycle commands the same 0.23° offset.
    **A treadmill.** It printed "parking… 1.2 rad to go" indefinitely while the
    number barely moved, raised nothing, and read as a controller that was merely
    slow. Julien reported PARK broken twice before this was found.

    TELEOP never had the bug, and that contrast is the proof: it integrates from
    `prev_q`, the **previously commanded** target, never from the measurement. When
    the arm lags, its command keeps advancing, the error grows, and the torque grows
    with it until the joint moves.

    This does the same. The command runs ahead of the arm as far as it needs to, and
    `SafeRobot.max_lag` (0.25 rad) is what stops it running away — exactly the right
    place for that guard, and already there.

    ⚠️ Completion must therefore be judged from the **measured** position, not from
    this command. The command arrives first, always.
    """
```

## park_target_from at source lines 311–337

```text
    """Build a PARK target from a saved pose that may not match this robot's shape.

    Returns `(target, warning_or_None)`. Pure, so it can be tested without an arm —
    which is the point, because the bug it fixes could only be found by reading.

    ⛔ TWO REAL DEFECTS LIVE HERE, both found on 2026-08-10 by reading the code.

    **1. A length mismatch used to drop the arm.** `config/park_pose.json` holds 7
    joints. Run with `--no-gripper` and the robot has 6, so the old
    `park_target - measured` raised `ValueError` — and that exception escaped the
    control loop, skipped the "the arm is HOLDING, press g or d" consent flow, and
    fell into `finally`, which **disables the motors**. A raised arm sags. And
    `--no-gripper` is precisely the escape hatch the gripper instructions tell you
    to fall back to, so the fallback was the broken path. Symmetrically, a pose
    saved *in* a no-gripper session has 6 entries and broke a later 7-DoF session.
    Fixed by starting from the measured pose and overlaying only the joints the
    saved pose actually carries: never invent a target for a joint we know nothing
    about.

    **2. PARK was the one path that bypassed the gripper clamp.** It commanded the
    saved jaw value directly, so a pose saved with the jaws resting on a mechanical
    stop would drive them back onto that stop and *hold* them there. Holding a
    position at a stop is stall torque — full current, no motion, no cooling — which
    is exactly how motor 7 cooked three times (FINDINGS §4, rule 1: never command
    the gripper to hold at a hard stop). `clamp` is applied here so no caller can
    forget it.
    """
```

## park_speed_factor at source lines 363–387

```text
    """How much of the park speed to use right now — a trapezoidal ramp, in [floor, 1].

    Ease in over the first `ramp` radians, cruise, ease out over the last `ramp`. On a
    move shorter than `2 * ramp` the profile degenerates to a triangle, which is the
    correct thing for a short hop rather than a special case.

    ⭐ WHY THIS IS A SEPARATE FUNCTION AND NOT A CHANGE TO `advance_park_command`.
    That function is confirmed on hardware, has 15 tests, and its one job — *advance
    the COMMAND, never the measurement* — cost two sessions to get right. Smoothing
    does not need to touch it: this returns a scale factor and the caller multiplies
    its step. The trajectory integrator stays exactly as it is.

    ⚠️ AND THAT SHAPE REMOVES THE RISK I ORIGINALLY WORRIED ABOUT. The plan in
    ROADMAP 6.5 said easing should be opt-in because *"a deceleration bug shows up as
    overshoot — the arm arriving somewhere it was not aimed"*. That is true of a new
    integrator carrying velocity state. It is **not** true here: `advance_park_command`
    is `command + clip(target - command, -step, step)`, so a step is already bounded
    by the distance that remains. **Scaling that step DOWN cannot overshoot**, only
    slow down. Checked by reading it rather than assumed, which is why this is on by
    default with `--no-smooth` as the escape hatch.

    ⚠️ `floor` matters: without it the factor reaches zero at both ends and the arm
    creeps for ever, which the stall detector would eventually — and wrongly — call
    an obstruction.
    """
```

## resolve_park_legs at source lines 395–408

```text
    """Turn typed digits into `(legs, missing)` — the poses to visit, in order.

    `"0"` always means the **base** pose, whatever it is called in the file. Anything
    else is a waypoint. A digit with nothing saved behind it lands in `missing` and is
    **skipped**, never substituted.

    ⛔ Why skipped and not substituted: the tempting alternative — fall back to the
    base when a waypoint is empty — would send the arm somewhere the operator did not
    ask for, in the middle of a sequence they are watching. A pose the arm moves to is
    never a default.

    Duplicates are kept: `p 1 2 1 Enter` visits slot 1, slot 2, then slot 1 again,
    which is the obvious reading and makes a there-and-back trivial to type.
    """
```

## park_verdict at source lines 422–464

```text
    """`"arrived"` · `"settled"` · `"blocked"` · `"moving"` — has the park finished?

    ⛔⭐ THE KNIFE EDGE THIS REMOVES, seen on hardware 2026-08-12. In one session the
    interleaved park reported **"PARK reached (0.020 rad off)"** and in the next the
    Ctrl-C park reported **"PARK STALLED — 0.021 rad still to go"**. Same arm, same
    pose, same code — and `PARK_TOLERANCE` is **0.02**. The arm was landing either
    side of the threshold by a thousandth of a radian.

    **That is not a fault, it is the noise floor.** A position-controlled arm holding
    itself against gravity has a steady-state error: the controller settles where its
    stiffness balances the load, a fraction of a degree short of the commanded pose.
    `park_target_from`'s own comment says so — *"it must allow for a position
    controller's steady-state error"* — and 0.02 rad (1.1°) turns out to sit right at
    it rather than safely above it.

    ⭐ **The fix is not a bigger number.** Simply loosening the tolerance would also
    make a genuinely obstructed arm look parked. What actually separates the two
    cases is **how far away it stopped**:

    - stopped improving and *close* → the controller has arrived as near as it can.
      That is `"settled"`, and it is a success.
    - stopped improving and *far* → something is in the way, or the pose is
      unreachable. That is `"blocked"`, and it must never be treated as arrival —
      it is the case that protects a hand or a clamp in the arm's path.

    So the threshold that was doing two jobs is split into two thresholds, each doing
    one. `tolerance` means "arrived cleanly"; `settled_band` means "close enough that
    the remaining error is the controller, not an obstruction".

    ⭐ `stopped_briefly` is a SECOND, much shorter timer, and adding it fixed a real
    annoyance. Julien, 2026-08-12: *"I quite dislike how long it takes for the last
    waypoint to be reached — it's nearly there, but then it moves millimetre by
    millimetre really slowly until it actually reaches its position."*

    He was watching a **wait**, not a crawl. `settled` was gated on the same 4-second
    stall timer as `blocked`, so every park that finished outside the 0.02 tolerance —
    which is most of them, because the controller settles ~0.04 rad short under load —
    sat apparently doing nothing for four seconds before admitting it had arrived.

    The two questions deserve different patience. *"Has the controller finished
    settling?"* is answered in a fraction of a second. *"Is something blocking the
    arm?"* deserves four. Splitting them costs one argument and removes the wait.
    """
```

## park_slots at source lines 477–488

```text
    """Every saved pose for `arm`, keyed by slot name.

    ⭐ Julien's design, 2026-08-12: *"it would also make sense to have more options to
    save more positions … hit s and then a number every time we wanna save a position,
    and then hitting p and then the number would park to that position."*

    ⚠️ **Accepts the legacy shape.** `config/park_pose.json` used to be
    `{"B": [q0, q1, …]}` — one pose per arm, no slots — and that file is *measured
    calibration* that exists on the rig right now. A format change that silently
    dropped it would cost bench time to recreate, so a bare list is read as the
    `default` slot and keeps working exactly as before. `q p d` is unaffected.
    """
```

## motor_temperatures at source lines 512–528

```text
    """`(temps, hottest, jaw)` from a chain's state list. Pure, so it can be tested.

    Each motor reports two temperatures — MOSFET and rotor — and the one that
    matters is whichever is higher, so they are combined per motor rather than
    picked between.

    ⚠️ `hottest` is **None** for an empty list, never 0.0. A temperature of zero is
    a plausible-looking reading that no motor in a warm room ever produces, and
    inventing one is how a thermal guard gets quietly disarmed — see `ThermalGuard`.

    ⭐ The gripper is returned SEPARATELY and is deliberately not folded into
    `hottest`. Motors 2 and 3 carry the arm's 4.3 kg and sit at 41-42 °C in normal
    equilibrium while an idle motor 7 is 31-36 °C, so a gripper climbing 33 → 41 °C
    is completely hidden behind the shoulder in a `max()`. Watching motor 7 plateau
    is the actual test of the 2π frame fix, and a test that cannot see the thing it
    tests is not a test (FINDINGS §0).
    """
```

## ThermalGuard at source lines 545–579

```text
    """Turns motor temperatures into a decision — **including when it cannot read them.**

    ⛔⭐ THE DEFECT THIS REPLACES, found by reading on 2026-08-12 and never yet
    triggered on hardware, which is the only reason it is not in FINDINGS §0's table.

    The session used to read temperatures inside a bare `try`, and its handler was:

        except Exception:
            temps, hottest, jaw_temp = [], 0.0, None

    So **any** failure of `chain.read_states()` — a CAN hiccup, a decode error, a
    short state list — set the hottest motor to **0 °C**. The comparison
    `if hottest >= TEMP_STOP` then could not fire, thermal protection was gone for
    that cycle, and the status line printed a calm `hottest 0°C`. Nothing warned.
    If the read failed persistently the session would run to completion with no
    thermal guard at all, reassuring the operator the whole way.

    That is three of this repo's own rules at once: it warns-and-continues past a
    hazard (working contract rule 4), it is a guard with a path straight around it
    (rule 7), and it fails by lying rather than by crashing (FINDINGS §0). Motor 7
    has been cooked three times on this rig; the thermal guard is not decoration.

    **So: a failed read is not a temperature.** Blindness is reported the first time
    it happens and becomes a *stop* if it persists — because a session that cannot
    see temperatures has lost the thing standing between the gripper and a stall
    burn, and continuing is a decision nobody made deliberately.

    ⭐ It also actually issues the warning the session has always advertised.
    `TEMP_WARN = 55.0` was printed in the startup plan — *"temperature : warn 55°C,
    stop 65°C"* — and, verified by an exhaustive grep on 2026-08-12, **was used
    nowhere else in the codebase.** The session promised a warning it could not
    give. Same defect class as the refusal that named the wrong arm (FINDINGS §16):
    the text is right, the behaviour is absent, and only a user at the bench finds
    out.
    """
```

## build_robot at source lines 687–700

```text
    """Construct the real robot. Returns `(robot, note)`.

    ⛔ This ENERGISES ALL SEVEN MOTORS and starts a 250 Hz control loop.

    `zero_gravity=False` (default) commands the arm to hold the pose it is
    already in — success looks like nothing happening. `zero_gravity=True` makes
    it back-drivable for hand-guiding, which is a genuinely different and looser
    physical state and should be asked for explicitly.

    `allow_calibration=True` permits the jaw-limit detection to run. Off by
    default precisely so it cannot happen by accident: it is a real motion into
    both mechanical stops, and once `config/gripper_limits.json` exists it never
    needs to happen again.
    """
```

## comment at source lines 708–733

```text
        # ⛔⭐ DEFAULT: DO NOT CONTROL THE GRIPPER. This is the only reliable fix
        # for the failure that cooked motor 7 three separate times on 2026-08-10.
        #
        # The jaws end up PHYSICALLY OUTSIDE the calibrated range -- measured, the
        # normalised position read **1.186**, i.e. 18.6% beyond "fully open".
        # `command_joint_pos` clips that back to 1.0, which maps to the end stop,
        # so the motor is commanded into a stop it is already past and pushes at
        # 7.71 Nm indefinitely. Worse, it is self-reinforcing: 7.7 Nm shoves the
        # jaws even further beyond the 0.3 Nm calibration's idea of the limit, so
        # every cycle makes the next one worse.
        #
        # No clamp above this layer can help, because the vendor's clip happens
        # BELOW it (`motor_chain_robot.py:390`). Releasing the command to the
        # measured value does not help either -- 1.186 clips to 1.0 all the same.
        #
        # With NO_GRIPPER the motor is never enabled and never commanded. Its own
        # 400 ms timeout leaves it damped and free. The six arm joints are
        # entirely unaffected, so teleop works completely.
        #
        # Re-enabling gripper control needs the calibration reworked to measure
        # limits at the torque the RUNTIME uses, not at 0.3 Nm. See docs/FINDINGS.md.
        #
        # ⛔ `ee_mass` IS NOT OPTIONAL HERE. NO_GRIPPER swaps the gravity-compensation
        # model as well as dropping motor 7, and without this the elbow is short by
        # 39% of its holding torque — which in GUIDE mode (kp=0) drops the arm. See
        # GRIPPER_MASS_KG above for the measurement and the incident.
```

## comment at source lines 792–798

```text
        # ⛔ THE ARM NAME MUST BE IN THE COMMAND. This message used to read
        # "uv run scripts/calibrate_gripper.py --yes" with no --arm, so following it
        # literally would re-calibrate B — driving the WRONG arm's jaws into both
        # mechanical stops — while the arm you were actually trying to start stayed
        # uncalibrated and the same refusal came back. Julien hit exactly this on
        # G's first run. A remediation message that names the wrong target is
        # worse than no message: it converts a clean refusal into a wrong action.
```

## SafeRobot at source lines 827–865

```text
    """A rate limiter that sits BELOW all control logic. Wraps any I2RT robot.

    ⭐ WHY THIS EXISTS — Julien, 2026-08-10, after the arm snapped:

        *"maybe there should be some safety mode where specific high speed
        movements just aren't possible… it would have to go over a more low
        level control before the output actually gets sent, because I'm guessing
        the actual snapping mistake wasn't an actual control that you sent, it
        was a misprogramming. So any type of safety would have to go lower than
        that."*

    He is exactly right, and the diagnosis was correct: the snap was not a
    commanded motion, it was a **stale cached variable** (`prev_q` was not reset
    when TELEOP was re-entered, so the first command after hand-guiding aimed at
    the pose from minutes earlier). No amount of care *inside* the teleop loop
    protects against a bug *in* the teleop loop. The guard has to be somewhere
    the buggy code cannot reach around.

    So every command passes through two independent limits here:

    1. **Rate limit on the command itself** — the commanded position may not move
       more than `max_speed · dt` per call, whatever it is asked for. A caller
       that suddenly demands a pose one radian away gets a ramp, not a jump.
    2. **Following-error limit** — the command may never run more than
       `max_lag` away from the *measured* position. This is the one that makes it
       genuinely low-level: it is anchored to physical reality rather than to any
       internal state, so it holds even if every variable above it is wrong.

    ⚠️ Why not clamp against the measured position alone (which would be simpler
    and stricter): the PD term is `kp · (command − measured)`, so a tight
    following-error limit is also a torque limit. Squeeze it too hard and the arm
    cannot overcome its own friction and goes sluggish. Two loose limits that
    each catch a different failure beat one tight limit that also breaks normal
    operation.

    This cannot prevent a *slow* wrong motion — nothing at this level can know
    that a direction is wrong. It bounds how fast anything can go wrong, which
    is what turns "dangerous" into "catchable".
    """
```

## comment at source lines 872–877

```text
        # ⭐ Velocity feedforward gain, 0.0 = OFF = exactly the old behaviour
        # (ROADMAP §8.2 item 44, FINDINGS §66.1). At f > 0 the motors are sent
        # `f ×` the rate-limited command's own derivative as their velocity
        # setpoint, so torque flows before position error builds. ⛔ OFF by
        # default because it changes what 4.3 kg does; Julien enables and
        # tests it slowly (his go, 2026-08-18). Live-editable like max_speed.
```

## comment at source lines 912–917

```text
        # ⭐ Velocity feedforward (ROADMAP §8.2 item 44). The DM motors' MIT-mode frame
        # carries a velocity setpoint and this stack always sent zero, so all torque had
        # to come from position error — the measured `0.033 s × speed` lag is that,
        # structurally (FINDINGS §66.1). ⭐ The derivative of the LIMITED command is used,
        # never the caller's raw target, so the feedforward can never ask for a speed the
        # rate limiter above just refused: |vel| ≤ max_speed × vel_ff by construction.
```

## comment at source lines 934–944

```text
        # ⛔⭐ NEVER PUSH A JOINT THAT IS ALREADY PAST ITS COMMAND (FINDINGS §68.3, from
        # Julien's jitter report at gain 3). Above gain 1 the motor is deliberately told
        # the target moves faster than it does, so the joint OVERSHOOTS the rate-limited
        # command; without this gate the setpoint kept pushing while the position term
        # pulled back — the forward jitter he felt. And at release the arm had been
        # driven PAST the command, so the position term visibly pulled it back, his
        # exact "it controls it back the other direction" question. The gate: a joint
        # whose position error opposes its setpoint gets zero push, IMMEDIATELY (the
        # smoothing memory is cleared too, so a crossing cuts hard while a release still
        # decays softly). At gains ≤ 1 the gate almost never engages, which leaves the
        # physically-exact setting unchanged.
```

## shutdown_robot at source lines 979–993

```text
    """Stop cleanly and return the motor IDs actually confirmed disabled.

    ⛔ Order is the whole point.

    1. **Stop the control thread.** It runs at 250 Hz and will otherwise be
       mid-`set_control` when the bus closes underneath it, which produced a
       thread-death traceback on the first real run.
    2. **Disable the motors, while the bus is still open.** `close()` does not do
       this, despite announcing that it has.
    3. **Then** close.

    Returns the IDs whose `motor_off` genuinely succeeded — not a fixed list.
    Reporting "all motors disabled" without checking is the same class of lie
    this codebase has produced all day.
    """
```

## check_grasp at source lines 1034–1065

```text
    """Did the gripper close on an object, or on itself?

    ⭐⭐ WHY THIS IS WORTH HAVING, AND WHY IT IS FREE. Two separate plans need the robot to
    know whether it succeeded, without a person watching:

    1. **Throwing failed episodes out of a dataset.** A failed grab is worse than a wasted
       minute: it is a bad demonstration labelled as a good one, and it trains the model on
       nothing ([ROADMAP.md](../docs/ROADMAP.md) §6.6).
    2. ⭐ **ENPIRE's whole method is a loop of reset, run, VERIFY, improve**
       ([ROADMAP.md](../docs/ROADMAP.md) §9.3). Automatic verification is a module of it, not
       a nicety, and this is the cheapest verification this rig can produce.

    ⭐ **And it needs no new hardware or per-object calibration.** The jaws are position
    controlled, and `get_joint_pos()[6]` is already a **normalised** opening, 0 closed to 1
    open, measured against the limits found by calibration. Calibration closes the jaws onto
    *themselves*, so 0 means empty. Command them shut and they stop at the object's width.
    **"Did it grab something" is therefore a number we already read every cycle.**

    ⚠️ WHAT IT CANNOT TELL YOU, and each of these is why `confident` exists:

    - **It cannot tell a good grab from an awkward one.** It reports that something is
      between the jaws, nothing more.
    - **A jammed or obstructed gripper reads exactly like a held object.** Same signal.
    - **It says nothing unless the jaws were told to close.** Commanding 0.5 and measuring
      0.5 carries no information, which is what `closed_enough` guards.
    - ⛔ **It is meaningless before the jaws stop moving.** Mid-close looks identical to
      holding a wide object, so the caller must pass `settled=True` and it is the caller's
      job to know. See `plan_gripper_stops` in `src/yam/motion.py`, which is where a run waits.

    `hold_threshold` is 0.03 of a 96 mm stroke, so roughly 3 mm. Below that the gap is
    sensor noise and the controller's own steady-state error rather than an object.
    """
```
