# Historical IK, mode and frame source notes

Archived September 16, 2026 from commit `f346a8d92c9c3fb4d07c2ba1eff610f0009cc56b`.
These are exact replaced passages, including obsolete interface and measurement claims.
Use the current source contracts and [BRIDGE](../BRIDGE.md) for present behavior.
Historical measurements retain their original scope; this archive is not a new verification.

## src/yam/teleop.py:1-25 (module)

```python
"""Cartesian teleoperation: a 6-DoF twist in, joint targets out.

This is the piece `docs/Setup-Plan.md` §4.2 says we have to build ourselves, and
the single largest deviation from the papers. ABC/ENPIRE/RoboTTT all teleoperate
with GELLO leader arms, which are passive copies of the robot and therefore hand
over **joint angles** directly. A SpaceMouse hands over a **cartesian twist** of
the end effector instead, and there is no fixed mapping between the two — it
depends on the arm's current configuration. Computing it is inverse kinematics:

    twist  →  integrate into a target EE pose  →  IK  →  joint targets  →  arm

⭐ Deliberately robot-agnostic. It talks to anything exposing I2RT's `Robot`
interface, which `get_yam_robot(sim=True)` and `get_yam_robot(sim=False)` both
do. The same code therefore drives the simulated arm and the real one, and
moving between them is a flag rather than a rewrite — so a sign error costs
nothing the first time it happens.

WHY THE TARGET POSE IS INTEGRATED, NOT COMMANDED DIRECTLY
---------------------------------------------------------
A SpaceMouse reports *deflection*, i.e. a velocity command, not a position.
Releasing it returns to zero, which must mean "stop", not "go home". So the
target pose is state that we integrate the twist into: push and the target
drifts, release and it stays where it was. This also makes a deadman trivial —
zero deflection integrates nothing.
"""
```

## src/yam/teleop.py:46-49 (# ⭐ The frames)

```python
# ⭐ The frames a twist can be expressed in. `world` is the original behaviour and
# stays the default; the others exist because Julien wants to drive while watching a
# wrist camera, and *"push forward"* then means forward IN THE IMAGE, which turns
# with the wrist. See `CartesianTeleop._twist_to_world()`.
```

## src/yam/teleop.py:58-65 (CartesianTeleop)

```python
    """Integrates a twist into an EE pose and solves IK for joint targets.

    Costs are the tuning surface. `position_cost` and `orientation_cost` trade
    translation accuracy against rotation accuracy; `posture_cost` is a weak pull
    toward a reference configuration that resolves the redundancy and, more
    importantly, keeps the solver away from singular configurations where joint
    velocities blow up.
    """
```

## src/yam/teleop.py:72-90 (# ⛔⭐ 0.05, NOT)

```python
        # ⛔⭐ 0.05, NOT 0.5. Lowered 10x on 2026-08-11 after measuring, and the
        # measurement is counter-intuitive enough to be worth keeping:
        #
        #   pos:ori    pure-roll tool wander    rotation achieved
        #   1.0:0.5          0.443 m                  7.9 deg     <- the old default
        #   1.0:0.2          0.034 m                129.5 deg
        #   1.0:0.05         0.002 m                134.6 deg     <- now
        #   1.0:0.01         0.000 m                 18.2 deg
        #
        # The old default was the WORST OF BOTH: it wandered 44 cm *and* achieved
        # the least rotation. A higher orientation cost produced LESS rotation,
        # because the effort went into satisfying an unreachable orientation by
        # translating, which drags the arm into a configuration that can rotate
        # even less. Verified at three different starting poses; small rotations
        # are unaffected and translation reach is unchanged (0.319 -> 0.320 m).
        #
        # The priority this encodes, in one line: **never sacrifice where the tool
        # IS to chase where it POINTS.** A wrist that cannot turn should simply not
        # turn — it should not drag the whole arm across the desk.
```

## src/yam/teleop.py:117-119 (# Joint limits enforced)

```python
        # Joint limits enforced inside the QP, so the solver never proposes a
        # velocity that would drive a joint past its stop. Cheaper and safer than
        # clamping afterwards, which would silently distort the requested motion.
```

## src/yam/teleop.py:125-127 (# ⛔⭐ ANTI-WINDUP)

```python
        # ⛔⭐ ANTI-WINDUP. How far the integrated goal may run ahead of the pose the
        # arm has actually achieved. This is `SafeRobot.max_lag` one layer up, and it
        # exists for a measured reason — see `step()`.
```

## src/yam/teleop.py:131-131 (# ⭐ WHICH FRAME)

```python
        # ⭐ WHICH FRAME THE PUCK'S TWIST IS EXPRESSED IN. See `_twist_to_world()`.
```

## src/yam/teleop.py:136-139 (# ⭐ Slow the puck)

```python
        # ⭐ Slow the puck down when the arm cannot keep up. See `_apply_speed_scale`.
        # 0.9 rad/s sits just under SafeRobot's 1.0 cap, so the twist is reined in
        # BEFORE the rate limiter has to intervene — the limiter stays a guard rather
        # than becoming part of normal operation.
```

## src/yam/teleop.py:142-146 (#: ⭐ The joint rate)

```python
        #: ⭐ The joint rate the last IK step ASKED for, before throttling — the measured
        #: quantity behind any SLOWED message. The old message asserted "near the reach
        #: limit" as the cause and was wrong on a comfortable pose (FINDINGS §41.2);
        #: showing this number instead lets the operator tell a singular pose (spikes
        #: only when extended) from an over-eager linear speed (high everywhere).
```

## src/yam/teleop.py:152-157 (CartesianTeleop.reset)

```python
        """Seed the IK state from the arm's *measured* joint positions.

        Always seed from reality rather than from zero: the IK's notion of where
        the arm is has to match where it actually is, or the very first solve
        commands a jump.
        """
```

## src/yam/teleop.py:168-172 (CartesianTeleop.step)

```python
        """Advance by one control cycle. Returns joint targets for joints 1-6.

        `twist` is [vx, vy, vz, wx, wy, wz] — linear m/s and angular rad/s,
        expressed in the world frame.
        """
```

## src/yam/teleop.py:180-183 (# World-frame integration)

```python
        # World-frame integration: rotation pre-multiplies, so a twist means the
        # same thing regardless of how the gripper happens to be oriented. Body
        # frame would be more natural to a hand holding the puck, and is a
        # deliberate later choice — not something to leave ambiguous now.
```

## src/yam/teleop.py:206-243 (CartesianTeleop._apply_speed_scale)

```python
        """Throttle the puck when the arm physically cannot follow it.

        ⛔⭐ THE PROBLEM, measured 2026-08-11. Julien: *"at high speeds the arm takes
        longer to follow the path that it's been told to move… I can only really
        control it at speeds of less than half a meter per second."*

        Pushing +X at 0.25 m/s from the home pose, watching the Jacobian's smallest
        singular value (`sigma_min`, which measures how close the arm is to a
        configuration where some direction of motion becomes unreachable):

            cycle  20   joint 0.68 rad/s   moved 0.05 m   sigma_min 0.170
            cycle 100   joint 1.34 rad/s   moved 0.25 m   sigma_min 0.121
            cycle 140   joint 2.93 rad/s   moved 0.35 m   sigma_min 0.048
            cycle 180   joint 0.12 rad/s   moved 0.38 m   sigma_min 0.005   (stalled)

        **The requested joint speed is not constant — it escalates as the arm
        extends.** The same 0.25 m/s at the tip costs 0.68 rad/s in the middle of the
        workspace and 2.93 rad/s near full reach, because as `sigma_min` collapses
        the arm must move its joints ever faster to produce the same tip motion.
        `SafeRobot` caps commands at 1.0 rad/s, so beyond that point the command is
        throttled, the arm falls behind, and it feels like latency.

        ⚠️ So this is **not really a speed problem**. Speed only decides how quickly
        you reach the part of the workspace where it happens. Raising the cap would
        not fix it either — it would just move the wall, at the cost of the guard
        that makes a wrong motion catchable, on a rig with no e-stop.

        **The fix is to ask for less.** If the last cycle wanted more joint speed
        than allowed, scale the twist by exactly the ratio; because tip speed and
        joint speed are locally proportional, that lands on the allowed rate in one
        step. Recovery is deliberately slower than reduction (5% per cycle, so ~0.2 s
        to return to full) — reacting instantly in both directions would oscillate at
        the boundary, which would feel worse than the lag it replaces.

        The result is that the arm slows down smoothly near its limits instead of
        silently lagging, and `speed_scale` is reported so the operator can see it
        happening rather than wonder why the arm feels heavy.
        """
```

## src/yam/teleop.py:254-281 (CartesianTeleop._twist_to_world)

```python
        """Re-express the puck's twist in the world frame, from whichever frame it
        was meant in. `world` returns it untouched.

        ⭐ WHY THIS EXISTS. Julien, 2026-08-11, wanting to mount a webcam on the arm:
        *"I can try to learn to control the arm from the point of view of the camera
        to get the tilts right and stuff."*

        World-frame control means "forward" is a fixed direction on the desk, however
        the wrist happens to be turned. That is the right default when you are looking
        AT the arm — it is predictable, and a wrong sign only nudges. But it is the
        wrong thing entirely when you are looking THROUGH a camera on the wrist:
        there, "push forward" means forward *in the image*, and the image turns with
        the wrist. `teleop.py` flagged this from the start as *"a deliberate later
        choice — not something to leave ambiguous now"*. This is that choice, made.

        The maths is small: a twist meant in frame F is `R_wf @ v` and `R_wf @ ω` in
        world coordinates, where `R_wf` is F's orientation in the world. Integration
        stays world-frame, so the anti-windup and the workspace box are untouched.

        ⚠️ **`camera` uses the MODELLED D405 mount** — the MJCF puts it on the flange
        at a 25° cant with `+Z` along the optical axis (ROS/OpenCV convention). That
        is correct for the real wrist cameras when they arrive and **wrong for a
        webcam cable-tied on by hand**, whose mounting transform nobody has measured.
        For the C920 stand-in use **`tool`**, mount the camera roughly looking the way
        the gripper points, and dial the rest out with the axis map. Using `camera`
        for an unmeasured mount would be inventing a transform, which is the single
        most repeated failure in FINDINGS.
        """
```

## src/yam/teleop.py:303-345 (CartesianTeleop._limit_lead)

```python
        """Stop the integrated goal running away from the pose actually achieved.

        ⛔⭐ THE BUG THIS FIXES — Julien, 2026-08-11: *"the inverse kinematics being
        weird and not working as intended, specifically when the robot gets into
        weird positions, and then it starts moving very, very incoherently."*

        `step()` advances `self.target` by the twist **unconditionally**. It never
        asks whether the arm followed. Measured in simulation, commanding pure roll
        at 0.6 rad/s from the park pose:

            t=4 s   target-vs-achieved gap 0.0004 m   tool point moved 0.000 m
            t=8 s   target-vs-achieved gap 0.238  m   tool point moved 0.238 m
            t=12 s                                    tool point moved 0.290 m

        **A pure rotation command moved the tool point 41 cm.** The chain is:

        1. A wrist joint hits its limit — the tight ones are ±1.5708. (Confirmed:
           the IK-vs-command gap pins at exactly 0.0800 rad, which *is*
           `JOINT_LIMIT_MARGIN`, so a joint is clamped at the margin while the IK
           believes it is at the true limit.)
        2. The orientation goal keeps integrating anyway, so it runs arbitrarily
           far past anything reachable.
        3. The QP now holds an impossible orientation target, and because
           `position_cost` (1.0) and `orientation_cost` (0.5) are traded against
           each other, it starts **moving the tool point to partially satisfy the
           unreachable rotation**.
        4. The workspace box then re-clamps translation, which fights the
           orientation task — hence the oscillation in the measurement above.

        So the arm does something the operator never asked for, in a direction that
        has no relation to the puck. That is exactly "incoherent".

        ⭐ The fix is the same idea as `SafeRobot.max_lag`, one layer up: a goal may
        only lead reality by a bounded amount. Translation and rotation are limited
        separately because they fail independently — the workspace box already
        happened to bound translation, which is why only rotation misbehaved.

        ⚠️ This deliberately does NOT try to detect singularities or predict
        reachability. It does not need to: whatever the reason the arm cannot
        follow — joint limit, singularity, rate limiter, a stalled motor — the
        symptom is the same, an unclosable gap, and bounding the gap bounds every
        one of those cases with no model of why.
        """
```

## src/yam/teleop.py:368-373 (CartesianTeleop.lead)

```python
        """How far the goal is currently ahead of the achieved pose: (metres, rad).

        Exposed so the session can show it. A goal that sits pinned at the limit is
        the signature of an arm that cannot follow, and that is worth seeing rather
        than inferring from the arm behaving oddly.
        """
```

## src/yam/teleop.py:386-427 (#: ⭐ THE WORKSPACE LIMIT)

```python
#: ⭐ THE WORKSPACE LIMIT, replacing a ±0.30 m cube on 2026-08-14. Julien's decision,
#: after the cube was measured stopping him at 71% of the arm's reach.
#:
#: `REACH_LIMIT` is a distance from the arm's base. The old cube re-centred on wherever
#: TELEOP was entered, so the wall sat somewhere different every session and nothing on
#: screen said where. His words, 2026-08-13: *"it stops moving in the direction I want it
#: to move even though the arm hasn't even close to fully extended."* Measured: he was
#: stopped 0.524 m from the base against a reachable 0.738 m (FINDINGS §41.1).
#:
#: ⛔⭐⭐ `FLOOR_LIMIT` EXISTS BECAUSE THE CUBE WAS QUIETLY PROVIDING ONE. A cube centred
#: on a tip at z = 0.475 bounded the tip above z = 0.175. **A bare sphere has no floor at
#: all**, and this arm can put its tip at **z = −0.377**, which is below its own base. So
#: swapping the cube for a radius alone would have removed real protection while looking
#: like a pure improvement. Working-contract rule 4: never continue past a hazard you have
#: correctly identified.
#:
#: ⛔⭐⭐ THE FLOOR IS 0.0 — EXACTLY THE BASE PLANE — AND IT TOOK TWO CORRECTIONS FROM
#: JULIEN TO GET THERE. Both were right and both are worth keeping, because the pair of
#: them defines what this number is allowed to be.
#:
#: **It shipped at +0.05 m.** He caught it before it ran: *"the bottom floor five
#: centimeter thing… sounds problematic because then I can't really pick anything up from
#: the table anymore."* A floor above the desk stops the tip short of everything lying on
#: it, and picking things up off the desk is what the rig is for.
#:
#: **I then over-corrected to −0.10 m**, reasoning that the floor should only bound a gross
#: plunge. He caught that too: *"ten centimeter below doesn't make any sense because then
#: it's still gonna crash into the table. So maybe do, like, one millimeter above or
#: something… or just do exactly on the base."*
#:
#: ⭐⭐ **He is right both times, and the two objections bracket the answer exactly.** Too
#: high forbids the task. Too low permits driving into the desk. **The base plane is the
#: only defensible value**, because the arm is bolted to the desk, so the desk is at or
#: just below z = 0 and the tip reaching z = 0 is a tip touching the desk.
#:
#: ⚠️ **It is his choice and it is meant to be tried**: *"we can test around with it
#: later."* If a flat object turns out to need a few mm below, `--floor -0.005` is one flag.
#: ⛔ Do NOT raise it above 0 again. That is the mistake this comment exists to prevent.
#:
#: ⭐ What it still protects against: this arm can otherwise put its tip at **z = −0.377 m**,
#: well below its own base. Every park pose sits at z ≥ 0.174, so the limit never
#: interferes with `q p d`. ⚠️ Measuring the true desk height remains open (ROADMAP §8.4).
```

## src/yam/teleop.py:432-449 (#: ⛔⭐⭐ HOW FAR PAST)

```python
#: ⛔⭐⭐ HOW FAR PAST THE STARTING POSE TO OPEN A WIDENED LIMIT, and it must not be zero.
#:
#: `effective_limits()` widens a limit to include the pose TELEOP started from. The first
#: version widened it to *exactly* that distance, which left an arm starting outside
#: sitting precisely on the wall — so the clamp fired on every single cycle.
#:
#: That is a knife edge, and this repo has been cut by one before: the park stall check
#: passed at 0.020 and stalled at 0.021 against a 0.02 tolerance (FINDINGS §26).
#:
#: ⛔ **And clamping every cycle has a known consequence.** A position clamp fights the
#: orientation task in the QP, which is written down in `_limit_lead`'s own notes: *"the
#: workspace box then re-clamps translation, which fights the orientation task — hence the
#: oscillation."* Measured: commanding pure roll from a folded pose moved the tool point
#: **0.178 m** with the limit on the wall, against under 0.002 m with room to spare.
#:
#: ⭐ 0.05 m because that is `max_lead_m`, the distance the goal is already allowed to run
#: ahead of the arm. A limit closer than one lead-length sits inside the controller's own
#: slack and will chatter by construction.
```

## src/yam/teleop.py:455-479 (effective_limits)

```python
    """Widen the limits, if needed, so they contain the pose TELEOP started from.

    ⛔⭐⭐ WHY THIS FUNCTION EXISTS AT ALL, and it is the one real hazard in swapping a
    moving limit for a fixed one.

    The old cube re-centred on the arm at the moment TELEOP began, so the arm was always
    at the exact centre and **the cube could never be entered from outside.** A fixed
    limit can be. If the arm is already at 0.65 m when `t` is pressed, clamping to a
    0.60 m sphere would command it 5 cm inward **the instant TELEOP starts**, with
    nobody having asked for it. An unrequested move at mode entry is the shape of
    several defects in this repo already.

    So the limits open just far enough to include the starting pose, and stay there for
    that session. ⚠️ **Deliberately no decay back to the nominal limit as the arm comes
    in.** A limit that moves during a session is what was wrong with the cube, and
    trading one moving wall for another would be a poor exchange for the small gain.

    ⭐ It needs no new state: `home_ee` is already the tip position at TELEOP entry, so
    the widening is derived rather than remembered. That also keeps the field earning
    its place after the cube stopped using it.

    Returns `(reach, floor)` unchanged whenever the starting pose is already inside,
    which is every pose the arm actually rests in — the furthest park pose on record
    puts the tip 0.433 m out and 0.306 m up.
    """
```

## src/yam/teleop.py:492-500 (clamp_to_workspace)

```python
    """Pull a tip position back inside the reach sphere and above the floor.

    Returns the position unchanged when it is already inside. Pass the limits through
    `effective_limits()` first, or an arm that started outside will be yanked.

    ⚠️ This clamps position only. It says nothing about speed. `_apply_speed_scale`
    handles the arm slowing near awkward configurations, and the two are independent —
    FINDINGS §41.2 is what happened when a message confused one for the other.
    """
```

## src/yam/teleop.py:514-518 (workspace_room)

```python
    """How much room is left: (metres of reach used, metres above the floor).

    ⭐ Reported on the status line every second. The cube was invisible, which is why
    hitting it read as the arm refusing to move rather than as a limit being reached.
    """
```

## src/yam/teleop.py:524-528 (scripted_twist)

```python
    """A slow horizontal circle, for validating IK with no input device involved.

    Debugging IK and debugging a HID decoder at the same time is how you end up
    unable to tell which one is wrong.
    """
```

## src/yam/session.py:20-25 (#: ⛔⭐⭐ HOW FAR THE JAWS)

```python
#: ⛔⭐⭐ HOW FAR THE JAWS MUST OPEN BEFORE A LATCHED BLOCK IS RELEASED, in normalised jaw
#: units where 0 is closed and 1 is fully open. **3% of full travel.**
#:
#: ⚠️ Chosen to sit far above sensor jitter and far below any deliberate movement. His
#: gripper-step default is 0.02 per keypress and the puck-button rate moves faster than that,
#: so one intentional open press already clears it while a wandering measurement never will.
```

## src/yam/session.py:60-62 (#: ⛔ Pushing hard)

```python
#: ⛔ Pushing hard while not moving is the definition of a stall, and stall is the worst
#: thermal case there is: full current, no motion, no cooling. Motor 7 was cooked three
#: times before this guard existed. Values copied from `teleop_session.py`.
```

## src/yam/session.py:67-74 (#: ⭐⭐ THE JAW PAUSE)

```python
#: ⭐⭐ THE JAW PAUSE (ROADMAP §6.6.2, items 3 + 10). A run splits wherever only the jaws
#: move, the arm holds at the split, and the run resumes when the jaws are DONE — which is
#: measured, never timed, because "how long the jaws take" is not a preference. A jaw that
#: stalls on an object counts as done: stopped-on-the-object IS the grab succeeding.
#:
#: ⚠️ All four are in NORMALISED jaw units (0 closed, 1 open) and seconds, because the pause
#: watches `get_joint_pos()[6]`, which is normalised. The raw-rad stall constants above watch
#: a different instrument (torque + raw velocity from the chain read) and stay separate.
```

## src/yam/session.py:200-210 (ArmSession)

```python
    """Own one arm's measurements, mode state, map, jaw latch and park execution.

    Controls-mode state lives here; key dispatch and selection live in the
    operator and UI handlers. SessionInput fills puck state. SessionHealth
    uses the thermal/stall methods and reports their decisions. Recording,
    playback and mirror coordination span arms and have session owners.

    The operator still applies per-mode commands and teleop clamps. SafeRobot
    applies its command limits beneath those modes; CartesianTeleop owns the
    workspace constraint. Device lifetimes belong to SessionResources.
    """
```

## src/yam/session.py:281-284 (# ⭐ The settle-gate)

```python
        # ⭐ The settle-gate before a jaw pause: the cursor finishing is not the arm
        # arriving, and jaws that close while the arm creeps its last millimetres close
        # in the wrong place (his 2026-08-18 grab missed exactly this way). `None` means
        # no gate is open; a value is the best arm lag seen since the gate opened.
```

## src/yam/session.py:289-292 (# ⛔ TWO CLOCKS)

```python
        # ⛔ TWO CLOCKS. `park_leg_t` resets at every waypoint so each leg reports its own
        # duration; `park_start_t` never resets so the arrival line can report the whole
        # park. Sharing one variable printed "PARK reached in 0.0s" after a 4.4 s park,
        # because the last waypoint is passed at the very end. FINDINGS §34.3.
```

## src/yam/session.py:316-320 (#: ⭐ Set to the block)

```python
        #: ⭐ Set to the block value when the latch CLEARS, so the loop can say so once. A
        #: latch that silently comes and goes is impossible to tell apart from a latch that
        #: never worked, which is precisely the ambiguity his 2026-08-17 log left behind:
        #: three stalls at 0.117, 0.098 and 0.104, and no way to know whether they were three
        #: deliberate squeezes or one latch being cleared twice by noise.
```

## src/yam/session.py:349-358 (ArmSession.alive)

```python
        """Is this arm still actually being commanded?

        ⛔ The single most important check. I2RT's control thread raises and exits on
        a motor fault and tells nobody; without this the loop commands a corpse while
        printing healthy numbers, which it did for 64 seconds on 2026-08-10.

        ⚠️ With N arms this becomes per-arm, and ROADMAP step 6 already ruled on what
        it means: **a fault on one arm stops BOTH.** A chain death on B must not leave
        G uncommanded and sagging.
        """
```

## src/yam/session.py:423-428 (ArmSession.resync)

```python
        """⛔ Re-anchor every cached variable to the measured pose.

        A mode change must re-read reality. Never carry cached state across one —
        `prev_q` surviving a hand-guide is what made the arm snap back to a pose from
        minutes earlier the first time GUIDE → TELEOP was tried.
        """
```

## src/yam/session.py:439-444 (ArmSession.enter_teleop)

```python
        """Leave zero-gravity and take the jaws exactly where they are.

        ⛔ Do NOT clamp the gripper here. Clamping on entry is a *command to move*,
        and nobody asked for that — an earlier version did, and if the jaws happened
        to sit outside the band the session drove them the moment teleop began.
        """
```

## src/yam/session.py:456-463 (ArmSession.enter_guide)

```python
        """Go weightless. Returns a warning string if the API is missing.

        ⛔⭐ UNDERSTAND WHAT THIS RESTS ON. Zero-gravity sets **kp = 0**, so the
        computed gravity compensation is the ONLY thing holding 4.3 kg up — there is
        no position term to absorb an error. Any shortfall in the model is an
        unopposed torque, which is how the arm fell on 2026-08-10. `guide_ref` is
        recorded here precisely so drift is measurable while it happens.
        """
```

## src/yam/session.py:475-475 (ArmSession.guide_drift)

```python
        """How far the arm has sunk since it went weightless, in radians."""
```

## src/yam/session.py:538-545 (ArmSession._arm_err)

```python
        """Worst ARM joint distance — the jaw is deliberately excluded.

        ⛔ The jaw legitimately sits far from its command whenever it holds an object
        (that is what a successful grab IS), so counting it would stall the cursor and
        fail the arrival verdict on every run that grips anything. Jaw completion has
        its own measured judgement in the jaw phase. Same rule, same reason, as the
        playback's by-index gripper exclusion in `teleop_session.py`.
        """
```

## src/yam/session.py:549-556 (ArmSession._command_park)

```python
        """Send a park command with the jaw routed through the block latch.

        ⛔⭐ THIS CLOSES A REAL HOLE: the park used to command the saved jaw value raw,
        every cycle, so a park whose pose closes the jaws onto an object would push into
        it at 90 Hz for the rest of the run — the stall guard latched a block and the
        park ignored it, the exact §58.2 shape one layer down. `hold_jaw` honours the
        latch (keep the grip, stop the push) and clears it on a deliberate open.
        """
```

## src/yam/session.py:576-581 (ArmSession.count_gripper_stops)

```python
        """How many times a run through these legs would pause for the jaws.

        ⭐ For the plan line, BEFORE Enter: a grab should be visible while the sequence
        is still being typed (ROADMAP §6.6.2 item 4). Same clamping and reconciliation
        as `begin_path`, so the count cannot disagree with the run.
        """
```

## src/yam/session.py:696-705 (# ⭐⭐ LET THE ARM SETTLE)

```python
            # ⭐⭐ LET THE ARM SETTLE BEFORE THE JAWS MOVE. The cursor finishing is not
            # the arm arriving — the arm still trails by its friction floor, and jaws
            # that close while it creeps close in the wrong place. His 2026-08-18 bench
            # pass measured exactly this: the grab missed by millimetres while the run
            # read clean. So the split pose keeps being commanded until the arm is
            # within `tolerance` or has stopped improving for `settle_seconds` — the
            # same two exits the final settle has — and only then do the jaws move.
            # Costs at most about half a second per stop; `jaw_arm_off` reports the
            # offset the arm actually settled at, which is the number that tells a
            # friction-floor miss from a badly taught pose.
```

## src/yam/session.py:725-730 (# ⭐ THE SETTLE PHASE)

```python
            # ⭐ THE SETTLE PHASE STILL COMMANDS AND STILL CREDITS PROGRESS — merged from
            # the script on 2026-08-18 (item 23 group ④), which had learned both after
            # this class was written. Re-sending the final point keeps the velocity
            # feedforward decaying to zero instead of freezing at its last value, and
            # crediting an err improvement keeps a slowly-settling arm from being
            # declared blocked at the stall timeout while it is still visibly closing.
```

## src/yam/session.py:737-747 (ArmSession.abandon_path)

```python
        """⛔ Leaving PARK abandons the rest of the run, and returns the rad dropped.

        An arm that resumes a queued trajectory after the operator pressed HOLD is doing
        something nobody asked for. Returning the distance rather than a waypoint count
        is deliberate: with one blended path there are no separate legs left to count,
        and "1.8 rad of path abandoned" is what an operator can actually picture.

        ⭐ Queued segments and a pause in progress are part of the run, so they are
        dropped — and counted — here too. A latched jaw block survives on purpose: it
        describes the object still between the jaws, not the abandoned motion.
        """
```

## src/yam/cameras/frame.py:1-4 (module)

```python
"""The per-sample camera record, field-aligned with the team's LaRobot `Frame`.

⭐ The alignment is the point (ROADMAP §10.6): the rebuild's `cameras/frame.py` carries `camera_name · sequence · camera_timestamp_ns | None · host_timestamp_ns · rgb · depth: None`-able, and matching those names means capture code written against this walkthrough lifts into the rebuild unchanged.
"""
```

## src/yam/cameras/frame.py:14-21 (Frame)

```python
    """One camera sample, timestamped honestly.

    ⛔ **`camera_timestamp_ns` is `None` on this stack, and that is a measurement, not laziness.** OpenCV's AVFoundation backend cannot report a device-side capture time (`CAP_PROP_POS_MSEC` is unreliable there, the same backend that cannot report FOURCC — FINDINGS §63.0's family). A fabricated device timestamp would be the fails-by-lying pattern applied to time; `None` says "the camera did not tell us", which is the truth the dataset needs to know about itself.

    **`host_timestamp_ns` is `time.monotonic_ns()` taken when the reader thread STORED the frame**, not when the control loop sampled it — the store moment is the closest observable to the exposure this backend offers. Monotonic, so it survives NTP adjustments; it shares a clock with nothing outside this process, and aligning it to joint data works because the recorder stamps samples from the same clock.

    ⛔ **`depth` is always `None` on this rig** — measured on 2026-08-17: every D405 mode over UVC on macOS is an ordinary colour photograph (FINDINGS §63.0). The field exists because LaRobot's record has it and the rebuild (Ubuntu + SDK) will fill it.
    """
```

## Frame field comments at f346a8d, lines 24-28

```python
    sequence: int                        # per camera, monotonically increasing, no gaps
    camera_timestamp_ns: int | None      # None on this stack — see the docstring
    host_timestamp_ns: int               # monotonic clock, stamped at frame-store time
    rgb: Any                             # the BGR ndarray as OpenCV delivers it
    depth: Any | None = None             # always None over UVC on macOS (FINDINGS §63.0)
```
