# Archived ArmSession source explanations

Moved September 16, 2026. These verbatim excerpts preserve historical reasoning
and can contain superseded claims. Source lines refer to the coordination
continuation based on `783c8f1`, before this prose pass. Current behavior is
described in the source and [CLEANUP](../CLEANUP.md).

## parse_arms docstring at source lines 83–110

```text
    """Turn `--arm` and `--arms` into the ordered list of arms a session drives.

        parse_arms(None, None,  ARM_SERIALS, "B")   -> ["B"]
        parse_arms(None, "B,G", ARM_SERIALS, "B")   -> ["B", "G"]
        parse_arms("G",  None,  ARM_SERIALS, "B")   -> ["G"]

    ⭐ WHY BOTH FLAGS EXIST. `--arm` is the spelling every other script here uses
    (`ping_motors.py`, `identify_arm.py`, `check_arms_match.py`), it is in every
    document, and it is what Julien types. `--arms` is the N-arm spelling ROADMAP §6.1
    step 2 asks for. **They are two spellings of one idea, not two ideas** — the same
    relationship `ö`/`ä` have to `[`/`]`, and the reason is the same: a working command
    must not stop working because the code grew a more general form.

    ⛔ THEY MUST AGREE. `--arm B --arms G` is refused rather than resolved by a
    precedence rule nobody would remember. This repo has already paid for the other
    approach: `--arm arm1` was deleted rather than aliased, because a flag that keeps
    working while its meaning has moved underneath is worse than one that fails loudly
    (`src/yam/can.py`, and the same call again for `--box`).

    ⛔ AND NO ARM MAY APPEAR TWICE. `--arms B,B` would build two `ArmSession` objects
    over one CAN bus, each with its own cached `prev_q`, both commanding the same seven
    motors every cycle. The last write of each cycle would win and nothing would raise,
    so the arm would follow a blend of two controllers. That is the FINDINGS §0 defect
    class exactly: a confident, plausible, wrong answer with no exception.

    Raises `ValueError` with a message written for the person at the keyboard. The
    caller turns it into `argparse`'s own error, so it prints like any other bad flag.
    """
```

## ArmSelector docstring at source lines 140–162

```text
    """Which arm a MODE key applies to. `a` cycles it: B → G → BOTH → B.

        sel = ArmSelector(["B", "G"])
        sel.label            # "B"
        sel.cycle()          # "G"
        sel.cycle()          # "BOTH"
        sel.names()          # ["B", "G"]

    ⛔⭐ WHY MODE KEYS NEED A SELECTOR AT ALL, and it is a safety argument rather than a
    convenience one. ROADMAP §6 decided it: a global `g` would put **8.6 kg** weightless
    in one keypress, and GUIDE is the mode where an error in the dynamics model becomes a
    *falling* arm rather than a droop ([FINDINGS §11.1](../docs/FINDINGS.md)). So a mode
    change is aimed at one arm unless the operator has deliberately selected BOTH.

    ⭐ **Driving is NOT selected.** Each arm follows its own puck, continuously, always —
    that is the whole point of two arms. Only mode changes and edits are aimed. Julien's
    own words for the goal are in ROADMAP §6, and the split is in its decision table.

    ⚠️ **With one arm there is nothing to cycle**, and `cycle()` says so by returning the
    same label rather than inventing a BOTH that means the same as B. A key that appears
    to do something while doing nothing is the `b` defect again
    ([FINDINGS §17.1](../docs/FINDINGS.md)).
    """
```

## ArmSession docstring at source lines 240–266

```text
    """Everything that is true of **one** arm during a session.

    Modes keep the script's own names — `guide`, `teleop`, `hold`, `park` — because they
    are Julien's mental model and renaming them would make every note in FINDINGS harder
    to follow.

    ⚠️ **`map` is deliberately absent, and this docstring used to claim it was here.**
    CONTROLS (`m`) is an interactive wizard: it asks the operator to move one axis at a
    time and waits for answers. That is a *session* activity, like key handling, so it
    stays in the script along with `last_active_axis`. **The old wording said "the same
    five modes" and the class only ever had four**, which is the kind of small untruth
    that makes a reader trust the rest of the file less. Found by the diff in
    [FINDINGS §36.5](../docs/FINDINGS.md).

    ⛔ **Still NOT here, and each is a decision rather than an oversight** — see
    [ROADMAP §6.1](../docs/ROADMAP.md):

    - **The teleop per-cycle clamp and the joint-limit clamp.** They belong here and are
      not here yet, and the argument is working-contract rule 7: *what path reaches the
      hazard without passing through the guard?* Today they live only in the teleop
      branch, and PARK already went around the gripper clamp once for exactly that reason
      (FINDINGS §9). Moving them into this class's single command path would close that
      whole class of defect. **Not done here**, because it changes what gets commanded
      and that deserves its own reviewable step.
    - **The workspace box.** A cartesian idea, so it stays with `CartesianTeleop`.
    - **Recording and playback.** They span both arms; see the module docstring.
    """
```

## comment at source lines 280–293

```text
        # ⭐⭐ WHICH PUCK DIRECTION DRIVES WHICH MOTION, for THIS arm in THIS frame.
        #
        # ⚠️ Passed to the constructor rather than assigned afterwards, and that is a
        # deliberate contrast with `mode`. `mode` cannot be a constructor argument, because
        # `build_robot()` reads it to decide zero-gravity and runs before the robot exists —
        # so the script has to hand it over on the next line, and a forgotten handover would
        # have been silent ([FINDINGS §50.2](../docs/FINDINGS.md)). The map has no such
        # constraint: `AxisMapStore` can be read before anything is built. **So it is
        # impossible to forget rather than merely tested for.**
        #
        # ⭐ `axis_map_at_start` is copied HERE rather than by the caller, because the two
        # must be taken at the same instant. `0` in CONTROLS reverts to it, and the closing
        # summary reports "was:" from it, so a copy taken a few lines later would quietly be
        # a copy of a different map.
```

## comment at source lines 297–314

```text
        # ⭐⭐ THIS ARM'S SAVED POSES, AND THE ONE Ctrl-C GOES TO.
        #
        # ⛔ THE BASE POSE AND THE WAYPOINTS ARE DIFFERENT THINGS, and it is a safety
        # requirement rather than a preference. Julien, 2026-08-12: *"the control-c park to
        # disable needs to always go back to the stable parking save. If I save a new
        # parking option it shouldn't go back to that and then disable."* Ctrl-C parks and
        # then RELEASES the motors, so the pose it chooses must be one that is safe to let
        # go in. A waypoint saved mid-task, with the arm extended over the desk holding
        # something, is exactly what that must never be.
        #
        # ⭐ Called `base_pose` and not `park`, deliberately: this file already has eleven
        # `park_*` fields describing the motion in progress, and a `park` beside them would
        # read as one of them. The old session local was named `park`, which is why
        # `check_restructure.py` carries it in RETIRED_LOCALS.
        #
        # ⚠️ `None` is a real state, not a missing value: it means no pose has ever been
        # saved for this arm. The script then defaults it to wherever the arm was when the
        # session started, which it can only do after the robot exists.
```

## comment at source lines 319–332

```text
        # ⭐⭐ THE PUCK THAT DRIVES THIS ARM. One `TwistReader`, already opened and bound to
        # one physical SpaceMouse by the wiggle assignment.
        #
        # ⚠️ THE MODULE DOCSTRING ABOVE USED TO SAY THIS CLASS DELIBERATELY DOES NOT OWN
        # "reading the SpaceMouse", and that is still half true. The *device layer* stays
        # shared and outside: enumerating, the wiggle assignment, `open_device`, and closing
        # the handle on the way out. **What belongs to the arm is the one reader it is
        # driven by**, because with two arms "which puck" is exactly as per-arm as "which
        # robot" — and a session-level reader is how both arms end up following one hand.
        #
        # ⛔ The HANDLE is deliberately NOT here. It has to be closed even when
        # `build_robot()` failed and no `ArmSession` was ever created, so the script keeps
        # its own dict of handles for teardown. Two references to one object, not two
        # copies of state.
```

## comment at source lines 340–356

```text
        # ⭐⭐ WHAT THE CONTROLS WIZARD REMEMBERS ABOUT *THIS* PUCK, and it is per arm for
        # the same reason the reader is: two pucks have two "controls you just used".
        #
        # ⛔ `last_active_axis` has NO TIMEOUT on purpose. In CONTROLS, `f` and `1`-`6` act
        # on "the control you just used", and that has to still be remembered after the puck
        # has sprung back to centre and the operator's hand has left it.
        #
        # ⭐ `last_input_kind` exists so that ONE key means one thing: `f` reverses whichever
        # control was last used, an axis by flipping its sign or a button by swapping
        # open/close.
        #
        # ⚠️ `buttons_prev` is what makes a press an EDGE rather than a state. Without it a
        # held button would re-fire its action every cycle at 100 Hz.
        #
        # ⚠️ The module docstring says key HANDLING stays in the script, and it still does:
        # which arm a keypress is aimed at is a session question, answered by `ArmSelector`.
        # What lives here is the state a key acts ON.
```

## comment at source lines 366–376

```text
        #: ⛔⭐⭐ HOW MANY TIMES THE JAWS HAVE STALLED IN A ROW, and when it was last said out
        #: loud. Julien, 2026-08-15: *"the gripper arm print was way too often, and it happened
        #: because I was pushing on the leader arm gripper and the follower was picking
        #: something up, so it pushed too far in."*
        #:
        #: ⚠️ THE GUARD IS WORKING; the REPORTING is not. In MIRROR the follower's jaw command
        #: is the leader's measured jaw position, re-sent every cycle. So the guard releases the
        #: jaws, the next cycle commands them back onto the object, and 0.4 s later it fires
        #: again — for as long as the operator squeezes the leader. Twenty identical lines in
        #: ten seconds is how a real warning gets trained into background noise, which
        #: [FINDINGS §0](../docs/FINDINGS.md) is a catalogue of.
```

## comment at source lines 386–392

```text
        # ⛔⭐ THE PARK IS ONE BLENDED PATH WITH A CURSOR ALONG IT, and this replaced a
        # queue of separate legs on 2026-08-13. The earlier model drove to each waypoint
        # and stopped dead, which is the thing Julien explicitly did not ask for:
        # *"instead of moving and then jittering ninety degrees to the next side, in a
        # smooth curve it would go to the next point."* `teleop_session.py` changed to
        # `JointPath` on 2026-08-12 at 15:15, one hour after this class was written, and
        # the class was left behind for a day. Audit: ROADMAP §6.1.
```

## comment at source lines 432–439

```text
        # ⛔⭐ THIS CYCLE'S READING, AND `None` MEANS BLIND RATHER THAN COLD. The status
        # row prints `??°C ⚠️BLIND` for `None`, never a number: a fabricated 0 °C is what
        # made a disarmed thermal guard look healthy on screen (FINDINGS §24.1), and the
        # readout is the only place a human would have noticed.
        #
        # ⭐ Per arm, because the row is per arm. As a session-level pair these were one
        # arm's temperatures painted on whichever row happened to be drawn — the shape of
        # error that would hide one arm's gripper behind the other arm's shoulder.
```

## comment at source lines 442–451

```text
        # ⛔⭐ THE LAST CHAIN READ THIS ARM MANAGED, kept because the incident record needs
        # it AFTER the chain has died. On 2026-08-14 the arm fell, the CAN link went away,
        # and every value describing that instant was lost — the gravity torques had to be
        # recovered by simulating joint angles the arm had already measured and thrown away
        # ([FINDINGS §45](../docs/FINDINGS.md)). A fresh read on a dead chain raises; the
        # last good reading is what actually describes the failure.
        #
        # ⚠️ `None` means the read failed, exactly like `hottest`. Never an empty list: an
        # empty list would read as "seven motors reporting nothing", which is a different
        # and much calmer claim than "I could not ask".
```

## comment at source lines 456–469

```text
        #: ⛔⭐⭐ WHERE THE JAWS GOT STUCK, LATCHED. `None` means nothing is blocking them.
        #:
        #: This exists because the stall RELEASE was being undone on the very next cycle.
        #: Julien's 2026-08-17 log shows it plainly: released to 0.152, then 0.151, then
        #: 0.150, then 0.147, and the message *"ARM G GRIPPER STALLED (14 times now)"*.
        #: Each release backed the command off to the measured jaw position, and each next
        #: cycle MIRROR copied the leader's jaw straight back over it. **A one-cycle
        #: correction against a source that re-commands every cycle can only ever nibble.**
        #:
        #: ⚠️ `teleop_session.py` already carried a comment describing exactly this — *"in
        #: MIRROR the follower's jaw command is the leader's measured jaw, re-sent every
        #: cycle, so squeezing the leader while the follower holds an object fires this
        #: every 0.4 s indefinitely"* — so the diagnosis was written down and the fix was
        #: never built. It is docs/ROADMAP.md §8.2 item 29.
```

## hold_jaw docstring at source lines 481–518

```text
        """The jaw value to actually command, honouring a latched block.

        ⭐ THE RULE, and the asymmetry is the whole point:

        | asked for | result |
        |---|---|
        | nothing latched | ✅ obeyed |
        | **further CLOSED** than the block | ⛔ held at the block, so it stops pushing |
        | **more OPEN** than the block by more than `JAW_CLEAR_MARGIN` | ✅ obeyed, **and the latch clears** |

        ⭐ Bigger is more open: `o` adds to `gripper_value` and `c` subtracts, and the jaws
        normalise to 0 closed and 1 open. So "further closed" is a smaller number.

        ⚠️ **Opening always clears the latch**, which matters more than it looks. The object
        may have been put down, the operator may have let go, or the leader's hand may have
        opened. Anything that moves away from the obstruction is evidence the obstruction is
        no longer being pushed into, and a latch that needed an explicit reset would
        eventually be the reason the jaws refused to work for a reason nobody could see.

        ⛔ It does NOT stop the jaws holding what they have. The block value IS the measured
        position where they stalled, so commanding it keeps the grip and stops the pushing.
        Releasing entirely would drop whatever is being held.

        ⛔⭐⭐ THE MARGIN EXISTS BECAUSE THE FIRST VERSION CLEARED ON ANY VALUE ABOVE THE
        BLOCK, AND THAT IS TOO EAGER. A jaw position read off a motor jitters, and the leader
        in MIRROR is a hand-held arm whose jaws are being squeezed, so its measured jaw wanders
        by a few thousandths every cycle. **One sample a hair above the block would unlatch
        it**, the next cycle would push again, and the stall would recur — which is the exact
        failure the latch was built to end.

        ⚠️⚠️ I ALSO WROTE A TEST ASSERTING THE WRONG BEHAVIOUR AND ARGUED FOR IT. The old
        `test_the_tiniest_opening_still_counts` said *"deliberately a strict inequality rather
        than a tolerance"*, reasoning that a tolerance would let commands a hair below the
        block through. **That was the wrong risk to weigh.** A hair below the block is
        harmless — it is still not pushing. A hair above it disarms the whole mechanism. ⭐ A
        test can encode a mistake as confidently as code can, and a docstring defending it
        makes the mistake harder to see rather than easier.
        """
```

## gripper_stall_release docstring at source lines 572–593

```text
        """Is the gripper pushing hard without moving? Returns a jaw value to back off to.

        ⛔⭐ WHY THIS EXISTS: motor 7 was cooked three times. Pushing at full current
        while not moving is the worst thermal case there is — full current, no motion, no
        cooling — and the jaws reach it whenever they are commanded past whatever they are
        holding. The release is to the **measured** jaw position, so the command stops
        fighting the object and the motor stops heating.

        ⭐ It returns a value instead of applying one, because *the class decides and the
        script narrates*: the caller sets `gripper_value` and prints the warning. Returning
        `None` means there is nothing to do.

        ⚠️ It needs `read_thermal()` to have run this cycle, because the torque and
        velocity come from the same chain read. Calling it without one is not an error; it
        simply reports nothing, which is the same "cannot see it, cannot judge it" rule the
        thermal guard uses.

        ⛔ **This was missing from this class for a day**, while `teleop_session.py` had it
        the whole time and this file even carried the `stall_since` variable with nothing
        writing to it. Found by a systematic diff rather than by anything failing.
        FINDINGS §36.5.
        """
```

## begin_path docstring at source lines 685–709

```text
        """Start a run through every leg, split wherever only the jaws move. Returns warnings.

        ⛔ Every waypoint goes through `park_target_from`, so the gripper clamp and the
        6-versus-7-joint reconciliation apply to all of them. A length mismatch on one
        leg once raised mid-park and dropped the arm (FINDINGS §11), and that path
        reaches every leg here, not only the first.

        ⭐⭐ THE SPLIT IS THE GRAB FEATURE (ROADMAP §6.6.2, item 3). A blended corner
        between "at the object, open" and "at the object, closed" closes the jaws during
        the descent, so `plan_gripper_stops` breaks the run at every jaws-only leg: one
        blended `JointPath` per segment, and between segments the arm holds while the
        jaws are commanded and WAITED FOR (see `step_path`'s jaw phase). A run with no
        jaws-only leg produces exactly one segment and behaves as it always has.

        ⚠️ A leg that moves the arm AND the jaws together is reported in the returned
        warnings rather than split — only the operator knows which he meant, and both
        readings are defensible. That rule and its reasoning live on `plan_gripper_stops`.
        `mixed_leg_advice=False` drops that advice (never the target warnings): a drive
        to a recording's start pose is one positioning move whose destination nobody can
        re-save, so "save a waypoint where only the jaws change" would be noise there.

        ⚠️ `smooth=False` is the caller's `--no-smooth`: the path is still blended, and
        only the easing ramp is switched off. Blending is the *shape*; easing is the
        *speed along it*. They are independent axes and Julien wants both adjustable.
        """
```

## step_path docstring at source lines 814–844

```text
        """Advance the park by one control cycle and report what happened.

        ⛔ Completion is judged from the **measured** pose, never from the command. The
        command always arrives first, so testing it would declare success while the arm
        was still travelling. That was a real bug and it hid for two sessions.

        ⛔⭐ ARRIVAL IS GATED ON THE CURSOR REACHING THE END OF THE PATH, not on the
        error alone. A run like `p 1 2 1` finishes where it started, so the distance to
        the final target is small at t=0 as well — judging on that would declare the
        whole sequence complete before the arm had moved at all.

        ⭐ The cursor waits when the arm falls behind. The trajectory is a *shape*, and a
        command racing ahead while the arm cuts its own corner is not the shape anyone
        chose. Progress means "the cursor moved OR the arm closed the gap": without the
        first half a legitimately slow leg looks stalled, and without the second an arm
        pinned against something never does.

        ⭐⭐ THE JAW PHASE (verdict "jaws"). At the end of every segment but the last the
        arm holds still, the next waypoint's jaw value is commanded (through the block
        latch), and the run resumes when the jaws are DONE — measured as "still for
        `JAW_SETTLE_SECONDS` after at least `JAW_MIN_WAIT`", never as a dwell time. A jaw
        stalled on an object is still, so a successful grab resumes the run by the same
        rule as an empty close. `JAW_TIMEOUT_SECONDS` bounds a jaw that never settles, so
        a jammed gripper cannot stop the run for ever; a timeout is reported, not hidden.
        On the resume cycle `check_grasp` grades a closing leg (item 10) and rides along
        on the step.

        ⛔ `err` and `lag` are ARM-ONLY — see `_arm_err`. A held object parks the jaw a
        finger's width from its command for the whole rest of the run, and counting that
        would freeze the cursor and turn every successful grab into a "blocked" park.
        """
```

