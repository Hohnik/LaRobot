# Archived teleop source explanations

Moved out of the operator application on September 15, 2026 during cleanup of
`de6bae7` and its continuation. These are verbatim source excerpts, including
superseded claims and historical wording. They are evidence of earlier reasoning,
not current operating instructions. Line numbers refer to the source before this
prose pass. Read [CLEANUP](../CLEANUP.md) for current behavior and remaining work.

## Comment at source lines 226-282

```text
# ⭐⭐ THE SAME PAIR OF KEYS, REACHABLE ON A GERMAN KEYBOARD. Julien, 2026-08-12:
# *"I don't like the fact that the brackets are used because I have a German
# keyboard, and they're awkward to reach. Maybe ä and ö could be used."*
#
# He is right, and it is worse than awkward: on a German QWERTZ layout `[` and `]` are
# **AltGr + 8** and **AltGr + 9** — a three-finger chord, on a rig whose whole input
# design rule is "no shift keys", for a knob he adjusts while 4.3 kg is moving.
#
# ⭐ `ö` and `ä` are single unshifted keys, adjacent, on the home row immediately right
# of `L` — the physical position US layouts give to `;` and `'`. They are the best pair
# available: `ü` and `+` are the other unshifted candidates and `+` already means
# "faster", while `^` and `´` are dead keys that emit nothing until a second press.
#
# ⚠️ ALIASES, NOT REPLACEMENTS. `[` and `]` keep working — they are in every doc, and
# a US keyboard (a colleague's, or a clone of this repo) must not lose the feature. Two
# spellings of one key is cheap; a key that exists only on one person's laptop is not.
#
# ⛔ These are non-ASCII, so they only arrive at all because `KeyReader` now decodes
# UTF-8 across reads — `ö` is two bytes and the old one-byte reader turned it into two
# replacement characters. See `src/yam/inputs/keyboard.py::_refill`.
#: ⛔⭐⭐ A CEILING ON THE LIVE SPEED KEY, BECAUSE THERE WAS NONE. `+` did
#: `linear_scale *= 1.25` with nothing above it, at two separate call sites. On 2026-08-17
#: Julien's CONTROLS readout showed **`lin 19.852 m/s`**, which is 165x the 0.12 m/s default
#: and about 23 presses of a key that repeats when held.
#:
#: ⚠️ Every other live adjustment in this file already had a bound — park speed clamps to
#: `min(teleop_speed, …)` and `max(0.05, …)`, and the gripper step has a 0.200 ceiling. The
#: two linear-speed sites were the exception, and the decrease direction had no floor either.
#:
#: ⭐ Set DELIBERATELY GENEROUS at 2.0 m/s, which is ~17x the default and far past anything
#: useful for hand-scale work, so it can never get in his way. **It is a backstop against a
#: held key, not a new restriction.** ⛔ A commanded speed this high does not move the arm
#: that fast — `SafeRobot` and the reach limit still bind — but it makes the IK target jump
#: the whole workspace in one cycle, so the arm slams to the boundary at whatever
#: `--max-speed` allows.
#: ⛔⭐⭐⭐ RAISED FROM 2.0 TO 15.0 BECAUSE 2.0 WAS A REGRESSION I INTRODUCED. Julien,
#: 2026-08-17: *"the linear limit is at two, the ceiling, and then the max speed was set way
#: higher, but that didn't make any difference to the teleop speed. I was limited by the two
#: before. And before, I was able to go much faster."*
#:
#: ⭐⭐ **He is right, and the reason is that THREE limits sit in series and the smallest one
#: binds.** In order, from the puck to the motor:
#:
#: | limit | units | what it bounds |
#: |---|---|---|
#: | `--linear-scale` | m/s | the CARTESIAN speed a full puck deflection asks for |
#: | `--teleop-speed` | rad/s | how far the IK answer may move any one JOINT per cycle |
#: | `--max-speed` | rad/s | `SafeRobot`'s rate cap, below all control logic |
#:
#: ⛔ At 2.0 m/s with a 0.4 m lever the joints only need about 5 rad/s, so a `--teleop-speed`
#: of 15 was never reached and raising it did nothing. **The cartesian limit was the binding
#: one and I had just capped it at a sixth of what he had been using** (he reached 19.852 m/s
#: before the cap existed, [FINDINGS §62.2](../docs/FINDINGS.md)).
#:
#: ⭐ 15.0 is chosen so it can NEVER be the binding limit: `teleop_speed`'s own ceiling is 20
#: rad/s, which at a 0.6 m lever is about 12 m/s of tip speed. ⚠️ So this stays a backstop
#: against a key that repeats when held, and stops being a speed limit.
```

## Comment at source lines 297-305

```text
# ⛔ Do not let the commanded cursor run further than this ahead of the arm. The
# trajectory is a SHAPE now, and a command that races ahead while the arm cuts its own
# corner is not the shape anyone chose. SafeRobot's 0.25 rad lag limit is the backstop
# below this; this keeps the path faithful rather than merely safe.
# ⭐ How fast the follower closes the initial gap in MIRROR mode. ⚠️ IMPORTED from
# `src/yam/mirror.py` rather than repeated, so there is ONE number. It is aliased here only
# because the plan line quotes it to the operator before they press Enter — the first draft
# of this line wrote `0.30` next to a comment claiming it came from the module, which is the
# staleness pattern in miniature: a duplicate plus a sentence asserting there is no duplicate.
```

## Comment at source lines 308-314

```text
# ⭐ Two DIFFERENT patiences, and separating them removed a four-second dead wait at
# the end of every park. "Has the controller finished settling?" is answered in a
# fraction of a second; "is something blocking the arm?" deserves four. They used to
# share PARK_STALL_SECONDS, so every park that finished outside the 0.02 tolerance —
# which is most of them — sat apparently doing nothing before admitting it had
# arrived. Julien: *"it moves millimetre by millimetre really slowly."* He was
# watching a wait, not a crawl.
```

## Comment at source lines 319-325

```text
# ⭐ "Close enough that what is left is the controller, not an obstruction."
# MEASURED on hardware 2026-08-12: the same arm parking to the same pose reported
# 0.020 rad off (pass) and 0.021 rad off (stall) in consecutive sessions, because a
# position-controlled arm settles a fraction of a degree short under its own weight.
# 0.02 rad sits ON that noise floor rather than above it. See yam_robot.park_verdict:
# the answer is not a bigger tolerance — it is a second threshold, so that stopping
# CLOSE is success while stopping FAR is still an obstruction.
```

## Comment at source lines 338-348

```text
# ⛔ NEVER command the gripper to 0.0 or 1.0. Those are the mechanical stops, and
# holding a position AT a stop is stall torque: full current, no motion, no
# cooling. That is what cooked motor 7 twice on 2026-08-10 -- the arm was simply
# told to "hold where you are" while the jaws happened to be resting on a stop.
# Keeping the command inside this band means the jaws are always free to move,
# so a hold command costs almost no torque.
# ⛔ These are only applied to values the OPERATOR asks for. The gripper is never
# forced away from where it already is. The earlier [0.15, 0.85] clamp was applied
# on entering TELEOP, which meant that if the jaws happened to sit outside the band
# the session COMMANDED THEM TO MOVE the moment teleop began -- a motion nobody
# asked for, into a mechanical stop when the limits were also mis-framed.
```

## Comment at source lines 369-374

```text
# ⭐ Hand-taught movements live OUTSIDE config/, and the distinction is deliberate.
# `config/` holds measured calibration that the code refuses to start without, so it is
# tracked in git. A recording is *data*: a five-minute one is megabytes, there will be
# hundreds, and losing one costs a minute of re-teaching rather than a session of
# re-calibrating. Gitignored, and the dataset itself will live somewhere else again
# (ROADMAP step 5).
```

## Comment at source lines 376-395

```text
# ⭐ One file per playback, named with a timestamp so nothing is ever overwritten. The
# recordings themselves are saved by slot digit and DO overwrite, which lost the two files
# an earlier measurement was taken from ([FINDINGS §33.2](../docs/FINDINGS.md)).
# ⚠️ Built as a LOCAL inside main() now, from `takes_dir`, so that a --sim session's
# tracking files follow its recordings into `recordings/sim/` instead of landing among the
# real measurements. There is deliberately no module-level constant for it any more: one
# would be shadowed by the local and read as though it were still in use.
# ⭐⭐ ONE CEILING FOR EVERY PLANNED MOTION, in radians per second for a single joint.
# Julien asked for this on 2026-08-13: *"max speed would just be limited by the actual
# safety things we have or the motors. Maybe we need an extra system for max speeds in
# general."* He is right, and the number already existed in the file twice over: TELEOP
# clamps the commanded joint change to MAX_JOINT_STEP per cycle, which at CONTROL_HZ is
# exactly this speed. Deriving it keeps the two from drifting apart, and it means a change
# to the teleop clamp automatically applies to playback.
#
# ⚠️ WHAT THIS IS NOT: a measurement of how fast the arm can actually track a command.
# Nobody has measured that on this rig. The evidence so far says it is LOWER — at 0.26x of
# a 2.67 rad/s recording, so ~0.7 rad/s commanded, the arm was already 0.105 rad behind
# against a 0.15 limit. So treat this as "the fastest we allow ourselves to ask for", and
# see ROADMAP §6.6 for the measurement that would replace it.
```

## Comment at source lines 597-613

```text
        # ⛔⭐⭐⭐ FORGET THE COMMAND HISTORY BEFORE THE FIRST COMMAND, AND THE ABSENCE OF
        # THIS LINE WAS A VISIBLE SPASM ON THE ARM. Julien, 2026-08-17: in the quit menu he
        # pressed `g` (weightless), moved an arm by hand, then `p` — and the arm *"quickly
        # spasmed for, like, a tenth of a second, for seemingly no reason"*.
        #
        # ⭐ The reason: `SafeRobot` is stateful. Its rate limiter walks from `_last_cmd`,
        # which still held the pose from BEFORE the hand-guiding, because GUIDE commands no
        # positions (kp = 0). The first park command was therefore pulled toward that stale
        # pose and clipped to measured ± max_lag — so the arm jerked up to 0.25 rad toward
        # where it USED to be, for the few cycles it took `_last_cmd` to converge. Exactly a
        # tenth of a second.
        #
        # ⛔ `SafeRobot.resync()`'s own docstring says "call this on EVERY mode transition",
        # and the in-session transitions all do. **This function is a mode transition that
        # lives outside the mode system**, which is why it was missed — the same reason the
        # two park implementations diverged (docs/ROADMAP.md §8.2 item 23).
        # ⚠️ `getattr`, because the test fakes are not SafeRobot-wrapped.
```

## Comment at source lines 672-677

```text
                # ⛔⭐ IT REPORTS WHAT IT MEASURED, then offers the guesses. This message read
                # *"Something is in the way, or the pose is unreachable"* and Julien's answer
                # to the same wording in MIRROR was *"the robot was never blocked by
                # anything."* The three numbers below distinguish the cases: how far the
                # COMMAND ran ahead of the arm, how often `SafeRobot` held it back, and
                # whether the arm was moving at all.
```

## Comment at source lines 696-701

```text
            # Same ease-in/ease-out as the interleaved park — Ctrl-C should not shut down
            # with a jerk at both ends when a mid-session park glides.
            # ⭐ EASINGS[2] is "out": full speed from the first step, soft landing. Julien on
            # Ctrl-C: *"I want it to move into its parking position quickly and swiftly,
            # without the excessive starting and pausing."* A shutdown move should leave at
            # once; only the arrival needs to be gentle.
```

## Comment at source lines 1300-1317

```text
    # ⭐⭐ THE THREE LAYERS, and the ORDER is the design:
    #
    #     built-in constant  →  config/session_defaults.json  →  command-line flag
    #
    # Julien, 2026-08-17: *"all of these flags should be default options that can be
    # changed… and then should be saved so that I don't always have to run with all of
    # the flags."*
    #
    # ⛔⭐ The cost of NOT having this was concrete. His working command had grown to six
    # flags, and the one he left out of it is the one that cost him three sessions: he
    # raised `--max-lag` three times chasing a mirror that kept stopping on
    # `--mirror-gap`, which sat at its built-in 0.35 because it was not in the command he
    # had been pasting. **A long command line is a place for a critical setting to go
    # missing in silence.**
    #
    # ⚠️ `set_defaults` is called AFTER every `add_argument` and BEFORE `parse_args`, so a
    # flag typed on the command line still wins. That precedence is a safety property: a
    # deliberate flag must never be quietly overridden by a file.
```

## Comment at source lines 1345-1354

```text
    # ⛔⭐⭐ TWO ARMS NOW RUN, AND `--start-mode guide` IS REFUSED FOR THEM.
    #
    # ROADMAP §6's ruling, and it is the one refusal that replaced the blanket one: **two
    # arms going weightless on a first run is the worst possible first run.** `g` reaches
    # the same state, but only after the operator has selected BOTH deliberately and
    # pressed a key at a rig they are watching. A flag does it before anything is on screen.
    #
    # ⚠️ The blanket "two arms cannot run yet" refusal lived here until 2026-08-14 and is
    # gone, along with the test that pinned it. That deletion is deliberate: the refusal
    # existed because the script below was single-arm, and it no longer is.
```

## Comment at source lines 1362-1377

```text
    # Rotation is ON by default now. Julien: "the gripper cannot be tilted
    # currently and cannot be twisted". It was off for the first hardware run
    # because a wrong rotation sign swings the wrist while a wrong translation
    # sign only nudges — that caution has served its purpose.
    # ⭐⭐ ALL OUTPUT IN THIS SESSION GOES THROUGH ONE STATUS LINE.
    #
    # ⛔ `print` is deliberately shadowed for the whole of main(). Julien changed the
    # park speed six times while choosing a run and got six copies of a two-line plan
    # interleaved with six status lines — *"that seems to be more of a bug."* It was:
    # a `\r` status with no newline and ordinary prints with newlines were fighting
    # over the same row.
    #
    # Shadowing rather than converting ~60 call sites is a deliberate trade. It makes
    # the output policy ONE thing in ONE place, and — the part that matters — a print
    # added later cannot forget to follow it. The rule: `end=""` means "this is the
    # live line, repaint it in place"; anything else scrolls above it. See src/yam/ui/screen.py.
```

## Comment at source lines 1416-1435

```text
    # ⚠️ No `axis_map` local any more. Each arm carries its own map and its own
    # start-of-session copy, handed to `ArmSession` at construction so it cannot be
    # forgotten. The store stays the source; this file only reads it here and at exit.
    # `checks/check_restructure.py` RETIRED_LOCALS keeps the old name from coming back.
    # ⭐⭐ THE BASE POSE AND THE WAYPOINTS ARE DIFFERENT THINGS. Julien's ruling,
    # 2026-08-12: *"the control-c park to disable needs to always go back to the stable
    # parking save. If I save a new parking option it shouldn't go back to that and then
    # disable. It should always go back to the base parking option."*
    #
    # ⛔ That is a safety requirement, not a preference. Ctrl-C is the "get me out of
    # here" key: it parks and then RELEASES the motors, so the pose it chooses must be
    # arm that is safe to be let go in. A waypoint saved mid-task — arm extended over
    # the desk, gripper holding something — is exactly what that must never be.
    #
    # So `park` (slot 0, the base) is only ever changed deliberately with `s 0`, while
    # `s 1`…`s 9` fill waypoints that Ctrl-C ignores completely.
    # ⚠️ Read here, per arm, and handed to `ArmSession` at construction. There is no
    # session-level `park` or `slots` any more: both are this arm's own, keyed by arm name
    # in `config/park_pose.json` since 2026-08-12. `check_restructure.py` RETIRED_LOCALS
    # keeps the old names from coming back.
```

## Comment at source lines 1437-1445

```text
    # ⚠️ The eleven park fields used to be initialised here. They are `ArmSession`
    # fields now, and the class's constructor sets every arm to the identical value —
    # `PARK_SPEED` 0.40 and `PARK_RAMP` 0.20 are the same constant in both files, which
    # was checked before the move rather than assumed.
    # ⛔ Leaving them here as `arm.park_* = …` would run BEFORE `arm` exists, which is
    # the fault `checks/check_restructure.py`'s ordering check exists to catch. It
    # caught all eleven. FINDINGS §48.
    # The blended path being followed, the cursor along it, and where each waypoint
    # falls so the readout can say which arm it is heading for.
```

## Comment at source lines 1448-1456

```text
    # ⛔⭐ TWO CLOCKS, AND CONFLATING THEM PRINTED A WRONG NUMBER FOR A DAY.
    # `park_leg_t` is reset every time the cursor passes a waypoint, because Julien asked
    # for each leg's own duration. `park_start_t` is set once and never reset, because the
    # arrival message wants the whole park. Using `park_leg_t` for both reported
    # **"PARK reached in 0.0s"** on a park that had just taken 4.4 seconds: the last leg's
    # mark is passed at the end of the path, so the reset happened moments before arrival.
    # See [FINDINGS §34.3](../docs/FINDINGS.md).
    # Recording and replay share one timeline across every arm (FINDINGS §35.4,
    # ROADMAP §9.2). ArmSession cannot own either clock independently.
```

## Comment at source lines 1503-1516

```text
    # ⛔⭐⭐ INITIALISED HERE BECAUSE IT WAS NOT, AND THAT CRASHED A SESSION. `replace_slot`
    # is assigned only inside the overwrite-guard branch, and the save handler reads it on
    # EVERY keypress at the save prompt. So the very first save of a session — the guard
    # never having fired — raised `UnboundLocalError: cannot access local variable
    # 'replace_slot'` and took the whole session down.
    #
    # ⭐⭐ FOUND BY A SIMULATED RUN, AND UNIT TESTS COULD NOT HAVE FOUND IT. The 12 tests in
    # `tests/test_save_slot.py` all pass, because they call `save_slot_action` directly and
    # hand it a `replace_slot` argument. **The defect was in the CALL SITE, not the function.**
    # Extracting a decision into a pure function and testing it does not test the code that
    # calls it — which is precisely the gap `--sim` was built to close.
    #
    # ⚠️ The safe stop did its job: both arms parked and all 14 motors were confirmed
    # disabled after the traceback. But the recording would have been lost.
```

## Comment at source lines 1541-1551

```text
    # ⭐⭐ ONE ARM FOLLOWS THE OTHER, joint for joint. Julien's idea, 2026-08-11: *"be able
    # to move one of the arms in the guide mode and have the second arm just mirror the exact
    # movements with zero latency."*
    #
    # ⚠️ SESSION-LEVEL, because a mirror is a RELATIONSHIP between two arms rather than a
    # property of either. The follower's `mode` is `"mirror"`; the leader keeps whatever mode
    # it is in, which is the point — hand-guide it in GUIDE and the follower copies.
    #
    # ⛔ The engagement logic lives in `src/yam/mirror.py::MirrorLink` and has 18 tests, because
    # the risky part is the FIRST cycle: the two arms are never in the same pose, so
    # commanding the leader's angles straight across would make the follower jump the gap.
```

## Comment at source lines 1562-1568

```text
    # ⭐⭐ COMPOSITE RUNS (ROADMAP §6.6.1a): the queue of legs still to run. Entries are
    # ["poses", [digits]] — consecutive poses group into ONE blended park, exactly like a
    # plain `p` run — or ("take", slot, Trajectory, Layout, arms). Takes are validated at
    # Enter, BEFORE any motion, so a bad take refuses the whole run instead of stranding
    # it halfway. `composite_wait` holds the arm names whose pose-park arrival is awaited;
    # `composite_total` > 0 means a composite is active even while its queue is empty
    # (the last leg is still running).
```

## Comment at source lines 1575-1581

```text
    # ⚠️ CONTROLS mode's memory of "the control you just used" — `last_active_axis`,
    # `last_active_value` and `last_input_kind` — plus `learn_button` and `buttons_prev`
    # were declared here. They are `ArmSession` fields now: two pucks have two answers to
    # "which control did you just use", and the class's constructor sets exactly the values
    # these lines did.
    # ⛔ Not left here as `arm.last_active_axis = None`: that would run before `arm` exists,
    # which is the fault `checks/check_restructure.py` check 3 catches.
```

## Comment at source lines 1616-1621

```text
    # ⭐ The ceiling that binds every mode, printed because it was invisible for four days
    # and explained every "why is it so slow" question in that time (FINDINGS §37.0).
    # ⭐⭐ EVERY LAYER, BECAUSE ONLY THE LOWEST ONE BINDS. Julien raised `--max-speed` to 5
    # and teleop felt identical, because the per-cycle IK clamp is a separate 1.5 rad/s and it
    # sits below. Four days were lost to the same invisibility once already (FINDINGS §37.0),
    # and the fix then was to name the number; the fix now is to show which one wins.
```

## Comment at source lines 1719-1734

```text
        # ⚠️ ONE PUCK, and with two arms this becomes arm call per arm with `exclude=` holding
        # the ones already taken — `pick_device_by_wiggle` already supports that and it is
        # tested (`tests/test_puck_assignment.py`). Not wired yet: ROADMAP §6.1 step 2.
        # ⭐⭐ ONE PUCK PER ARM, ASSIGNED BY BEING MOVED. Two SpaceMice both report an EMPTY
        # serial number (measured 2026-08-10), so the trick that made the CAN adapters
        # unambiguous — select by serial, never by position — does not transfer. The device
        # identifies itself by being wiggled.
        #
        # ⛔ `exclude` IS THE PART THAT MATTERS WITH TWO ARMS, and without it this function can
        # hand the SAME puck to both: the single-device shortcut returns it unconditionally, and
        # nothing stops the operator moving the arm they already assigned. Both failures are
        # silent, and the symptom — two arms following arm hand — reads as a control bug rather
        # than a device-assignment bug (`src/yam/inputs/spacemouse.py`, 6 tests).
        #
        # ⚠️ Opened BEFORE `build_robot()`, deliberately: if a puck is missing the session
        # returns here, with nothing energised.
```

## Comment at source lines 1736-1745

```text
            # ⭐⭐ NO SPACEMOUSE IN SIMULATION, and a still puck rather than a random one.
            # The whole point of --sim is running on a machine with nothing attached, and the
            # wiggle assignment below cannot work without a device to wiggle.
            #
            # ⚠️ A puck reporting ZERO deflection is the honest stand-in: nobody's hand is on
            # it, so TELEOP holds still. That is not a limitation to work around — the parts
            # of the loop worth testing without hardware are the mode transitions, the
            # cursors, the playback sequencing and the teardown, and every one of those is
            # driven by KEYS. ⛔ It does mean --sim can say nothing about driving feel or about
            # the axis map, which needs a real hand on a real puck.
```

## Comment at source lines 1752-1760

```text
            # ⭐⭐ AN ARM WITHOUT A PUCK STILL JOINS THE SESSION (item 47, FINDINGS §68.5).
            # With one SpaceMouse and two arms this used to refuse outright — which killed
            # MIRROR and two-arm playback, although a mirror follower and a replaying arm
            # never need a hand. If every attached puck is already assigned, the remaining
            # arm gets the same zero-deflection reader --sim uses: HOLD, GUIDE, playback,
            # scrub and MIRROR-follower all work; only its own TELEOP is inert.
            # ⚠️ Deliberately ONLY when no unassigned device exists. If a free puck IS
            # attached and the operator just did not move it, the abort below stands —
            # falling back silently there would hand him a dead TELEOP he asked to assign.
```

## Comment at source lines 1781-1800

```text
        # Fully initialized sessions drive the loop; `robots` owns every acquired handle
        # for cleanup, including handles whose ArmSession could not initialize.
        # ⛔⭐ DECLARED HERE, BEFORE THE `try`, AND IT IS None ON PURPOSE. FINDINGS §48.3.
        #
        # The closing summary at the bottom of this function reads a field off `arm`, and it
        # runs on the path where `build_robot()` FAILED — the `except Exception` prints the
        # error and falls through. That is the path Julien sees whenever the CAN adapters are
        # in DFU, which happens often. Without this declaration that line would raise
        # `UnboundLocalError` and replace a clear "No candleLight CAN adapter found" with a
        # traceback.
        #
        # ⚠️ So every read of `arm` outside the `try` MUST be guarded by `if arm is not None`.
        # `checks/check_restructure.py` finds the construction point by locating the
        # `ArmSession(` call rather than this line, so it can still catch a genuine
        # use-before-build.
        # ⭐⭐ ONE PUCK, N ARMS → THE PUCK FOLLOWS THE SELECTION. His design, 2026-08-18
        # (FINDINGS §68.8): with a single real SpaceMouse in a multi-arm session, `a` aims
        # the puck as well as the mode keys — B, then G, then BOTH, where BOTH drives both
        # arms at once, each from its own pose (his call: no mirror needed for that).
        # Unaimed arms read a centred puck. With one puck per arm nothing changes.
```

## Comment at source lines 1810-1826

```text
        # ⛔⭐ DECLARED HERE FOR THE SAME REASON `arm` IS, and the reason is now stronger than
        # it was for `arm`. The `finally` block and the closing summary both iterate `arms` —
        # to save each arm's axis map, and to report it — and both run on the path where
        # `build_robot()` FAILED. An unbound name there would replace *"No candleLight CAN
        # adapter found"* with a `NameError`, on the failure Julien hits most often.
        # ⭐ An empty list is better than a `None` guard: the loops simply do not run, so
        # there is no second code path to keep correct. FINDINGS §48.3.
        # ⛔⭐⭐ `start_mode` IS A SEPARATE NAME FROM `arm.mode`, AND THAT IS THE WHOLE REASON
        # `mode` WAS THE LAST FIELD TO MOVE. `build_robot()` below is called with
        # `zero_gravity=(start_mode == "guide")`, and it runs BEFORE the robot exists — so
        # before the `ArmSession` that would hold the mode can exist either. The name with the
        # most references (48) was therefore the last arm that could move, which is the
        # opposite of the order anyone would choose for comfort. FINDINGS §50.
        #
        # ⚠️ It is deliberately NOT the same variable. Keeping arm `mode` and assigning it
        # twice would put the script and the object out of step for the lines in between,
        # which is the state neither models.
```

## Comment at source lines 1828-1842

```text
        # ⚠️ `teleop` was declared here as None. It is `ArmSession.teleop` now, and the class's
        # own constructor already sets it to None. ⛔ Leaving this line would run before `arm`
        # exists, which the ordering check in checks/check_restructure.py catches.
        # ⚠️ The thermal guard used to be created here. It is `ArmSession`'s now, built by its
        # constructor from the same `warn_at=TEMP_WARN, stop_at=TEMP_STOP` this line passed.
        # ⛔ Leaving it here as `arm.thermal = …` would run before `arm` exists.
        #
        # ⭐ It stays an object rather than a pair of floats, and the reason is worth keeping:
        # "I cannot read the temperature" is a state that has to be tracked and acted on, and
        # it used to be indistinguishable from 0 °C. See `ThermalGuard`.
        # ⚠️ `hottest` and `jaw_temp` were declared here. They are `ArmSession` fields now, so
        # each arm reports its OWN temperatures on its own status row — as session locals they
        # were arm arm's reading painted on whichever row was being drawn.
        # ⛔ Not left here as `arm.hottest = None`: that would run before `arm` exists, which
        # is the ordering fault `checks/check_restructure.py` check 3 catches.
```

## Comment at source lines 1875-1884

```text
        # ⭐⭐⭐ ONE ROBOT PER ARM, BUILT IN ORDER. ROADMAP §6.1 step 3.
        #
        # ⛔ `build_robot()` energises motors and is the single most dangerous call in the
        # project, so it stays here, visible, in the script — never inside `ArmSession`.
        # With two arms it happens twice, and the second arm starts while the first is
        # already holding its pose under power.
        #
        # ⚠️ If the SECOND build fails, the first arm is already energised. The `finally`
        # block disables every arm it finds in `arms`, and the arm is appended as soon as it
        # is constructed, so a half-built session still shuts the built half down properly.
```

## Comment at source lines 1912-1923

```text
            # ⛔⭐⭐ THIS LINE IS NOT OPTIONAL, AND ITS ABSENCE WOULD HAVE BEEN SILENT.
            #
            # `ArmSession.__init__` sets `self.mode = "hold"`, which is the right default for
            # a class that may be built before anyone has chosen a mode. **The script has
            # already chosen arm**, from `--start-mode`, and `build_robot()` above has
            # already acted on it by deciding `zero_gravity`.
            #
            # ⛔ Without this assignment, `--start-mode guide` would build a WEIGHTLESS robot
            # and then run the loop believing it was in HOLD. **Nothing would raise.** The arm
            # would hang from gravity compensation alone while the screen said HOLD, which is
            # the defect class FINDINGS §0 exists for. Found by asking what the class's own
            # default is, not by anything failing. FINDINGS §50.2.
```

## Comment at source lines 1928-1938

```text
            # ⭐ DEFAULT PARK POSE = WHEREVER THIS ARM STARTED. Julien: *"if the standard set
            # position for park mode is just the starting position, then I can always just
            # press p and then d, and I don't have to do anything with my hands."*
            #
            # ⭐ Per arm, which removes a real dependency he flagged: the two arms do NOT have
            # to be physically placed the same way before a session, because each arm parks
            # back to its own measured start rather than to a pose recorded from the other.
            #
            # ⚠️ It is only as good as the pose you start in. Start with the arm drooped and
            # PARK will faithfully return it to drooped, which is why the plan prints the
            # actual numbers rather than just saying "default".
```

## Comment at source lines 1952-1970

```text
        # ⛔ DELIBERATELY NOT RE-CHECKING THE GRIPPER FRAME HERE. Do not add it back.
        #
        # A stale-limit check used to live at this point, and it was worse than
        # nothing for two reasons. It compared the raw jaw position against the
        # *unshifted* limits from the file, so it re-flagged exactly the cases
        # `frame_correct_gripper_limits()` had legitimately reconciled: at the
        # measured raw −1.380 it printed "STALE GRIPPER LIMITS … re-run
        # calibrate_gripper" while the frame was in fact correct and the jaws
        # normalised to 0.3005. And it then **warned and continued** — in the wording
        # of the one rule this project wrote in blood (FINDINGS §3.5) — so the real
        # message and a false alarm were indistinguishable, and the advice it gave
        # was to run a routine that drives the jaws into both mechanical stops.
        #
        # `build_robot()` already gates this twice, better, and BEFORE any control
        # loop starts: it refuses if no ±2π shift reconciles the saved range, and it
        # reads the normalised jaw position back from the runtime and shuts
        # everything down if it is outside [0,1]. Both raise. The `note` printed
        # above carries the verified value ("jaws normalise to 0.030 ✓"), which is
        # also the baseline to watch during the thermal test.
```

## Comment at source lines 1975-1992

```text
        # ⭐⭐ EVERY FUNCTION BELOW TAKES THE ARM IT ACTS ON. Step 2 plumbing, 2026-08-14.
        #
        # They were closures over the single `robot` and the single `arm`, which is the last
        # structural reason two arms could not run: a closure cannot be pointed at a second
        # arm. Passing the arm changes nothing at N=1 — the same object is handed in — and
        # it is what lets the mode dispatch loop over `selection.names()`.
        #
        # ⚠️ `clamp_gripper` above takes NO arm on purpose: the band is a property of the
        # gripper hardware, identical on both arms, and it is passed to `park_target_from`
        # as a one-argument callable.
        #
        # ⛔ These are still the script's own copies rather than `ArmSession`'s methods, and
        # that is a recorded decision, not an oversight: FINDINGS §52.1. Collapsing them is
        # ROADMAP §8.2 item 23, it changes what the arm is commanded at the margins, and it
        # therefore needs its own bench pass.
        # ⭐ resync lives on ArmSession too (item 23): every class enter_* method
        # calls self.resync() itself, so the script no longer owns any mode hygiene.
        # The GUIDE→TELEOP snap story lives in ArmSession.resync's docstring.
```

## Comment at source lines 2103-2112

```text
            # ⛔⭐⭐ A PARK STARTED BY ANYTHING ELSE CANCELS A PENDING PLAYBACK, and without
            # this the safety property of the whole playback feature can be broken by one
            # keypress. `l` parks to the recording's START POSE and hands over on arrival; if
            # the operator presses `p 0` while that park is running, the path is replaced by a
            # park to the BASE pose — and the arrival still hands over, so the recording would
            # begin from a pose it was never taught. The first command of a playback is the
            # only dangerous one, and that is exactly the one this would get wrong.
            #
            # ⚠️ Found by auditing rather than by hitting it: it needs `p` pressed inside the
            # two or three seconds the start-pose park takes.
```

## Comment at source lines 2307-2326

```text
        # ⭐⭐ THE MEASURED LENGTH OF THE LAST CYCLE, next to the nominal one. Found on
        # 2026-08-13 because Julien's playback summary did not add up: a 3.6 s recording
        # reported "3.6 s of movement plus 0.4 s waiting" and finished in 4.6 s. The
        # missing 0.6 s is the loop running below 100 Hz. `dt` is a constant, and the
        # sleep at the bottom is `max(0, dt - elapsed)`, so a cycle that overruns is not
        # compensated: nominal time falls behind the wall clock, roughly 87 Hz against 100.
        #
        # ⛔⭐⭐ AND THAT EXPLANATION WAS INCOMPLETE, corrected 2026-08-20. An overrunning
        # cycle is not the main reason. `time.sleep` itself returns LATE, and on macOS it
        # returns about 1.9 ms late at this scale, every pass. An empty loop with no arms,
        # no cameras and no work at all, using this exact line, runs at 84.3 Hz on the Mac
        # and 97.3 Hz on the Linux station. So the Mac's whole shortfall is the wait, the
        # 87 Hz figure is a macOS number rather than a property of this program, and moving
        # work off the loop cannot recover it. [PERFORMANCE.md](../docs/PERFORMANCE.md) §2
        # has both machines' numbers; it answers ROADMAP §8.2 item 14.
        #
        # ⛔ Anything that has to match real time must use `real_dt`, not `dt`. A playback
        # is exactly that: the whole point is reproducing the timing a hand taught.
        # ⚠️ Clamped, so one long stall (a slow disk, a thermal read retry) cannot make the
        # cursor jump forward and command a step the arm never asked for.
```

## Comment at source lines 2341-2360

```text
            # ⛔⭐ CTRL-C MUST NOT RELEASE THE ARM, and it used to.
            #
            # This whole session exists partly because "quitting released the arm on a
            # timer" and *a 5 s countdown is not consent* — so `q` was changed to go to
            # HOLD and wait for an explicit second key. **Ctrl-C went around all of
            # that.** SIGINT raised past the consent flow, past the `with`, into the
            # outer handler and straight to `finally`, which calls `shutdown_robot()`
            # and disables the motors. On a raised arm that is a sag, and Ctrl-C is
            # what everyone presses when something looks wrong.
            #
            # Found by reading on 2026-08-12, never yet triggered on hardware. It is
            # working contract rule 7 exactly: *what path reaches the hazard without
            # passing through your guard?* — this one, and the guard was the newest
            # code in the file.
            #
            # A handler rather than a try/except so the ~540-line loop body is not
            # re-indented for it. The first Ctrl-C sets a flag and restores the DEFAULT
            # handler, so the loop stops at the top of its next cycle and falls into
            # the same consent flow `q` uses — and a **second** Ctrl-C is a real force
            # quit, which is what someone pressing it twice means.
```

## Comment at source lines 2454-2473

```text
                            # ⛔⭐⭐ ALREADY LATCHED, SO SAY NOTHING AND DO NOTHING. This is the
                            # last piece of the repeated-message problem and the latch was not
                            # the fault.
                            #
                            # The detector fires on **high torque and no movement**. Holding
                            # the jaws at the block is exactly that: a position controller
                            # sitting at its target against an object still produces torque and
                            # still is not moving. ⚠️ So the condition stays true for as long as
                            # the grip is held, and every cycle re-reported a stall the latch
                            # had already dealt with.
                            #
                            # ⭐ His 2026-08-17 log is unambiguous. Arm B: released to 0.304,
                            # 0.311, 0.314, 0.315, 0.315, 0.315, 0.315, 0.316 — **eight
                            # messages, all creeping OPEN by a thousandth**, then one
                            # "opened past the block". Arm G did the same four times. The latch
                            # was working correctly throughout.
                            #
                            # ⛔ Re-latching was also wrong on its own: it moved the block to
                            # each new measured position, so a jaw slowly relaxing dragged the
                            # block open with it and the protection loosened by itself.
```

## Comment at source lines 2483-2488

```text
                                # ⛔⭐⭐ LATCH IT. Setting `gripper_value` alone was undone on
                                # the very next cycle by whatever is commanding the jaws —
                                # in MIRROR that is the leader's jaw, re-sent at 90 Hz. His
                                # 2026-08-17 log: released to 0.152, 0.151, 0.150, 0.147 and
                                # "(14 times now)". The latch is what makes the release stick
                                # ([ROADMAP §8.2](../docs/ROADMAP.md) item 29).
```

## Comment at source lines 2492-2497

```text
                                # ⛔⭐ SAID ONCE, THEN AT MOST EVERY FIVE SECONDS WITH A COUNT.
                                # The release happens every time; only the printing is rationed.
                                # In MIRROR the follower's jaw command is the leader's measured
                                # jaw, re-sent every cycle, so squeezing the leader while the
                                # follower holds an object fires this every 0.4 s indefinitely.
                                # Twenty identical lines is how a real warning becomes noise.
```

## Comment at source lines 2532-2550

```text
                    # ⭐⭐ WHICH ARMS THIS KEYPRESS IS AIMED AT. ROADMAP §6's split, and the
                    # three names below are the whole of it:
                    #
                    #   `aimed`    — every selected arm. Mode changes, poses, the gripper and
                    #                the park knobs act on all of them.
                    #   `edit_arm` — the FIRST selected arm, and the only one a MAP edit ever
                    #                touches. ⛔ Not a shortcut: `AxisMapStore.for_arm()`
                    #                returns the SAME `AxisMap` object to both arms while the
                    #                scope is SHARED (the default), so applying an edit to
                    #                each selected arm would flip a motion TWICE, back to
                    #                where it started, printing two confirmations
                    #                (FINDINGS §53.5). A map edit also comes from one physical
                    #                gesture on one puck, so one arm is the honest target.
                    #   `wizard`   — the arm inside CONTROLS, if any. At most one can be:
                    #                `m` refuses when two arms are selected, and `a` refuses
                    #                while CONTROLS is open.
                    #
                    # ⚠️ Recomputed every keypress, because `a` changes the selection inside
                    # this very loop.
```

## Comment at source lines 2555-2563

```text
                    # ---- a pending s/p consumes the NEXT key as its argument ----
                    # ⭐ Two-key sequences, because a bare digit is already taken: 1-3
                    # flip rotation axes in the drive modes and 1-6 select motions in
                    # CONTROLS. A digit AFTER s or p is an argument, not a command, so
                    # nothing has to be re-bound. This is the shape Julien proposed.
                    #
                    # ⚠️ It is a mode, in a loop that drives 4.3 kg — so it is bounded:
                    # exactly one keypress wide for `s`, echoed at every keystroke for
                    # `p`, and cancelled by anything unexpected rather than guessing.
```

## Comment at source lines 2567-2574

```text
                            # ⭐ EVERY SELECTED ARM SAVES ITS OWN POSE into that slot. The file
                            # is keyed by arm, so `s 1` with BOTH selected records two
                            # different poses under one digit, which is what a two-arm
                            # waypoint is.
                            # ⚠️ `data` is threaded through the loop and written ONCE, because
                            # `with_park_slot` returns a new dict rather than mutating. Saving
                            # inside the loop would write the first arm's version and then
                            # overwrite it with a copy that never saw it.
```

## Comment at source lines 2597-2629

```text
                        # ⛔⭐⭐ AN OCCUPIED SLOT ASKS ONCE MORE, and this is the fifth time
                        # the repo has paid for it not doing so. Twice an overwrite destroyed
                        # the only copy of a measurement ([FINDINGS §33.2](../docs/FINDINGS.md),
                        # §34.7), once again three hours later, and on 2026-08-14 Julien's
                        # first two-arm recording landed on `1.json` and replaced a
                        # hand-guided take from the day before. **Every time, the prompt said
                        # nothing about what was already there.**
                        #
                        # ⭐ Same shape as `l` and `p`: it names what is in the slot and the
                        # SAME digit confirms. Anything else discards, which is the contract
                        # the first prompt already states.
                        # ⛔⭐⭐ A DIFFERENT DIGIT RE-AIMS AT THAT SLOT. IT DOES NOT
                        # DISCARD. Julien, 2026-08-17: *"when I pressed save seven, then the
                        # guard came up. But then when I pressed a different number, I
                        # wanted to save it on, it's still discarded."*
                        #
                        # ⛔ He was exactly right, and the old rule was *"the SAME digit
                        # confirms, anything else discards"*. That turned the guard into a
                        # dead end: the only ways out were overwrite the take you were
                        # trying to protect, or lose the new recording. **The obvious third
                        # thing a person wants — put it somewhere else — was the one thing
                        # it would not do.**
                        #
                        # ⚠️ A recording is minutes of his time and cannot be re-taken
                        # identically, so discarding it should take a deliberate act, never
                        # be the default outcome of aiming at a busy slot.
                        #
                        # ⭐ The rule now, and it is simpler than what it replaces:
                        #   a digit on a FREE slot            → saves
                        #   a digit on an OCCUPIED slot       → asks
                        #   the SAME digit again              → replaces
                        #   a DIFFERENT digit while asking    → re-aims at that slot
                        #   anything that is not a digit      → discards
```

## Comment at source lines 2710-2719

```text
                        # ⭐⭐ ARROW KEYS MOVE THE SELECTION, because that is what a person
                        # presses at a list. Julien's first ever use of this screen produced
                        # `(key '\x1b[B' does nothing here)` twice: he reached for up and
                        # down, got a raw escape sequence echoed back, and a UI that prints
                        # its own escape codes at you looks broken even when it is not.
                        # ⭐⭐ A ONE-LINER FOR A CHANGE, THE FULL SCREEN ONLY WHEN ASKED.
                        # Reprinting all fifteen lines after every keypress gave him THIRTEEN
                        # copies in one session, which buries everything else the session did
                        # and makes the scrollback useless for reading back what happened.
                        # The rest of this file already works this way.
```

## Comment at source lines 2827-2840

```text
                        # ⭐⭐ 1.00x IS THE TAUGHT SPEED, AND THE DEFAULT IS WHATEVER THE ARM
                        # CAN ACTUALLY FOLLOW. Reworked 2026-08-13, on Julien's suggestion:
                        # *"maybe one x should just be the original speed, and then you could
                        # go up and down. So max speed would just be limited by the actual
                        # safety things we have or the motors."*
                        #
                        # ⛔ WHY THE OLD VERSION COULD NOT EXPLAIN ITSELF. Hand-guiding a
                        # weightless arm reaches 2.4 to 2.9 rad/s (his own three recordings,
                        # 99th percentile), while MAX_PLANNED_JOINT_SPEED is 1.5. So every
                        # recording came back reading "max 1.00x", he played it at 1.00x, and
                        # it took 2.3 s longer than the recording because the loop kept
                        # holding the clock to let the arm catch up. Nothing on screen said
                        # why. Now the plan states both numbers and starts at a speed that
                        # will actually track.
```

## Comment at source lines 2875-2887

```text
                            # ⛔⭐ PARK TO THE START POSE FIRST, AND THIS IS THE SAFETY
                            # POINT OF THE WHOLE FEATURE. Playback commands poses the arm
                            # is known to reach, because a hand physically put it there.
                            # The dangerous command is the FIRST one: if the arm is
                            # somewhere else right now, commanding the recording's opening
                            # pose is a jump across whatever separates them. So the
                            # existing, tested, interruptible park drives there, and only
                            # when it arrives does playback begin.
                            # ⭐⭐ EVERY ARM THE RECORDING DRIVES PARKS TO ITS OWN SLICE of
                            # the start pose. This is the safety point of the whole feature:
                            # playback commands poses a hand physically put the arm in, and
                            # the only dangerous command is the FIRST one, which would jump
                            # from wherever the arm is now.
```

## Comment at source lines 2987-2998

```text
                            # ⭐ Start/stop easing is INDEPENDENT of corner blending.
                            # Julien wanted the ends toggleable without giving up the
                            # smooth corners; `none` leaves immediately, which is what
                            # a shutdown move wants.
                            #
                            # ⚠️ Handled HERE as well as in the main dispatch below, and
                            # that duplication is deliberate: an unrecognised key while
                            # typing a sequence CANCELS the run, so a key that is only
                            # bound further down would abort the very move it was meant
                            # to configure. `+/-` and `,/.` are duplicated for the same
                            # reason. The two differ only in what they show — the whole
                            # plan while choosing, just the profile otherwise.
```

## Comment at source lines 3006-3011

```text
                            # ⭐⭐ COMPOSITE RUNS (ROADMAP §6.6.1a, his idea): inside a run,
                            # `w` then a digit names a RECORDING as a leg — `p 1 w2 3` means
                            # pose 1, then PLAY take 2, then pose 3. Precision lands where
                            # the task needs it (the taught take) and variation where it
                            # tolerates it (the planned poses). The two-key idiom matches
                            # `s <digit>` and `l <digit>`.
```

## Comment at source lines 3094-3106

```text
                            # ⛔⭐ THE STALE ROW JULIEN THEN PRESSED KEYS AT. `hint()`
                            # was never cleared anywhere in this file, so the
                            # `RUN 1 → 2 → 3 … ease s-curve over 0.20 (e, ö/ä) ·
                            # Enter=go` row stayed live for the rest of the session —
                            # after the run was cancelled, through mode changes, next
                            # to a `[HOLD]` status. It is a live row, repainted every
                            # cycle, which is why the same block appears twice in his
                            # paste with only the timestamp differing.
                            #
                            # Worse than untidy: it advertises `e` and `ö/ä`, he pressed
                            # `e`, and the reply was `(key 'e' does nothing)`. A hint
                            # that outlives the thing it describes is the `b` defect
                            # again (FINDINGS §17.1) — right text, wrong context.
```

## Comment at source lines 3227-3234

```text
                        # ⛔ Save the map for the frame being LEFT before switching.
                        # Each frame owns its own wiring, so carrying one frame's map
                        # into another would silently overwrite it — the same
                        # blast-radius bug as editing a shared map believing it was
                        # per-arm.
                        # ⭐ ONE arm's frame, like every other map-scoped edit (§53.5). Two
                        # arms may sit in different frames, which is the point: one driven in
                        # `world` while the other follows its own wrist in `tool`.
```

## Comment at source lines 3241-3247

```text
                        # ⛔⭐ THERE IS NOW ONE COPY OF THE FRAME, AND THAT IS THE REAL FIX
                        # FOR WHAT FINDINGS §52.7 CAUGHT. `ArmSession.frame` used to sit
                        # beside a session-level `control_frame` local, and only the local
                        # was updated here — so the object silently disagreed with the
                        # session after the first `v`. It was patched by assigning both.
                        # ⭐ Deleting the local is better than keeping them in step: a
                        # second copy that has to be maintained will eventually not be.
```

## Comment at source lines 3395-3404

```text
                        # ⭐ CONTROLS mode DRIVES the arm — that is the whole point, and it
                        # is why this calls enter_teleop() rather than enter_hold(). The
                        # previous version held the arm still, which made it useless for
                        # the actual task: you cannot decide that a direction is wrong
                        # until you have watched the arm go that way. Julien:
                        # *"the actual mapping has to happen while the arm is moving so I
                        # can see what the different directions are doing."*
                        # ⛔ ORDER: the class's enter_teleop() sets mode="teleop", so MAP is
                        # written AFTER it — the reverse would run CONTROLS while the row
                        # said TELEOP (the §52.1 trap, third instance).
```

## Comment at source lines 3447-3457

```text
                        # ⛔ ALREADY IN THAT MODE — say so, do not call it unrecognised.
                        # Julien's paste has `⭐ MODE: GUIDE` immediately followed by
                        # `(key 'g' does nothing — press ? for the list)`, which reads
                        # as the program having lost track of its own state. It had not:
                        # the mode branches above are guarded by `mode != …`, so a second
                        # press fell past them to the catch-all for unknown keys.
                        #
                        # ⚠️ The mode is deliberately NOT re-entered. `enter_guide()`
                        # re-arms gravity compensation and re-takes the drift reference,
                        # and this cannot be tested on the arm from here — so this fixes
                        # the message and changes nothing the motors see.
```

## Comment at source lines 3465-3470

```text
                        # ⭐ LABEL A STRETCH while recording (ROADMAP §8.2 item 8, the
                        # keypress half of his microphone idea): press k when it goes
                        # wrong, press k again when it is good again. A recording starts
                        # implicitly good, so the first press always means BAD-from-here.
                        # ⚠️ Labels are DATA for the dataset export, never control: a bad
                        # stretch still plays back, and nothing about the motion changes.
```

## Comment at source lines 3481-3488

```text
                        # ⭐ START OR STOP RECORDING. Deliberately allowed in EVERY mode,
                        # not only GUIDE. Hand-guiding is the intended use and the reason
                        # the feature exists, but a teleop run is also a demonstration and
                        # refusing to record one would be an arbitrary restriction. The
                        # mode in force is written into the metadata instead.
                        #
                        # ⚠️ Recording moves nothing, so a mis-press is harmless. That is
                        # why `w` needs no confirmation while `l` does.
```

## Comment at source lines 3512-3524

```text
                            # ⛔⭐ STOP MEANS STOP, AND THIS WAS A REAL BUG FOUND ON THE ARM
                            # ON 2026-08-13. `take` was left in place while the "which slot?"
                            # prompt waited for a digit, and the per-cycle sampler keys off
                            # `take is not None` — so the recording kept growing for as long
                            # as Julien took to answer. Measured from his own three files:
                            # 1.8 s, 4.4 s and 3.3 s of extra samples appended AFTER the stop
                            # keypress, with 0.1 to 0.7 rad of movement in the tail. He
                            # described it exactly: *"the recordings played for like two
                            # seconds longer than I actually recorded, it was just standing
                            # still for that time."*
                            #
                            # Moving it to a second name is the whole fix: the sampler stops
                            # on this line, and the prompt then saves something frozen.
```

## Comment at source lines 3549-3559

```text
                        # ⛔ PLAY A RECORDING, AND IT ASKS TWICE ON PURPOSE. `l` sits next
                        # to `ö` and `ä` on a German keyboard, which now adjust the ease
                        # ramp — so a slip lands on a key that would otherwise start 4.3 kg
                        # moving. Showing the plan and waiting for Enter means a stray `l`
                        # can never move the arm. Same shape as `p 1 2 3 Enter`.
                        # ⛔ REFUSED WHILE A PLAYBACK IS RUNNING (FINDINGS §68.4): on
                        # 2026-08-18 `l` mid-scrub opened this prompt ON TOP of the running
                        # playback — two live states again, the item 38 family — and the
                        # scrub kept driving the arm underneath the menu. One playback at
                        # a time; stopping it stays an explicit act (h or t), never a side
                        # effect of asking for the next one.
```

## Comment at source lines 3613-3631

```text
                        # ⭐⭐ ö AND ä MEAN THE EASE RAMP EVERYWHERE NOW. Changed
                        # 2026-08-13, after Julien hit the confusion on the arm.
                        #
                        # ⛔ WHAT WENT WRONG. `e` cycles the ease profile in any mode, and
                        # its own message read *"(ö/ä adjusts how long)"*. But outside a
                        # park prompt those keys were bound to the GRIPPER STEP, so the
                        # message told him to press keys that did something else. He
                        # pressed them and pushed the gripper step to its 0.200 ceiling by
                        # accident, which makes every later `o` or `c` move the jaws a
                        # fifth of their travel. His words: *"the German characters ö and ä
                        # don't quite work as I think they should… they change the gripper
                        # step speed, which in itself might be cool, but not necessary
                        # currently and all the time."*
                        #
                        # ⭐ The fix is one meaning per key, not a cleverer message. A
                        # message that has to explain which of two things a key does today
                        # is a design admitting it is wrong. The gripper step moves to
                        # `--gripper-step`: it is a preference set once, and he said
                        # outright that it does not need a live key.
```

## Comment at source lines 3641-3650

```text
                        # ⭐ WORKS EVERYWHERE, and the message now says WHAT IT AFFECTS.
                        # Julien, on the arm: *"the easing outside of parking, I don't
                        # really know what that means. Does it work for recording, or does
                        # it work for teleoperating? … what's the point of that?"* A fair
                        # question, and the answer is neither. Easing shapes only PLANNED
                        # moves: `p` runs and the Ctrl-C park. It does nothing for driving
                        # by hand or for a playback, which follows recorded timing instead.
                        # Pressing `e` elsewhere sets up the next planned move. A knob whose
                        # effect you cannot see has to say where its effect lives, every
                        # time it is touched.
```

## Comment at source lines 3732-3740

```text
                # ⛔ Leaving PARK for ANY reason abandons the rest of the run. One
                # place rather than a clear() in each of g/t/h/m/blocked, because the
                # one that gets forgotten is the one that matters: an arm resuming a
                # planned trajectory after the operator pressed HOLD is doing something
                # nobody asked for.
                # ⛔ Leaving MIRROR by any route drops the link. One place rather than a
                # cancel in each of g/t/h/i, because the one that gets forgotten is the one
                # that matters: a follower still tracking after the operator pressed HOLD is
                # an arm moving for a reason nobody can see.
```

## Comment at source lines 3760-3778

```text
                    # ⛔ A park that was interrupted must not hand over to a playback. The
                    # handover lives in the arrival branch, but this is the second gate:
                    # pressing h or t while driving to the start pose cancels the whole
                    # thing, rather than leaving a recording queued to fire later.
                    #
                    # ⛔⭐⭐ `unfinished` IS THE WHOLE FIX, and without it two-arm playback
                    # could never start. This used to cancel whenever it found a path on an
                    # arm that had left `park`, which includes an arm that ARRIVED. With one
                    # arm that never showed, because the arrival handed over in the same
                    # cycle and left nothing pending. With two arms the first arrival waits
                    # for the second, so the pending playback was still there to cancel:
                    # *"arm B is at the start pose; waiting for G"* and then *"playback
                    # cancelled — it never reached the start pose"*, one line apart, in
                    # Julien's own log. FINDINGS §57.1.
                    #
                    # ⭐ Deciding from the MEASURED remaining path rather than from whether
                    # some other branch remembered to tidy up is the fix that cannot rot: a
                    # future exit that forgets to clear the path still cannot cancel a
                    # playback whose park actually finished.
```

## Comment at source lines 3784-3794

```text
                # ⛔ Same rule for a playback in progress: leaving the mode abandons it.
                # An arm resuming a recorded movement after the operator pressed HOLD is
                # doing something nobody asked for.
                # ⚠️ Session-level, because the playback cursor is.
                # ⛔⭐⭐ ANY arm leaving replay ends the playback for EVERY replay arm
                # (FINDINGS §69.1). This said "when NO arm is in replay" until 2026-08-18,
                # and mode keys only re-mode the AIMED arm — so with a two-arm playback,
                # `m` aimed at B left arm G a ZOMBIE: mode REPLAY, cursor frozen (the
                # advance needs ALL replay arms in the mode), no message, for the rest of
                # Julien's session. A playback is one thing; it ends as one thing, and
                # the arms it releases go to HOLD through the class.
```

## Comment at source lines 3817-3835

```text
                # ---- 3.4 sample the recording, if one is running ---------------
                # ⭐ EVERY CYCLE, IN EVERY MODE, and before the mode acts. Recording is a
                # property of the session rather than of a mode, so putting it in a branch
                # would silently stop capturing the moment the operator switched modes —
                # which is the same defect shape as the puck being read only inside the
                # teleop branch, fixed just below.
                #
                # ⚠️ It records the MEASURED position. For a hand-guided demonstration that
                # is the only thing that means anything: in GUIDE the position gain is zero,
                # so there is no command to record, and the arm is wherever the hand put it.
                #
                # ⭐⭐ EVERY ARM, CONCATENATED IN `--arms` ORDER, which is exactly the shape
                # ABC wants: 14 states per timestep, two arms in ONE timeline (ROADMAP §9.2).
                # `src/yam/recording.py::Layout` owns the mapping and its tests.
                #
                # ⚠️ Sampled in ONE list comprehension so every arm's position comes from the
                # same cycle. Reading the arms in separate statements would put a control
                # cycle between them, and a demonstration whose two halves are 10 ms apart is
                # a demonstration with a lie in it.
```

## Comment at source lines 3837-3845

```text
                    # ⛔⭐ A RECORDING PROBLEM MUST NEVER TAKE DOWN THE SESSION, and this
                    # wrapper is the whole reason the block exists. `append` raises on a
                    # non-monotonic timestamp or a changed joint count. Unwrapped, that
                    # exception would leave the control loop, skip the "the arm is HOLDING,
                    # press g or d" consent flow, and fall into `finally`, which disables
                    # the motors. On a raised arm that is a sag. It is the exact path that
                    # dropped 4.3 kg once already (FINDINGS §11, park_target_from), and
                    # recording is a convenience feature: it has no business being able to
                    # release the arm.
```

## Comment at source lines 3921-3929

```text
                            # ⛔⭐⭐ AN UNPLUGGED PUCK MUST NEVER DROP THE ARMS. On
                            # 2026-08-18 Julien pulled a SpaceMouse mid-session; `read()`
                            # raised `OSError: read error`, the exception skipped the
                            # auto-park entirely, and the `finally` disabled every motor
                            # with the arm wherever it stood (FINDINGS §68.2). His ruling:
                            # a graceful quit. So: the dead puck reads as CENTRED (zero
                            # deflection, the same honest stand-in --sim uses), and the
                            # stop_reason routes through the SAFE STOP — every live arm
                            # parks, then the motors disable.
```

## Comment at source lines 3944-3950

```text
                        # ⭐ TWO PROMPTS NEVER STAY OPEN AT ONCE — the newest wins (item 38,
                        # FINDINGS §63.3, his ruling "do that sensibly"). Button learning
                        # can only be ARMED while no keyboard prompt is open (prompt
                        # handlers consume every key, b included), so if both are armed
                        # the keyboard prompt came second and the learning yields. Without
                        # this, a puck press he made for the prompt era was consumed by
                        # the learner and silently rebound a gripper button.
```

## Comment at source lines 3986-3996

```text
                # ⭐⭐ EVERY ARM ACTS ON ITS OWN MODE, in `--arms` order. ROADMAP §6.1.
                #
                # ⛔ Driving is NOT aimed by the selector: each arm follows its own puck
                # every cycle, which is the whole point of two arms. Only mode changes and
                # edits are aimed (`a`, and `ArmSelector`).
                #
                # ⚠️ The replay branch reads SESSION state — one cursor, one recording —
                # because ABC wants both arms in one timeline (ROADMAP §9.2). With two arms
                # in replay it would drive them from the same slice, which is wrong; that is
                # why `l` refuses when more than one arm is connected until the two-arm
                # recorder exists.
```

## Comment at source lines 4029-4041

```text
                        # ⭐⭐ THE WORKSPACE LIMIT, changed on 2026-08-14 by Julien's decision.
                        #
                        # It used to be a ±0.30 m cube centred on wherever TELEOP was entered.
                        # Measured on the arm, that stopped him at 0.524 m from the base while
                        # the arm reaches 0.738, and the wall sat somewhere different every
                        # session. It is now a fixed 0.60 m sphere around the base plus a floor.
                        # Why a floor: the cube had been providing one for free, and this arm
                        # can put its tip below its own base. See teleop.clamp_to_workspace and
                        # FINDINGS §43.
                        #
                        # ⚠️ Clamped against the ACHIEVED position so the limit ratchets inward
                        # and never yanks an arm that starts outside it. The old cube could not
                        # be entered from outside; a fixed one can.
```

## Comment at source lines 4079-4089

```text
                            # ⛔ The link stopped itself. Continuing would keep commanding a
                            # pose the follower cannot reach, which is how a motor ends up
                            # held against a stop.
                            #
                            # ⭐ The reason NAMES the joint and the measured leader speed, and
                            # this line adds the joint's real name plus how to start again —
                            # both were missing when Julien first hit it.
                            # ⛔⭐ ONE FACT PER LINE. `StatusLine.say()` truncates each line
                            # to the terminal width while the live block is on screen, and
                            # Julien's first high-speed run lost the end of a long stop message
                            # to an ellipsis — the half that named the cause.
```

## Comment at source lines 4097-4102

```text
                            # ⭐⭐ THE HARDWARE'S OWN EVIDENCE, which the pure class cannot
                            # see. `SafeRobot` counts every cycle on which one of its two
                            # limits actually bit, and one of those limits is the 0.25 rad
                            # following-error clip. A high count during a mirror run says the
                            # COMMAND was being held back from running ahead of the arm, which
                            # is the mechanism behind the "the arm could not track" case.
```

## Comment at source lines 4142-4152

```text
                                # ⛔⭐ THIS LINE USED TO CITE "2.4-3.7 rad/s" AS WHAT A HAND
                                # CAN DO, and on 2026-08-17 it printed that **in the same
                                # message as a measured leader speed of 5.66 rad/s**. The
                                # range came from [FINDINGS §37.2](../docs/FINDINGS.md),
                                # measured from three playbacks in August, and his hand has
                                # since been measured at 5.66 and 6.83 rad/s.
                                #
                                # ⚠️ A hardcoded range in a message that also prints a live
                                # measurement will eventually contradict itself, and the
                                # contradiction discredits the whole message. **Quote the
                                # measurement, not the memory.**
```

## Comment at source lines 4225-4232

```text
                            # ⛔⭐⭐ THE PURPOSE IS TAKEN BEFORE THE QUEUE MAY ADVANCE (FINDINGS §72.1).
                            # The composite advance two lines down can ARM A TAKE LEG inside this very
                            # event; without this capture, the pose-leg arrival being handled right now
                            # fell through into the ready-check below and was credited as "this arm is
                            # at the recording's start" — the recording then played while the arm still
                            # had 1.28 rad of park to go, measured on the bench 2026-08-19. The credit
                            # is only valid for the park THIS arrival completed, and that park's purpose
                            # was stamped when it began.
```

## Comment at source lines 4258-4264

```text
                                    # ⛔⭐ MEASURE BEFORE PLAYING — the second, independent defence
                                    # (FINDINGS §72.1): the bookkeeping above was wrong once, so the
                                    # gate no longer trusts it alone. Every replay arm's ARM joints
                                    # (jaws excluded — a jaw holding an object sits off on purpose)
                                    # must actually BE at the recording's start, measured NOW. A
                                    # refusal here costs a retry; a pass-through costs a playback
                                    # dragging the arm from the wrong pose under the max-lag ratchet.
```

## Comment at source lines 4430-4442

```text
                    # ⭐ THE PER-JOINT ANSWER TO "HOW FAST CAN THE ARMS MOVE", collected from
                    # motion Julien is already running rather than from a speed sweep that
                    # would command the arm faster than any existing code allows. Reasoning
                    # in src/yam/recording.py::TrackingLog and ROADMAP §7.5.
                    # ⛔⭐⭐ `measured`, NOT `q`. When this block moved out of the per-arm
                    # loop it kept reading `q`, which had been that arm's measured pose. `q` is
                    # still a bound local of `main()` from OTHER branches — the park, the save
                    # handler — so it did not raise. It silently fed the tracking log a stale
                    # 7-element snapshot from the end of the park, against 14-element targets,
                    # and `TrackingLog.observe` quietly compared only the first seven joints.
                    # ⚠️ Worse than a crash: that table is the only measurement anyone has of
                    # what the arm can follow, and it would have been wrong with no sign.
                    # FINDINGS §57.2.
```

## Comment at source lines 4517-4523

```text
                            # ⭐⭐ AND KEEP IT. This table is the only measurement anyone has
                            # of what the arm can physically follow, and on 2026-08-13 the
                            # only copy of it was a paste into a chat window. Saved per
                            # playback under a timestamp, so nothing overwrites anything.
                            # ⚠️ Never lets a failed write end a session: the arm is in HOLD
                            # at this point and a missing diagnostic file is not worth a
                            # traceback. FINDINGS §34.4.
```

## Comment at source lines 4525-4531

```text
                                # ⭐ `names` is built ONCE above and shared with the printed
                                # table. It used to be computed here as well, and the two
                                # copies drifted: this one labelled every row with its arm
                                # and the printed one did not. Names per ARM matter because
                                # with two arms the table is 14 rows while `YAM_JOINTS` only
                                # names seven, and "base_yaw" appearing twice with no way to
                                # tell which arm is a measurement nobody can act on.
```

## Comment at source lines 4637-4647

```text
                    # ⭐⭐ ONE ROW PER ARM. ROADMAP §6.1 step 2.
                    #
                    # ⭐ THE SESSION FACTS RIDE THE FIRST ROW AND ARE NOT REPEATED. The
                    # clock, the recording and the loop rate belong to the session, not to
                    # an arm, and printing them twice would invite reading two arms as
                    # having two clocks. Later rows are padded to the same width so the
                    # temperature columns line up down the block.
                    #
                    # ⚠️ Padded with `display_width`, not `len`: `⏺` and `⚠️` are one
                    # character and two columns, so `len` would misalign the rows by
                    # exactly the number of symbols (`src/yam/ui/screen.py::display_width`).
```

## Comment at source lines 4679-4721

```text
            # ⭐ CTRL-C IS A GRACEFUL SHUTDOWN, NOT A QUESTION. Julien, 2026-08-12:
            # *"if we hit control c it should just instantly go back into the starting
            # position and then disable itself, without allowing for the options,
            # because control c is typically just a quit."*
            #
            # He is right about the convention, and the park pose defaults to the pose
            # the arm was in when the session began — so this returns it to where he
            # left it and lets go, which is what "quit" should mean here.
            #
            # ⚠️ Note what this means: **Ctrl-C now MOVES the arm.** That is unusual
            # for an interrupt and is why it announces itself first and why a second
            # Ctrl-C stops the motion immediately. The motion itself is the same slow,
            # bounded, stall-guarded trajectory PARK always uses.
            #
            # ⛔ And it is automatic ONLY on the happy path. If the park stalls or the
            # chain dies, the arm is NOT released — it falls through to the consent
            # flow below, because "I could not reach the safe pose" is exactly when a
            # human should decide rather than a default.
            # ⭐⭐ EXTENDED 2026-08-14 FROM CTRL-C TO EVERY UNPLANNED STOP. Julien's
            # request, after a chain death dropped the arm: *"when the robot is being
            # moved and stuff, and then it crashes for some reason, it should always
            # resort to trying to do the safe crash… It should be, like, when I do
            # control c."*
            #
            # ⛔⭐ BE EXACT ABOUT WHAT THIS CAN AND CANNOT DO, because the failure that
            # prompted it is the one case it cannot help. When the CAN link dies —
            # which is what happened, all seven motors latching `0xD loss of
            # communication` (FINDINGS §46) — **the arm cannot be commanded at all**,
            # so no park is possible and it sags. `chain_alive()` is the gate, and it
            # was already false in that session.
            #
            # ⭐ What it DOES cover is every other way a session can end badly, and
            # those are the majority: an exception in our own loop, a thermal stop, a
            # guard refusing, an IK failure. In all of those the chain is still alive
            # and the arm was previously left holding until a human answered a menu.
            #
            # ⚠️ A thermal stop parks too, deliberately. Holding keeps current in a hot
            # motor indefinitely and disabling drops the arm; parking gets it to a
            # supported pose and THEN removes current, which is better than both.
            #
            # ⛔ `q` is deliberately NOT auto-parked. Julien uses `q p d` and may want
            # `g` instead, so a planned quit keeps its menu. Only unplanned stops park
            # themselves.
```

## Comment at source lines 4767-4779

```text
                        # ⭐⭐ PARK THEN DISABLE, ONE KEY. Julien's request, 2026-08-14:
                        # *"There should be an option that just combines park and disable.
                        # Maybe just pressing q should allow for park and disable."*
                        #
                        # `q q` is now the keyboard equivalent of Ctrl-C, which has parked
                        # and disabled in one action since 2026-08-12. Same motion, same
                        # guards, same interruptibility.
                        #
                        # ⛔ IT ONLY RELEASES THE ARM IF THE PARK ACTUALLY ARRIVED. That is
                        # the same rule the Ctrl-C path follows: *"I could not reach the safe
                        # pose"* is exactly when a human should decide rather than a default.
                        # A stalled or interrupted park leaves the arm holding and the menu
                        # open.
```

## Comment at source lines 4886-4900

```text
        # ⛔ DO NOT make this an unconditional save again.
        #
        # It was one, and on 2026-08-10 it wrote a map mangled by the old bind-on-
        # deflection MAP mode straight over Julien's hand-dialled file. The values had
        # been produced on real hardware and were only recoverable because the file
        # happened to be committed. Two changes: nothing is written unless the map
        # actually changed, and the previous contents are kept alongside it.
        # ⚠️ THIS IS THE `finally` BLOCK, so it also runs on the path where
        # `build_robot()` failed and no arm was ever created — the path Julien hits
        # whenever the CAN adapters are in DFU (FINDINGS §48.3). `arms` is empty there,
        # so the loop simply does not run and nothing is written unless `--fork-map`
        # already changed the store.
        #
        # ⭐ Each arm saves ITS map under the frame IT ended in. Both live on the object,
        # which is why this no longer needs a session-level copy of either.
```

## Comment at source lines 4914-4936

```text
            # ⭐⭐ RECORD THE MOMENT, IF SOMETHING WENT WRONG. FINDINGS §45.
            #
            # On 2026-08-14 the arm fell because a motor stopped answering the CAN bus,
            # and everything about that instant was lost: no torques, no temperatures,
            # and no record of the USB bus, which is where the leading explanation
            # turned out to be. Recovering the gravity torques took a simulation of the
            # joint angles the arm had already measured and discarded.
            #
            # ⛔ PLACED HERE ON PURPOSE: after `shutdown_robot()`, so the motors are
            # already off before any of this is attempted. `write_incident` cannot
            # raise, and every field it gathers is individually guarded, because half
            # of them throw on a chain that has already died. A crash report that
            # delays the teardown would be worse than no crash report.
            #
            # ⚠️ Only on a bad stop. A normal `q p d` writes nothing.
            #
            # ⛔⭐ THE WHOLE BLOCK IS WRAPPED, and the reason is FINDINGS §42.0: a dry run
            # returns long before this line, and no headless test can reach it either,
            # because it needs a real robot. So this code path's FIRST execution will be
            # on the arm, during a failure. `src/yam/incident.py` is unit-tested and every
            # field is individually guarded — this outer guard exists because a path that
            # cannot be tested should not be able to add a second traceback on top of the
            # one the operator is already reading.
```

## Comment at source lines 4940-4948

```text
                # ⚠️ Every value below is the LAST one the loop managed to read, not a
                # fresh read. A fresh read on a dead chain raises, and the last good
                # reading is what actually describes the failure. Each arm keeps its own
                # last read in `one.states` / `one.temps` for exactly this.
                #
                # ⭐ ONE ENTRY PER ARM, in `arms` order. With two arms a single `"arm"` key
                # would have to name one of them, and the other arm's torques at the moment
                # of the fall are the ones that might explain it. `arms` is empty on the
                # failed-build path, so the list is simply empty rather than guarded.
```

## Comment at source lines 4999-5011

```text
    # ⛔⭐ GUARDED, AND THIS IS THE WHOLE REASON `arm` IS DECLARED None ABOVE.
    #
    # This runs after the `finally`, which means it also runs when `build_robot()` FAILED
    # and the `except Exception` printed the error. That is the path Julien sees whenever
    # the CAN adapters are in DFU. Without the guard, `arm.thermal` would raise there and
    # replace *"No candleLight CAN adapter found"* with a traceback. FINDINGS §48.3.
    #
    # ⭐ The failed-build branch also reads better than what it replaced: it used to print
    # `hottest motor seen this session: 0°C`, which is a fabricated number for a session
    # that never ran. A thermal guard reporting a plausible zero is the exact defect
    # `ThermalGuard` was written to remove ([FINDINGS §24](../docs/FINDINGS.md)).
    # ⭐ One temperature report per arm, and `arms` is EMPTY when the build failed — the
    # same guard as everywhere else in this teardown, expressed as a loop that does not run.
```

## Docstring: module

```text
"""⭐ Interactive arm session: switch between hand-guiding and SpaceMouse at will.

    uv run apps/teleop_session.py            # dry run: explains the keys
    uv run apps/teleop_session.py --yes      # LIVE

⛔ MOVES THE WHOLE ARM. Desk clear, hand near the power.

    KEY   MODE
    g     GUIDE   — zero gravity. The arm is weightless; push it where you like.
    t     TELEOP  — the SpaceMouse drives the end effector.
    h     HOLD    — the arm holds its current pose. The safe idle.
    p     PARK    — slowly drive back to the saved park pose.
    m     CONTROLS — set up the mouse. The arm MOVES, one isolated axis, half speed.
    s     save the current pose as the park pose.
    x/y/z flip that motion of the SpaceMouse mapping (saved on exit).
    +/-   faster / slower.
    ?     print this again.
    q     QUIT — goes to HOLD first and asks; it never just releases the arm.
    ^C    the SAME as q — stops and asks. Press it twice to force a real quit.

⛔ CONTROLS mode is where the axis map gets set up, ON THE ARM, and that is not a
convenience. An earlier version held the arm still and this docstring recommended
`apps/map_axes.py` for the "first dial-in… with the arms unplugged". Julien
showed that was wrong: **you cannot decide a direction is wrong until you have
watched the arm go that way.** The map is not a property of the input device — it
is a property of the device *and* how the arm is turned on the desk, and only one
of those is in a file. `map_axes.py` remains useful for sign tweaks away from the
bench; it cannot tell you what a direction *is*.

WHY IT IS ONE SESSION AND NOT A SEQUENCE OF SCRIPTS
---------------------------------------------------
Julien, after the first run: *"it shouldn't be in phases. It should be more like
going forward and backward. I should be able to control when the weightless or
SpaceMouse-controlled things are happening."* He is right, and there is a safety
argument for it beyond convenience: **zero gravity cancels the arm's weight, so
the instant any process stops commanding, the weight is back.** Every gap between
scripts is a moment when a raised arm sags. One long-lived session with mode
switches has no gaps.

FOUR REAL FAILURES FROM THE PREVIOUS RUN, ALL FIXED HERE
--------------------------------------------------------
1. **The control thread died and the loop carried on regardless.** Motor 7 hit an
   over-temperature fault at t≈24 s; I2RT's control thread raised and exited. The
   teleop loop kept solving IK and kept calling `command_joint_pos` into a dead
   robot for another **64 seconds**, printing plausible EE numbers the whole time
   while the arm did nothing. Julien saw exactly that. **Now every cycle checks
   the chain is alive and stops the instant it is not.** A loop that cannot tell
   whether its commands are arriving is worse than one that crashes.
2. **The arm drooped when the thread died.** With no commands arriving, the
   motors' own 400 ms timeout damps them — so the arm sank slowly under gravity
   rather than holding. Now the session detects the death immediately and says so
   loudly, so a human can catch it while it is still a slow sag.
3. **Motor over-temperature came as a surprise.** Temperatures are now read every
   cycle from the chain, shown live, warned at 55 °C and stopped at 65 °C — below
   the firmware's own trip, so the session ends in a controlled way instead of
   the thread dying underneath it.
4. **Quitting released the arm on a timer.** A 5 s countdown is not consent. Now
   `q` moves to HOLD and waits for an explicit second key. ⚠️ **And that fix had a
   hole in it for two days: Ctrl-C went around the consent flow entirely** and
   disabled the motors from `finally`. Fixed 2026-08-12 — the first Ctrl-C is now
   the same request `q` is, the second forces the quit.

ON RECOVERING A DROOPED ARM — yes, and nothing is lost
------------------------------------------------------
The encoders report true joint positions at all times; that is why a hand-twist
of the gripper this morning read back exactly. When the arm drooped, the system
did **not** lose track of where it was — it lost the loop that was commanding it.
So PARK is safe: it reads the true current pose, then interpolates slowly to the
saved one. It is not dead reckoning and it cannot be "miscalibrated" by a droop.
⚠️ What it cannot know is what is now in the way, so it moves slowly and can be
stopped with any key.
"""
```

## Docstring: park_arms

```text
    """Drive EVERY arm to its own base pose, **blocking**, and say how it ended.

    Returns the worst outcome across the arms: `"dead"` · `"stalled"` · `"stopped"`
    (a key was pressed) · `"arrived"`. Per-arm detail is printed as it happens.

    ⭐⭐ EVERY ARM ADVANCES ON EVERY CYCLE rather than one arm after another, and the
    reason is not speed. Sequential parking would make *"any key stops it"* stop only
    the arm currently moving, and it would leave the other arm holding a pose for the
    whole of the first arm's park with nobody watching it. One loop, N commands.

    ⚠️ A DEAD ARM IS SKIPPED, NOT FATAL. If one chain has died that arm cannot be
    commanded at all — it is already sagging — while the live arm can still be parked,
    which is better than leaving it holding and far better than disabling it. The dead
    one is named, loudly, because someone may need to catch it.

    ⛔ THE DUPLICATION THIS REMOVES, kept from the single-arm version because it is still
    the reason this function exists: the quit path used to carry its own copy of this
    loop, and the code's own comment admitted the risk — *"This is a SECOND park loop,
    and duplication is what has bitten this repo four times."*

    ⚠️ The *interleaved* park (mode == "park") is deliberately NOT folded in here. That
    one advances a single step per control cycle so the operator can still press keys and
    the temperature guard still runs; this one blocks because the session is already
    ending. Same trajectory maths, different scheduling — collapsing them would mean a
    blocking call inside the 100 Hz loop.

    ⚠️ `stall_seconds` is a parameter only so the tests can ask for a 0.2 s patience
    instead of waiting the real 4 s. Nothing in the session passes it.
    """
```

## Docstring: save_slot_action

```text
    """What should pressing `key` at the SAVE prompt do? Pure, so it can be tested.

    Returns one of:
      `("save", slot)` · `("ask", slot, already_asking)` · `("discard", kept_slot_or_None)`

    ⛔⭐⭐ THIS DECIDES WHETHER MINUTES OF JULIEN'S WORK SURVIVE, and it had no test until
    2026-08-17. `occupied` is the description of what is already in the slot, or None if the
    slot is free.

    ⭐ THE RULES, and rule four is the one he asked for:

    | keypress | outcome |
    |---|---|
    | a digit on a **free** slot | ✅ saves |
    | a digit on an **occupied** slot | ⚠️ asks first |
    | the **same** digit again | ✅ replaces |
    | ⭐ a **different** digit while asking | **re-aims at that slot** |
    | anything **not a digit** | ⛔ discards |

    ⛔ Rule four used to be "discard". Julien, 2026-08-17: *"when I pressed save seven, then
    the guard came up. But then when I pressed a different number, I wanted to save it on,
    it's still discarded."* That made the guard a dead end whose only exits were overwriting
    the take he was protecting or losing the new one. **The obvious third thing a person
    wants, put it somewhere else, was the one thing it would not do.**

    ⚠️ A recording cannot be re-taken identically, so discarding it must be a deliberate act
    rather than the default outcome of aiming at a busy slot.
    """
```

## Docstring: flat_joint_names

```text
    """`["B base_yaw", …, "G base_yaw", …]` for a flat multi-arm sample.

    ⛔⭐⭐ WHY THIS IS A FUNCTION RATHER THAN AN EXPRESSION IN TWO PLACES. It WAS an
    expression in two places, and the two drifted. The saved tracking JSON labelled every
    row with its arm; the printed table used `YAM_JOINTS.get(i + 1)` on the **flat** index,
    so with two arms it named arm B's joints correctly and called every one of arm G's
    "joint" — flat indices 7-13 become keys 8-14, and `YAM_JOINTS` only holds 1-7.

    ⛔ Julien's 2026-08-17 playback log is the evidence: six named rows, six anonymous
    ones, and nothing saying which arm any row belonged to, so the six named ones read as
    "the arm" when they were only arm B. ⚠️ Same family as the `label_verdict` defect —
    code written for one arm producing confident, plausible, wrong output with two, raising
    nothing.

    `total` clips the list to however many joints were actually tracked.
    """
```

## Docstring: status_row

```text
    """ONE arm's heartbeat row: its mode, its temperatures, its pose, its warnings.

    `lead` is the session's own facts — the clock, the recording, the loop rate — which
    ride the FIRST row only. Later rows get a run of spaces of the same display width, so
    the temperature columns line up down the block.

    ⭐⭐ WHY THIS IS A FUNCTION AND NOT SIXTY LINES INSIDE THE LOOP. It was those sixty
    lines until 2026-08-14, which meant the row a human reads to know what the arm is
    doing could only ever be executed on the arm — no test could reach it, and a
    formatting error in it would surface as a session dying one second after starting.
    As a function it is testable against a fake arm, which is the same argument
    `ArmSession` itself is built on: **the class decides, the script narrates, and the
    narration is worth proving too.**

    ⚠️ It reads and formats. It must never command anything, and it must never raise —
    a display fault has no business stopping a session that is holding 4.3 kg.
    """
```


## Incident assembly extraction — September 16, 2026

Historical source comments from the operator at `142d5a9`; current schema assembly lives in `yam.incident.session_facts`.

```python
# ⛔ A LAMBDA, not `safe_fact(loop_timer.to_dict)`. `_safe_fact`'s own docstring says a local here may be unbound if the loop never ran a cycle, and the attribute access in the shorter form happens OUTSIDE its try, so on the failed-build path it would raise and take the whole incident file with it.
# ⭐ The field whose absence cost the most on 2026-08-14: the
# gravity torques at the moment of failure had to be recovered by
# simulating the joint angles, when the arm had measured them and
# thrown them away.
# ⭐ Read at the top of the teardown, never here: after the
# disable loop above, `one.alive()` is False on every path.
# The key is renamed on purpose, so old incident files (whose
# `chain_alive` was always the meaningless post-shutdown read)
# cannot be confused with files carrying the real measurement.
```


## Remaining long operator comments — September 16, 2026

Historical explanations moved during the resource/reporting continuation after `142d5a9`. Line numbers refer to the intermediate source before this comment-only pass. Current contracts remain at the call sites; some old wording (especially that CONTROLS does not move motors) is historical and does not describe current behavior. Executable ASTs matched exactly before and after this pass. Relative documentation links below are adjusted for this archive.

### Operator comments near line 1224

```python
                # ---- 1. is every robot still there? -----------------------
                # ⛔⭐ A FAULT ON ONE ARM STOPS ALL OF THEM. ROADMAP §6's ruling, and the
                # reason is physical: a chain death on B must not leave G uncommanded and
                # sagging while the operator is still looking at B.
                #
                # ⛔⭐⭐ NOTE THE SHAPE, BECAUSE `break` CHANGED MEANING HERE. This used to
                # be `if not chain_alive(robot): stop_reason = …; break`, straight out of the
                # `while`. Inside a `for one in arms:` a `break` leaves only the FOR, so the
                # cycle would carry on commanding arms with a stop already decided. The stop
                # is recorded in the loop and acted on after it.
```

### Operator comments near line 1244

```python
                # ---- 2. temperatures and the gripper stall guard -----------
                # ⛔⭐ ONLY THE READ IS WRAPPED. The decisions are not, and that is the
                # entire point of this shape. The previous version wrapped the read AND
                # every check that followed in one `try`, whose handler set
                # `hottest = 0.0` — so a failed read silently disarmed the thermal stop
                # and printed a calm "hottest 0°C". A guard with a path around it is
                # the defect this repo keeps paying for (working contract rule 7);
                # here the path was its own exception handler. See ThermalGuard.
                #
                # ⚠️ Per arm, and each arm keeps its OWN last reading, because the incident
                # record wants the last good values from a chain that may now be dead.
```

### Operator comments near line 1273

```python
                        # ---- gripper stall guard ------------------------------
                        # ⚠️ With --no-gripper the chain has 6 motors, so states[6] would
                        # IndexError. It used to be guarded by raising StopIteration out of
                        # the shared try — which worked, but meant the "no gripper" path and
                        # the "read failed" path were the same code path. Now it is just an
                        # if, because there is nothing left to jump out of.
                        # ⭐⭐ SAY IT ONCE WHEN THE LATCH LETS GO. His 2026-08-17 log showed
                        # three stalls at 0.117, 0.098 and 0.104 and there was **no way to
                        # tell** whether those were three deliberate squeezes or one latch
                        # being cleared twice by a jittering measurement. A latch that
                        # silently comes and goes is indistinguishable from one that never
                        # worked, so the next run must not leave the same ambiguity.
```

### Operator comments near line 1439

```python
                        # ⭐ `i` AT THE PROMPT SWITCHES copy ↔ mirror and re-prints the plan.
                        # Without it, discovering that `copy` is the wrong choice for how the
                        # arms are standing means quitting the session and restarting with
                        # `--mirror mirror`, which costs a puck assignment and two builds.
                        # ⚠️ The plan line says what `i` does here, so this is not one key
                        # with two hidden meanings — it is the mirror key, inside the mirror
                        # prompt, changing the mirror.
```

### Operator comments near line 1458

```python
                            # ⛔ The follower goes under POSITION control before anything is
                            # commanded. If it were left weightless the commands would do
                            # nothing at all, and the readout would show it tracking.
                            # ⛔ ORDER: the class's enter_hold() sets mode="hold",
                            # so MIRROR is written AFTER it — the reverse order would
                            # leave a mirror running while the row said HOLD (§52.1).
```

### Operator comments near line 1514

```python
                            # ⭐ How LONG the ease lasts, separately from its shape.
                            # Julien: *"the smoothing should maybe be adjustable at the
                            # beginning of the park, similar to the parking speed."*
                            # ö/ä (or [/]) mean gripper step elsewhere, which is
                            # meaningless while choosing a park — same
                            # context-dependence as +/-.
```

### Operator comments near line 1576

```python
                    # ---- device configuration: works in EVERY mode ------------
                    # ⛔ `b` USED TO LIVE IN THE CONTROLS BRANCH ONLY, while the
                    # "press b to set the gripper buttons" hint printed in TELEOP as
                    # well. So in TELEOP the hint appeared and b fell through to the
                    # catch-all and did nothing. Julien hit exactly that: *"it says
                    # press b to set the gripper, and then b does nothing either."*
                    #
                    # A message that tells you to press a key which does nothing
                    # where you are is the same defect class as the refusal that
                    # named the wrong arm (FINDINGS §16) — the text is right, the
                    # context is wrong, and it costs the user a session to find out.
                    # Button assignment is a property of the DEVICE, not of the
                    # arm's mode, so it belongs above the mode dispatch entirely.
```

### Operator comments near line 1597

```python
                        # ⭐⭐ WHICH ARM THE MODE KEYS AIM AT — ROADMAP §6's decision, and
                        # the reason is in `ArmSelector`: `g` on two arms at once is 8.6 kg
                        # going weightless on one keypress.
                        #
                        # ⚠️ Handled HERE, above the mode dispatch, for the same reason `b`
                        # and `v` are: which arm a key applies to is a property of the
                        # SESSION, not of a mode. A selector that worked only in TELEOP
                        # would be the `b` defect again (FINDINGS §17.1).
```

### Operator comments near line 1606

```python
                            # ⛔ CONTROLS is a wizard that belongs to the arm it was
                            # entered on: it asks the operator to push one axis at a time
                            # and edits that arm's map from the answers. Re-aiming the
                            # keys underneath it would write one arm's answers into
                            # another arm's map, which is the blast-radius bug the
                            # per-arm map store exists to prevent.
```

### Operator comments near line 1627

```python
                        # ⭐⭐ MIRROR MODE. The selected arm leads; the other follows.
                        #
                        # ⛔ IT ASKS TWICE, exactly like `l`. Engaging starts a MOTION on the
                        # follower — it ramps to the leader's pose — and the operator's hands
                        # and eyes are on the leader at that moment. A single keypress that
                        # moves an arm nobody is looking at is the one thing this session's
                        # design refuses.
```

### Operator comments near line 1704

```python
                    # ---- MAP mode owns the keyboard while it is active --------
                    # ⚠️ 1-6 mean "select a motion" here and "flip a rotation sign" in
                    # the drive modes. Overloading is a real footgun in a codebase
                    # whose motto is that this stack fails by lying, so it is bounded:
                    # MAP mode is entered explicitly, announces itself loudly, holds
                    # the arm still, and echoes the effect of every key. Nothing it
                    # can do moves a motor.
```

### Operator comments near line 1750

```python
                                    # ⭐ SWAP, not steal. Julien's request after using this
                                    # on the arm: the commonest edit is two controls in
                                    # each other's places, and stealing left an orphan he
                                    # then had to notice and re-bind. A straight exchange
                                    # is also an involution, so pressing the same key
                                    # again undoes it. See AxisMap.swap().
```

### Operator comments near line 1788

```python
                        # ⚠️ The rotation pair was MISSING here while the linear pair was
                        # present, so in CONTROLS mode roll/pitch/yaw could not be sped up
                        # or slowed down at all — Julien found it on the wizard. The keys were
                        # copied from the drive-mode handler and the second pair was
                        # dropped. Both scales are also printed in the status line now, so
                        # a key that silently does nothing is visible rather than inferred.
```

### Operator comments near line 1963

```python
                        # ⭐ Both folders in a --sim session, with the simulated ones
                        # marked, because "saved: 1, 2, 7" that silently mixes real
                        # demonstrations with simulated ones is the confusion the folder
                        # split exists to prevent.
                        # ⛔ `listing` and not `glob`: a macOS `._5.json` sidecar in a hand-copied
                        # recordings folder was offered here as a playable slot "._5" on the Linux
                        # station (FINDINGS §76). It is not a recording and it cannot be loaded.
```

### Operator comments near line 2040

```python
                        # ⚠️ These now flip a ROBOT MOTION, not a puck axis. Under the
                        # identity map that is the same arithmetic, which is why the
                        # hand-dialled file still means what it meant. Under a
                        # permutation it is the only reading that stays useful: when
                        # Julien presses x he means "the gripper goes the wrong way",
                        # which is a statement about the arm, not about the device.
```

### Operator comments near line 2061

```python
                        # ⭐ In PARK these mean the park speed. The teleop linear scale
                        # is meaningless while the puck is not driving, and a key that
                        # does nothing where you are is the defect class that made `b`
                        # look broken (FINDINGS §17.1).
                        # ⭐ In a SCRUB they mean the full-push pace — his time-lapse dial
                        # (FINDINGS §68.5): "more than normal speed if I fully press the
                        # control forward". Safe high: a fast cursor is held back by the
                        # lag hold, so only the clock is fast, never the arm.
```

### Operator comments near line 2188

```python
                # ---- 4. act on the mode -----------------------------------
                # ---- 3.5 the puck, read EVERY cycle in EVERY mode -------------
                # ⛔ This used to sit inside the teleop/map branch, which had two
                # consequences: the buttons were dead in GUIDE and HOLD, and the HID
                # reports queued up while in those modes and then arrived in a burst
                # on the next mode switch. Reading unconditionally costs nothing —
                # TwistReader.read() is non-blocking by construction — and it is what
                # makes button assignment work from wherever Julien happens to be.
                # ⭐⭐ EVERY ARM READS ITS OWN PUCK, EVERY CYCLE, IN EVERY MODE.
                #
                # ⛔ This whole block used to read `arm.reader` once, outside any loop.
                # With two arms that reads ONE hand and hands its deflection to both
                # arms — and the leaked loop variable at the bottom of it made the
                # gripper follow whichever arm the previous loop ended on
                # (FINDINGS §54.1).
                # ⭐⭐ ONE PUCK, N ARMS: THE PUCK FOLLOWS THE SELECTION (his design,
                # 2026-08-18, FINDINGS §68.8): a → B drives B, a → G drives G, a → BOTH
                # drives both arms at once, each from its own pose. The shared reader is
                # read ONCE per cycle — two arms draining one HID queue would split the
                # event stream between them — and unaimed arms read as centred.
```

### Operator comments near line 2290

```python
                            # ⭐ AXIS ISOLATION — Julien's design: only the strongest puck
                            # direction is applied, so the arm performs exactly one motion and
                            # it is obvious which gesture caused it. Half speed, because this
                            # is the mode you experiment in.
                            #
                            # ⛔ Note what is NOT here: any call that edits the map. Deflection
                            # observes; keys edit. The mode this replaced bound on deflection
                            # and destroyed the hand-dialled map (FINDINGS §11).
```

### Operator comments near line 2373

```python
                                # ⛔⭐⭐ NAME BOTH FLAGS, AND NAME THE ONE THAT ACTUALLY
                                # FIRED FIRST. This branch used to say only *"That
                                # allowance is `--max-speed`. Raise it one step."*
                                #
                                # ⛔ On 2026-08-17 Julien raised `--max-lag` from 0.25 to
                                # 0.4 to 1.0 across three sessions chasing this message,
                                # and **none of it could ever have helped**: the stop is
                                # triggered by the gap passing `--mirror-gap`, which was
                                # sitting at its 0.35 default because he had not set it.
                                # His own earlier run with `--mirror-gap 0.6` is the one he
                                # described as working *"much better"*.
                                #
                                # ⚠️ Third time a speed-layer confusion has cost him a
                                # session ([FINDINGS §58.3](../FINDINGS.md)). The
                                # message named the limit's VALUE ("limit 0.35") and never
                                # named the FLAG that sets it, so the number was unusable.
                                #
                                # ⭐ Two independent routes out, and both are stated,
                                # because they do different things: a wider tolerance means
                                # it does not stop, a faster follower means the gap does not
                                # grow.
```

### Operator comments near line 2433

```python
                        # ⭐⭐ item 23 group ④ (2026-08-18): the CLASS advances the park.
                        # `ArmSession.step_path` owns the cursor, the lag hold, the easing,
                        # the stall guard and the verdict — 48 tests. This branch only
                        # narrates the ParkStep it returns and performs the handovers,
                        # which is the split the whole restructure was for: the class
                        # decides, the script narrates.
```

### Operator comments near line 2470

```python
                            # ⛔⭐⭐ THE ARRIVAL CLEARS ITS OWN PATH — the fix for the bug
                            # that killed the first two-arm playback. The generic "leaving
                            # PARK abandons the run" block fires for any arm whose mode is
                            # no longer `park` while `park_path` is still set; an ARRIVAL
                            # used to leave the path in place, so with two arms the FIRST
                            # arrival (which waits for the second) had its pending playback
                            # cancelled as "abandoned". FINDINGS §57.1.
```

### Operator comments near line 2603

```python
                # ---- 4a. the playback: ONE cursor, every arm it was recorded from ----
                #
                # ⭐⭐ SESSION-LEVEL, AND IT HAS TO BE. The cursor is a clock, and one clock
                # drives every arm. Inside the per-arm loop this block called `replay_step`
                # once per arm, which with two arms would advance the SAME cursor twice per
                # cycle — a playback running at double speed, silently.
                #
                # ⭐ FOLLOW THE RECORDING IN TIME, not along its length. A park traverses a
                # *shape* at a constant joint speed, which throws away the thing hand-guiding
                # provides: human timing and hesitation are the signal (ROADMAP §6.6).
```

### Operator comments near line 2657

```python
                        # ⭐⭐ SAY WHERE THE EXTRA TIME WENT. Julien's first playbacks ran
                        # 2.3 s longer than the recording and the old message reported only
                        # the total, so it read as a bug with no explanation. The whole
                        # difference is the loop holding the clock while the arm catches up,
                        # which is a decision this code makes on purpose. A readout has to
                        # show what can go wrong, not only what looks tidy — the same lesson
                        # as showing the jaw temperature separately (FINDINGS §11).
```

### Operator comments near line 2688

```python
                            # ⚠️ MEASURED, so read it as such. The playback holds its clock
                            # once the arm falls behind, so the speeds here are not an even
                            # sweep, and load changes with the arm's pose. It is the cheap
                            # first answer; ROADMAP §7.5 has the active sweep if this is
                            # ambiguous.
                            # ⭐⭐ ONE list of names for BOTH the printed table and the
                            # saved file, and they DISAGREED until 2026-08-17. The saved
                            # JSON below already labelled every row with its arm, and its
                            # comment says exactly why that is necessary — while this
                            # print used `YAM_JOINTS.get(i + 1)` on a **flat** index. With
                            # two arms that names arm B's seven joints correctly and then
                            # labels every one of arm G's simply "joint", because the flat
                            # indices 7-13 become keys 8-14 and `YAM_JOINTS` only holds
                            # 1-7.
                            #
                            # ⛔ Julien's 2026-08-17 playback log is the evidence: six
                            # named rows, six anonymous ones, and **nothing saying which
                            # arm any row belonged to**. The six named rows read as "the
                            # arm" when they were only arm B.
                            #
                            # ⚠️ Same family as the `label_verdict` defect: code written
                            # for one arm that produces confident, plausible, wrong output
                            # with two, and raises nothing. Building the list once is the
                            # actual fix, because two copies is what let them drift.
```

### Operator comments near line 2791

```python
                # ---- 5. report --------------------------------------------
                # CONTROLS mode reports continuously, not once a second: he is watching
                # the arm and the readout together to attribute a motion to a gesture,
                # and a 1 Hz readout is useless for that.
                # ⚠️ CONTROLS owns the whole live block while it is open, so the other arm's
                # row is not painted during it. That is a real gap at N>1 and it is deliberate
                # for now: the wizard is a full-screen conversation with one arm, and `m`
                # refuses when two arms are selected. Tracked in FINDINGS §54.2.
```

### Operator comments near line 2801

```python
                    # ⛔ NOT `arm = wizard`. Rebinding the session's own `arm` here would repoint it
                    # at the wizard for the REST of the loop, including the incident record and
                    # the shutdown. At N=1 it is the same object, so it would have worked and
                    # proved nothing — the leaked-variable defect again (FINDINGS §54.1).
                    # Both scales are always shown. Julien could not tell that ,/. were
                    # doing nothing here because only the active axis's resulting speed
                    # was displayed — a missing key looked identical to a key that worked.
```


## Broader September review: remaining operator notes

Verbatim excerpts before this prose pass, after the status/drive-control extractions.
Line numbers are historical. Several claims were obsolete; read current source contracts.

### Former operator line 25

```text
# ⚠️ `Any` was used in this file's annotations since long before this import existed, and
# it worked only because `from __future__ import annotations` never evaluates them. A real
# import is needed the moment it appears on a variable inside `main()`, because
# `checks/check_restructure.py` check 4 resolves every name used there.
```

### Former operator line 144

```text
# Faster than the first run, which Julien found "very slow". Still well short of
# what the hardware can do — this is a human-in-the-loop speed, not a limit.
# ⭐ Defined in src/yam/inputs/axis_map.py so `apps/map_axes.py` reports the exact speeds
# this session commands. Dialling a mapping against speeds the arm does not use
# would teach the wrong feel.
```

### Former operator line 152

```text
# ⚠️ `WORKSPACE_BOX = 0.30` used to live here. The workspace limit is now
# `REACH_LIMIT` and `FLOOR_LIMIT` in `src/yam/teleop.py`, next to the code that applies
# them, because the old constant sat in the script while the clamp it fed was an
# untested inline block. FINDINGS §43.
```

### Former operator line 164

```text
# ⭐ Ease in and out over this much joint travel. A constant-rate park starts and
# stops with a jerk; with sequences that jerk lands at every waypoint. 0.20 rad is
# ~half a second of ramp at the default 0.4 rad/s, and a move shorter than twice it
# simply never reaches full speed. `--no-smooth` sets it to 0.
```

### Former operator line 169

```text
# ⭐ How much the path may cut a corner, in radians of the fastest joint. `sharp`
# reproduces the old stop-at-every-waypoint behaviour exactly. Julien's words for what
# the others are for: *"instead of moving and then jittering ninety degrees to the next
# side, in a smooth curve it would go to the next point."*
```

### Former operator line 211

```text
# Hold-to-move rate for the puck buttons. A gripper wants squeeze-and-hold, not a
# staircase of keypresses. 0.6/s crosses the whole normalised stroke in ~1.6 s,
# which is deliberate and slow: the jaws close on real objects, and the stall guard
# should be a backstop rather than the thing that routinely stops you.
```

### Former operator line 217

```text
# Gripper stall guard. Catches the CAUSE (jaws pushing against something they
# cannot move) rather than the symptom (temperature). Torque high while velocity
# is ~0 is the definition of a stall, and stall is the worst thermal case there
# is: full current, no motion, no cooling.
```

### Former operator line 236

```text
# ⚠️ How long one recording may run before it stops itself. ~16 minutes at 100 Hz, which
# is well past the ~4.5 minutes of context a long-horizon policy wants (ROADMAP §9.3).
# It exists because nothing else would ever stop a recording, and an unbounded list in a
# process that is driving an arm is a memory problem waiting for the worst moment.
```

### Former operator line 413

```text
    # ⛔ DISCARD ANYTHING TYPED BEFORE THIS MOVE EXISTED. "Any key stops it" must mean a
    # key pressed *at* the moving arm, not one left over from teleop or from the menu that
    # led here. Julien saw a park announce itself and stop in the same breath — the stale
    # keystroke that cancelled it had been typed seconds earlier.
```

### Former operator line 516

```text
    # ⭐⭐ THE LIST OF ARMS THIS SESSION DRIVES. ROADMAP §6.1 step 2.
    #
    # ⚠️ `arm_names[0]` appears below wherever a line still assumes one arm, on purpose:
    # each one marks a site step 2's remaining work has to turn into a loop, and it is
    # greppable. Sites that run after the object exists use `arm.name` instead.
```

### Former operator line 604

```text
    # ⭐⭐ SAY WHICH SETTINGS CAME FROM THE FILE, AND FLAG A PERMANENT LOOSENING. A flag
    # typed on the command line is visible in the shell history and on screen; a saved
    # default is not. ⛔ Without these lines a session could run at three times the built-in
    # speed limit with nothing on screen explaining why.
```

### Former operator line 704

```text
        # ⚠️ `gripper_value` and `stall_since` used to be initialised here. They are now
        # `ArmSession` fields, and the class's own constructor sets exactly the same values
        # (0.0 and None). ⛔ Leaving the assignments here as `arm.gripper_value = 0.0` would
        # run BEFORE `arm` exists, which is nine lines below inside the `try`. See the
        # ordering check in checks/check_restructure.py.
```

### Former operator line 742

```text
                # ⭐⭐ THE WHOLE POINT OF --sim, AND IT IS ONE BRANCH ON PURPOSE. Everything
                # below this line is the same code in both modes, so a simulated session
                # exercises the real loop rather than a parallel one. `build_fake_robot`
                # returns the same `(robot, note)` tuple for exactly that reason.
```

### Former operator line 779

```text
        # ⛔ Mode keys are AIMED; driving never is. A global `g` would put 8.6 kg weightless
        # in arm keypress, and GUIDE is where a dynamics-model error becomes a falling arm
        # rather than a droop (FINDINGS §11.1). Each arm always follows its own puck.
        # `src/yam/session.py::ArmSelector` holds the cycle and its tests.
```

### Former operator line 801

```text
        # ⭐ enter_hold lives on ArmSession now (item 23 group ①, 2026-08-18): the class
        # method resyncs, commands the measured pose AND sets mode="hold" — so the two
        # sites that want a DIFFERENT mode afterwards (the park seed, the mirror engage)
        # write their mode AFTER the call. The script's own copy is gone.
```

### Former operator line 806

```text
        # ⭐ enter_guide lives on ArmSession now (item 23 group ②, 2026-08-18). The class
        # method records guide_ref, sets mode="guide" and RETURNS the "NOT weightless"
        # warning instead of printing it — every caller prints the return, so the
        # warning that once explained a falling arm (FINDINGS §11) cannot be dropped.
        # The kp=0 physics and the API-name history live in the class docstring.
```

### Former operator line 843

```text
            # ⭐ A grab is visible BEFORE Enter (ROADMAP §6.6.2 item 4): a leg where only
            # the jaws move splits the run and pauses it, and the count says so here,
            # while the sequence is still being typed.
            # ⚠️ Take legs (`w<digit>`, ROADMAP §6.6.1a) are counted separately: their
            # jaw motion is whatever the hand taught, so no stop-counting applies.
```

### Former operator line 884

```text
            # ⭐ Same rule one level up (ROADMAP §6.6.1a trap ①): a park the COMPOSITE did
            # not start replaces the composite. Its own pose legs pass for_composite=True
            # and its take legs pass for_replay=True, so only operator-initiated parks
            # land here — which is exactly who may abandon a queued run.
```

### Former operator line 890

```text
            # ⭐ item 23 group ④: the CLASS builds and runs the park now — the tested
            # `begin_path`/`step_path` pair with its 48 tests is finally the code that
            # moves the arm. The session's live dials are copied on at start, and the
            # `e` key keeps `easing` current mid-park.
```

### Former operator line 1004

```text
                # ⚠️ GUIDE at startup is established by build_robot(zero_gravity=True), not
                # by enter_guide() — so the drift reference has to be taken here too, or the
                # readout silently shows nothing for the whole first GUIDE period. That gap
                # is exactly the 33 seconds in which the arm sank unremarked on 2026-08-10.
```

### Former operator line 1137

```text
                        # ⭐ SPEED AND CORNERS ADJUSTABLE WHILE TYPING, not only while
                        # moving. Julien: *"I can change the park speeds whilst it's
                        # parking, but not whilst I'm putting in the numbers, which is
                        # a bit annoying."* Deciding how a move should feel belongs to
                        # the moment you are choosing the move.
```

### Former operator line 1255

```text
                        # ⭐ Cycle which frame the puck's directions mean. Safe to do
                        # live: the twist is a VELOCITY, so a frame change alters the
                        # interpretation from the next cycle onward and leaves no
                        # stale cached state behind — unlike a mode change, which is
                        # why this does not need resync().
```

### Former operator line 1318

```text
                    # ⛔ Unrecognised keys are IGNORED. They used to fall through to a
                    # catch-all that cancelled PARK, so pressing Enter out of habit
                    # right after `p` killed the move in the same keyboard batch --
                    # which looked exactly like "park just went to hold". A control
                    # character must never be an action.
```

### Former operator line 1326

```text
                        # ⛔ CONTROLS EDITS ONE MAP FROM ONE WIGGLE, so it cannot be aimed at
                        # two arms. Refused rather than silently applied to the first: the
                        # operator who selected BOTH and pressed `m` asked for something this
                        # wizard has no meaning for.
```

### Former operator line 1347

```text
                    # ⭐⭐ MODE KEYS APPLY TO EVERY SELECTED ARM, which is what `a` is for.
                    # ⛔ `g` on two arms is 8.6 kg going weightless in one keypress, and GUIDE
                    # is the mode where an error in the dynamics model becomes a FALLING arm
                    # rather than a droop (FINDINGS §11.1). That is why the selector exists at
                    # all, and why it starts on one arm rather than on BOTH.
```

### Former operator line 1379

```text
                    # ⚠️ `w` and `l` REFUSED with two arms until 2026-08-14 night, because
                    # `Trajectory` held one arm's joints and a two-arm demonstration would have
                    # been saved as half of itself. The recorder now samples every arm into one
                    # timeline, which is ABC's own shape (ROADMAP §9.2), so the refusal is gone
                    # along with the test that pinned it.
```

### Former operator line 1608

```text
                        # ⚠️ A bound, because this grows in memory for as long as it runs and
                        # nothing else would ever stop it. 100 000 samples is ~16 minutes at
                        # 100 Hz, comfortably past the ~4.5 minutes a long-context policy
                        # wants (ROADMAP §9.3). Stopping and saying so beats running out of
                        # memory in a process that is driving an arm.
```

### Former operator line 1684

```text
                        # ⭐⭐ ONE ARM FOLLOWS THE OTHER. Every decision is `MirrorLink`'s
                        # (18 tests, no robot handle); this branch reads the two poses,
                        # carries the command out, and narrates. Same split as `replay_step`
                        # and `ArmSession` — the code that commands an arm is the code that
                        # cannot be tested without one, so it is kept as thin as possible.
```

### Former operator line 1699

```text
                            # ⛔ The jaws go through the clamp, never straight from the leader.
                            # A leader whose jaws rest on a stop would otherwise drive the
                            # follower's onto its own stop and HOLD there, which is stall
                            # torque and is how motor 7 was cooked three times (FINDINGS §4).
```

### Former operator line 1704

```text
                                # ⛔⭐ THROUGH THE LATCH, so a stalled follower stops being
                                # pushed further closed by the leader every cycle. Opening
                                # clears it, so letting go of the leader's jaws frees the
                                # follower's immediately.
```

### Former operator line 1736

```text
                            # ⭐ Total and settling answer different questions: the total
                            # is what speed/corner/ease tuning changes, and the settling
                            # is how long the arm closed the last gap after the commanded
                            # path ran out. The class stamps both from the right clocks —
                            # park_start_t, NOT park_leg_t, for the total (FINDINGS §34.3).
```

### Former operator line 1751

```text
                            # ⭐ Composite (ROADMAP §6.6.1a): a pose-leg's park arrived.
                            # The leg is done when EVERY awaited arm has arrived; only
                            # then does the queue advance — in the ARRIVAL branch, never
                            # a key branch, which is the §57.1 rule.
```

### Former operator line 1756

```text
                            # ⭐ The handover from "drive to the start pose" to "play the
                            # recording" lives HERE, in the arrival branch, so a park that
                            # was blocked or interrupted can never roll into a playback:
                            # only a park that actually arrived does — and only a park that
                            # was FOR the playback (`arrived_purpose`), never a pose leg's.
```

### Former operator line 1763

```text
                                # ⛔⭐ EVERY ARM MUST ARRIVE BEFORE ANY ARM PLAYS. Each one
                                # parks a different distance and finishes at a different
                                # moment; starting on the first arrival would have the
                                # second arm still parking while the recording ran.
```

### Former operator line 1817

```text
                            # ⭐⭐ THE JAW PAUSE (items 3 + 10): the run split at a
                            # waypoint where only the jaws move. The class holds the arm,
                            # drives the jaws and measures when they are done; this
                            # branch only says what is happening, so a pause never reads
                            # as a stall.
```

### Former operator line 1824

```text
                                # ⭐ The settled offset is the miss-diagnosis number: at
                                # the friction floor (~0.02-0.04 rad) a missed grab means
                                # the POSE was taught off; well above it, the arm never
                                # got there (FINDINGS §70.5).
```

### Former operator line 1837

```text
                                # ⭐ item 10: `check_grasp` grades a CLOSING leg from
                                # where the jaws stopped. It stays silent when it cannot
                                # know (an opening leg, a timeout) — `confident` is the
                                # gate, and printing a guess would be the §0 pattern.
```

### Former operator line 1853

```text
                            # ⛔ BLOCKED. Never spin silently: say so and hold. The wording
                            # keeps the old two shapes — mid-path (the arm stopped
                            # following) and at the end (it stopped closing) — decided by
                            # the remaining path, which the ParkStep carries.
```

### Former operator line 1882

```text
                    # ⛔ The grippers are left out of the "is it keeping up" check by INDEX,
                    # because with two arms the first gripper sits in the middle of the
                    # vector. Jaws legitimately sit far from their commanded value while
                    # closing on an object, and counting that as lag would stall every
                    # playback that grips anything.
```

### Former operator line 1889

```text
                        # ⭐ SCRUB: the puck is the clock. EITHER puck works — during
                        # playback nobody's hand is driving an arm, so whichever hand is
                        # free is the deadman. The forward/back axis (index 1) is the
                        # natural "push to play" gesture; largest deflection wins.
```

### Former operator line 1907

```text
                            # ⛔ Through the clamp, never straight from the file. A recording
                            # made while the jaws rested on a stop would otherwise drive them
                            # back onto it and HOLD there. That is stall torque, and it is how
                            # motor 7 was cooked three times (FINDINGS §4).
```

### Former operator line 1925

```text
                        # ⛔ THE TWO NUMBERS MUST RECONCILE, and on 2026-08-13 they did not:
                        # a 3.6 s recording reported 3.6 + 0.4 and finished in 4.6. The gap
                        # was the loop running below 100 Hz while the cursor advanced in
                        # nominal time. That is fixed, and this check stays so a future
                        # version cannot reintroduce it silently.
```

### Former operator line 1989

```text
                        # ⛔ NEVER WAIT FOR EVER. Holding the clock is right for a moment
                        # and wrong for ever: an arm that cannot catch up is blocked, and a
                        # playback that sits silently holding its clock is the treadmill
                        # bug again (FINDINGS §24). Same patience the park uses.
```

### Former operator line 2052

```text
                    # ⭐ RECORDING HAS TO BE VISIBLE ON THE HEARTBEAT, not only in the
                    # message that started it. A session where recording is silently still
                    # running produces a demonstration full of whatever happened next, and
                    # the operator finds out at training time.
```

### Former operator line 2157

```text
        # ⛔ BOTH lines print in the SAME frame, and the frame is NAMED. This summary once
        # printed the current map in the frame the arm ENDED in (camera) against a `was:`
        # line in world labels — different motion names for the same store — and it read
        # as a scrambled, saved map. Verifying that nothing was actually written cost real
        # bench time, twice (FINDINGS §66.2, ROADMAP §8.2 item 45).
```

### Former operator line 307

```text
    """The one-line answer to *"what does easing even do here?"*

    ⭐ It names where the effect lives, because that is the question Julien actually asked
    on the arm: *"the easing outside of parking, I don't really know what that means. Does
    it work for recording, or does it work for teleoperating?"* Neither. Easing shapes how
    a **planned** move starts and stops, which means `p` runs and the Ctrl-C park, and
    nothing else. Driving by hand has no plan to shape, and a playback follows the timing
    it was taught rather than an eased ramp.
    """
```

### Former operator line 321

```text
    """Is the robot still actually being commanded?

    ⛔ The single most important check in this file. I2RT's control thread raises
    and exits on a motor fault; nothing tells the caller. Without this, the loop
    keeps issuing commands into a corpse and reporting healthy-looking numbers,
    which is what happened for 64 s on 2026-08-10.
    """
```

### Former operator line 338

```text
    """Silence ONE known, expected traceback — and only while we are shutting down.

    ⛔ The noise this removes. Every clean exit printed:

        Exception in thread robot_server:
        RuntimeError: … motor_chain_robot's motor chain is not running, exiting the
        robot server

    …immediately before `motors confirmed disabled: [1, 2, 3, 4, 5, 6, 7]`. It is the
    I2RT SDK's background server thread noticing the chain has stopped — **because we
    stopped it**. Nothing is wrong, and the shutdown it appears to indict has in fact
    succeeded.

    ⚠️ Why bother, when it is harmless? Because a scary traceback printed on every
    successful exit is a training exercise in ignoring tracebacks, and this project
    depends on people reading the ones that matter. FINDINGS §0 is a catalogue of
    failures that looked calm; the inverse — a success that looks like a failure — has
    the same cost, paid in attention.

    ⛔ Deliberately narrow, because blanket exception-swallowing is the other half of
    that catalogue: it fires only during our own shutdown, only for that thread, only
    for `RuntimeError`, and only for that message. Anything else goes to the real hook
    and prints in full.
    """
```

### Former operator line 539

```text
        """A value the operator just changed, on its own live row above the status.

        ⭐ `linear speed → 0.188 m/s` printed as a MESSAGE six times is six rows of
        scrollback saying the same word. As a hint it is one row whose number changes
        — and, crucially, it no longer loses a race with the once-a-second status,
        which is what made a knob change flash up and vanish.
        """
```

### Former operator line 718

```text
            """Where to LOOK for a recording. Writes always go to `takes_dir`.

            ⭐⭐ A --sim SESSION CAN STILL PLAY A REAL RECORDING, and that is deliberate. When
            the folder split was first written it applied to reads as well, which quietly
            removed one of the best uses of a simulator: **replaying a real take against
            simulated arms to check the playback before committing it to 4.3 kg of hardware.**
            Sim recordings win when both exist, so a sim session never silently reaches past
            its own work.
            """
```
