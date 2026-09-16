# Archived recording source explanations

Verbatim passages from `216fda0`, moved September 16, 2026. These preserve
historical reasoning and may contain superseded claims. Current contracts live
in src/yam/recording.py; see [CLEANUP](../CLEANUP.md) for current evidence.

## MODULE at source lines 1–26

```text
"""A movement taught by hand, stored so it can be played back.

⭐ WHY THIS EXISTS. Julien, 2026-08-12: *"One good idea is definitely recording
everything in the guide mode and then replaying it. That's a smart idea,
definitely."*

Instead of saving a handful of poses and letting the code interpolate between them,
stream the joint positions continuously while he hand-guides the arm through a whole
task. Then play that back with the scene reset, and record the cameras on the playback
run. The full argument, including why this may beat waypoints, is
[ROADMAP.md](../docs/ROADMAP.md) §6.6.

⭐ WHAT THIS FILE IS AND IS NOT. It is the **motion** recorder: a hand-taught path that
the arm can repeat. It is **not** the dataset recorder (ROADMAP step 5), which has to
write MCAP in ABC's schema and is deliberately still deferred. Keeping them apart means
this file does not have to guess at a format someone else is specifying:
`amazon-far/abc` wants `states_actions.bin` with 14 states and 14 actions per timestep
([ROADMAP.md](../docs/ROADMAP.md) §9.2), and guessing at that would be exactly the
mistake step 5 was deferred to avoid.

⛔ THE RULE THIS FOLLOWS, AND IT IS WHY THERE ARE NO PRINTS AND NO ROBOT HANDLE HERE.
*The class decides, the script narrates.* Same rule as `src/yam/session.py`. Every
decision below can therefore be proven with no arm plugged in, which matters on a rig
where three changes have passed their tests and then failed on first contact with the
hardware ([FINDINGS §11](../docs/FINDINGS.md)).
"""
```

## safe_time_scale at source lines 52–73

```text
    """Playback multiplier at which no joint is commanded faster than `cap` rad/s.

    ⭐ It reports, it does not decide. `1.0` means the recording is exactly at the cap,
    below `1.0` means the recording is faster than the cap and playing it at full speed
    will ask the arm for more than the cap allows, above `1.0` means there is headroom to
    speed the playback up. **The caller chooses what to do with that**, which is the same
    split as the rest of this file: the module measures, the session decides.

    ⛔ WHY IT NO LONGER FLOORS THE ANSWER AT 1.0, and this changed on 2026-08-13 after a
    real run. The floor was there to express "replaying at the taught speed is always
    allowed", which is a *policy* and belongs in the session. Hiding it here made the
    session unable to see the one fact it most needed: **that the recording is faster than
    the arm can follow.** Julien hit that immediately. Hand-guiding a weightless arm
    reaches 2.4 to 2.9 rad/s at the 99th percentile, while this code permits 1.5 rad/s for
    any planned motion, so every hand-taught recording came back with "max 1.00x" and then
    played back slower than 1x anyway because the arm kept falling behind. The number was
    right and it was reported in a form that could not explain what he was seeing.

    ⚠️ `cap` is the fastest any planned motion is allowed to command, not a measurement of
    what the arm can physically track. Those are different, and the second one has never
    been measured on this rig. See [ROADMAP.md](../docs/ROADMAP.md) §6.6.
    """
```

## Sample at source lines 83–96

```text
    """One instant: when it happened, and where every joint was.

    ⭐ `q` is the **measured** position, not the commanded one, and for a hand-taught
    recording those are different things by definition. In GUIDE mode the position gain
    is zero and gravity compensation is the only thing holding the arm up, so there is
    no meaningful command; the arm is wherever the hand put it. The measurement is the
    demonstration.

    ⚠️ For a PLAYBACK run the distinction reverses and matters even more: what must be
    stored is what the arm was **actually told to do**, because storing the tidy plan
    instead produces a dataset that claims the arm was on track while the picture shows
    it off to one side. That warning belongs to the dataset recorder, and it is written
    down in [ROADMAP.md](../docs/ROADMAP.md) §6.6 so it cannot be lost.
    """
```

## Layout at source lines 104–118

```text
    """How one flat sample maps onto arms. `arms` in order, `per_arm` joints each.

        Layout(("B", "G"), 7).slice_for("G")        -> slice(7, 14)
        Layout(("B", "G"), 7).tracked_indices(6)    -> [0..5, 7..12]   (no grippers)

    ⭐⭐ WHY A RECORDING IS ONE FLAT VECTOR AND NOT A DICT PER ARM. `amazon-far/abc`, the
    training format this rig is aiming at, stores **14 states and 14 actions per timestep —
    two arms in ONE timeline** ([ROADMAP §9.2](../docs/ROADMAP.md)). Concatenating the arms
    in `--arms` order gives exactly that, so the internal format and the target format have
    the same shape and no conversion can silently reorder them.

    ⛔ AND IT IS WHY THE ARM ORDER IS METADATA RATHER THAN A CONVENTION. A recording made
    with `--arms B,G` and played back in a `--arms G,B` session would drive each arm with
    the other's joints. The names travel with the samples so the caller can check.
    """
```

## Trajectory at source lines 168–176

```text
    """A hand-taught movement: samples in time order, plus how it was made.

    ⭐ `meta` is free-form on purpose, because the fields that matter are a *provenance*
    question rather than a code question, and Julien answered it on 2026-08-12: he wants
    *"the history and everything we found and problems we had, and being able to
    reproduce everything and connect it to other research papers."* The names the caller
    should fill are listed in `META_FIELDS`. ⛔ All of it is cheap to write while
    recording and impossible to reconstruct afterwards.
    """
```

## start_pose at source lines 217–225

```text
        """Where the arm has to be before playback can begin.

        ⛔ SAFETY, AND IT IS THE MAIN ONE FOR THIS FEATURE. Playback commands positions
        the arm is known to reach, because a hand physically put it there. What is *not*
        safe is the very first command: if the arm is somewhere else when playback
        starts, commanding the recording's first pose is a jump across whatever distance
        separates them. So a caller must drive to this pose first, through the existing
        park machinery, and only then follow the samples.
        """
```

## mark at source lines 237–244

```text
        """Label the stretch from `t` (seconds into the recording) onward.

        ⭐ ROADMAP §8.2 item 8, the keypress half of his idea: press a key when a stretch
        goes wrong, press it again when it is good again. A recording starts implicitly
        `good`, and each mark holds until the next one. ⛔ Stored in `meta`, so the file
        format is unchanged and every recording saved before labels existed still loads —
        it simply has no marks, which reads as all-good, which is what it always was.
        """
```

## joint_speed at source lines 315–358

```text
        """Joint speed at a percentile of the sampled steps, in radians per second.

        ⭐⭐ WHY A PERCENTILE EXISTS ALONGSIDE THE PLAIN MAXIMUM, and it is a measurement
        that forced the question. **The measurement that decided it, 2026-08-13 at 09:35,
        slot 4 under commit `e89b745`:** `max 3.31 · p99 2.40 · p95 2.02 · median 0.47`.

        ⚠️ The maximum is 3.31 and the 99th percentile is 2.40. **A single sample is
        dragging the maximum up by 38%.** At 100 Hz one noisy reading of 0.033 rad is
        enough to do that, and a weightless arm being pushed by hand is exactly where such
        a reading comes from. Sizing a playback speed off the maximum therefore lets one
        bad sample veto the whole recording.

        ⚠️ The figures above are a re-measurement of that file made at 15:22 with this
        method. The original note here, and [FINDINGS §30](../docs/FINDINGS.md), recorded
        `p99 2.36` and *"40%"* for the same file. The 0.04 difference was never explained
        and the file was overwritten at 16:35, so it can no longer be settled. It changes
        nothing about the decision, and it is left visible rather than harmonised.

        ⛔ So use `joint_speed(99)` to decide a speed and `max_joint_speed()` to report what
        actually happened. Do not collapse them into one number: the maximum is the honest
        answer to "how fast did this go", and the percentile is the useful answer to "how
        fast is this, ignoring noise". Reporting only the percentile would hide a real fast
        movement, which is the failure this repo is named after.

        ⛔⭐⭐ THAT MEASUREMENT IS DATED BECAUSE THE FILE IT CAME FROM NO LONGER EXISTS, AND
        A LIVE TABLE HERE WENT STALE TWICE IN ONE DAY. This docstring used to carry a table
        of all five recordings. Version one was written at 10:01; Julien recorded over slot
        1 at 12:55 and over slots 3 and 4 at 16:34 and 16:35. **Recordings are saved by slot
        digit and `recordings/` is gitignored, so each overwrite destroys the file a written
        number described, permanently.** The same defect bit the same paragraph twice inside
        six hours ([FINDINGS §33.2](../docs/FINDINGS.md), [§34.7](../docs/FINDINGS.md)).

        ⭐ **The fix is not a fresher table, it is no table.** One dated measurement stays,
        because it is the evidence for a design decision and a decision's evidence does not
        expire. For current numbers run **`uv run scripts/check_recordings.py`**, which
        reads the files and therefore cannot lie. Every recording carries `commit` and
        `recorded_at`, which is the only reason either staleness was ever detectable.

        ⛔ So use `joint_speed(99)` to decide a speed and `max_joint_speed()` to report
        what actually happened. Do not collapse them into one number: the maximum is the
        honest answer to "how fast did this go", and the percentile is the useful answer to
        "how fast is this, ignoring noise". Reporting only the percentile would hide a real
        fast movement, which is the failure this repo is named after.
        """
```

## max_joint_speed at source lines 370–381

```text
        """Fastest any single joint moved, in radians per second.

        ⭐ WHY A CALLER NEEDS THIS BEFORE PLAYING ANYTHING BACK. A hand-taught path is
        safe at the speed it was taught, because a person was holding the arm at the
        time. Played back faster it is a different motion, and this rig has no emergency
        stop ([HANDOFF §4.5](../docs/HANDOFF.md)) so there is no hardware backstop under
        a bad guess. This is the number to check a speed multiplier against.

        ⚠️ It reports the fastest *sampled* step, so a spike between two samples at
        100 Hz is real but a spike inside 10 ms is invisible. It bounds what was
        recorded, not what the hand did.
        """
```

## trailing_still_seconds at source lines 391–422

```text
        """Seconds of near-motionless time at the END of the recording.

        ⭐⭐ WHY THIS IS A FUNCTION AND NOT A SENTENCE IN A DOCUMENT. The padding defect
        of [FINDINGS §30.1](../docs/FINDINGS.md) — `w` stopped the recording at the slot
        digit rather than at the keypress, so every file carried the time the save prompt
        spent waiting — was recorded in prose as *"slots 1, 3, 4, 5, 6 are all padded,
        discard them"*. ⛔ **That sentence was already wrong when it was written**, and
        nothing could see it: three of the five were recorded *after* the fix. Measuring
        it takes 20 lines, so it is measured.

        ⭐ The measured separation is wide, which is why one threshold works. Julien's five
        recordings on 2026-08-13, `still=0.05`:

            slot  commit    padding   share    verdict
              3   e89b745    4.46 s   57.3 %   before the fix
              4   e89b745    2.64 s   43.9 %   before the fix
              1   0e268ed    0.03 s    0.5 %   after the fix
              5   0e268ed    0.00 s    0.0 %   after the fix
              6   0e268ed    0.25 s    7.0 %   after the fix

        ⚠️ A padded tail is NOT motionless, which is the trap that makes a naive
        zero-speed test find nothing. A weightless arm held by a hand, or resting against
        its own gravity compensation, wobbles at a steady 0.032 to 0.038 rad/s — the two
        padded tails above sit on that floor for seconds at a time, dead flat. A threshold
        of 0.01 reports zero padding on all five files and hides the defect completely.

        ⚠️ It measures the *tail*, so it cannot see a pause in the middle, and a
        deliberate pause at the end of a demonstration reads the same as padding. Slot 6's
        0.25 s is that case: a quarter of a second is nowhere near the 1.8 to 4.4 s the
        defect produced, and pressing `w` shortly after the arm comes to rest looks exactly
        like this. ⛔ So read it as evidence about seconds, never about tenths.
        """
```

## resampled at source lines 437–449

```text
        """The same movement on an evenly spaced clock.

        ⭐ WHY THIS IS NEEDED RATHER THAN NICE. A 100 Hz control loop does not produce
        evenly spaced samples: every cycle takes slightly different work. Two things
        downstream assume an even clock. ABC's training file has one row per timestep
        with no timestamp column, and RoboTTT-style policies predict a *block* of future
        moves at once, which only means anything if the steps are equally spaced
        ([ROADMAP.md](../docs/ROADMAP.md) §9.2, §9.3).

        ⚠️ Resampling is lossy in one direction only: going *down* in rate drops detail
        that cannot come back. So record at the loop rate, save that, and resample when
        writing a dataset. Never the other way round.
        """
```

## describe_slot at source lines 540–551

```text
    """One line about the recording already in `path`, or `None` if there is none.

    ⛔⭐⭐ WHY THIS EXISTS: A SLOT HAS NOW BEEN OVERWRITTEN FIVE TIMES. Twice it destroyed the
    only copy of a measurement ([FINDINGS §33.2](../docs/FINDINGS.md), [§34.7](../docs/FINDINGS.md)),
    once more three hours later, and on 2026-08-14 Julien's first two-arm recording landed on
    `1.json` and replaced a hand-guided one-arm take from the day before. **Every time, the
    save prompt said nothing about what was already there.**

    ⭐ It reads the file rather than guessing from the name, and it never raises: a slot whose
    file is corrupt reports that, because "I cannot read what is in here" is exactly as
    important to the person about to overwrite it.
    """
```

## replay_step at source lines 586–612

```text
    """Advance a playback by one control cycle.

    ⭐ WHY A CLOCK AND NOT A DISTANCE ALONG THE PATH. The waypoint runner
    (`src/yam/motion.py`) walks a *shape* at a constant joint speed, which is right for a
    planned move between saved poses. It is wrong here, because it discards the one thing
    hand-guiding provides: **human timing and hesitation**. Those are the signal
    ([ROADMAP.md](../docs/ROADMAP.md) §6.6), so the cursor is measured in seconds.

    ⛔ WHY THE CLOCK CAN BE HELD. If the arm has fallen behind the pose being commanded,
    advancing anyway widens the gap, and the motion stops being the one that was recorded.
    Worse, when whatever was holding it back lets go, the arm crosses the accumulated gap
    at once, which is a lurch. So the clock waits for the arm. Borrowed from the park loop,
    where the same reasoning produced `MAX_CURSOR_LAG`.

    ⚠️ `held` is returned rather than acted on, because "the arm has been stuck for four
    seconds" is a *session* judgement: only the caller knows how long it has been true and
    what to say about it. Same division of labour as `src/yam/session.py`.

    `compare` lists which joint INDICES count towards the lag. Leave the grippers out: the
    jaws legitimately sit far from their commanded value while closing on an object, and
    counting that as "the arm cannot follow" would stall every playback that grips anything.

    ⚠️ IT USED TO BE `n_compare`, a count of leading joints, which worked while the gripper
    was the LAST element of a one-arm sample. With two arms the vector is `[B0..B6, G0..G6]`
    and arm B's gripper sits in the middle, so a prefix can no longer express "skip the
    grippers". `Layout.tracked_indices()` builds the list.
    """
```

## scrub_step at source lines 664–683

```text
    """One control cycle of PUCK-SCRUBBED playback: the hand is the clock.

    The same shape as `replay_step`, with three deliberate differences:

    1. **The cursor moves at `scrub_rate(deflection)` and can run BACKWARDS.** The pose
       at every cursor value is one a hand physically put the arm in, so playing the
       samples in reverse commands only poses the recording already proved reachable.
    2. **It clamps at both ends and never finishes.** Reaching the end of the recording
       under a scrub means "the dial hit the last frame", not "the run is over" — the
       operator decides when it is over, by leaving the mode. `finished` is always False.
    3. **The lag hold works in both directions.** If the arm falls `max_lag` behind the
       commanded pose, the cursor freezes exactly as in a normal playback, whichever way
       the hand is dragging it.

    `max_rate` is the full-push pace in recording-seconds per second — his time-lapse
    dial (2026-08-18: *"more than normal speed if I fully press the control forward"*).
    ⚠️ A high value is safe by construction: the cursor can outrun the arm, and then the
    lag hold freezes it until the arm catches up, so the ARM's speed is still bounded by
    `SafeRobot` and the recording's own motion — only the CLOCK is fast.
    """
```

## TrackingLog at source lines 698–724

```text
    """Per joint: how far behind the arm ran, and how fast it was being asked to move.

    ⭐⭐ WHY THIS EXISTS RATHER THAN A SPEED SWEEP. Julien asked on 2026-08-13 how fast the
    arms can really move, and whether the 1.5 rad/s limit could be raised. The obvious way
    to find out is a script that drives one joint faster and faster until it cannot keep up.
    ⛔ **That script would deliberately command the arm faster than any existing code
    allows, and the agent cannot test it.** Session 4 is the standing warning: three changes
    passed their tests and produced three failures on first hardware contact, one of which
    dropped 4.3 kg ([FINDINGS §11](../docs/FINDINGS.md)).

    ⭐ **There is a version that needs no new motion at all.** Every playback already
    commands a hand-taught path and already measures how far behind the arm is. Recording
    that per joint, against the speed each joint was being asked for, answers the same
    question using hardware time Julien is already spending. His recordings reach 2.9 rad/s
    at the 99th percentile, so the interesting range is already covered.

    ⚠️ WHAT THIS CANNOT TELL YOU, and it matters when reading the table:

    - The playback holds its clock once the arm falls behind, so the commanded speed is not
      a clean sweep. The pairs are still real; the coverage is uneven.
    - Load depends on the arm's pose, so the same joint at the same speed lags differently
      with the arm extended and folded.
    - It only ever reports speeds a recording happened to contain.

    ⭐ **So this is the cheap first answer.** If it comes out ambiguous, the active sweep is
    designed in [ROADMAP.md](../docs/ROADMAP.md) §7.5 and can be built then, with a reason.
    """
```
