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
*The class decides, the script narrates.* Same rule as `src/arm_session.py`. Every
decision below can therefore be proven with no arm plugged in, which matters on a rig
where three changes have passed their tests and then failed on first contact with the
hardware ([FINDINGS §11](../docs/FINDINGS.md)).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

__all__ = ["Sample", "Trajectory", "ReplayStep", "replay_step", "safe_time_scale",
           "scrub_rate", "scrub_step",
           "TrackingLog"]

# ⚠️ Radians, rounded on save. 1e-5 rad is 0.0006°, which is roughly a thousand times
# finer than the controller's own steady-state error (0.02 rad is "arrived" for a park,
# see PARK_TOLERANCE). So this loses nothing real and roughly halves the file.
SAVE_PRECISION = 5

# ⛔ Refuse to record a sample that arrived before the previous one. A control loop
# should never produce that, and if it does, every later calculation here (speed,
# resampling, duration) silently returns nonsense rather than raising.
MIN_STEP = 0.0


def safe_time_scale(recorded_speed: float, cap: float) -> float:
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
    if cap <= 0:
        raise ValueError(f"cap must be positive, got {cap}")
    if recorded_speed <= 0:
        return float("inf")             # nothing moved, so no speed can breach the cap
    return cap / recorded_speed


@dataclass(frozen=True)
class Sample:
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

    t: float
    q: tuple[float, ...]


@dataclass(frozen=True)
class Layout:
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

    arms: tuple[str, ...]
    per_arm: int

    @property
    def n_joints(self) -> int:
        return len(self.arms) * self.per_arm

    def slice_for(self, arm: str) -> slice:
        """Which part of a flat sample belongs to `arm`."""
        i = self.arms.index(arm)
        return slice(i * self.per_arm, (i + 1) * self.per_arm)

    def tracked_indices(self, n_arm: int) -> list[int]:
        """Every ARM joint, with the grippers left out.

        ⛔ The grippers must not count towards "has the arm fallen behind". Jaws sit far
        from their commanded value while closing on an object, and counting that as lag
        would stall every playback that grips anything. With one arm a prefix count said
        this; with two the gripper sits in the MIDDLE of the vector, so it takes indices.
        """
        return [i * self.per_arm + j
                for i in range(len(self.arms))
                for j in range(min(n_arm, self.per_arm))]

    def to_meta(self) -> dict[str, Any]:
        return {"arms": list(self.arms), "joints_per_arm": self.per_arm}

    @classmethod
    def from_meta(cls, meta: dict[str, Any], n_joints: int) -> Layout:
        """Read the layout back, INCLUDING from recordings made before it existed.

        ⭐ Old files carry `meta["arm"] = "B"` and 7 joints and nothing else. They are still
        playable, and they must be: Julien has recordings in slots 1 to 6 that were made
        before two arms existed, and a format change that quietly stopped reading them would
        throw away hardware time he has already spent.
        """
        arms = meta.get("arms")
        if isinstance(arms, list) and arms:
            per_arm = int(meta.get("joints_per_arm") or (n_joints // len(arms)) or n_joints)
            return cls(tuple(str(a) for a in arms), per_arm)
        # ⚠️ One arm, from a file written before the layout was recorded. Fall back to the
        # single `arm` field, and to the sample width for the joint count.
        one = meta.get("arm")
        return cls((str(one) if one else "?",), n_joints)


@dataclass
class Trajectory:
    """A hand-taught movement: samples in time order, plus how it was made.

    ⭐ `meta` is free-form on purpose, because the fields that matter are a *provenance*
    question rather than a code question, and Julien answered it on 2026-08-12: he wants
    *"the history and everything we found and problems we had, and being able to
    reproduce everything and connect it to other research papers."* The names the caller
    should fill are listed in `META_FIELDS`. ⛔ All of it is cheap to write while
    recording and impossible to reconstruct afterwards.
    """

    samples: list[Sample] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)

    #: What a caller should put in `meta`. Not enforced, because a missing field must
    #: never be the reason a demonstration is lost, but a recording without these is
    #: much less useful later. See ROADMAP §6.6 and §9.2.
    META_FIELDS = ("arm", "commit", "recorded_at", "nominal_hz", "method", "notes")

    # ---------------------------------------------------------------- building ----

    def append(self, t: float, q: Sequence[float]) -> None:
        """Add one sample. ⛔ Refuses to go backwards in time."""
        if self.samples and t <= self.samples[-1].t + MIN_STEP:
            raise ValueError(
                f"sample at t={t} is not after the previous one at "
                f"t={self.samples[-1].t}; a recording must move forwards"
            )
        if self.samples and len(q) != len(self.samples[0].q):
            raise ValueError(
                f"this sample has {len(q)} joints and the recording has "
                f"{len(self.samples[0].q)}; a joint count cannot change mid-recording"
            )
        self.samples.append(Sample(float(t), tuple(float(x) for x in q)))

    # ---------------------------------------------------------------- reading ----

    def __len__(self) -> int:
        return len(self.samples)

    @property
    def duration(self) -> float:
        """Seconds from the first sample to the last. Zero if there is nothing to play."""
        return 0.0 if len(self.samples) < 2 else self.samples[-1].t - self.samples[0].t

    @property
    def n_joints(self) -> int:
        return len(self.samples[0].q) if self.samples else 0

    def start_pose(self) -> tuple[float, ...] | None:
        """Where the arm has to be before playback can begin.

        ⛔ SAFETY, AND IT IS THE MAIN ONE FOR THIS FEATURE. Playback commands positions
        the arm is known to reach, because a hand physically put it there. What is *not*
        safe is the very first command: if the arm is somewhere else when playback
        starts, commanding the recording's first pose is a jump across whatever distance
        separates them. So a caller must drive to this pose first, through the existing
        park machinery, and only then follow the samples.
        """
        return self.samples[0].q if self.samples else None

    def pose_at(self, t: float) -> tuple[float, ...]:
        """Where the arm was at time `t`, interpolated between the two nearest samples.

        Clamped at both ends, so a caller cannot walk off the recording and get a
        surprise. Times are measured from the first sample.
        """
        if not self.samples:
            raise ValueError("an empty recording has no pose")
        first = self.samples[0].t
        want = first + max(0.0, min(self.duration, t))
        if len(self.samples) == 1:
            return self.samples[0].q
        lo = 0
        hi = len(self.samples) - 1
        while hi - lo > 1:                       # binary search, so long recordings are cheap
            mid = (lo + hi) // 2
            if self.samples[mid].t <= want:
                lo = mid
            else:
                hi = mid
        a, b = self.samples[lo], self.samples[hi]
        span = b.t - a.t
        f = 0.0 if span <= 0 else (want - a.t) / span
        return tuple(x + (y - x) * f for x, y in zip(a.q, b.q))

    def joint_speed(self, percentile: float = 100.0) -> float:
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
        speeds = sorted(
            max(abs(y - x) for x, y in zip(a.q, b.q)) / (b.t - a.t)
            for a, b in zip(self.samples, self.samples[1:])
            if b.t > a.t
        )
        if not speeds:
            return 0.0
        idx = min(len(speeds) - 1, max(0, round(percentile / 100.0 * len(speeds)) - 1))
        return speeds[idx]

    def max_joint_speed(self) -> float:
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
        fastest = 0.0
        for a, b in zip(self.samples, self.samples[1:]):
            dt = b.t - a.t
            if dt <= 0:
                continue
            fastest = max(fastest, max(abs(y - x) for x, y in zip(a.q, b.q)) / dt)
        return fastest

    def trailing_still_seconds(self, still: float = 0.05) -> float:
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
        if len(self.samples) < 2:
            return 0.0
        i = len(self.samples) - 1
        while i > 0:
            a, b = self.samples[i - 1], self.samples[i]
            dt = b.t - a.t
            if dt > 0 and max(abs(y - x) for x, y in zip(a.q, b.q)) / dt > still:
                break
            i -= 1
        return self.samples[-1].t - self.samples[i].t

    # ------------------------------------------------------------- reshaping ----

    def resampled(self, hz: float) -> Trajectory:
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
        if hz <= 0:
            raise ValueError(f"hz must be positive, got {hz}")
        if len(self.samples) < 2:
            return Trajectory(list(self.samples), dict(self.meta))
        out = Trajectory(meta={**self.meta, "resampled_hz": hz})
        step = 1.0 / hz
        n = int(self.duration / step) + 1
        for i in range(n):
            t = i * step
            out.samples.append(Sample(t, self.pose_at(t)))
        # ⚠️ The final sample is kept even when the grid misses it, so the recording
        # still ends where the hand left the arm. Dropping it would quietly shorten
        # every playback by up to one step.
        if out.samples[-1].t < self.duration:
            out.samples.append(Sample(self.duration, self.samples[-1].q))
        return out

    def time_scaled(self, factor: float) -> Trajectory:
        """The same path, played `factor` times faster. ⚠️ Above 1.0 this is a NEW motion.

        ⛔ The class does not refuse a fast factor, because refusing is a session
        decision and this class has no way to know what the operator can see or reach.
        It gives the caller `max_joint_speed()` to check against instead. Same division
        of labour as `src/arm_session.py`.
        """
        if factor <= 0:
            raise ValueError(f"factor must be positive, got {factor}")
        scaled = Trajectory(meta={**self.meta, "time_scale": factor})
        base = self.samples[0].t if self.samples else 0.0
        for s in self.samples:
            scaled.samples.append(Sample((s.t - base) / factor, s.q))
        return scaled

    # ----------------------------------------------------------------- files ----

    def to_dict(self) -> dict[str, Any]:
        """⭐ Rows of `[t, q0 … qN]`, not a list of objects.

        A five-minute recording at 100 Hz is 30 000 samples. As objects with named keys
        that is several megabytes of repeated field names; as rows it stays readable in
        an editor and diffable in git, which is the reason this is JSON at all rather
        than a binary blob.
        """
        return {
            "meta": self.meta,
            "n_joints": self.n_joints,
            "samples": [
                [round(s.t, SAVE_PRECISION), *[round(x, SAVE_PRECISION) for x in s.q]]
                for s in self.samples
            ],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Trajectory:
        traj = cls(meta=dict(data.get("meta", {})))
        for row in data.get("samples", []):
            traj.samples.append(Sample(float(row[0]), tuple(float(x) for x in row[1:])))
        return traj

    def save(self, path: Path) -> None:
        """⛔ Refuses to write a recording with nothing in it.

        An empty file that looks like a demonstration is worse than no file: it survives
        into a dataset and trains on nothing. This repo's rule is that a stack which
        fails by lying is the dangerous kind ([FINDINGS §0](../docs/FINDINGS.md)).
        """
        if len(self.samples) < 2:
            raise ValueError(
                f"refusing to save a recording with {len(self.samples)} sample(s); "
                "there is no movement in it"
            )
        path.write_text(json.dumps(self.to_dict(), indent=1))

    @classmethod
    def load(cls, path: Path) -> Trajectory:
        return cls.from_dict(json.loads(path.read_text()))


@dataclass(frozen=True)
class ReplayStep:
    """What one cycle of playback should do. Every field is a decision, not a report."""

    cursor: float                    # where the clock now stands, in seconds
    target: tuple[float, ...]        # the pose to command
    lag: float                       # how far the arm is behind that pose, radians
    finished: bool                   # the recording has been played to its end
    held: bool                       # the clock did NOT advance, because of lag


def describe_slot(path: Path) -> str | None:
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
    if not path.is_file():
        return None
    try:
        traj = Trajectory.load(path)
    except Exception as exc:  # noqa: BLE001
        return f"an unreadable file ({type(exc).__name__})"
    arms = traj.meta.get("arms") or ([traj.meta.get("arm")] if traj.meta.get("arm") else [])
    who = ",".join(str(a) for a in arms) or "?"
    when = str(traj.meta.get("recorded_at", "?"))[:16]
    return (f"{traj.duration:.1f}s on {who}, {traj.meta.get('method', '?')}, "
            f"recorded {when}")


def replay_step(traj: Trajectory, cursor: float, measured: Sequence[float], dt: float,
                speed: float = 1.0, max_lag: float = 0.15,
                compare: Sequence[int] | None = None) -> ReplayStep:
    """Advance a playback by one control cycle.

    ⭐ WHY A CLOCK AND NOT A DISTANCE ALONG THE PATH. The waypoint runner
    (`src/motion.py`) walks a *shape* at a constant joint speed, which is right for a
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
    what to say about it. Same division of labour as `src/arm_session.py`.

    `compare` lists which joint INDICES count towards the lag. Leave the grippers out: the
    jaws legitimately sit far from their commanded value while closing on an object, and
    counting that as "the arm cannot follow" would stall every playback that grips anything.

    ⚠️ IT USED TO BE `n_compare`, a count of leading joints, which worked while the gripper
    was the LAST element of a one-arm sample. With two arms the vector is `[B0..B6, G0..G6]`
    and arm B's gripper sits in the middle, so a prefix can no longer express "skip the
    grippers". `Layout.tracked_indices()` builds the list.
    """
    if not traj.samples:
        raise ValueError("cannot play back an empty recording")
    if speed <= 0:
        raise ValueError(f"speed must be positive, got {speed}")
    target = traj.pose_at(cursor)
    usable = min(len(target), len(measured))
    idx = range(usable) if compare is None else [i for i in compare if i < usable]
    lag = max((abs(target[i] - measured[i]) for i in idx), default=0.0)
    held = lag >= max_lag
    moved = cursor if held else cursor + dt * speed
    return ReplayStep(
        cursor=moved,
        target=target,
        lag=lag,
        finished=moved >= traj.duration,
        held=held,
    )


#: ⭐ The scrub dial (ROADMAP §7.6, item 13 — his idea, built 2026-08-18). Deflection below
#: this is treated as "hands off", ON TOP of the reader's own hardware deadzone, because the
#: scrub must freeze the instant the hand leaves the puck — that release-to-stop property is
#: the whole safety argument for the feature on a rig with no e-stop.
SCRUB_DEADBAND = 0.15

#: ⭐ Full deflection scrubs at 1.5× the recorded pace, either direction. Deliberately at the
#: same ceiling `safe_time_scale` would allow a normal playback, so scrubbing can never ask
#: the arm for speeds a plain `l` run could not — and SafeRobot still binds underneath.
SCRUB_MAX_RATE = 1.5


def scrub_rate(deflection: float, deadband: float = SCRUB_DEADBAND,
               max_rate: float = SCRUB_MAX_RATE) -> float:
    """Puck deflection (−1..1) → signed playback rate, in recording-seconds per second.

    ⭐ Push forward = the recording runs forward; pull back = it runs backwards; let go =
    it freezes. A SpaceMouse is spring-centred, so the neutral state is STOPPED and the
    dial is a deadman by construction (ROADMAP §7.6). The response is linear past the
    deadband, so half a push is half the pace — a scrub wheel, not a switch.
    """
    d = float(deflection)
    if abs(d) <= deadband:
        return 0.0
    span = min(1.0, (abs(d) - deadband) / (1.0 - deadband))
    return math.copysign(span * max_rate, d)


def scrub_step(traj: Trajectory, cursor: float, measured: Sequence[float], dt: float,
               deflection: float, max_lag: float = 0.15,
               compare: Sequence[int] | None = None) -> ReplayStep:
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
    """
    if not traj.samples:
        raise ValueError("cannot scrub an empty recording")
    target = traj.pose_at(cursor)
    usable = min(len(target), len(measured))
    idx = range(usable) if compare is None else [i for i in compare if i < usable]
    lag = max((abs(target[i] - measured[i]) for i in idx), default=0.0)
    held = lag >= max_lag
    moved = cursor if held else min(traj.duration,
                                    max(0.0, cursor + dt * scrub_rate(deflection)))
    return ReplayStep(cursor=moved, target=target, lag=lag, finished=False, held=held)


class TrackingLog:
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

    def __init__(self, n_joints: int) -> None:
        self.n_joints = n_joints
        self.cycles = 0
        self._worst_lag = [0.0] * n_joints
        self._speed_at_worst_lag = [0.0] * n_joints
        self._top_speed = [0.0] * n_joints
        self._lag_at_top_speed = [0.0] * n_joints

    def observe(self, target: Sequence[float], prev_target: Sequence[float],
                measured: Sequence[float], dt: float) -> None:
        """One control cycle. `dt` must be MEASURED time, not the nominal loop period.

        ⚠️ The nominal period is wrong by about 13% on this rig, because the loop runs near
        87 Hz rather than 100 ([FINDINGS §31.1](../docs/FINDINGS.md)). Feeding it here would
        overstate every speed by the same amount and quietly bias the answer.
        """
        if dt <= 0:
            return
        self.cycles += 1
        n = min(self.n_joints, len(target), len(prev_target), len(measured))
        for i in range(n):
            speed = abs(target[i] - prev_target[i]) / dt
            lag = abs(target[i] - measured[i])
            if lag > self._worst_lag[i]:
                self._worst_lag[i] = lag
                self._speed_at_worst_lag[i] = speed
            if speed > self._top_speed[i]:
                self._top_speed[i] = speed
                self._lag_at_top_speed[i] = lag

    def rows(self) -> list[tuple[int, float, float, float, float]]:
        """`(joint index, worst lag, speed then, top speed, lag then)`, one row per joint.

        ⭐ Both pairs are reported because they answer different questions. "How far behind
        did it get, and how fast was it going" tells you where the limit is. "How fast was it
        asked to go, and did it manage" tells you whether that speed is usable at all.
        """
        return [
            (i, self._worst_lag[i], self._speed_at_worst_lag[i],
             self._top_speed[i], self._lag_at_top_speed[i])
            for i in range(self.n_joints)
        ]

    def to_dict(self, joint_names: Sequence[str] | None = None) -> dict[str, Any]:
        """The table as data, so a playback's measurement outlives the terminal it printed in.

        ⭐⭐ WHY THIS EXISTS, and it is a defect in how this project records measurements
        rather than a defect in code. On 2026-08-13 this table was printed to Julien's
        terminal, pasted into a chat, and analysed there. **That is the only copy.** Three
        separate documented numbers went stale inside a day for exactly that reason
        ([FINDINGS §33.3](../docs/FINDINGS.md)), and the arm-speed answer is worth more than
        a paste: it is the first measurement of what this hardware can physically follow.

        ⭐ Written per playback and stamped with a time, so nothing overwrites anything. That
        is deliberately unlike the recordings themselves, which are saved by slot digit and
        silently replace each other — the defect of [FINDINGS §33.2](../docs/FINDINGS.md),
        and it bit the very files this table was measured from, twice in one afternoon.

        ⚠️ Read the caveats on this class before drawing a conclusion from a saved file. The
        speeds are whatever a recording happened to contain, the coverage is uneven because
        the playback holds its clock when the arm falls behind, and load depends on the pose.
        """
        return {
            "cycles": self.cycles,
            "n_joints": self.n_joints,
            "joints": [
                {
                    "index": i,
                    "name": (joint_names[i] if joint_names and i < len(joint_names)
                             else f"joint{i + 1}"),
                    "worst_lag_rad": round(worst, 5),
                    "speed_at_worst_lag": round(at_speed, 5),
                    "top_speed": round(top, 5),
                    "lag_at_top_speed": round(lag_top, 5),
                }
                for i, worst, at_speed, top, lag_top in self.rows()
            ],
        }
