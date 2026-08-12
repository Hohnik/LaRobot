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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

# ⚠️ Radians, rounded on save. 1e-5 rad is 0.0006°, which is roughly a thousand times
# finer than the controller's own steady-state error (0.02 rad is "arrived" for a park,
# see PARK_TOLERANCE). So this loses nothing real and roughly halves the file.
SAVE_PRECISION = 5

# ⛔ Refuse to record a sample that arrived before the previous one. A control loop
# should never produce that, and if it does, every later calculation here (speed,
# resampling, duration) silently returns nonsense rather than raising.
MIN_STEP = 0.0


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
