"""Measured joint trajectories, replay/scrub decisions and tracking summaries.

Trajectory stores measured poses on one timeline plus caller-supplied metadata.
RecordingSession owns live acquisition/writers; PlaybackSession coordinates
replay across arms. This module contains no robot handle or motion commands.
Dataset/MCAP export exists separately in yam.dataset and yam.episode; inferred
next-state actions are not a recorded command stream.

Historical rationale is preserved in `docs/archive/recording-source-notes.md`.
Current contracts below describe the code, including its validation limits.
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
    """Return cap / recorded_speed; reject a nonpositive cap.

    Values below one expose recordings faster than the cap; do not hide that
    by flooring the result at one. Nonpositive recorded speed yields infinity.
    The caller chooses playback speed; this ratio is not a measured physical
    tracking capability.
    """
    if cap <= 0:
        raise ValueError(f"cap must be positive, got {cap}")
    if recorded_speed <= 0:
        return float("inf")             # nothing moved, so no speed can breach the cap
    return cap / recorded_speed


@dataclass(frozen=True)
class Sample:
    """One timestamp and flattened joint-position tuple.

    The live recorder supplies measured positions. Source semantics are the
    caller's responsibility; this container does not distinguish a measured
    pose from an invented/commanded one. Export action inference is separate.
    """

    t: float
    q: tuple[float, ...]


@dataclass(frozen=True)
class Layout:
    """Map flattened samples to arms in recorded order, per_arm joints each.

    Legacy metadata with one "arm" is supported; unknown ownership is "?".
    tracked_indices selects arm joints across every slice, excluding jaws when
    n_arm is smaller than per_arm. Consumers validate layout compatibility;
    metadata parsing alone does not establish a complete valid recording.
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
    """Joint samples and free-form provenance on a common time axis.

    append checks increasing timestamps and consistent joint counts. Loading
    JSON does not run those append checks; consumers must validate data for
    their purpose. Metadata is intentionally permissive so missing provenance
    does not itself prevent saving measured samples.
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
        """Return the first measured pose, or None for an empty trajectory.

        The caller parks there and checks measured arrival before replay starts.
        Returning this pose does not itself command or validate any motion.
        """
        return self.samples[0].q if self.samples else None

    # ---------------------------------------------------------------- labels ----

    #: The only two labels a stretch can carry. Two on purpose: his idea (ROADMAP §6.6)
    #: is marking BAD stretches of an otherwise good demonstration, so the model is
    #: "everything is good until marked bad, and bad until marked good again" — a
    #: richer taxonomy would mean deciding categories before any dataset exists to need them.
    LABELS = ("good", "bad")

    def mark(self, t: float, label: str) -> None:
        """Append a good/bad label boundary at recording-relative time t.

        Reject unknown labels and backwards mark times. Marks describe export
        selection and never change the trajectory or command the arm.
        """
        if label not in self.LABELS:
            raise ValueError(f"label must be one of {self.LABELS}, not {label!r}")
        marks = self.meta.setdefault("marks", [])
        if marks and float(t) < float(marks[-1]["t"]):
            raise ValueError("marks must arrive in time order, like samples")
        marks.append({"t": float(t), "label": label})

    def label_at(self, t: float) -> str:
        """The label in force at `t`: `good` until a mark at or before `t` says otherwise."""
        label = "good"
        for m in self.meta.get("marks", []):
            if float(m["t"]) <= t:
                label = str(m["label"])
            else:
                break
        return label

    def label_spans(self) -> list[tuple[float, float, str]]:
        """Merged `(start, end, label)` spans covering the whole recording.

        Marks outside the sampled range clamp to it, consecutive same-label marks merge,
        and an empty recording has no spans. This is the shape the dataset export (item 7)
        consumes: it needs stretches, not keypresses.
        """
        if not self.samples:
            return []
        t0, t1 = self.samples[0].t, self.samples[-1].t
        spans: list[tuple[float, float, str]] = []
        cur_label, cur_start = "good", t0
        for m in self.meta.get("marks", []):
            mt = min(max(float(m["t"]), t0), t1)
            if m["label"] == cur_label:
                continue
            if mt > cur_start:
                spans.append((cur_start, mt, cur_label))
            cur_label, cur_start = str(m["label"]), mt
        if t1 > cur_start or not spans:
            spans.append((cur_start, t1, cur_label))
        return spans

    def bad_seconds(self) -> float:
        """How much of the recording is marked bad, for summaries and for filtering."""
        return sum(e - s for s, e, label in self.label_spans() if label == "bad")

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
        """Percentile of maximum per-joint finite-difference speed, in rad/s.

        For each positive-duration interval, take its fastest coordinate, then
        select the requested percentile across intervals (rank clamped to the
        available samples). Return zero when none exist. This measures sampled
        motion, not unsampled peaks or motor capability. Use the maximum when a
        percentile would conceal a brief excursion.
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
        """Maximum sampled per-coordinate speed in rad/s; zero without intervals.

        Skip nonpositive-duration pairs. A spike between measurements is invisible;
        this is evidence about the recorded sequence, not an absolute speed bound.
        """
        fastest = 0.0
        for a, b in zip(self.samples, self.samples[1:]):
            dt = b.t - a.t
            if dt <= 0:
                continue
            fastest = max(fastest, max(abs(y - x) for x, y in zip(a.q, b.q)) / dt)
        return fastest

    def trailing_still_seconds(self, still: float = 0.05) -> float:
        """Measure contiguous low-speed time at the end of the samples.

        Walk backwards until any coordinate's finite-difference speed exceeds
        still (rad/s). Deliberate end pauses look like unwanted padding; this
        cannot diagnose cause or detect pauses in the middle. Return zero for
        fewer than two samples.
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
        """Interpolate onto a positive-rate clock starting at zero seconds.

        Keep an off-grid final endpoint. With fewer than two samples, return a copy
        without interpolation. Metadata is copied and the requested rate recorded;
        this does not add information between the original measurements.
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
        of labour as `src/yam/session.py`.
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
    """Describe the file currently in a slot before proposing an overwrite.

    Return None for no file and an explicit unreadable description on load
    failure. Never infer that a corrupt/unreadable slot is empty or disposable.
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


def slot_overview(takes_dir: Path, slots: str = "0123456789") -> list[str]:
    """One line per occupied slot plus which are free — the whole shelf, once, up front.

    ⭐ WHY ([FINDINGS §71.6](../docs/FINDINGS.md)): on 2026-08-19 every slot held a recording, and the save prompt revealed that one digit at a time — Julien pressed ELEVEN keys and read nine one-slot warnings before finding one he was willing to replace. `describe_slot` already knew everything each warning said; this shows all of it before the first digit, and the per-digit replace confirmation stays as the guard it always was.
    """
    lines: list[str] = []
    free: list[str] = []
    for k in slots:
        desc = describe_slot(takes_dir / f"{k}.json")
        if desc is None:
            free.append(k)
        else:
            lines.append(f"     {k}: {desc}")
    lines.append("     free: " + (" ".join(free) if free
                                  else "none — every slot is occupied, so any save replaces"))
    return lines


def replay_step(traj: Trajectory, cursor: float, measured: Sequence[float], dt: float,
                speed: float = 1.0, max_lag: float = 0.15,
                compare: Sequence[int] | None = None) -> ReplayStep:
    """Return the current target and next shared playback cursor.

    Compare current target with measured coordinates; hold the cursor when
    lag reaches max_lag, otherwise advance by dt * speed. compare selects
    tracked coordinates (normally arm joints, excluding held jaws); default
    uses all shared coordinates. Reject empty trajectories/nonpositive speed.
    The target precedes cursor advancement, and finished concerns the cursor,
    not physical arrival. The caller issues constrained commands.
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
               compare: Sequence[int] | None = None,
               max_rate: float = SCRUB_MAX_RATE) -> ReplayStep:
    """Move the shared cursor forward/backward according to puck deflection.

    Use the same target-versus-measured lag gate as replay; hold when lagged.
    Clamp cursor to the trajectory endpoints and never finish automatically:
    a released puck or endpoint holds until the caller changes mode.
    The returned target precedes cursor advancement; no robot is commanded.
    """
    if not traj.samples:
        raise ValueError("cannot scrub an empty recording")
    target = traj.pose_at(cursor)
    usable = min(len(target), len(measured))
    idx = range(usable) if compare is None else [i for i in compare if i < usable]
    lag = max((abs(target[i] - measured[i]) for i in idx), default=0.0)
    held = lag >= max_lag
    moved = cursor if held else min(traj.duration,
                                    max(0.0, cursor + dt * scrub_rate(
                                        deflection, max_rate=max_rate)))
    return ReplayStep(cursor=moved, target=target, lag=lag, finished=False, held=held)


class TrackingLog:
    """Track per-joint requested target speed and target-minus-measured lag.

    Keep the speed at worst lag and lag at top requested speed. Speeds come
    from successive replay targets before downstream command limits; they are
    not measured motor speeds. Ignore nonpositive dt and compare only shared
    coordinates. This records evidence without changing motion.
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
