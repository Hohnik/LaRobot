"""Export a saved recording as one MCAP episode in the team's contracted shape.

⭐⭐ THE CONTRACT IS [Setup-Anleitung.md](../../docs/Setup-Anleitung.md) C3, verbatim: the topic names below, arm-state dimension 6 and end-effector (jaw) dimension 1 per side, **tick = 33,333,333 ns** (30 Hz) with every stream synchronous, and the ACTION SPACE IS JOINT SPACE — the commanded joint target, never the SpaceMouse input. *"MCAP ist Single Source of Truth"* is that document's own emphasis: get the names and dimensions exact and ABC's whole training chain runs unchanged.

⛔⭐ WHAT THIS MODULE CANNOT PROMISE, said once and honestly: the Anleitung specifies names, dimensions and the tick — it does NOT specify ABC's message encoding byte for byte (that lives in ABC's own `export_mcap.py`, which is not in this repo). This exporter writes ROS2-encoded messages (`mcap-ros2-support`, the library the team's own plan put in the dependencies) with a minimal `yam_msgs/Vector` schema. **The Anleitung's own C4 gate exists for exactly this**: convert 2-3 mini-episodes and verify against ABC's loader BEFORE collecting for real. Until that gate has run, this file's output is "the contract as written", not "verified ABC input".

⭐ THE ACTION-POLICY DECISION, recorded where it is made: a hand-taught recording holds MEASURED positions only — in GUIDE there is no command stream to record, because the position gain is zero and the hand is the controller. So the exported action at tick k is the state at tick k+1 (the next-tick target), which is the standard demonstrated-action construction for position-controlled teleop data. The episode's own metadata names this policy, so a training run can never mistake it for a recorded command.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from yam.recording import Layout, Trajectory

#: Setup-Anleitung C3: 30 Hz, and the tick is given in NANOSECONDS, exactly.
CONTRACT_TICK_NS = 33_333_333
ARM_DIM = 6   # joints per arm-state/-action message
EE_DIM = 1    # the jaw, normalised 0..1, per ee-state/-action message

#: The eight vector topics of the contract, exactly as C3 spells them.
SIDES = ("left", "right")
VECTOR_TOPICS = tuple(
    f"/{side}-{kind}-{what}"
    for side in SIDES for kind in ("arm", "ee") for what in ("state", "action")
)

#: Everything else rides on one extra topic, which C3 explicitly encourages
#: ("Zusätzlich alles mitloggen … in eigenen Topics"): provenance, labels, the arm
#: mapping, and the action policy, as one JSON message at t0.
META_TOPIC = "/episode-meta"

VECTOR_MSGDEF = "float64[] data"
TEXT_MSGDEF = "string data"


@dataclass(frozen=True)
class ExportReport:
    """What was written, for the caller to narrate and the tests to assert on."""

    path: Path
    ticks: int
    duration_s: float
    mapping: dict[str, str]          # {"left": "B", "right": "G"}
    bad_spans: int
    warnings: tuple[str, ...] = field(default=())


def export_episode(traj: Trajectory, left: str, right: str, out_path: Path,
                   source: str = "?") -> ExportReport:
    """Write one episode. Refuses loudly rather than mislabelling training data.

    ⛔ `left` and `right` are REQUIRED and never defaulted: the contract's sides are
    physical positions on the bench, nothing in a recording can derive them, and a
    silently wrong side is the worst kind of dataset poison — every episode would be
    mirrored and nothing would raise. The mapping is written into the episode's own
    metadata so it can be audited later.
    """
    from mcap_ros2.writer import Writer as Ros2Writer  # noqa: PLC0415 — import cost only when exporting

    layout = Layout.from_meta(traj.meta, traj.n_joints)
    if sorted((left, right)) != sorted(layout.arms):
        raise ValueError(
            f"this recording holds arm(s) {', '.join(layout.arms)} and the mapping says "
            f"left={left}, right={right} — an ABC episode needs BOTH sides, so a "
            "recording that does not carry exactly these two arms cannot become one. "
            "Record with --arms and both arms in the session."
        )
    if layout.per_arm != ARM_DIM + EE_DIM:
        raise ValueError(
            f"the contract wants {ARM_DIM}+{EE_DIM} values per arm and this recording "
            f"carries {layout.per_arm} — refusing rather than guessing which to drop."
        )
    if len(traj.samples) < 2:
        raise ValueError("an episode needs at least two samples to have a duration")

    t0 = traj.samples[0].t
    duration = traj.duration
    ticks = int(duration * 1e9 // CONTRACT_TICK_NS) + 1
    warnings: list[str] = []
    warnings.append(
        "no camera topics were written: camera frames are not wired into the recorder "
        "yet (yam.cameras exists; the session does not sample it while recording). An "
        "episode without images cannot train ABC — this export proves the joint/action "
        "side of the contract."
    )

    spans = traj.label_spans()
    meta = {
        "contract": "Setup-Anleitung C3",
        "tick_ns": CONTRACT_TICK_NS,
        "mapping": {"left": left, "right": right},
        "action_policy": "next-tick measured state (hand-taught demo; no command stream exists in GUIDE)",
        "labels": [{"start": s, "end": e, "label": lab} for s, e, lab in spans],
        "source": source,
        "recording_meta": dict(traj.meta),
        "time_base": "tick 0 = the recording's first sample; log_time = k * tick_ns",
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        writer = Ros2Writer(f)
        vec_schema = writer.register_msgdef("yam_msgs/Vector", VECTOR_MSGDEF)
        txt_schema = writer.register_msgdef("yam_msgs/Text", TEXT_MSGDEF)
        writer.write_message(META_TOPIC, txt_schema, {"data": json.dumps(meta)},
                             log_time=0, publish_time=0)

        for k in range(ticks):
            log_time = k * CONTRACT_TICK_NS
            t = min(t0 + k * CONTRACT_TICK_NS / 1e9, t0 + duration)
            t_next = min(t0 + (k + 1) * CONTRACT_TICK_NS / 1e9, t0 + duration)
            state, action = traj.pose_at(t), traj.pose_at(t_next)
            for side, arm in (("left", left), ("right", right)):
                sl = layout.slice_for(arm)
                s7, a7 = list(state[sl]), list(action[sl])
                for topic, values in (
                    (f"/{side}-arm-state", s7[:ARM_DIM]),
                    (f"/{side}-ee-state", s7[ARM_DIM:]),
                    (f"/{side}-arm-action", a7[:ARM_DIM]),
                    (f"/{side}-ee-action", a7[ARM_DIM:]),
                ):
                    writer.write_message(topic, vec_schema,
                                         {"data": [float(v) for v in values]},
                                         log_time=log_time, publish_time=log_time,
                                         sequence=k)
        writer.finish()

    return ExportReport(path=out_path, ticks=ticks, duration_s=duration,
                        mapping={"left": left, "right": right},
                        bad_spans=sum(1 for _, _, lab in spans if lab == "bad"),
                        warnings=tuple(warnings))
