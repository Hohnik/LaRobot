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

#: C3's three camera topics, by ROLE. ⛔ A role is a physical bench position, exactly
#: like the arm sides: nothing in a recording can derive which camera looked from the
#: top, so the role→camera mapping is REQUIRED input whenever frames were recorded,
#: and it is written into the episode's own metadata for auditing.
CAMERA_ROLES = ("top", "left-wrist", "right-wrist")
CAMERA_TOPICS = {role: f"/{role}-camera" for role in CAMERA_ROLES}

VECTOR_MSGDEF = "float64[] data"
TEXT_MSGDEF = "string data"
#: ⚠️ Same honesty caveat as the module docstring: the Anleitung names the camera
#: TOPICS, not their message encoding — that lives in ABC's own `export_mcap.py`. Until
#: the C4 gate has run, this minimal jpeg-bytes message is "the contract as written".
IMAGE_MSGDEF = "string format\nuint8[] data"


@dataclass(frozen=True)
class ExportReport:
    """What was written, for the caller to narrate and the tests to assert on."""

    path: Path
    ticks: int
    duration_s: float
    mapping: dict[str, str]          # {"left": "B", "right": "G"}
    bad_spans: int
    warnings: tuple[str, ...] = field(default=())
    cameras: dict[str, str] = field(default_factory=dict)   # {"top": "c920", ...}


def nearest_frame_per_tick(entries: list[list[int]], mono0_ns: int, t0: float,
                           ticks: int, tick_ns: int = CONTRACT_TICK_NS) -> list[int]:
    """For every tick, the index into `entries` of the nearest frame by timestamp.

    Pure, so the join is testable without a single image on disk. `entries` is a camera's index list of `[seq, host_stamp_ns]` in write order; `mono0_ns` is the monotonic stamp the recorder took at the `w` keypress, which is the same instant as sample time zero — so `(host_ns - mono0_ns)/1e9` and `Sample.t` live on ONE axis (both clocks are `mach_absolute_time()` on this Mac, verified 2026-08-19). Tick k sits at `t0 + k*tick` on that axis, and the nearest frame wins; a camera slower than 30 Hz simply serves one frame to several ticks, which is the honest join (the alternative — refusing gaps — would make every dropped frame kill an export).
    """
    if not entries:
        raise ValueError("a camera with no frames cannot be joined to ticks")
    rel = [(host_ns - mono0_ns) / 1e9 for _, host_ns in entries]
    out: list[int] = []
    j = 0
    for k in range(ticks):
        want = t0 + k * tick_ns / 1e9
        while j + 1 < len(rel) and abs(rel[j + 1] - want) <= abs(rel[j] - want):
            j += 1
        out.append(j)
    return out


#: ⭐⭐ THE COLUMN LAYOUT OF ONE TRAINING ROW, and it is the C3 topic order flattened
#: ([Setup-Anleitung.md](../../docs/Setup-Anleitung.md) C4: "(num_steps, 28) float64 = 14
#: state + 14 action"). Written into every episode's metadata as well, because a dataset
#: whose column order is only known to the code that wrote it cannot be checked by anyone
#: else — and a silently transposed pair of arms is exactly the poison C4 exists to catch.
ROW_COLUMNS = tuple(
    [f"state.{side}.{part}" for side in ("left", "right")
     for part in [f"arm{i}" for i in range(ARM_DIM)] + ["ee"]]
    + [f"action.{side}.{part}" for side in ("left", "right")
       for part in [f"arm{i}" for i in range(ARM_DIM)] + ["ee"]]
)


def state_action_rows(traj: Trajectory, layout: Any, left: str, right: str,
                      ticks: int, t0: float, duration: float) -> list[list[float]]:
    """One `(ticks, 28)` table of states and actions, on the contract's exact tick grid.

    ⭐⭐ ONE FUNCTION, BOTH EXPORTERS. The MCAP writer (C3) and the training-directory writer
    (C4, `yam/dataset.py`) both call this, so the two outputs cannot drift apart — and if the
    column order or the action policy is ever wrong, it is wrong once, in one place, for both.
    That matters more than it sounds: the C3 file and the C4 directory describe the same
    demonstration, and a dataset where they disagree is worse than either alone.

    The action at tick k is the state at tick k+1 — the standard demonstrated-action
    construction, and the only honest one for a hand-taught demo, where GUIDE has no command
    stream because the hand IS the controller ([FINDINGS §70.13](../../docs/FINDINGS.md)).
    Both clocks are clamped to the recording's end, so the final action drives to the true
    final pose rather than past it.
    """
    rows: list[list[float]] = []
    for k in range(ticks):
        t = min(t0 + k * CONTRACT_TICK_NS / 1e9, t0 + duration)
        t_next = min(t0 + (k + 1) * CONTRACT_TICK_NS / 1e9, t0 + duration)
        state, action = traj.pose_at(t), traj.pose_at(t_next)
        row: list[float] = []
        for values in (state, action):
            for arm in (left, right):
                row.extend(float(v) for v in values[layout.slice_for(arm)])
        rows.append(row)
    return rows


def load_frame_index(recording_path: Path, meta_cameras: dict[str, Any],
                     name: str) -> tuple[Path, dict[str, Any]]:
    """One camera's frames directory and its `index.json`, verified against the meta.

    ⛔ Refuses on a count mismatch rather than exporting what happens to be there: the index is written at flush time and the meta at save time from the same numbers, so a disagreement means the frames directory was moved, half-copied or edited — and an episode built from it would carry images that do not belong to its joints.
    """
    from yam.cameras.specs import camera_dir_name  # noqa: PLC0415 — avoids a cycle at import time

    cam_dir = recording_path.parent / meta_cameras["dir"] / camera_dir_name(name)
    index_path = cam_dir / "index.json"
    if not index_path.is_file():
        raise ValueError(f"camera {name!r}: no index at {index_path} — the frames "
                         "directory named in the recording's meta is gone or incomplete.")
    index = json.loads(index_path.read_text())
    expected = meta_cameras.get("per_camera", {}).get(name, {}).get("written")
    if expected is not None and expected != index.get("written"):
        raise ValueError(
            f"camera {name!r}: the recording says {expected} frames were written and the "
            f"index says {index.get('written')} — the frames on disk are not the ones "
            "this recording made. Refusing rather than exporting someone else's images."
        )
    return cam_dir, index


def export_episode(traj: Trajectory, left: str, right: str, out_path: Path,
                   source: str = "?", cameras: dict[str, str] | None = None,
                   recording_path: Path | None = None) -> ExportReport:
    """Write one episode. Refuses loudly rather than mislabelling training data.

    ⛔ `left` and `right` are REQUIRED and never defaulted: the contract's sides are
    physical positions on the bench, nothing in a recording can derive them, and a
    silently wrong side is the worst kind of dataset poison — every episode would be
    mirrored and nothing would raise. The mapping is written into the episode's own
    metadata so it can be audited later.

    ⛔ `cameras` maps a C3 ROLE to a recorded camera name (`{"top": "c920", ...}`) and is
    required exactly when the recording carries frames, for the same reason the sides
    are: a role is a physical bench position. A recording WITH frames exported without a
    mapping would silently produce an image-less episode that looks complete; a mapping
    against a recording WITHOUT frames names images that do not exist. Both refuse.
    `recording_path` locates the frames directory the recording's meta names.
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

    recorded = traj.meta.get("cameras") or {}
    recorded_names = set(recorded.get("per_camera", {}))
    if recorded_names and not cameras:
        raise ValueError(
            f"this recording carries frames from {sorted(recorded_names)} and no camera "
            "roles were given — exporting would silently drop the images and the episode "
            "would look complete without them. Name the roles: --top / --left-wrist / "
            "--right-wrist."
        )
    if cameras and not recorded_names:
        raise ValueError("camera roles were given but this recording carries no frames — "
                         "the mapping names images that do not exist.")
    if cameras:
        bad_roles = set(cameras) - set(CAMERA_ROLES)
        if bad_roles:
            raise ValueError(f"unknown camera role(s) {sorted(bad_roles)} — the contract's "
                             f"roles are {list(CAMERA_ROLES)}.")
        mapped = list(cameras.values())
        if sorted(set(mapped)) != sorted(mapped):
            raise ValueError(f"one camera is mapped to two roles: {cameras} — every "
                             "recorded camera serves exactly one bench position.")
        unmapped = recorded_names - set(mapped)
        unknown = set(mapped) - recorded_names
        if unmapped or unknown:
            raise ValueError(
                f"the mapping and the recording disagree: recorded cameras are "
                f"{sorted(recorded_names)}, the mapping names {sorted(mapped)}"
                + (f" — {sorted(unmapped)} recorded but unmapped" if unmapped else "")
                + (f" — {sorted(unknown)} mapped but never recorded" if unknown else "")
                + ". Every recorded camera must serve exactly one role."
            )
        if recording_path is None:
            raise ValueError("cameras were mapped but recording_path was not given, so "
                             "the frames directory cannot be located.")

    t0 = traj.samples[0].t
    duration = traj.duration
    ticks = int(duration * 1e9 // CONTRACT_TICK_NS) + 1
    warnings: list[str] = []
    if not recorded_names:
        warnings.append(
            "no camera topics were written: this recording carries no frames (record "
            "with --cameras in the session). An episode without images cannot train "
            "ABC — this export proves the joint/action side of the contract."
        )
    else:
        missing_roles = [r for r in CAMERA_ROLES if r not in (cameras or {})]
        if missing_roles:
            warnings.append(
                f"the contract wants all three camera topics and only "
                f"{len(cameras or {})} camera(s) were recorded — "
                f"{[CAMERA_TOPICS[r] for r in missing_roles]} not written."
            )
        for name, counts in recorded.get("per_camera", {}).items():
            if counts.get("dropped", 0) or counts.get("write_errors", 0):
                warnings.append(
                    f"camera {name}: {counts.get('dropped', 0)} frame(s) dropped and "
                    f"{counts.get('write_errors', 0)} write error(s) during recording — "
                    "those instants have no image and the nearest neighbour serves them."
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
    if cameras:
        meta["cameras"] = {
            "roles": dict(cameras),
            "encoding": "jpeg in a minimal yam_msgs/CompressedImage — the contract as "
                        "written; ABC's own encoding is C4's question",
            "join": "nearest recorded frame per tick, on the recording's own clock",
        }

    # Read every mapped camera's index BEFORE the file opens, so a refusal cannot leave a half-written episode behind.
    joined: dict[str, tuple[Path, list[list[int]], list[int]]] = {}
    if cameras:
        mono0 = int(recorded.get("mono0_ns", 0))
        for role, name in cameras.items():
            cam_dir, index = load_frame_index(recording_path, recorded, name)
            picks = nearest_frame_per_tick(index["entries"], mono0, t0, ticks)
            joined[role] = (cam_dir, index["entries"], picks)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        writer = Ros2Writer(f)
        vec_schema = writer.register_msgdef("yam_msgs/Vector", VECTOR_MSGDEF)
        txt_schema = writer.register_msgdef("yam_msgs/Text", TEXT_MSGDEF)
        img_schema = (writer.register_msgdef("yam_msgs/CompressedImage", IMAGE_MSGDEF)
                      if joined else None)
        writer.write_message(META_TOPIC, txt_schema, {"data": json.dumps(meta)},
                             log_time=0, publish_time=0)

        # One image per tick per mapped role, C3's "alle Streams synchron": the nearest recorded frame serves each tick, so every camera stream has exactly `ticks` messages on exactly the vector streams' log_times. A frame slower than the tick serves several ticks and is READ once (the one-slot cache), not re-encoded.
        for role, (cam_dir, entries, picks) in joined.items():
            cached: tuple[int, bytes] | None = None
            for k, j in enumerate(picks):
                if cached is None or cached[0] != j:
                    seq = entries[j][0]
                    jpg = cam_dir / f"{seq:06d}.jpg"
                    if not jpg.is_file():
                        raise ValueError(f"camera {cameras[role]!r}: the index names frame "
                                         f"{seq} and {jpg.name} is not on disk — the frames "
                                         "directory is incomplete.")
                    cached = (j, jpg.read_bytes())
                writer.write_message(CAMERA_TOPICS[role], img_schema,
                                     {"format": "jpeg", "data": cached[1]},
                                     log_time=k * CONTRACT_TICK_NS,
                                     publish_time=k * CONTRACT_TICK_NS, sequence=k)

        # ⭐ The rows come from `state_action_rows`, the same function the C4 training export
        # uses, so the MCAP file and the training directory can never describe the demo
        # differently. Each 28-wide row unflattens back into the eight contract topics.
        rows = state_action_rows(traj, layout, left, right, ticks, t0, duration)
        for k, row in enumerate(rows):
            log_time = k * CONTRACT_TICK_NS
            for offset, kind in ((0, "state"), (14, "action")):
                for i, side in enumerate(("left", "right")):
                    seven = row[offset + i * 7:offset + i * 7 + 7]
                    for topic, values in (
                        (f"/{side}-arm-{kind}", seven[:ARM_DIM]),
                        (f"/{side}-ee-{kind}", seven[ARM_DIM:]),
                    ):
                        writer.write_message(topic, vec_schema,
                                             {"data": [float(v) for v in values]},
                                             log_time=log_time, publish_time=log_time,
                                             sequence=k)
        writer.finish()

    return ExportReport(path=out_path, ticks=ticks, duration_s=duration,
                        mapping={"left": left, "right": right},
                        bad_spans=sum(1 for _, _, lab in spans if lab == "bad"),
                        warnings=tuple(warnings), cameras=dict(cameras or {}))
