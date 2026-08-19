"""Turn a recording into the training-directory format, with timecodes that line up exactly.

⭐⭐ WHAT THIS IS AND WHY IT IS SEPARATE FROM `yam/episode.py`. The team's guide specifies two things, and this repo now writes both. C3 is the LOG: one MCAP file with the eight state/action topics and the camera topics (`yam/episode.py`). **C4 is the TRAINING SET**: a directory per episode holding a flat numeric table, one video with the camera views stacked, and a metadata file ([Setup-Anleitung.md](../../docs/Setup-Anleitung.md) C4):

    episode_<id>/
      states_actions.bin              (num_steps, 28) float64 = 14 state + 14 action
      combined_camera-images-rgb.mp4  the views stacked vertically, 30 fps
      episode_metadata.json

⛔⭐⭐ THE ENCODING IS THE WHOLE POINT, AND IT IS STRICT FOR ONE REASON: **the trainer does not decode the video to find frame k, it computes where frame k is.** So the file must make that arithmetic true — constant 30 fps, timebase 1/15360, PTS exactly 512·k, a keyframe every 30 frames with no scene-cut detection moving them, and NO B-frames (which reorder presentation against decode order and break the mapping). The guide's own warning: a wrongly encoded file makes the data loader about **70× slower**, and it says so because someone measured it. `FFMPEG_VIDEO_ARGS` below is that spec expressed once, and `checks/check_dataset.py` reads the finished file back with `ffprobe` and checks every one of those properties — so this repo can clear its own half of the C4 gate instead of waiting.

⚠️ WHAT REMAINS THE TEAM'S HALF, stated plainly. The guide says *"best advice: do not encode it yourself, ABC's `export_mcap.py` does it right"* — and that file is not in this repo. So what is verifiable here is that the output matches **the published spec**, byte-level property by property. What only ABC's loader can say is whether the spec as published is complete. That is the C4 gate, and it stays open ([FINDINGS §74.1](../../docs/FINDINGS.md)).

⚠️ ONE AMBIGUITY, NAMED RATHER THAN GUESSED. The guide writes the stacked video as "3 views vertically stacked, 224×224", which reads as each view 224×224 (so 224 wide × 672 high for three). But the team's own simulation renders its cameras at 224×168. Both cannot be right, and a wrong choice here is a silently mis-shaped dataset. So: 224×224 per view is the DEFAULT because the guide says so, `--view-height` overrides it, and the number actually used is written into every episode's metadata where a loader mismatch will point straight at it.
"""

from __future__ import annotations

import json
import subprocess
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

from yam.episode import (
    CAMERA_ROLES,
    CONTRACT_TICK_NS,
    ROW_COLUMNS,
    load_frame_index,
    nearest_frame_per_tick,
    state_action_rows,
)
from yam.recording import Layout, Trajectory

__all__ = ["EPISODE_FPS", "TIMEBASE", "PTS_STEP", "GOP", "VIEW_SIZE", "ROW_WIDTH",
           "STATES_FILE", "VIDEO_FILE", "META_FILE", "FFMPEG_VIDEO_ARGS",
           "ffmpeg_command", "stack_views", "DatasetReport", "export_dataset"]

#: The contract's frame rate. Identical to C3's 33,333,333 ns tick by construction.
EPISODE_FPS = 30
#: Timebase denominator from the guide: PTS is counted in 1/15360 of a second.
TIMEBASE = 15_360
#: 15360 / 30. Frame k must land on PTS = 512·k for the analytic index to be right.
PTS_STEP = TIMEBASE // EPISODE_FPS
#: Keyframe interval, from the guide. With scene-cut detection off, keyframes land on k%30==0.
GOP = 30
#: Per-view square size. See the ambiguity note in the module docstring.
VIEW_SIZE = 224
#: 14 state + 14 action.
ROW_WIDTH = len(ROW_COLUMNS)

STATES_FILE = "states_actions.bin"
VIDEO_FILE = "combined_camera-images-rgb.mp4"
META_FILE = "episode_metadata.json"

#: ⛔⭐ THE ENCODING SPEC, one list, quoted from Setup-Anleitung C4. Kept as data rather than
#: buried in a string so a test can assert every flag is present and `check_dataset.py` can
#: verify the RESULT of each one. Both belts are worn on purpose: the modern `-x264-params`
#: form and the older top-level flags say the same thing, and libx264 honours whichever the
#: local ffmpeg build understands — an encoder that silently ignored one would otherwise
#: produce a plausible file with the wrong GOP, which is this stack's favourite kind of bug.
FFMPEG_VIDEO_ARGS = [
    "-an",                                  # no audio track at all
    "-c:v", "libx264",
    "-preset", "fast",
    "-crf", "18",
    "-pix_fmt", "yuv420p",
    "-bf", "0",                             # ⛔ no B-frames: they reorder PTS vs decode order
    "-g", str(GOP),                         # keyframe every 30 frames
    "-keyint_min", str(GOP),                # ...and never sooner
    "-x264-params", f"keyint={GOP}:min-keyint={GOP}:scenecut=0:open_gop=0:bframes=0",
    "-video_track_timescale", str(TIMEBASE),  # 1/15360 → PTS 512·k at 30 fps
    "-movflags", "+faststart",              # moov atom first, so the loader can seek at once
]


def ffmpeg_command(out_path: Path, width: int, height: int,
                   fps: int = EPISODE_FPS) -> list[str]:
    """The exact ffmpeg argv for a contract-shaped video, fed raw BGR frames on stdin.

    Pure, so the encoding spec is testable without encoding anything — and the test that
    asserts these flags is the one that would catch a well-meaning "cleanup" of them.

    Raw frames rather than a directory of PNGs: a 10-second three-view episode is ~300
    frames, and writing then re-reading them costs disk and time for no gain. OpenCV hands
    over BGR, and `-pix_fmt bgr24` on the INPUT tells ffmpeg exactly that, so no channel
    swap happens anywhere in this pipeline (getting that wrong yields a colour-swapped
    dataset that trains, badly, and looks fine in a thumbnail).
    """
    return [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height}",
        "-framerate", str(fps), "-i", "-",
        *FFMPEG_VIDEO_ARGS,
        str(out_path),
    ]


def stack_views(views: Sequence[Any], view_width: int = VIEW_SIZE,
                view_height: int = VIEW_SIZE) -> Any:
    """The camera views resized and stacked vertically into one frame.

    ⛔ The ORDER is the caller's and must be the contract's role order (top, left-wrist,
    right-wrist). A stacked video whose views are in a different order than the metadata
    claims trains a policy to look in the wrong place, and nothing raises.
    """
    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415

    if not views:
        raise ValueError("no views to stack")
    resized = [cv2.resize(v, (view_width, view_height), interpolation=cv2.INTER_AREA)
               for v in views]
    return np.vstack(resized)


@dataclass(frozen=True)
class DatasetReport:
    """What one exported episode directory contains."""

    path: Path
    episode_id: str
    steps: int
    duration_s: float
    view_size: tuple[int, int]        # (width, height) per view
    frame_size: tuple[int, int]       # (width, height) of the stacked video
    roles: tuple[str, ...]            # the roles actually written, in stack order
    mapping: dict[str, str]
    warnings: tuple[str, ...] = field(default=())


def export_dataset(traj: Trajectory, left: str, right: str, cameras: dict[str, str],
                   recording_path: Path, out_root: Path,
                   view_width: int = VIEW_SIZE, view_height: int = VIEW_SIZE,
                   episode_id: str | None = None, source: str = "?") -> DatasetReport:
    """Write one `episode_<id>/` directory in the C4 training shape. Refuses rather than guessing.

    ⛔ Every refusal here is a dataset-poisoning case: no frames at all (a training set needs
    images), a role naming a camera the recording does not carry, and a recording with fewer
    than two samples (no duration, so no tick grid). The arm sides and the camera roles are
    required for the same reason they are in the MCAP export — they are physical facts about
    the bench that no file can derive ([FINDINGS §70.13](../../docs/FINDINGS.md)).
    """
    import cv2  # noqa: PLC0415
    import numpy as np  # noqa: PLC0415

    if len(traj.samples) < 2:
        raise ValueError("an episode needs at least two samples to have a duration")
    recorded = traj.meta.get("cameras") or {}
    recorded_names = set(recorded.get("per_camera", {}))
    if not recorded_names:
        raise ValueError(
            "this recording carries no camera frames, and a C4 training episode is a VIDEO "
            "plus a table — there is nothing to encode. Record with --cameras, or export "
            "the joints-only MCAP with apps/export_episode.py instead.")
    if not cameras:
        raise ValueError(f"name the camera roles: this recording carries "
                         f"{sorted(recorded_names)} and the contract wants "
                         f"{list(CAMERA_ROLES)}.")
    bad_roles = set(cameras) - set(CAMERA_ROLES)
    if bad_roles:
        raise ValueError(f"unknown camera role(s) {sorted(bad_roles)} — the contract's roles "
                         f"are {list(CAMERA_ROLES)}.")
    unknown = set(cameras.values()) - recorded_names
    if unknown:
        raise ValueError(f"role(s) name camera(s) {sorted(unknown)} that this recording does "
                         f"not carry; it has {sorted(recorded_names)}.")

    layout = Layout.from_meta(traj.meta, traj.n_joints)
    if left not in layout.arms or right not in layout.arms:
        raise ValueError(f"this recording holds arms {list(layout.arms)}, so it cannot be "
                         f"exported with left={left} right={right}: a C4 episode needs BOTH "
                         "sides, and a missing side cannot be invented.")
    t0 = traj.samples[0].t
    duration = traj.duration
    steps = int(duration * 1e9 // CONTRACT_TICK_NS) + 1
    rows = state_action_rows(traj, layout, left, right, steps, t0, duration)

    #: The stack order is the contract's role order, and only roles that were recorded.
    roles = tuple(r for r in CAMERA_ROLES if r in cameras)
    warnings: list[str] = []
    if len(roles) != len(CAMERA_ROLES):
        missing = [r for r in CAMERA_ROLES if r not in cameras]
        warnings.append(
            f"only {len(roles)} of the contract's 3 views were recorded, so the stacked video "
            f"is {len(roles)} high instead of 3 and {missing} are absent. A loader expecting "
            "three views will need its camera_keys changed to match.")

    mono0 = int(recorded.get("mono0_ns", 0))
    joined: dict[str, tuple[Path, list[list[int]], list[int]]] = {}
    for role in roles:
        cam_dir, index = load_frame_index(recording_path, recorded, cameras[role])
        picks = nearest_frame_per_tick(index["entries"], mono0, t0, steps)
        joined[role] = (cam_dir, index["entries"], picks)
        counts = recorded.get("per_camera", {}).get(cameras[role], {})
        if counts.get("dropped") or counts.get("write_errors"):
            warnings.append(
                f"camera {cameras[role]}: {counts.get('dropped', 0)} dropped and "
                f"{counts.get('write_errors', 0)} failed frame(s) during recording — those "
                "instants are served by their nearest neighbour, so the video repeats there.")

    episode_id = episode_id or uuid.uuid4().hex
    out_dir = Path(out_root) / f"episode_{episode_id}"
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- the numeric table: (steps, 28) float64, C-contiguous, little-endian native ----
    table = np.asarray(rows, dtype=np.float64)
    if table.shape != (steps, ROW_WIDTH):
        raise ValueError(f"internal: built {table.shape}, contract wants ({steps}, {ROW_WIDTH})")
    (out_dir / STATES_FILE).write_bytes(table.tobytes(order="C"))

    # ---- the stacked video, encoded to the contract's exact properties ----
    frame_size = (view_width, view_height * len(roles))
    cmd = ffmpeg_command(out_dir / VIDEO_FILE, *frame_size)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    cache: dict[str, tuple[int, Any]] = {}
    try:
        for k in range(steps):
            views = []
            for role in roles:
                cam_dir, entries, picks = joined[role]
                j = picks[k]
                got = cache.get(role)
                if got is None or got[0] != j:
                    seq = entries[j][0]
                    jpg = cam_dir / f"{seq:06d}.jpg"
                    img = cv2.imread(str(jpg))
                    if img is None:
                        raise ValueError(f"camera {cameras[role]!r}: frame {jpg.name} is "
                                         "missing or unreadable — the frames directory is "
                                         "incomplete, refusing to encode a gap.")
                    cache[role] = (j, img)
                    got = cache[role]
                views.append(got[1])
            proc.stdin.write(stack_views(views, view_width, view_height).tobytes())
        proc.stdin.close()
    except BrokenPipeError as exc:
        # ⚠️ NOT communicate() here: stdin is (or is being) closed, and communicate would try
        # to flush it and raise ValueError over the top of the real ffmpeg error. Read the
        # pipes directly instead — the encoder's own words are what diagnoses this.
        err = proc.stderr.read() or b""
        proc.wait()
        raise ValueError(f"ffmpeg stopped early: {err.decode(errors='replace')[:400]}") from exc
    except Exception:
        proc.kill()
        proc.wait()
        raise
    err = proc.stderr.read() or b""
    if proc.wait() != 0:
        raise ValueError(f"ffmpeg failed ({proc.returncode}): "
                         f"{err.decode(errors='replace')[:400]}")

    # ---- the metadata: everything a loader or a human needs to check the other two ----
    meta = {
        "episode_id": episode_id,
        "contract": "Setup-Anleitung C4",
        "fps": EPISODE_FPS,
        "tick_ns": CONTRACT_TICK_NS,
        "num_steps": steps,
        "duration_s": duration,
        "states_actions": {
            "file": STATES_FILE, "dtype": "float64", "shape": [steps, ROW_WIDTH],
            "order": "C", "columns": list(ROW_COLUMNS),
            "action_policy": "next-tick measured state (hand-taught demo; GUIDE has no "
                             "command stream, the hand is the controller)",
            "units": "arm joints in radians; ee (jaw) NORMALISED 0..1 — ⛔ the team's own "
                     "Observation carries the gripper in METRES, and C3 gives the ee "
                     "dimension without a unit, so this is the open question of "
                     "docs/PLAN.md §4",
        },
        "video": {
            "file": VIDEO_FILE, "views": list(roles), "stack": "vertical",
            "view_width": view_width, "view_height": view_height,
            "frame_width": frame_size[0], "frame_height": frame_size[1],
            "timebase": f"1/{TIMEBASE}", "pts_step": PTS_STEP, "gop": GOP,
            "encoder_args": FFMPEG_VIDEO_ARGS,
            "note": "each view resized independently, then stacked in the order above; "
                    "frame k of this file is state/action row k of " + STATES_FILE,
        },
        "cameras": {"roles": dict(cameras),
                    "join": "nearest recorded frame per tick, on the recording's own clock"},
        "mapping": {"left": left, "right": right},
        "labels": [{"start": s, "end": e, "label": lab} for s, e, lab in traj.label_spans()],
        "source": source,
        "recording_meta": dict(traj.meta),
        "verified_against_abc_loader": False,
        "verification_note": "This directory matches the PUBLISHED C4 spec, property by "
                             "property (checks/check_dataset.py re-reads it with ffprobe). "
                             "It has NOT been through ABC's own loader — that is the C4 "
                             "gate and it is still open.",
    }
    (out_dir / META_FILE).write_text(json.dumps(meta, indent=2) + "\n")

    return DatasetReport(path=out_dir, episode_id=episode_id, steps=steps,
                         duration_s=duration, view_size=(view_width, view_height),
                         frame_size=frame_size, roles=roles,
                         mapping={"left": left, "right": right}, warnings=tuple(warnings))
