#!/usr/bin/env python3
"""Tests for the C4 training-directory export — the encoding spec, the table, the refusals.

    uv run tests/test_dataset.py

⭐ The encoding-spec test is the important one and it is pure: it asserts that
`ffmpeg_command` still carries every flag [Setup-Anleitung.md](../docs/Setup-Anleitung.md) C4
names. Those flags look like noise and are the difference between a dataset the loader reads
analytically and one it reads ~70× slower, so a well-meaning tidy-up of them must fail here.

⭐ The end-to-end test encodes a REAL video with ffmpeg and reads the table back off disk,
because asserting on what was passed to the writer would measure the wrong instrument
([FINDINGS §36.3](../docs/FINDINGS.md)). ⚠️ It therefore needs `ffmpeg` on PATH: the dataset
feature does not work without it, so this fails loudly rather than skipping quietly.
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

import numpy as np  # noqa: E402

from yam.dataset import (  # noqa: E402
    EPISODE_FPS,
    GOP,
    META_FILE,
    PTS_STEP,
    ROW_WIDTH,
    STATES_FILE,
    TIMEBASE,
    VIDEO_FILE,
    STD_FLOOR,
    export_dataset,
    ffmpeg_command,
    norm_stats,
    stack_views,
)
from yam.episode import ROW_COLUMNS  # noqa: E402
from yam.recording import Trajectory  # noqa: E402


def two_arm_take_with_frames(seconds: float = 0.6, hz: float = 90.0,
                             cameras: tuple[str, ...] = ("c920", "d405-a", "d405-b")):
    """A two-arm recording plus real JPEGs on disk, shaped exactly as the session writes them."""
    import cv2

    d = Path(tempfile.mkdtemp())
    traj = Trajectory(meta={"arms": ["B", "G"], "joints_per_arm": 7, "method": "sim:test"})
    n = int(seconds * hz)
    for i in range(n + 1):
        s = i / hz
        b = [s * 0.1 * (j + 1) for j in range(6)] + [0.9]
        g = [-s * 0.1 * (j + 1) for j in range(6)] + [0.1]
        traj.append(s, b + g)
    mono0 = 7_000_000_000
    per_camera = {}
    for c, name in enumerate(cameras):
        cam_dir = d / "frames" / "5" / name
        cam_dir.mkdir(parents=True)
        entries = []
        for i in range(int(seconds * 30) + 2):
            seq = i + 1
            # A frame that differs per camera AND per tick, so a stacking or ordering
            # mistake shows up as a wrong pixel rather than as nothing at all.
            img = np.zeros((72, 128, 3), dtype=np.uint8)
            img[:, :, c] = 40 + (i * 3) % 200
            cv2.imwrite(str(cam_dir / f"{seq:06d}.jpg"), img)
            entries.append([seq, mono0 + int(i / 30 * 1e9)])
        (cam_dir / "index.json").write_text(json.dumps(
            {"camera": name, "written": len(entries), "dropped": 0, "write_errors": 0,
             "flushed": True, "entries": entries}))
        per_camera[name] = {"written": len(entries), "dropped": 0, "write_errors": 0}
    traj.meta["cameras"] = {"dir": "frames/5", "mono0_ns": mono0, "per_camera": per_camera}
    return traj, d / "5.json"


ROLES = {"top": "c920", "left-wrist": "d405-a", "right-wrist": "d405-b"}


def test_the_encoding_spec_carries_every_flag_the_contract_names() -> None:
    cmd = ffmpeg_command(Path("out.mp4"), 224, 672)
    joined = " ".join(cmd)
    for flag, why in (
        ("-c:v libx264", "the contract names the codec"),
        ("-preset fast", "the contract names the preset"),
        ("-crf 18", "the contract names the quality"),
        ("-pix_fmt yuv420p", "the contract names the pixel format"),
        ("-bf 0", "B-frames break the analytic frame index"),
        (f"-g {GOP}", "the contract fixes the keyframe interval"),
        ("scenecut=0", "scene detection would move keyframes off k%30"),
        ("open_gop=0", "an open GOP makes a keyframe depend on the previous one"),
        (f"-video_track_timescale {TIMEBASE}", "this is what makes PTS 512·k"),
        ("+faststart", "the loader must read the index without seeking"),
    ):
        assert flag in joined, f"missing {flag!r}: {why}"
    assert "-pix_fmt bgr24" in joined, \
        "the INPUT format must say bgr24 — OpenCV hands over BGR, and a swap here would " \
        "produce a colour-inverted dataset that looks fine in a thumbnail"
    assert cmd[0] == "ffmpeg" and cmd[-1] == "out.mp4"


def test_the_pts_arithmetic_is_the_contracts_own() -> None:
    assert TIMEBASE % EPISODE_FPS == 0, "a non-integer PTS step could never land on 512·k"
    assert PTS_STEP == 512 == TIMEBASE // EPISODE_FPS


def test_stacking_preserves_order_and_resizes_each_view() -> None:
    a = np.full((10, 20, 3), 10, dtype=np.uint8)
    b = np.full((30, 40, 3), 200, dtype=np.uint8)
    out = stack_views([a, b], view_width=8, view_height=8)
    assert out.shape == (16, 8, 3), f"two 8x8 views stack to 8 wide x 16 high, got {out.shape}"
    assert out[:8].mean() < out[8:].mean(), "the FIRST view must land on top, not the second"
    try:
        stack_views([])
    except ValueError:
        pass
    else:
        raise AssertionError("stacking nothing must refuse")


def test_a_full_export_reads_back_exactly() -> None:
    assert shutil.which("ffmpeg"), \
        "ffmpeg is not on PATH, and the dataset export cannot work without it " \
        "(macOS: brew install ffmpeg · Ubuntu: sudo apt install ffmpeg)"
    traj, rec_path = two_arm_take_with_frames()
    out_root = Path(tempfile.mkdtemp())
    report = export_dataset(traj, left="G", right="B", cameras=ROLES,
                            recording_path=rec_path, out_root=out_root,
                            view_width=32, view_height=32, episode_id="unit")
    assert report.path.name == "episode_unit"
    assert report.roles == ("top", "left-wrist", "right-wrist"), \
        "the stack order must be the contract's role order, not dict insertion order"
    assert report.frame_size == (32, 96), "three 32px views stack to 96 high"

    raw = (report.path / STATES_FILE).read_bytes()
    assert len(raw) == report.steps * ROW_WIDTH * 8
    table = np.frombuffer(raw, dtype=np.float64).reshape(report.steps, ROW_WIDTH)
    assert np.isfinite(table).all()
    # left=G, and G's jaw is 0.1 while B's is 0.9 — so column 6 (left ee) proves the
    # MAPPING decided the sides, not the recording's own arm order.
    assert abs(table[0][6] - 0.1) < 1e-9, \
        f"left was mapped to G (jaw 0.1); column 6 holds {table[0][6]}"
    assert abs(table[0][13] - 0.9) < 1e-9, "column 13 is the right side's jaw (B, 0.9)"
    # The action half is the next tick's state, in the bytes themselves.
    assert np.allclose(table[:-2, 14:], table[1:-1, :14])

    meta = json.loads((report.path / META_FILE).read_text())
    assert meta["num_steps"] == report.steps
    assert meta["states_actions"]["columns"] == list(ROW_COLUMNS)
    assert meta["video"]["views"] == list(report.roles)
    assert meta["verified_against_abc_loader"] is False, \
        "the episode must never claim a verification that has not happened"
    assert (report.path / VIDEO_FILE).stat().st_size > 0


def test_every_dataset_poisoning_case_refuses() -> None:
    traj, rec_path = two_arm_take_with_frames()
    out = Path(tempfile.mkdtemp())
    cases = [
        ({}, "roles", "no roles named at all"),
        ({"overhead": "c920"}, "unknown camera role", "a role the contract does not have"),
        ({"top": "nope"}, "does not carry", "a role naming an unrecorded camera"),
    ]
    for cameras, expect, why in cases:
        try:
            export_dataset(traj, left="G", right="B", cameras=cameras,
                           recording_path=rec_path, out_root=out, episode_id="x")
        except ValueError as e:
            assert expect in str(e), f"{why}: wanted {expect!r} in {e}"
        else:
            raise AssertionError(f"{why} must refuse")

    # A frameless recording cannot become a training episode: there is no video to write.
    bare = Trajectory(meta={"arms": ["B", "G"], "joints_per_arm": 7})
    for i in range(10):
        bare.append(i / 90, [0.0] * 14)
    try:
        export_dataset(bare, left="G", right="B", cameras={"top": "c920"},
                       recording_path=rec_path, out_root=out, episode_id="y")
    except ValueError as e:
        assert "no camera frames" in str(e)
    else:
        raise AssertionError("a frameless recording must refuse")

    # One arm cannot fill two sides, and inventing the missing one would mirror a dataset.
    one = Trajectory(meta={"arms": ["B"], "joints_per_arm": 7,
                           "cameras": {"dir": "frames/5", "mono0_ns": 0,
                                       "per_camera": {"c920": {"written": 1}}}})
    for i in range(10):
        one.append(i / 90, [0.0] * 7)
    try:
        export_dataset(one, left="G", right="B", cameras={"top": "c920"},
                       recording_path=rec_path, out_root=out, episode_id="z")
    except ValueError as e:
        assert "BOTH" in str(e) or "cannot be exported" in str(e)
    else:
        raise AssertionError("a one-arm recording must refuse a two-side export")


def test_a_missing_view_warns_rather_than_pretending() -> None:
    assert shutil.which("ffmpeg"), "ffmpeg is required for the dataset export"
    traj, rec_path = two_arm_take_with_frames(cameras=("c920",))
    report = export_dataset(traj, left="G", right="B", cameras={"top": "c920"},
                            recording_path=rec_path, out_root=Path(tempfile.mkdtemp()),
                            view_width=32, view_height=32, episode_id="onlytop")
    assert report.frame_size == (32, 32) and report.roles == ("top",)
    assert any("3 views" in w for w in report.warnings), \
        "a 1-view stack where the contract wants 3 must say so — a loader expecting three " \
        "would read stacked rows that are not there"


def test_the_normalisation_statistics_normalise_their_own_data() -> None:
    """⭐ The consequence, not the intention: normalising the input with the produced stats
    must give zero mean and unit variance. A subtly wrong stats file trains a model badly
    and raises nothing (guide C5)."""
    rng = np.random.default_rng(7)
    rows = rng.normal(3.0, 2.0, size=(500, ROW_WIDTH))
    rows[:, 4] = 1.234                      # a column that never moved
    stats = norm_stats(rows)
    mean, std = np.asarray(stats["mean"]), np.asarray(stats["std"])
    live = [i for i in range(ROW_WIDTH) if i != 4]
    normalised = (rows - mean) / std
    assert abs(normalised[:, live].mean()) < 1e-12
    assert abs(normalised[:, live].std(axis=0) - 1.0).max() < 1e-9
    assert stats["count"] == 500 and stats["columns"] == list(ROW_COLUMNS)


def test_a_motionless_column_is_floored_and_NAMED() -> None:
    rows = np.zeros((20, ROW_WIDTH))
    rows[:, 0] = np.arange(20)              # only one column carries any signal
    stats = norm_stats(rows)
    assert stats["std"][1] == STD_FLOOR, "a zero-variance column must be floored, not zero"
    assert stats["std"][0] > STD_FLOOR
    assert len(stats["floored_columns"]) == ROW_WIDTH - 1
    assert ROW_COLUMNS[1] in stats["floored_columns"], \
        "a floored column must be NAMED — silently amplifying noise on a dead column is the " \
        "kind of plausible-looking dataset this repo exists to catch"
    # Dividing by the floor must never produce an infinity, which is the whole point of it.
    assert np.isfinite((rows - np.asarray(stats["mean"])) / np.asarray(stats["std"])).all()


def test_statistics_over_nothing_or_the_wrong_width_refuse() -> None:
    for rows, why in ((np.zeros((0, ROW_WIDTH)), "no rows"),
                      (np.zeros((5, 3)), "the wrong column count")):
        try:
            norm_stats(rows)
        except ValueError:
            pass
        else:
            raise AssertionError(f"statistics over {why} must refuse")


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
