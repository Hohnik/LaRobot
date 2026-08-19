#!/usr/bin/env python3
"""Tests for the MCAP episode export — the contract's names, dims, tick and honesty.

    uv run tests/test_episode.py

⭐ Every test writes a real MCAP file and READS IT BACK with the decoder, because the export's whole job is what lands in the file — asserting on the writer's inputs would be measuring the wrong instrument (FINDINGS §36.3's rule).
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from mcap.reader import make_reader  # noqa: E402
from mcap_ros2.decoder import DecoderFactory  # noqa: E402

from yam.episode import (  # noqa: E402
    CAMERA_TOPICS,
    CONTRACT_TICK_NS,
    VECTOR_TOPICS,
    export_episode,
    nearest_frame_per_tick,
)
from yam.recording import Trajectory  # noqa: E402


def two_arm_take(seconds: float = 0.5, hz: float = 90.0) -> Trajectory:
    """A synthetic two-arm recording: B ramps up, G ramps down, jaws distinct."""
    t = Trajectory(meta={"arms": ["B", "G"], "joints_per_arm": 7, "method": "sim:test"})
    n = int(seconds * hz)
    for i in range(n + 1):
        s = i / hz
        b = [s * 0.1 * (j + 1) for j in range(6)] + [0.9]
        g = [-s * 0.1 * (j + 1) for j in range(6)] + [0.1]
        t.append(s, b + g)
    return t


def read_back(path: Path):  # noqa: ANN201
    with path.open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return list(reader.iter_decoded_messages())


def export_tmp(traj, **kw):  # noqa: ANN001, ANN201
    d = Path(tempfile.mkdtemp())
    report = export_episode(traj, left=kw.pop("left", "B"), right=kw.pop("right", "G"),
                            out_path=d / "e.mcap", **kw)
    return report, read_back(report.path)


def test_the_contracts_topic_names_and_dims_are_exact() -> None:
    report, msgs = export_tmp(two_arm_take())
    seen = {}
    for schema, channel, message, decoded in msgs:
        seen.setdefault(channel.topic, []).append(decoded)
    for topic in VECTOR_TOPICS:
        assert topic in seen, f"contract topic {topic} is missing"
        dim = 6 if "-arm-" in topic else 1
        assert all(len(m.data) == dim for m in seen[topic]), \
            f"{topic} must carry exactly {dim} values per message"
    assert "/episode-meta" in seen


def test_every_stream_is_synchronous_on_the_exact_tick() -> None:
    report, msgs = export_tmp(two_arm_take())
    times = {}
    for schema, channel, message, decoded in msgs:
        if channel.topic == "/episode-meta":
            continue
        times.setdefault(channel.topic, []).append(message.log_time)
    counts = {len(v) for v in times.values()}
    assert counts == {report.ticks}, f"stream lengths differ: { {k: len(v) for k, v in times.items()} }"
    for topic, ts in times.items():
        gaps = {b - a for a, b in zip(ts, ts[1:])}
        assert gaps == {CONTRACT_TICK_NS}, \
            f"{topic} ticks are not exactly {CONTRACT_TICK_NS} ns: {gaps}"


def test_the_action_is_the_next_ticks_state() -> None:
    _, msgs = export_tmp(two_arm_take())
    per = {}
    for schema, channel, message, decoded in msgs:
        per.setdefault(channel.topic, []).append(list(decoded.data))
    states, actions = per["/left-arm-state"], per["/left-arm-action"]
    for k in range(len(states) - 1):
        assert actions[k] == states[k + 1], \
            f"tick {k}: the action must be the NEXT tick's state (the documented policy)"
    # ⚠️ The last tick rarely lands exactly on the recording's end, so the final action
    # is the clamped END pose — the recording's true final position, not the final
    # tick's state. (This test's first version asserted the wrong invariant and failed
    # honestly against correct code.)
    traj = two_arm_take()
    end_left_arm = list(traj.pose_at(traj.samples[-1].t))[:6]
    assert actions[-1] == end_left_arm, "the last action drives to the recording's final pose"


def test_sides_come_from_the_mapping_never_from_file_order() -> None:
    traj = two_arm_take()
    _, msgs = export_tmp(traj, left="G", right="B")
    for schema, channel, message, decoded in msgs:
        if channel.topic == "/left-ee-state":
            assert abs(decoded.data[0] - 0.1) < 1e-9, \
                "left was mapped to G, whose jaw is 0.1 — the mapping must decide sides"
            break


def test_the_metadata_names_the_mapping_policy_and_labels() -> None:
    traj = two_arm_take()
    traj.mark(0.1, "bad")
    traj.mark(0.3, "good")
    report, msgs = export_tmp(traj, source="slot 7")
    metas = [decoded for schema, channel, message, decoded in msgs
             if channel.topic == "/episode-meta"]
    assert len(metas) == 1
    meta = json.loads(metas[0].data)
    assert meta["mapping"] == {"left": "B", "right": "G"}
    assert "next-tick" in meta["action_policy"]
    assert meta["labels"] and meta["labels"][0]["label"] == "good"
    assert meta["tick_ns"] == CONTRACT_TICK_NS and meta["source"] == "slot 7"
    assert report.bad_spans == 1


def test_a_single_arm_recording_is_refused_with_a_reason() -> None:
    t = Trajectory(meta={"arm": "B"})
    for i in range(10):
        t.append(i / 90.0, [0.0] * 7)
    try:
        export_episode(t, left="B", right="G", out_path=Path(tempfile.mkdtemp()) / "e.mcap")
    except ValueError as e:
        assert "BOTH sides" in str(e)
    else:
        raise AssertionError("a one-arm recording cannot become a two-side episode")


def test_a_wrong_mapping_is_refused() -> None:
    try:
        export_episode(two_arm_take(), left="B", right="Q",
                       out_path=Path(tempfile.mkdtemp()) / "e.mcap")
    except ValueError:
        pass
    else:
        raise AssertionError("a mapping naming an arm the recording lacks must refuse")


def test_the_report_warns_that_no_camera_topics_exist_yet() -> None:
    report, msgs = export_tmp(two_arm_take())
    assert any("camera" in w for w in report.warnings), \
        "an episode without images must say so — silence here poisons a dataset"
    topics = {channel.topic for schema, channel, message, decoded in msgs}
    assert not any("camera" in t for t in topics)


# ---------------------------------------------------------------- item 48: frames ----

def take_with_frames(cameras: tuple[str, ...] = ("c920",), fps: float = 30.0):  # noqa: ANN201
    """A recording + a real frames directory, shaped exactly as the session writes them.

    `mono0_ns` is an arbitrary non-zero epoch on purpose: the join must work off the OFFSET between frame stamps and sample times, and an epoch of zero would let an absolute-time bug pass.
    """
    d = Path(tempfile.mkdtemp())
    traj = two_arm_take()
    mono0 = 5_000_000_000
    per_camera = {}
    for name in cameras:
        cam_dir = d / "frames" / "7" / name
        cam_dir.mkdir(parents=True)
        entries = []
        n = int(traj.duration * fps) + 1
        for i in range(n):
            seq = i + 1
            (cam_dir / f"{seq:06d}.jpg").write_bytes(f"jpeg-{name}-{seq}".encode())
            entries.append([seq, mono0 + int(i / fps * 1e9)])
        (cam_dir / "index.json").write_text(json.dumps(
            {"camera": name, "written": len(entries), "dropped": 0,
             "write_errors": 0, "flushed": True, "entries": entries}))
        per_camera[name] = {"written": len(entries), "dropped": 0, "write_errors": 0}
    traj.meta["cameras"] = {"dir": "frames/7", "mono0_ns": mono0,
                            "per_camera": per_camera}
    return traj, d / "7.json"


def test_camera_topics_ride_the_same_ticks_with_jpeg_payloads() -> None:
    traj, rec_path = take_with_frames(("c920", "d405-255"))
    report, msgs = export_tmp(traj, cameras={"top": "c920", "left-wrist": "d405-255"},
                              recording_path=rec_path)
    per: dict = {}
    for schema, channel, message, decoded in msgs:
        per.setdefault(channel.topic, []).append((message.log_time, decoded))
    for role, name in (("top", "c920"), ("left-wrist", "d405-255")):
        topic = CAMERA_TOPICS[role]
        assert topic in per, f"{topic} must be written"
        assert len(per[topic]) == report.ticks, "one image per tick, C3's synchrony"
        times = [t for t, _ in per[topic]]
        assert times[0] == 0 and times[1] - times[0] == CONTRACT_TICK_NS
        first = per[topic][0][1]
        assert first.format == "jpeg"
        assert bytes(first.data) == f"jpeg-{name}-1".encode(), \
            "tick 0's image is the camera's first frame, byte for byte"
    assert report.cameras == {"top": "c920", "left-wrist": "d405-255"}
    assert any("right-wrist" in w for w in report.warnings), \
        "two cameras cannot fill three contract topics, and the report must say so"


def test_the_nearest_join_is_nearest_and_slow_cameras_repeat() -> None:
    # Frames at 0, 40 and 80 ms; 30 Hz ticks at 0, 33.3 and 66.7 ms → nearest is 1, 2, 3.
    entries = [[1, 0], [2, 40 * 1_000_000], [3, 80 * 1_000_000]]
    assert nearest_frame_per_tick(entries, mono0_ns=0, t0=0.0, ticks=3) == [0, 1, 2]
    # One lone frame serves every tick — a camera slower than the tick repeats, honestly.
    assert nearest_frame_per_tick([[9, 0]], mono0_ns=0, t0=0.0, ticks=3) == [0, 0, 0]


def test_frames_without_a_mapping_refuse_and_a_mapping_without_frames_refuses() -> None:
    traj, rec_path = take_with_frames()
    try:
        export_episode(traj, left="B", right="G",
                       out_path=Path(tempfile.mkdtemp()) / "e.mcap",
                       recording_path=rec_path)
    except ValueError as e:
        assert "roles" in str(e), "dropping recorded images silently is the failure"
    else:
        raise AssertionError("a recording WITH frames must refuse a role-less export")
    try:
        export_episode(two_arm_take(), left="B", right="G",
                       out_path=Path(tempfile.mkdtemp()) / "e.mcap",
                       cameras={"top": "c920"})
    except ValueError as e:
        assert "no frames" in str(e)
    else:
        raise AssertionError("a mapping naming images that do not exist must refuse")


def test_bad_role_names_unmapped_cameras_and_double_mappings_refuse() -> None:
    traj, rec_path = take_with_frames(("c920", "d405-255"))
    for cameras, expect in (
        ({"overhead": "c920", "left-wrist": "d405-255"}, "unknown camera role"),
        ({"top": "c920"}, "recorded but unmapped"),
        ({"top": "c920", "left-wrist": "c920"}, "two roles"),
        ({"top": "c920", "left-wrist": "nope"}, "never recorded"),
    ):
        try:
            export_episode(traj, left="B", right="G",
                           out_path=Path(tempfile.mkdtemp()) / "e.mcap",
                           cameras=cameras, recording_path=rec_path)
        except ValueError as e:
            assert expect in str(e), f"{cameras}: wanted {expect!r} in {e}"
        else:
            raise AssertionError(f"{cameras} must refuse")


def test_a_frames_directory_that_disagrees_with_the_meta_refuses() -> None:
    traj, rec_path = take_with_frames()
    traj.meta["cameras"]["per_camera"]["c920"]["written"] = 999
    try:
        export_episode(traj, left="B", right="G",
                       out_path=Path(tempfile.mkdtemp()) / "e.mcap",
                       cameras={"top": "c920"}, recording_path=rec_path)
    except ValueError as e:
        assert "999" in str(e), "the refusal must show both counts"
    else:
        raise AssertionError("frames that are not the recording's own must never export")


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
