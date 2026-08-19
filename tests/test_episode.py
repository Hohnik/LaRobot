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

from yam.episode import CONTRACT_TICK_NS, VECTOR_TOPICS, export_episode  # noqa: E402
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
