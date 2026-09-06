import json

import numpy as np
from mcap.reader import make_reader
from io import BytesIO

from robot.recording.recorder import Recorder
from robot.recording.sample import Sample
from robot.cameras.frame import Frame


def test_recording_preserves_samples(tmp_path):
    state = np.arange(14, dtype=np.float32)
    action = state + np.float32(0.25)

    with Recorder(tmp_path) as recorder:
        for timestamp in (1.0, 2.0):
            recorder.record(
                Sample(
                    timestamp_s=timestamp,
                    frames=(),
                    state=state,
                    action=action,
                )
            )

    with (tmp_path / "episode.mcap").open("rb") as file:
        messages = list(make_reader(file).iter_messages())

    assert len(messages) == 2

    for index, (_, channel, message) in enumerate(messages):
        assert channel.topic == "/robot/state_action"
        assert message.sequence == index

        expected_time = (index + 1) * 1_000_000_000
        assert message.log_time == expected_time
        assert message.publish_time == expected_time

        payload = json.loads(message.data)
        assert payload["state"] == state.tolist()
        assert payload["action"] == action.tolist()

def test_recording_preserves_images_and_skips_cached_frames(tmp_path):
    first = Frame(
        camera_name="left",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        timestamp_s=0.0,
        depth=np.ones((2, 2), dtype=np.uint16),
    )
    second = Frame(
        camera_name="left",
        rgb=np.full((2, 2, 3), 100, dtype=np.uint8),
        timestamp_s=0.2,
    )
    joints = np.zeros(14, dtype=np.float32)

    with Recorder(tmp_path) as recorder:
        for timestamp, frame in (
            (0.0, first),
            (0.1, first),
            (0.2, second),
        ):
            recorder.record(
                Sample(
                    timestamp_s=timestamp,
                    frames=(frame,),
                    state=joints,
                    action=joints,
                )
            )

    with (tmp_path / "episode.mcap").open("rb") as file:
        messages = list(
            make_reader(file).iter_messages(topics=["/cameras/left"])
        )

    assert len(messages) == 2

    for (_, channel, message), expected in zip(messages, (first, second)):
        assert channel.message_encoding == "npz"
        assert message.publish_time == round(
            expected.timestamp_s * 1_000_000_000
        )

        with np.load(BytesIO(message.data), allow_pickle=False) as arrays:
            np.testing.assert_array_equal(arrays["rgb"], expected.rgb)

            if expected.depth is None:
                assert "depth" not in arrays.files
            else:
                np.testing.assert_array_equal(arrays["depth"], expected.depth)