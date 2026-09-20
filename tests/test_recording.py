from io import BytesIO

import h5py
import numpy as np
import pytest
from PIL import Image

from robot.cameras.frame import Frame
from robot.recording.recorder import Recorder
from robot.recording.sample import Sample


def test_recording_preserves_timestamps_states_and_actions(tmp_path):
    state = np.arange(14, dtype=np.float32)
    action = state + np.float32(0.25)
    recorder = Recorder(tmp_path)

    with recorder:
        for timestamp_ns in (1_000_000_000, 2_000_000_000):
            recorder.record(
                Sample(
                    timestamp_ns=timestamp_ns,
                    frames=(),
                    state=state,
                    action=action,
                )
            )

    with h5py.File(recorder.file_path, "r") as file:
        np.testing.assert_array_equal(
            file["timestamps"][:],
            np.array([1_000_000_000, 2_000_000_000], dtype=np.uint64),
        )
        np.testing.assert_array_equal(file["states"][:], [state, state])
        np.testing.assert_array_equal(file["actions"][:], [action, action])


def test_recording_writes_each_camera_frame_once_as_jpeg(tmp_path):
    state = np.zeros(14, dtype=np.float32)
    first_rgb = np.full((8, 10, 3), (20, 80, 140), dtype=np.uint8)
    second_rgb = np.full((8, 10, 3), (140, 80, 20), dtype=np.uint8)
    first = Frame("overhead", 100, first_rgb)
    second = Frame("overhead", 200, second_rgb)
    recorder = Recorder(tmp_path)

    with recorder:
        for timestamp_ns, frame in ((100, first), (150, first), (200, second)):
            recorder.record(Sample(timestamp_ns, (frame,), state, state))

    with h5py.File(recorder.file_path, "r") as file:
        camera = file["cameras/overhead"]
        np.testing.assert_array_equal(camera["timestamps"][:], [100, 200])
        assert len(camera["frames"]) == 2
        assert camera["frames"].attrs["encoding"] == "jpeg"
        assert camera["frames"].attrs["color_space"] == "RGB"

        decoded = np.asarray(
            Image.open(BytesIO(np.asarray(camera["frames"][0]).tobytes()))
        )
        np.testing.assert_allclose(decoded, first_rgb, atol=2)


def test_recording_requires_an_open_context(tmp_path):
    state = np.zeros(14, dtype=np.float32)
    sample = Sample(0, (), state, state)
    recorder = Recorder(tmp_path)

    with pytest.raises(RuntimeError, match="must be opened"):
        recorder.record(sample)

    with recorder:
        pass

    with pytest.raises(RuntimeError, match="must be opened"):
        recorder.record(sample)


def test_recording_rejects_decreasing_camera_timestamps(tmp_path):
    state = np.zeros(14, dtype=np.float32)
    rgb = np.zeros((8, 10, 3), dtype=np.uint8)

    with Recorder(tmp_path) as recorder:
        recorder.record(Sample(200, (Frame("overhead", 200, rgb),), state, state))
        with pytest.raises(ValueError, match="timestamps must not decrease"):
            recorder.record(Sample(300, (Frame("overhead", 100, rgb),), state, state))


def test_recording_keeps_camera_streams_independent(tmp_path):
    state = np.zeros(14, dtype=np.float32)
    rgb = np.zeros((8, 10, 3), dtype=np.uint8)
    recorder = Recorder(tmp_path)

    with recorder:
        recorder.record(
            Sample(
                100,
                (
                    Frame("overhead", 100, rgb),
                    Frame("wrist", 50, rgb),
                ),
                state,
                state,
            )
        )
        recorder.record(
            Sample(
                200,
                (
                    Frame("overhead", 200, rgb),
                    Frame("wrist", 150, rgb),
                ),
                state,
                state,
            )
        )

    with h5py.File(recorder.file_path, "r") as file:
        np.testing.assert_array_equal(
            file["cameras/overhead/timestamps"][:], [100, 200]
        )
        np.testing.assert_array_equal(file["cameras/wrist/timestamps"][:], [50, 150])


def test_recording_rejects_an_invalid_state_shape(tmp_path):
    sample = Sample(
        timestamp_ns=0,
        frames=(),
        state=np.zeros(6, dtype=np.float32),
        action=np.zeros(14, dtype=np.float32),
    )

    with (
        Recorder(tmp_path) as recorder,
        pytest.raises(ValueError, match="state must have shape"),
    ):
        recorder.record(sample)
