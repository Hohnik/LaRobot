from unittest.mock import Mock

import numpy as np
import pytest

rs = pytest.importorskip("pyrealsense2")

from robot.cameras.d405_camera import D405Camera


@pytest.fixture
def sdk(monkeypatch):
    rgb_frame = Mock(spec=rs.video_frame)
    rgb_frame.get_data.return_value = np.array(
        [[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8
    )
    rgb_frame.get_frame_metadata.side_effect = {
        rs.frame_metadata_value.frame_timestamp: 1_234_567,
    }.__getitem__
    frameset = Mock(spec=rs.composite_frame)
    frameset.get_color_frame.return_value = rgb_frame
    pipeline = Mock(spec=rs.pipeline)
    pipeline.wait_for_frames.return_value = frameset
    monkeypatch.setattr(rs, "pipeline", Mock(return_value=pipeline))
    monkeypatch.setattr("robot.cameras.d405_camera.monotonic_ns", lambda: 9_876_543_210)
    return pipeline, rgb_frame


def test_read_frame(sdk):
    with D405Camera("test-serial", "left") as camera:
        frame = camera.read()
    assert frame.timestamp_ns_capture == 1_234_567_000
    assert frame.timestamp_ns_host == 9_876_543_210
    np.testing.assert_array_equal(frame.rgb, [[[10, 20, 30], [40, 50, 60]]])


def test_read_copies_rgb(sdk):
    _, rgb_frame = sdk
    with D405Camera("test-serial", "left") as camera:
        frame = camera.read()
    rgb_frame.get_data.return_value[:] = 0
    np.testing.assert_array_equal(frame.rgb, [[[10, 20, 30], [40, 50, 60]]])


def test_read_failure_closes_pipeline(sdk):
    pipeline, _ = sdk
    failure = RuntimeError("Frame didn't arrive")
    pipeline.wait_for_frames.side_effect = failure
    with (
        pytest.raises(RuntimeError) as raised,
        D405Camera("test-serial", "left") as camera,
    ):
        _ = camera.read()
    assert raised.value is failure
    pipeline.stop.assert_called_once()
