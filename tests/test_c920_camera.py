from fractions import Fraction
from pathlib import Path
from unittest.mock import Mock

import av
import numpy as np
import pytest

from robot.cameras.c920_camera import C920Camera


@pytest.fixture
def backend(monkeypatch):
    rgb = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    frame = Mock(spec=av.VideoFrame)
    frame.pts = 442_242_128_478
    frame.time_base = Fraction(1, 1_000_000)
    frame.to_ndarray.return_value = rgb
    container = Mock()
    container.decode.side_effect = lambda **kwargs: iter([frame])
    open_camera = Mock(return_value=container)
    monkeypatch.setattr(av, "open", open_camera)
    monkeypatch.setattr(
        "robot.cameras.c920_camera.monotonic_ns", lambda: 442_242_142_554_528
    )
    return open_camera, container, frame


def test_read_rgb_and_driver_timestamp(backend):
    open_camera, container, raw_frame = backend
    with C920Camera("/dev/c920", "overhead") as camera:
        frame = camera.read()

    assert frame.camera_name == "overhead"
    assert frame.timestamp_ns_capture == 442_242_128_478_000
    assert frame.timestamp_ns_host == 442_242_142_554_528
    assert frame.rgb.dtype == np.uint8
    assert frame.rgb.shape == (1, 2, 3)
    assert frame.rgb.flags.owndata
    assert frame.depth is None
    raw_frame.to_ndarray.assert_called_once_with(format="rgb24")
    raw_frame.to_ndarray.return_value[:] = 0
    np.testing.assert_array_equal(frame.rgb, [[[10, 20, 30], [40, 50, 60]]])
    assert open_camera.call_args.args == ("/dev/c920",)
    options = open_camera.call_args.kwargs["options"]
    assert options["input_format"] == "mjpeg"
    assert options["timestamps"] == "default"
    container.close.assert_called_once()


def test_rgb_conversion_with_real_pyav_frame(backend):
    _, container, _ = backend
    frame = av.VideoFrame.from_ndarray(
        np.array([[[0, 0, 255], [255, 0, 0]]], dtype=np.uint8), format="bgr24"
    )
    frame.pts = 1_234_567
    frame.time_base = Fraction(1, 90_000)
    container.decode.side_effect = lambda **kwargs: iter([frame])
    with C920Camera("/dev/c920", "overhead") as camera:
        result = camera.read()
    np.testing.assert_array_equal(result.rgb, [[[255, 0, 0], [0, 0, 255]]])
    assert result.timestamp_ns_capture == 13_717_411_111


def test_connect_close_and_reconnect(backend):
    open_camera, container, _ = backend
    camera = C920Camera("/dev/c920", "overhead")
    with pytest.raises(RuntimeError, match=r"call connect\(\)"):
        camera.read()
    camera.close()
    camera.connect()
    camera.connect()
    open_camera.assert_called_once()
    camera.read()
    camera.close()
    camera.close()
    container.close.assert_called_once()
    with pytest.raises(RuntimeError, match=r"call connect\(\)"):
        camera.read()
    with camera:
        assert camera.read().camera_name == "overhead"
    assert open_camera.call_count == 2


def test_custom_mode(backend):
    open_camera, _, _ = backend
    with C920Camera("/dev/c920", "overhead", width=1920, height=1080):
        assert open_camera.call_args.kwargs["options"]["video_size"] == "1920x1080"


def test_decode_error_closes_device(backend):
    _, container, _ = backend
    failure = OSError("camera disconnected")

    def failing_stream(**kwargs):
        raise failure
        yield  # Make the failure happen on read, like PyAV's decoder.

    container.decode.side_effect = failing_stream
    with (
        pytest.raises(OSError) as raised,
        C920Camera("/dev/c920", "overhead") as camera,
    ):
        camera.read()
    assert raised.value is failure
    container.close.assert_called_once()


def test_availability_uses_capture_node_without_opening_stream(
    backend, monkeypatch, tmp_path
):
    open_camera, _, _ = backend
    capture = tmp_path / "usb-046d_HD_Pro_Webcam_C920-video-index0"
    metadata = tmp_path / "usb-046d_HD_Pro_Webcam_C920-video-index1"
    metadata.symlink_to("/dev/null")
    glob = Path.glob
    monkeypatch.setattr(Path, "glob", lambda self, pattern: glob(tmp_path, pattern))
    assert not C920Camera.is_available()
    capture.symlink_to("/dev/null")
    assert C920Camera.is_available()
    capture.unlink()
    capture.symlink_to(tmp_path / "unplugged")
    assert not C920Camera.is_available()
    open_camera.assert_not_called()
