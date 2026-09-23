from collections.abc import Iterator
from pathlib import Path
from time import monotonic_ns
from typing import final, override

import av

from robot.cameras.camera import Camera
from robot.cameras.frame import Frame


@final
class C920Camera(Camera):
    """Read RGB frames and V4L2 timestamps from a C920 selected by device path."""

    def __init__(
        self,
        device_path: str,
        name: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ) -> None:
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("width, height and fps must be positive")
        self.device_path = device_path
        self.name = name
        self.width = width
        self.height = height
        self.fps = fps
        self._container: av.container.InputContainer | None = None
        self._frames: Iterator[av.VideoFrame] | None = None

    @classmethod
    @override
    def is_available(cls) -> bool:
        """Check for a C920 without starting a stream.

        Returns
        -------
        True if a C920 capture node is present, regardless of device path.
        ```
        bool
        ```
        """
        return any(
            path.is_char_device()
            for path in Path("/dev/v4l/by-id").glob("*C920*video-index0")
        )

    @override
    def connect(self) -> None:
        """Start colour capture and verify the stream mode."""
        if self._container is not None:
            return

        self._container = av.open(
            self.device_path,
            mode="r",
            format="v4l2",
            options={
                "input_format": "mjpeg",
                "video_size": f"{self.width}x{self.height}",
                "framerate": str(self.fps),
                "timestamps": "default",  # use the driver's clock
                "probesize": "32",  # limit startup buffering
                "analyzeduration": "0",
            },
        )

    @override
    def read(self) -> Frame:
        """Wait for a colour frame and retain its V4L2 timestamp.

        Returns
        -------
        Named frame containing an owned RGB uint8 array of shape
        (height, width, 3), a capture and host timestamp in nanoseconds.
        ```
        Frame
        ```

        Raises
        ------
        RuntimeError
            The camera is not connected, the stream ended or the timestamp is missing.
        """
        if self._frames is None:
            raise RuntimeError(f"{self.name}: call connect() before read()")

        try:
            rgb_frame = next(self._frames)
        except StopIteration:
            raise RuntimeError(f"{self.name}: camera stream ended") from None
        host_time_ns = monotonic_ns()

        return Frame(
            camera_name=self.name,
            timestamp_ns_capture=int(
                rgb_frame.pts * rgb_frame.time_base * 1e9
            ),
            timestamp_ns_host=host_time_ns,
            rgb=rgb_frame.to_ndarray(format="rgb24").copy(),
        )

    @override
    def close(self) -> None:
        self._frames = None
        container, self._container = self._container, None
        if container is not None:
            container.close()
