from time import monotonic_ns
from typing import final, override

import numpy as np
import pyrealsense2 as rs

from robot.cameras.camera import Camera
from robot.cameras.frame import Frame


@final
class D405Camera(Camera):
    """Read RGB frames and timestamps from a D405 selected by serial."""

    def __init__(
        self,
        serial_number: str,
        name: str,
        width: int = 1280, # max width supported by D405
        height: int = 720, # max height supported by D405
        fps: int = 30, # max fps supported by D405 when running at max resolution
        timeout_ms: int = 5000, # no idea if this is a sensible default
    ) -> None:
        self.serial_number = serial_number
        self.name = name
        self.width = width
        self.height = height
        self.fps = fps
        self.timeout_ms = timeout_ms
        self._pipeline: rs.pipeline | None = None

    @classmethod
    @override
    def is_available(cls) -> bool:
        """Check for a D405 without starting a stream.

        Returns
        -------
        True if the SDK detects at least one D405, regardless of serial number.
        ```
        bool
        ```
        """
        return any(
            device.get_info(rs.camera_info.name).endswith("D405")
            for device in rs.context().query_devices()
        )

    @override
    def connect(self) -> None:
        """Start colour capture and verify that the first frame arrives."""
        if self._pipeline is not None:
            return

        config = rs.config()
        config.enable_device(self.serial_number)
        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps
        )

        pipeline = rs.pipeline()
        _ = pipeline.start(config)
        self._pipeline = pipeline

    @override
    def read(self) -> Frame:
        """Wait for a colour frame and retain its timestamp.

        Returns
        -------
        Named frame containing an owned RGB uint8 array of shape
        (height, width, 3), a captured and host-timestamp in nanoseconds.
        ```
        Frame
        ```

        Raises
        ------
        RuntimeError
            The camera is not connected
        """
        if self._pipeline is None:
            raise RuntimeError(f"{self.name}: call connect() before read()")
        frameset = self._pipeline.wait_for_frames(self.timeout_ms)
        rgb_frame = frameset.get_color_frame()
        capture_time = rgb_frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp) # this is in microseconds
        host_time_ns = monotonic_ns()

        return Frame(
            camera_name=self.name,
            timestamp_ns_capture=capture_time * 1_000,
            timestamp_ns_host=host_time_ns,
            rgb=np.asanyarray(rgb_frame.get_data()).copy(),
            depth=None,
        )

    @override
    def close(self) -> None:
        pipeline, self._pipeline = self._pipeline, None
        if pipeline is not None:
            pipeline.stop()
