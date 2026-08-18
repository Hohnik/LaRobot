"""The per-sample camera record, field-aligned with the team's LaRobot `Frame`.

⭐ The alignment is the point (ROADMAP §10.6): the rebuild's `cameras/frame.py` carries `camera_name · sequence · camera_timestamp_ns | None · host_timestamp_ns · rgb · depth: None`-able, and matching those names means capture code written against this walkthrough lifts into the rebuild unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Frame:
    """One camera sample, timestamped honestly.

    ⛔ **`camera_timestamp_ns` is `None` on this stack, and that is a measurement, not laziness.** OpenCV's AVFoundation backend cannot report a device-side capture time (`CAP_PROP_POS_MSEC` is unreliable there, the same backend that cannot report FOURCC — FINDINGS §63.0's family). A fabricated device timestamp would be the fails-by-lying pattern applied to time; `None` says "the camera did not tell us", which is the truth the dataset needs to know about itself.

    **`host_timestamp_ns` is `time.monotonic_ns()` taken when the reader thread STORED the frame**, not when the control loop sampled it — the store moment is the closest observable to the exposure this backend offers. Monotonic, so it survives NTP adjustments; it shares a clock with nothing outside this process, and aligning it to joint data works because the recorder stamps samples from the same clock.

    ⛔ **`depth` is always `None` on this rig** — measured on 2026-08-17: every D405 mode over UVC on macOS is an ordinary colour photograph (FINDINGS §63.0). The field exists because LaRobot's record has it and the rebuild (Ubuntu + SDK) will fill it.
    """

    camera_name: str
    sequence: int                        # per camera, monotonically increasing, no gaps
    camera_timestamp_ns: int | None      # None on this stack — see the docstring
    host_timestamp_ns: int               # monotonic clock, stamped at frame-store time
    rgb: Any                             # the BGR ndarray as OpenCV delivers it
    depth: Any | None = None             # always None over UVC on macOS (FINDINGS §63.0)
