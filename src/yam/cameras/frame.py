"""A cached camera sample with explicit identity and clock fields.

The current LaRobot branches use different Frame contracts. Adapters must
translate clock units and image channels; see docs/BRIDGE.md. Previous claims
of identical interfaces are preserved in the source-note archive.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Frame:
    """One sample from the reference camera readers.

    host_timestamp_ns uses the process's monotonic clock at frame-store time;
    it is not a hardware exposure timestamp. Recorder alignment uses that clock.
    The OpenCV reader leaves camera_timestamp_ns and depth unknown (None).
    Despite its legacy name, rgb contains OpenCV BGR pixels. Consumers must handle
    that channel order explicitly. Sequence increases at the producer; a consumer
    may miss frames when reading a latest-only cache or after writer queue drops.
    """

    camera_name: str
    sequence: int                        # producer count; consumers can observe gaps
    camera_timestamp_ns: int | None      # None when the backend cannot provide it
    host_timestamp_ns: int               # monotonic clock, stamped at frame-store time
    rgb: Any                             # the BGR ndarray as OpenCV delivers it
    depth: Any | None = None             # unused by the current OpenCV colour reader
