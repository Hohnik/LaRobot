"""Structural interfaces for current input and camera implementations.

These describe the listed implementations. Device discovery, button mapping and
command freshness remain application responsibilities. See docs/BRIDGE.md for
the distinct interfaces in the team repository.
"""
from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

__all__ = ["CommandSource", "FrameSource", "FrameConsumer"]


@runtime_checkable
class CommandSource(Protocol):
    """Movement requests supplied by TwistReader.

    A policy adapter also needs an explicit freshness and failure policy, plus
    application wiring for discovery, ownership and any required buttons.
    """

    def read(self) -> list[float]:
        """Six numbers: translation then rotation, each between -1 and 1.

        Values are fractions of configured maximum speed. This call must not
        wait for new input. TwistReader retains axis state between HID reports;
        an empty device read does not mean a neutral puck. Device errors may
        raise. The operator catches them, zeros the command and requests a stop.
        A policy adapter must define when its latest command expires.
        """
        ...


@runtime_checkable
class FrameSource(Protocol):
    """Latest camera pictures buffered by CaptureSet's reader threads."""

    @property
    def names(self) -> list[str]:
        """Camera names in stable order, as recorded in the dataset."""
        ...

    def sample(self) -> dict[str, Any]:
        """Return the newest Frame per camera without waiting for capture.

        A camera with no picture returns None. Repeated frames retain their
        sequence and host_timestamp_ns; consumers decide whether to store them.
        """
        ...

    def stop(self) -> None:
        """Attempt to release every device; report cleanup failures to the caller."""
        ...


@runtime_checkable
class FrameConsumer(Protocol):
    """Per-take camera writing implemented by FrameSink.

    Retain the sink and its directory through completion. Encoding and final
    index writes happen in workers. Changing the storage format also requires
    changing the episode exporter; this interface does not cover file schemas.
    """

    def offer(self, samples: dict[str, Any]) -> None:
        """Queue frames whose sequence advanced, without waiting for disk I/O.

        Full queues drop their oldest frame and count the drop. Unexpected
        failures may raise; the operator stops the take and retains its files.
        """
        ...

    def request_stop(self) -> None:
        """Stop accepting samples and request draining without waiting."""
        ...

    @property
    def finished(self) -> bool:
        """True when no worker can write again, including after a failure."""
        ...

    def poll_stop(self) -> dict[str, dict[str, Any]] | None:
        """Final per-camera indexes, None while busy, or a completion failure."""
        ...

    def stop(self) -> dict[str, dict[str, Any]]:
        """Blocking convenience for teardown and standalone tools.

        A timeout can return flushed=False. Those files are still owned by the
        caller and must not be published or discarded until workers terminate.
        Use request_stop/poll_stop inside the control loop.
        """
        ...
