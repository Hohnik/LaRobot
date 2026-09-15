"""Ownership while a set of camera readers is being opened."""
from __future__ import annotations

from typing import Any, Callable


class CameraStartup:
    """Release partial startup on failure; successful readers belong to the caller.

    A raw capture is owned until explicitly released or handed to a reader.
    If a later camera fails, every earlier reader is stopped as well. Exceptions
    from cleanup are attached to the original failure and do not skip other devices.
    """

    def __init__(self) -> None:
        self._captures: dict[int, Any] = {}
        self._readers: list[Any] = []

    def __enter__(self) -> CameraStartup:
        return self

    def own(self, capture: Any) -> Any:
        if capture is not None:
            self._captures[id(capture)] = capture
        return capture

    def release(self, capture: Any) -> None:
        capture.release()
        self._captures.pop(id(capture), None)

    def start_reader(self, capture: Any, factory: Callable[[Any], Any]) -> Any:
        reader = factory(capture)
        self._readers.append(reader)
        self._captures.pop(id(capture), None)
        return reader

    def __exit__(self, exc_type, failure, traceback) -> bool:
        if failure is not None:
            cleanup = [reader.stop for reader in reversed(self._readers)]
            cleanup.extend(capture.release for capture in self._captures.values())
            for close in cleanup:
                try:
                    close()
                except Exception as cleanup_error:  # noqa: BLE001
                    failure.add_note(f"Camera startup cleanup failed: {cleanup_error}")
        return False
