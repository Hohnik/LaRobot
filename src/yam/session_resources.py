"""Acquired session handles and motor-first cleanup, including partial startup.

Register each returned handle before configuring or reading it. ArmSession objects
are deliberately absent: a constructor failure must not hide its enabled robot.
Motion/parking and process signal hooks remain the application's responsibility.
"""
from __future__ import annotations

from typing import Any, Callable

from yam.recording_session import RecordingSession


class SessionResources:
    def __init__(self, recording: RecordingSession, *,
                 shutdown_robot: Callable[[Any], list[int]],
                 emit: Callable[[str], None] = print) -> None:
        self.recording = recording
        self._shutdown_robot = shutdown_robot
        self._emit = emit
        self._robots: list[tuple[str, Any, frozenset[int]]] = []
        self._pucks: list[Any] = []
        self._captures: list[Any] = []
        self._result: bool | None = None

    @property
    def robot_names(self) -> tuple[str, ...]:
        return tuple(name for name, _, _ in self._robots)

    def own_robot(self, name: str, robot: Any, *, motors: int) -> None:
        self._robots.append((name, robot, frozenset(range(1, motors + 1))))

    def own_puck(self, handle: Any) -> None:
        self._pucks.append(handle)

    def own_capture(self, capture: Any) -> None:
        self._captures.append(capture)

    def close(self) -> bool:
        """Attempt every cleanup after ordinary failures; return confirmed success.

        Disable all acquired robots before closing inputs, finishing recording
        work or stopping readers. Repeated completed closes return the same result.
        KeyboardInterrupt/SystemExit retain their normal process-level behavior.
        """
        if self._result is not None:
            return self._result
        ok = True
        for name, robot, expected in self._robots:
            try:
                disabled = self._shutdown_robot(robot)
                self._emit(f"\narm {name} motors confirmed disabled: {disabled}")
                missing = sorted(expected - set(disabled))
                if missing:
                    ok = False
                    self._emit(f"\n⛔ arm {name}: could not confirm motors {missing} disabled.")
                    self._emit("   Treat that arm as live. Support it and cut the mains.")
            except Exception as exc:
                ok = False
                self._emit(f"\n⛔ arm {name}: could not confirm the motors are disabled: "
                           f"{type(exc).__name__}: {exc}")
                self._emit("   Treat that arm as live. Support it and cut the mains.")
        for handle in self._pucks:
            try:
                handle.close()
            except Exception as exc:
                ok = False
                self._emit(f"\n⚠️ could not close SpaceMouse: {exc}")
        # Recording owns writers; capture owns the longer-lived camera readers.
        cleanups = [("recording", self.recording.shutdown)]
        cleanups.extend(("camera readers", lambda capture=capture: capture.stop())
                        for capture in self._captures)
        for label, cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                ok = False
                self._emit(f"\n⚠️ could not clean up {label}: {exc}")
        self._result = ok
        return ok
