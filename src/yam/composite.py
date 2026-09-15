"""Sequence pose and recording legs after all participating arms arrive.

The operator supplies the existing park and replay actions. This owner validates
all takes before starting, owns the remaining queue, and clears it on cancellation.
It never treats a pose-leg arrival as a replay-start arrival; the operator must
capture each completed path's purpose before calling arrived().
"""
from __future__ import annotations

from typing import Any, Callable, Sequence

from yam.robot import resolve_park_legs


class CompositeRun:
    def __init__(self, *, load_take: Callable[[str], tuple | None],
                 begin_path: Callable[..., None], start_take: Callable[..., None],
                 emit: Callable[[str], None] = print,
                 clear_hint: Callable[[], None] = lambda: None) -> None:
        self._load_take = load_take
        self._begin_path = begin_path
        self._start_take = start_take
        self._emit = emit
        self._clear_hint = clear_hint
        self._queue: list = []
        self._waiting: set[str] = set()
        self._total = 0
        self._arms: list[Any] = []

    @property
    def active(self) -> bool:
        return bool(self._queue or self._waiting or self._total)

    def begin(self, wanted: Sequence[str], arms: Sequence[Any]) -> bool:
        """Validate all take legs before any motion; preserve selection by value."""
        entries: list = []
        for entry in wanted:
            if entry.startswith("w"):
                entries.append(("take", entry[1:]))
            elif entries and entries[-1][0] == "poses":
                entries[-1][1].append(entry)
            else:
                entries.append(["poses", [entry]])
        resolved: list = []
        for entry in entries:
            if entry[0] == "take":
                got = self._load_take(entry[1])
                if got is None:
                    self._emit("     the whole composite run is refused — nothing has "
                               "moved. Fix the take and retype it.\n")
                    return False
                resolved.append(("take", entry[1], *got))
            else:
                resolved.append(entry)
        self._queue = resolved
        self._waiting = set()
        self._total = len(resolved)
        self._arms = list(arms)
        self._emit(f"\n⭐ COMPOSITE RUN: {self._total} leg(s) — poses park, takes "
                   "play, h or t abandons the rest.\n")
        self._next()
        return True

    def abandon(self, why: str) -> None:
        """Cancel once; later arrivals cannot start queued work."""
        if not self.active:
            return
        left = len(self._queue)
        self._queue, self._waiting, self._total = [], set(), 0
        self._emit(f"\n  ⚠️  composite run abandoned ({why}) — "
                   f"{left} queued leg(s) dropped.\n")

    def arrived(self, name: str) -> None:
        """Credit only an arm awaited by the current pose leg."""
        if name in self._waiting:
            self._waiting.discard(name)
            if not self._waiting:
                self.leg_done()

    def leg_done(self) -> None:
        """A pose group or a replay completed; start the next or report completion."""
        if self._queue:
            self._next()
        elif self._total:
            done, self._total = self._total, 0
            self._clear_hint()
            self._emit(f"\n⭐ COMPOSITE RUN complete — all {done} leg(s) done.\n")

    def _next(self) -> None:
        while self._queue:
            entry = self._queue.pop(0)
            left = len(self._queue)
            if entry[0] == "poses":
                wanted = entry[1]
                self._waiting = set()
                for arm in self._arms:
                    legs, missing = resolve_park_legs(wanted, arm.base_pose, arm.slots)
                    if missing:
                        self._emit(f"\n  ⚠️  arm {arm.name}: skipping empty slot(s) "
                                   f"{', '.join(missing)}.\n")
                    if legs:
                        self._begin_path(arm, legs, " → ".join(n for n, _ in legs),
                                         for_composite=True)
                        self._waiting.add(arm.name)
                if self._waiting:
                    self._emit(f"  ▶ composite: {left} leg(s) still queued after this one.")
                    return
                continue
            self._start_take(entry[1], entry[2], entry[3], entry[4])
            self._emit(f"  ▶ composite: {left} leg(s) still queued after this one.")
            return
        self.leg_done()
