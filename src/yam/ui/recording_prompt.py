"""Recording slot confirmation and asynchronous-operation feedback.

The RecordingSession owns the take, writers and disk work. This controller owns
only the prompt, replacement choice and success summary; it never commands arms.
While busy, keys are consumed except q, which returns to the operator's quit flow.
When choosing a slot, every non-digit (including q) retains its discard meaning.
"""
from __future__ import annotations

from enum import Enum, auto
from pathlib import Path
from typing import Callable

from yam.recording import describe_slot
from yam.recording_session import RecordingSession
from yam.recording_store import SavedTake, save_take


class SaveState(Enum):
    CLOSED = auto()
    CHOOSING = auto()
    REPLACING = auto()
    SAVING = auto()
    DISCARDING = auto()


def save_slot_action(key: str, has_take: bool, was_replacing: bool,
                     replace_slot: str | None, occupied: str | None) -> tuple:
    """Choose save, ask or discard for the recording slot prompt.

    An occupied slot needs the same digit twice. A different digit re-aims.
    A non-digit discards the new take and keeps the occupied slot. Returns
    (save, slot), (ask, slot, reaimed), or (discard, previous_slot_or_None).
    """
    if not (key.isdigit() and has_take):
        return ("discard", replace_slot if was_replacing else None)
    confirmed = was_replacing and key == replace_slot
    if occupied is not None and not confirmed:
        return ("ask", key, bool(was_replacing and key != replace_slot))
    return ("save", key)


class RecordingPrompt:
    def __init__(self, recording: RecordingSession, root: Path, *,
                 saver: Callable[..., SavedTake] = save_take,
                 emit: Callable[[str], None] = print) -> None:
        self.recording = recording
        self.root = root
        self.saver = saver
        self.emit = emit
        self._state = SaveState.CLOSED
        self._replace_slot: str | None = None
        self._summary: tuple[str, float, int] | None = None

    @property
    def active(self) -> bool:
        return self._state is not SaveState.CLOSED

    def open(self) -> None:
        self._state = SaveState.CHOOSING
        self._replace_slot = None
        self._summary = None

    def discard(self) -> None:
        self.recording.request_discard()
        self._state = SaveState.DISCARDING
        self._replace_slot = None
        self._summary = None

    def poll(self) -> None:
        """Observe completion once per cycle, without waiting for disk work."""
        recording = self.recording
        had_finishing_sink = recording.active is None and recording.sink is not None
        recording.poll()
        if had_finishing_sink and recording.sink is None:
            if recording.frame_error:
                self.emit(f"\n  {recording.frame_error}. Frames retained at {recording.frames}.\n")
            elif recording.frame_report is not None:
                for camera, report in recording.frame_report["per_camera"].items():
                    self.emit(f"  📷 {camera}: {report['written']} frame(s), "
                          f"{report['dropped']} dropped, {report['write_errors']} write error(s).")
                self.emit("  Camera files finished. Choose a save slot or discard.\n")
        if self._state in (SaveState.SAVING, SaveState.DISCARDING) and not recording.busy:
            if recording.save_error is not None:
                self.emit(f"\n  Recording disk operation failed: {recording.save_error}")
                for note in getattr(recording.save_error, "__notes__", []):
                    self.emit(f"     {note}")
                self.emit("     The new take is still pending. Choose a slot to retry, "
                      "or a non-digit to discard.\n")
                self.open()
            elif self._state is SaveState.SAVING and recording.saved is not None:
                slot, seconds, count = self._summary
                saved = recording.saved
                if saved.warning:
                    self.emit(f"  {saved.warning}")
                self.emit(f"\n  ✓ recording {slot} saved: {seconds:.1f}s, "
                      f"{count} samples → {saved.path.name}"
                      + (f" + frames/{slot}/" if saved.has_frames else ""))
                self.emit(f"     (l then {slot} plays it back)\n")
                self._state = SaveState.CLOSED
            else:
                self.emit("\n  recording discarded.\n")
                self._state = SaveState.CLOSED

    def handle(self, key: str) -> bool:
        """Consume a prompt key; return False when normal dispatch should run."""
        if not self.active:
            return False
        recording = self.recording
        if recording.busy:
            if key == "q":
                self._state = SaveState.CLOSED
                return False
            self.emit("  Recording files are still finishing. Wait, or q to quit; "
                      "unfinished files are retained.")
            return True
        was_replacing = self._state is SaveState.REPLACING
        occupied = describe_slot(self.root / f"{key}.json") if key.isdigit() else None
        action = save_slot_action(key, recording.pending is not None,
                                  was_replacing, self._replace_slot, occupied)
        if action[0] == "ask":
            self._replace_slot = slot = action[1]
            self._state = SaveState.REPLACING
            if action[2]:
                self.emit(f"\n  ⭐ aiming at recording {slot} instead.")
            self.emit(f"\n  ⚠️  recording {slot} already holds {occupied}.")
            self.emit(f"     Press {slot} again to REPLACE it, or "
                      "another digit to aim somewhere else.")
            self.emit("     Any NON-digit discards the new recording and keeps "
                      "what is there.\n")
        elif action[0] == "discard":
            self.discard()
            if action[1] is not None:
                self.emit(f"\n  kept recording {action[1]}; the new one is being discarded.\n")
            else:
                self.emit("\n  finishing recording discard.\n")
        else:
            try:
                self._summary = (key, recording.pending.duration, len(recording.pending))
                recording.request_save(self.root, key, saver=self.saver)
                self._state = SaveState.SAVING
                self.emit(f"\n  Saving recording {key}… control remains active.\n")
            except Exception as exc:
                self.open()
                self.emit(f"\n  Recording was not saved: {exc}")
                self.emit("     The new take is still pending. Choose a slot to retry, "
                          "or a non-digit to discard.\n")
        return True
