"""Recording selection, speed preview and explicit playback confirmation.

A START result requests the application's existing park-to-start flow. This prompt
never commands arms or bypasses the measured-pose check before replay begins.
"""
from __future__ import annotations

from enum import Enum, auto

from yam.playback_session import PlaybackSession
from yam.recording import safe_time_scale


class PlaybackChoice(Enum):
    STAY = auto()
    CLOSE = auto()
    START = auto()


class PlaybackPrompt:
    def __init__(self, playback: PlaybackSession, *, load_take, hint, emit=print):
        self.playback = playback
        self.load_take = load_take
        self.hint = hint
        self.emit = emit
        self.open()

    def open(self) -> None:
        self._choosing_slot = True

    def plan(self, planned_speed: float) -> str:
        playback = self.playback
        if playback.pending is None:
            return ""
        taught = playback.pending.joint_speed(99)
        note = ""
        if taught > planned_speed:
            note = (f" ⚠️ taught {taught:.1f} rad/s exceeds the "
                    f"{planned_speed:.1f} allowed, so 1.00x will lag")
        return (f"PLAY {playback.slot} · {playback.pending.duration:.1f}s taught at "
                f"{taught:.2f} rad/s · speed {playback.speed:.2f}x (-/+){note} · Enter=go "
                f"· j=puck scrub")

    def handle(self, key: str, planned_speed: float) -> PlaybackChoice:
        playback = self.playback
        if self._choosing_slot:
            if not key.isdigit():
                self.emit("\n  play cancelled.\n")
                return PlaybackChoice.CLOSE
            got = self.load_take(key)
            if got is None:
                return PlaybackChoice.CLOSE
            loaded, layout, arms = got
            playback.prepare(loaded, layout, arms, key, planned_speed)
            self._choosing_slot = False
            self.hint(self.plan(planned_speed))
            return PlaybackChoice.STAY
        if key in ("+", "="):
            # Fast taught motion may still use 1x, but never a higher multiplier.
            ceiling = max(1.0, safe_time_scale(playback.pending.joint_speed(99), planned_speed))
            playback.speed = min(ceiling, playback.speed * 1.25)
            self.hint(self.plan(planned_speed))
            return PlaybackChoice.STAY
        if key == "-":
            playback.speed = max(0.05, playback.speed / 1.25)
            self.hint(self.plan(planned_speed))
            return PlaybackChoice.STAY
        if key in ("j", "\r", "\n", " ") and playback.pending is not None:
            playback.scrub = key == "j"
            return PlaybackChoice.START
        playback.cancel_pending()
        self.hint("")
        self.emit("\n  play cancelled.\n")
        return PlaybackChoice.CLOSE
