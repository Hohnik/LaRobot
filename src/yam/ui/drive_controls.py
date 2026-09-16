"""Shared speed-key and wrist-rotation policy for normal and mapping modes.

This owner adjusts requested rates, never joint commands or safety limits. Linear
and scrub values stay in the live settings namespace so settings-panel edits are
visible immediately. Mode transitions and key precedence belong to the caller.
"""
from __future__ import annotations

import math

from yam.settings import LIVE_BOUNDS, adjust

# Requested angular-rate bounds; downstream joint-rate and lag constraints still apply.
MAX_ANGULAR_SCALE = 12.0
MIN_ANGULAR_SCALE = 0.02


class DriveControls:
    def __init__(self, *, rotation: bool, angular_scale: float, hint, emit=print):
        self.rotation = rotation
        self.angular_scale = angular_scale
        self.hint, self.emit = hint, emit

    def handle(self, key: str, *, values, aimed, scrub: bool = False,
               mapping: bool = False) -> bool:
        """Handle r , . - + =; return False for keys owned by other interactions.

        In normal mode, scrub pace takes priority over selected-arm park speed,
        then linear speed. Mapping mode always adjusts linear speed. If any
        selected arm is parking, preserve the existing all-selected speed update.
        """
        if key == 'r':
            self.rotation = not self.rotation
            message = f"wrist rotation {'ON' if self.rotation else 'OFF'}"
            if mapping:
                self.emit(f"\n  {message}"
                          f"{'' if self.rotation else ' — ROLL/PITCH/YAW will not move'}\n")
            else:
                self.hint(message)
        elif key in (',', '.'):
            self.angular_scale = (min(MAX_ANGULAR_SCALE, self.angular_scale * 1.25)
                                  if key == '.' else
                                  max(MIN_ANGULAR_SCALE, self.angular_scale / 1.25))
            if mapping:
                self.emit(f"\n  rotation speed → {self.angular_scale:.2f} rad/s "
                          f"({math.degrees(self.angular_scale):.0f}°/s)\n")
            else:
                self.hint(f"rotation speed {self.angular_scale:.2f} rad/s")
        elif key in ('+', '=', '-'):
            increase = key != '-'
            if not mapping and scrub:
                values.scrub_max = adjust('scrub_max', values.scrub_max, increase)
                self.hint(f"scrub pace: full push = {values.scrub_max:g}x the "
                          "recording's own speed")
            elif not mapping and any(one.mode == 'park' for one in aimed):
                for one in aimed:
                    one.park_speed = (min(values.teleop_speed, one.park_speed * 1.25)
                                      if increase else max(.05, one.park_speed / 1.25))
                self.hint(f"park speed {aimed[0].park_speed:.2f} rad/s")
            else:
                values.linear_scale = adjust('linear_scale', values.linear_scale, increase)
                self.hint(f"linear speed {values.linear_scale:.3f} m/s"
                          + (' (ceiling)' if increase and
                             values.linear_scale >= LIVE_BOUNDS['linear_scale'][1] else ''))
        else:
            return False
        return True
