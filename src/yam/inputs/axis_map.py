"""Which SpaceMouse axis drives which robot motion, and in which direction.

⭐ WHAT THIS ADDS OVER THE OLD `sign` ARRAY. Until 2026-08-10 the mapping was six
sign flips and nothing else: puck axis *i* always drove robot motion *i*, and the
only choice was its direction. Julien asked for the other half — *"the option to
choose which spacemouse control direction controls what part of the arm"* — which
is a **permutation**, not a sign. This module is that permutation plus the sign,
in one place.

    twist[i] = axes[source[i]] * sign[i]          source[i] == UNBOUND -> 0.0

`source` is indexed by **robot motion**, not by puck axis, and that choice is
load-bearing. When Julien presses the flip key for UP he means *"the gripper goes
the wrong way when I do that"* — a statement about the robot, not about the
device. Row-oriented indexing makes the flip keys keep that meaning under any
permutation. Under the identity map the two indexings are numerically identical,
which is why the old files keep working untouched (see `load`).

⛔ WHY THIS IS ITS OWN MODULE, not inline in the session script. `src/yam/inputs/spacemouse.py`
exists because device logic had been copy-pasted into two scripts and a bug fix
landed in only one of them. The same trap was open here: `scripts/teleop_sim.py`
had its own `twist_from_axes()` that applied **no** sign at all, so the simulator —
the one place a wrong axis convention is free to discover — was the one place that
could not reproduce the mapping. One copy, three callers.

THE LABELS ARE MEASURED, NOT ASSUMED
------------------------------------
`CartesianTeleop.step()` integrates the twist in the **world** frame, so a motion
means the same thing however the wrist happens to be turned. What the world axes
physically *are* was measured in simulation on 2026-08-10 rather than taken from
convention, because a prompt that says "push the puck the way you want the gripper
to go UP" is a lie if +Z is not up:

    gravity                     (0, 0, -9.81)      => +Z is up
    joint 1 (base_yaw) rotates about world Z       => the arm stands Z-up
    twist [0.05,0,0] for 1 s -> tcp moved [+0.0499, 0, 0]
    twist [0,0.05,0] for 1 s -> tcp moved [0, +0.0499, 0]
    twist [0,0,0.05] for 1 s -> tcp moved [0, 0, +0.0498]
    each rotation component -> rotation about exactly that world axis, and the
      tool point drifted <= 0.3 mm over 17 deg

⚠️ Deliberately NOT claimed: which way is "forward" or "left" for Julien. That
depends on how the arm is physically turned on the desk, which no file in this
repo records. So +X and +Y are described by geometry that is true of the model
("out from the base at base_yaw = 0"), and the operator supplies the rest by
watching the arm. Inventing a "forward" label would be exactly the confident,
plausible, wrong answer this stack specialises in (FINDINGS §0).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

UNBOUND = -1

# Robot motions, in twist order. `short` is what fits in a status line; `world`
# and `note` are the measured facts above.
ROBOT_MOTIONS: list[dict[str, str]] = [
    {"short": "X",     "long": "X+ / X−",     "world": "+X",       "note": "horizontal, straight out from the base at base_yaw = 0"},
    {"short": "Y",     "long": "Y+ / Y−",     "world": "+Y",       "note": "horizontal, 90° left of +X seen from above"},
    {"short": "UP",    "long": "UP / DOWN",   "world": "+Z",       "note": "straight up, away from the table"},
    {"short": "ROLL",  "long": "ROLL",        "world": "about +X", "note": "twists the tool about +X; the tool point stays put"},
    {"short": "PITCH", "long": "PITCH",       "world": "about +Y", "note": "tips the tool about +Y; the tool point stays put"},
    {"short": "YAW",   "long": "YAW",         "world": "about +Z", "note": "spins the tool about vertical; the tool point stays put"},
]

# ⭐⭐ THE SAME SIX COMPONENTS MEAN DIFFERENT PHYSICAL THINGS IN DIFFERENT FRAMES,
# so their names and descriptions must change with the frame. Julien, 2026-08-11:
# *"the description in the m mode needs to be suitable for the v mode… they need to
# be different to when I'm in T mode and press M."*
#
# The twist ORDER is fixed — [0,1,2] are the three translations, [3,4,5] the
# rotations about those same axes — but *which direction* component 0 is depends
# entirely on the frame the twist gets interpreted in.
#
# ⭐ These labels are MEASURED (2026-08-11, at arm B's saved home pose):
#
#   tool +Z -> world [0.993, 0.080, 0.088]   dot with "out of the gripper" = +1.000
#   tool +X -> world [0.082, 0.079, -0.993]  (straight down at a level gripper)
#   tool +Y -> world [-0.087, 0.994, 0.072]  (left at a level gripper)
#
# **+Z out of the gripper holds at EVERY pose** — it is the gripper's own axis, set
# by the model, not an accident of this pose. "Down" and "left" for X and Y hold
# only while the gripper is upright and unrolled, which their notes say rather than
# quietly assume.
#
# The rotation names follow from the translation ones: turning about the DOWN axis
# swings the forward direction sideways, which a camera operator calls a **pan**;
# turning about the LEFT axis tips it vertically, a **tilt**; turning about the
# FORWARD axis spins the picture, a **roll**.
TOOL_MOTIONS: list[dict[str, str]] = [
    {"short": "DOWN",  "long": "DOWN / UP",    "world": "tool +X",       "note": "down in the picture, when the gripper is upright"},
    {"short": "LEFT",  "long": "LEFT / RIGHT", "world": "tool +Y",       "note": "left in the picture, when the gripper is upright"},
    {"short": "FWD",   "long": "FORWARD / BACK", "world": "tool +Z",     "note": "straight out of the gripper, into the picture — true at EVERY pose"},
    {"short": "PAN",   "long": "PAN",          "world": "about tool +X", "note": "look left / right; the tool point stays put"},
    {"short": "TILT",  "long": "TILT",         "world": "about tool +Y", "note": "look up / down; the tool point stays put"},
    {"short": "ROLL",  "long": "ROLL",         "world": "about tool +Z", "note": "spin the picture around its centre; the tool point stays put"},
]

MOTIONS_BY_FRAME = {
    "world": ROBOT_MOTIONS,
    "tool": TOOL_MOTIONS,
    "camera": TOOL_MOTIONS,   # same axis roles; the camera frame is just canted
}


def motions_for(frame: str) -> list[dict[str, str]]:
    return MOTIONS_BY_FRAME.get(frame, ROBOT_MOTIONS)


# ⭐ HOW A TOOL-FRAME MAP IS SEEDED FROM THE WORLD-FRAME ONE, so Julien does not
# start from nothing. Julien: *"maybe think about what sensible controls would be
# for the v mode, like a default mode so that I don't have to do all the edits
# myself."*
#
# Measured at the home pose, the two frames line up like this:
#
#     tool X ≈ −world Z        tool Y ≈ +world Y        tool Z ≈ +world X
#
# So if each tool component borrows the puck axis that drives its world counterpart
# — negating where the correspondence is negative — the controls feel **identical
# to the world map at the home pose**, and only start differing as the wrist turns.
# That is exactly the wanted behaviour: nothing new to learn on day one, and the
# controls follow the camera the moment you tilt.
#
# Each entry is (world component to borrow from, sign multiplier).
WORLD_TO_TOOL: list[tuple[int, int]] = [
    (2, -1),   # tool X  (down)    <- world UP,    negated: tool +X is world −Z
    (1, +1),   # tool Y  (left)    <- world Y
    (0, +1),   # tool Z  (forward) <- world X
    (5, -1),   # about tool X (pan)  <- world YAW,   negated for the same reason
    (4, +1),   # about tool Y (tilt) <- world PITCH
    (3, +1),   # about tool Z (roll) <- world ROLL
]

# As reported by `spacemouse.TwistReader.read()`. ⚠️ No physical gesture is named
# for these on purpose: which way you have to push the puck to get +z has not been
# measured, so the teach flow describes the operator's own gesture back to them
# from the reading instead of asserting one.
PUCK_AXES = ["x", "y", "z", "roll", "pitch", "yaw"]

N = 6

# ⚠️ Speed policy, which is arguably not this module's business — but it lives here
# because `scripts/map_axes.py` has to report the *same* metres per second the
# session will actually command, or the dial-in tool teaches a mapping at speeds
# the arm does not use. One definition with two importers beats two definitions
# that agree today. Both were tuned on hardware 2026-08-10: Julien found the first
# run "very slow", and these are still well inside what the arm can do.
DEFAULT_LINEAR_SCALE = 0.12    # m/s at full deflection
DEFAULT_ANGULAR_SCALE = 0.60   # rad/s at full deflection

# Gesture detection. The puck cross-talks — a firm push produces a little rotation
# too — so a binding needs a clear winner, not merely a maximum.
GESTURE_MIN = 0.35        # well clear of TwistReader's 0.06 deadzone: a deliberate push
GESTURE_DOMINANCE = 2.5   # the winner must beat the runner-up by this factor
GESTURE_HOLD_S = 0.15     # ...and hold it, on the same axis, for this long


class AxisMap:
    """A permutation-with-signs from six puck axes to six robot motions.

    Kept injective by construction: binding a puck axis that already drives
    something else **unbinds** the previous owner rather than letting one gesture
    drive two motions. That is always a mistake in a 6-DoF teleop, and a silent
    double-binding would present as "pitch also drifts when I go up", which is
    hours of debugging the IK for a config-file problem.
    """

    def __init__(
        self,
        source: list[int] | None = None,
        sign: list[int] | None = None,
        button_open: int | None = None,
        button_close: int | None = None,
    ):
        self.source = list(source) if source is not None else list(range(N))
        self.sign = list(sign) if sign is not None else [1] * N
        # ⭐ The two puck buttons, as raw HID bitmasks. Julien: *"there are two
        # buttons on the left and the right. One could be open, one could be
        # closed."*
        # ⛔ Stored as MASKS LEARNED BY PRESSING, never as "bit 0" and "bit 1".
        # Which physical button sets which bit has never been measured on this
        # unit, and the whole family of bugs this repo keeps hitting — the CAN
        # adapter by index, the puck by index, the gripper limits in the wrong
        # frame — is "assumed an identity that was never checked". Asking the
        # operator to press the button they mean costs two seconds and cannot be
        # wrong. Same idiom as pick_device_by_wiggle().
        self.button_open = int(button_open) if button_open else None
        self.button_close = int(button_close) if button_close else None
        self._validate()

    def _validate(self) -> None:
        if len(self.source) != N or len(self.sign) != N:
            raise ValueError(f"axis map must have {N} entries per field")
        self.source = [int(s) if s == UNBOUND or 0 <= int(s) < N else UNBOUND for s in self.source]
        self.sign = [1 if int(s) >= 0 else -1 for s in self.sign]

    # ---- persistence -------------------------------------------------------

    @classmethod
    def load(cls, path: Path) -> AxisMap:
        """Read a map, tolerating every older shape of the file.

        ⛔ Backward compatibility is not a nicety here — `config/spacemouse_map.json`
        currently holds `{"sign": [1, -1, -1, 1, 1, 1]}`, dialled in by hand on real
        hardware. A missing `source` therefore means the identity permutation, under
        which the new indexing is numerically identical to the old, so that file
        keeps behaving exactly as it did. Losing it would cost bench time to redo.
        """
        if not path.exists():
            return cls()
        try:
            raw: Any = json.loads(path.read_text())
        except Exception:  # noqa: BLE001
            return cls()
        if not isinstance(raw, dict):
            return cls()
        try:
            return cls(
                source=raw.get("source"), sign=raw.get("sign"),
                button_open=raw.get("button_open"), button_close=raw.get("button_close"),
            )
        except Exception:  # noqa: BLE001
            return cls()

    def as_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"source": self.source, "sign": self.sign}
        if self.button_open is not None:
            d["button_open"] = self.button_open
        if self.button_close is not None:
            d["button_close"] = self.button_close
        return d

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.as_dict(), indent=2) + "\n")

    def copy(self) -> AxisMap:
        return AxisMap(source=self.source, sign=self.sign,
                       button_open=self.button_open, button_close=self.button_close)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, AxisMap)
            and self.source == other.source
            and self.sign == other.sign
            and self.button_open == other.button_open
            and self.button_close == other.button_close
        )

    # ---- the mapping itself ------------------------------------------------

    def apply(self, axes: Any) -> np.ndarray:
        """Six puck axes in, six robot motions out. Still normalised — the caller
        applies the linear/angular scales, because that is a speed policy and this
        is a wiring diagram."""
        axes = np.asarray(axes, dtype=float)
        out = np.zeros(N)
        for i, src in enumerate(self.source):
            if src != UNBOUND:
                out[i] = axes[src] * self.sign[i]
        return out

    # ---- editing ----------------------------------------------------------

    def bind(self, motion: int, puck_axis: int, value: float) -> int | None:
        """Bind `motion` to `puck_axis`, with the sign taken from the gesture.

        `value` is what the puck actually read when the operator gestured. The sign
        is chosen so that **this gesture produces positive motion**, i.e. the thing
        the prompt asked for. Pull the puck backwards to mean "go up" and the sign
        comes out negative, which is correct and is the whole point.

        Returns the motion that lost this puck axis, or None.
        """
        displaced = None
        for other, src in enumerate(self.source):
            if other != motion and src == puck_axis:
                self.source[other] = UNBOUND
                displaced = other
        self.source[motion] = puck_axis
        self.sign[motion] = 1 if value > 0 else -1
        return displaced

    def swap(self, motion_a: int, motion_b: int) -> None:
        """Exchange the two motions' controls outright — puck axis AND direction.

        ⭐ Julien's request, 2026-08-10, after using CONTROLS mode on the arm:
        *"instead of only changing it to that specific thing and then just deleting
        the other one, that instead it would just swap whatever was on the other…
        if I have a roll on six and yaw on five, then when I'm on roll and I press
        five, it would also switch yaw to six. That will make it a lot easier."*

        He is right, and the reason is that `bind()`'s steal-and-unbind leaves the
        map **worse than before** for the commonest edit there is: two controls in
        each other's places. Stealing costs him two more actions — notice the
        orphan, then re-bind it — and in between, a motion silently does nothing.

        Two properties worth having, both free with a straight exchange:

        - **It is an involution.** Swapping the same pair again restores the
          original, so a mistaken swap is undone by repeating it. Nothing to
          remember and nothing to lose.
        - **Injectivity is preserved by construction** — exchanging two entries of
          a permutation is still a permutation, so this can never produce one
          gesture driving two motions.

        The *sign travels with the puck axis*, because the unit being exchanged is
        the whole control (which axis, pushed which way), not just the wiring. That
        keeps "push this axis that way" meaning the same thing before and after.
        """
        self.source[motion_a], self.source[motion_b] = self.source[motion_b], self.source[motion_a]
        self.sign[motion_a], self.sign[motion_b] = self.sign[motion_b], self.sign[motion_a]

    def swap_buttons(self) -> None:
        """Exchange which puck button opens and which closes.

        ⭐ Reached by the SAME `f` key that reverses an axis, and that is not a
        coincidence. Julien described it as *"pressing whatever the switch button
        was, I think f, could then switch it back"* — so `f` means one thing
        throughout: **reverse the control I just used.** If the last control was an
        axis, reversing it flips its sign; if it was a button, reversing it swaps
        open and close. One rule, no new vocabulary to remember.

        Also an involution, like `swap()`, so a wrong guess costs one keypress.
        """
        self.button_open, self.button_close = self.button_close, self.button_open

    def learn_button(self, action: str, mask: int) -> str | None:
        """Assign a pressed button to 'open' or 'close'. Returns a warning, or None.

        ⚠️ Refuses to give one physical button both jobs. A single button that both
        opens and closes is not a control, it is a coin flip, and the resulting
        "the gripper does random things" would be debugged in the gripper code.
        """
        if not mask:
            return "no button was pressed"
        other = self.button_close if action == "open" else self.button_open
        if other is not None and other == mask:
            return "that is already the other button — press the OTHER one"
        if action == "open":
            self.button_open = mask
        else:
            self.button_close = mask
        return None

    def button_action(self, buttons: int) -> str | None:
        """Which gripper action the currently-held buttons mean, if any."""
        if self.button_open and (buttons & self.button_open):
            return "open"
        if self.button_close and (buttons & self.button_close):
            return "close"
        return None

    def buttons_row(self) -> str:
        def fmt(mask: int | None) -> str:
            return "—— not set ——" if not mask else f"button 0x{mask:02x}"
        return f"  GRIPPER  open ← {fmt(self.button_open)}   close ← {fmt(self.button_close)}"

    def flip(self, motion: int) -> None:
        self.sign[motion] *= -1

    def unbind(self, motion: int) -> None:
        self.source[motion] = UNBOUND

    def unbound(self) -> list[int]:
        return [i for i, s in enumerate(self.source) if s == UNBOUND]

    def motion_driven_by(self, puck_axis: int) -> int | None:
        """Which motion this puck axis currently drives, or None.

        The reverse lookup exists for CONTROLS mode's flip key. Julien's design:
        *"move the space mouse in different directions and only the strongest
        direction is actually moved, and then I can press some key which reverses
        the direction of that specific control."* "That specific control" is the
        axis under his hand — so the flip needs to resolve from puck axis back to
        motion, which the row-oriented map does not do by construction.
        """
        for motion, src in enumerate(self.source):
            if src == puck_axis:
                return motion
        return None

    # ---- reporting --------------------------------------------------------

    # ⚠️ Every display method takes the FRAME, because the same component index is a
    # different physical direction in each one. Defaulting to "world" keeps older
    # callers correct rather than silently mislabelling them.

    def row(self, motion: int, frame: str = "world") -> str:
        src = self.source[motion]
        m = motions_for(frame)[motion]
        if src == UNBOUND:
            return f"{m['short']:>5}  ({m['world']:>13})  ←  —— nothing ——"
        direction = "+" if self.sign[motion] > 0 else "−"
        return f"{m['short']:>5}  ({m['world']:>13})  ←  puck {PUCK_AXES[src]:<5} {direction}"

    def describe(self, frame: str = "world") -> str:
        motions = motions_for(frame)
        lines = [f"  motion ({frame} frame)              driven by", "  " + "-" * 48]
        lines += ["  " + self.row(i, frame) for i in range(N)]
        missing = self.unbound()
        if missing:
            names = ", ".join(motions[i]["short"] for i in missing)
            lines.append(f"  ⚠️  UNBOUND (will not move): {names}")
        return "\n".join(lines)

    def explain(self, frame: str = "world") -> str:
        """The full picture: what each motion IS, and which gesture drives it.

        This is what CONTROLS mode shows on entry. `describe()` alone lists the
        wiring but never says what "FWD" or "UP" physically mean, which is useless
        the first time you enter a frame you have not driven before.
        """
        motions = motions_for(frame)
        lines = [f"  what the six controls mean in the {frame.upper()} frame:", ""]
        for i, m in enumerate(motions):
            lines.append(f"    {i + 1}  {m['short']:<5} {m['world']:<14} {m['note']}")
        lines += ["", self.describe(frame)]
        return "\n".join(lines)

    def one_line(self, frame: str = "world") -> str:
        motions = motions_for(frame)
        return " ".join(
            "—" if self.source[i] == UNBOUND
            else f"{motions[i]['short']}←{PUCK_AXES[self.source[i]]}{'+' if self.sign[i] > 0 else '−'}"
            for i in range(N)
        )

    def seeded_from_world(self) -> AxisMap:
        """A tool/camera-frame map derived from this world-frame one.

        See `WORLD_TO_TOOL` for the measured correspondence and why this is the right
        starting point: at the home pose the result feels **identical** to the world
        map, and only diverges as the wrist turns. So there is nothing to re-learn on
        day one, and the controls follow the camera the moment you tilt.

        Buttons are carried across unchanged — they open and close the gripper and
        have no direction to reinterpret.
        """
        src: list[int] = []
        sgn: list[int] = []
        for world_idx, mult in WORLD_TO_TOOL:
            src.append(self.source[world_idx])
            sgn.append(self.sign[world_idx] * mult)
        return AxisMap(source=src, sign=sgn,
                       button_open=self.button_open, button_close=self.button_close)


ISOLATE_HYSTERESIS = 1.3


def isolate(axes: Any, current: int | None = None, hysteresis: float = ISOLATE_HYSTERESIS
            ) -> tuple[int | None, float]:
    """Keep only the axis the operator is pushing hardest. Returns (axis, value).

    ⭐ THIS IS THE HEART OF CONTROLS MODE, and it is Julien's design rather than
    mine. His words: *"only the strongest direction is actually moved."* The arm
    then performs exactly one motion at a time, which makes it obvious which
    gesture caused it — and that is the whole difficulty with a SpaceMouse, because
    a firm push produces deflection on three or four axes at once and the resulting
    diagonal motion is unattributable.

    ⚠️ Note what this deliberately does NOT do: it never edits the map. Deflection
    observes, keys edit. The mode this replaces bound whichever motion was selected
    the instant any clear deflection arrived, and auto-advanced — so the natural act
    of *"let me see what this does"* silently rewrote the map. It destroyed Julien's
    hand-dialled file on 2026-08-10 (FINDINGS §11). Separating observation from
    editing is not a preference; it is the fix.

    `hysteresis` keeps the previously-chosen axis until another beats it by that
    factor, so two near-equal axes cannot flicker and make the arm jitter between
    two motions. Unlike `dominant_axis`, this does not refuse an ambiguous reading:
    for *driving*, argmax is deterministic and zeroing the rest already isolates it.
    Refusing would just make the arm feel dead.
    """
    a = np.asarray(axes, dtype=float)
    if a.size != N:
        return None, 0.0
    mag = np.abs(a)
    if float(mag.max()) <= 0.0:
        return None, 0.0
    best = int(np.argmax(mag))
    if (current is not None and current != best and 0 <= current < N
            and mag[current] > 0.0 and mag[best] < mag[current] * hysteresis):
        best = current
    return best, float(a[best])


def isolated_axes(axes: Any, keep: int | None) -> np.ndarray:
    """`axes` with everything except `keep` zeroed."""
    out = np.zeros(N)
    a = np.asarray(axes, dtype=float)
    if keep is not None and 0 <= keep < N:
        out[keep] = a[keep]
    return out


def dominant_axis(axes: Any) -> tuple[int, float] | None:
    """The one axis the operator is clearly pushing, or None if it is ambiguous.

    ⛔ Returns None rather than a best guess. A wrong binding here is silent: the
    map looks plausible, the arm moves along an axis nobody asked for, and the
    natural conclusion is that the IK is broken. Refusing an unclear gesture costs
    one repeat; guessing costs a debugging session.
    """
    a = np.abs(np.asarray(axes, dtype=float))
    if a.size != N:
        return None
    order = np.argsort(a)[::-1]
    best, runner = int(order[0]), int(order[1])
    if a[best] < GESTURE_MIN:
        return None
    if a[runner] * GESTURE_DOMINANCE > a[best]:
        return None
    return best, float(np.asarray(axes, dtype=float)[best])


class GestureDetector:
    """Requires the same dominant axis for `GESTURE_HOLD_S` before it commits.

    A single 10 ms sample is not a gesture: the puck passes through other axes on
    the way to the one you meant. Holding removes that without asking the operator
    to be careful.
    """

    def __init__(self, hold_s: float = GESTURE_HOLD_S):
        self.hold_s = hold_s
        self._axis: int | None = None
        self._value = 0.0
        self._since: float | None = None

    def reset(self) -> None:
        self._axis = None
        self._since = None

    def feed(self, axes: Any, now: float) -> tuple[int, float] | None:
        """Returns (axis, value) once a gesture has been held long enough."""
        hit = dominant_axis(axes)
        if hit is None:
            self.reset()
            return None
        axis, value = hit
        if axis != self._axis:
            self._axis, self._value, self._since = axis, value, now
            return None
        # Keep the largest deflection seen, so the sign comes from the firm part
        # of the push rather than from whatever it happened to be at timeout.
        if abs(value) > abs(self._value):
            self._value = value
        if self._since is not None and now - self._since >= self.hold_s:
            out = (axis, self._value)
            self.reset()
            return out
        return None


def ambiguity_note(axes: Any) -> str | None:
    """Why a gesture was not accepted, in the operator's terms."""
    a = np.abs(np.asarray(axes, dtype=float))
    if a.size != N or float(a.max()) < 0.02:
        return None
    order = np.argsort(a)[::-1]
    best, runner = int(order[0]), int(order[1])
    if a[best] < GESTURE_MIN:
        return f"too gentle: strongest is {PUCK_AXES[best]} {a[best]:.2f}, need {GESTURE_MIN:.2f}"
    if a[runner] * GESTURE_DOMINANCE > a[best]:
        return (
            f"ambiguous: {PUCK_AXES[best]} {a[best]:.2f} vs {PUCK_AXES[runner]} {a[runner]:.2f}"
            " — move ONE way only"
        )
    return None


class AxisMapStore:
    """Control maps, keyed by **scope** (shared or a single arm) and by **frame**.

    Two independent reasons a map has to be duplicated, and they compose:

    **Per arm.** Julien: *"maybe one of the directions might want to be different for
    the arms… Probably the same, actually. But maybe that should be options to map
    them separately."* So the default is one map for both, and an override is created
    only when explicitly asked for. A map per arm from the start would let the two
    silently diverge, after which a puck that feels wrong on G is indistinguishable
    from a map that was never copied across.

    **Per frame.** Julien, 2026-08-11: *"the controls in the v mode … need to be able
    to edit with the m mode."* A map says which puck axis drives twist component 0 —
    but component 0 is "out from the base" in the world frame and "down in the
    picture" in the tool frame. The same wiring therefore produces two different
    feels, and tuning one necessarily mis-tunes the other. They must be separate.

    ⛔ **Buttons are NOT per frame.** They open and close the gripper; there is no
    direction in them to reinterpret. They live on the scope and are mirrored onto
    whichever map is handed out, so setting them in one frame sets them in all.

    ⛔ **Whatever reads this must state which scope it is editing.** Tuning G while
    still on the shared map silently changes B — the same shape as the bug that
    destroyed the hand-dialled map, an edit whose blast radius exceeded what the
    operator believed.

    On disk::

        {"shared": {"buttons": {"open": 2, "close": 1},
                    "frames": {"world":  {"source": [...], "sign": [...]},
                               "tool":   {"source": [...], "sign": [...]}}},
         "G": { ... optional, same shape ... }}

    ⚠️ Both older shapes still load. A flat `{"sign": [...]}` and the intermediate
    `{"shared": {"source": ..., "sign": ..., "button_open": ...}}` are both read as
    the **world** map, with tool/camera seeded from it — so nothing hand-dialled on
    hardware is ever lost to a format change.
    """

    SHARED = "shared"
    FRAMES = ("world", "tool", "camera")

    def __init__(self, scopes: dict[str, dict[str, AxisMap]] | None = None,
                 buttons: dict[str, tuple[int | None, int | None]] | None = None):
        self._scopes: dict[str, dict[str, AxisMap]] = scopes or {self.SHARED: {}}
        self._buttons: dict[str, tuple[int | None, int | None]] = buttons or {}
        self._scopes.setdefault(self.SHARED, {})

    # ---- construction ------------------------------------------------------

    @staticmethod
    def _map_from(d: Any) -> AxisMap:
        if not isinstance(d, dict):
            return AxisMap()
        try:
            return AxisMap(source=d.get("source"), sign=d.get("sign"))
        except Exception:  # noqa: BLE001
            return AxisMap()

    @classmethod
    def _scope_from(cls, d: Any) -> tuple[dict[str, AxisMap], tuple[int | None, int | None]]:
        """Read one scope, accepting every historical shape."""
        if not isinstance(d, dict):
            return {}, (None, None)
        if "frames" in d and isinstance(d["frames"], dict):
            frames = {f: cls._map_from(m) for f, m in d["frames"].items() if f in cls.FRAMES}
            b = d.get("buttons") or {}
            return frames, (b.get("open"), b.get("close"))
        # Older shape: the scope IS the world map, buttons inline.
        return {"world": cls._map_from(d)}, (d.get("button_open"), d.get("button_close"))

    @classmethod
    def load(cls, path: Path) -> AxisMapStore:
        if not path.exists():
            return cls()
        try:
            raw: Any = json.loads(path.read_text())
        except Exception:  # noqa: BLE001
            return cls()
        if not isinstance(raw, dict):
            return cls()
        scopes: dict[str, dict[str, AxisMap]] = {}
        buttons: dict[str, tuple[int | None, int | None]] = {}
        if "source" in raw or "sign" in raw:
            # Oldest shape: the whole FILE is the world map.
            frames, b = cls._scope_from(raw)
            scopes[cls.SHARED] = frames
            buttons[cls.SHARED] = b
        else:
            for key, val in raw.items():
                if not isinstance(val, dict):
                    continue
                frames, b = cls._scope_from(val)
                scopes[key] = frames
                buttons[key] = b
        scopes.setdefault(cls.SHARED, {})
        return cls(scopes=scopes, buttons=buttons)

    def save(self, path: Path) -> None:
        data: dict[str, Any] = {}
        for scope in sorted(self._scopes, key=lambda k: (k != self.SHARED, k)):
            frames = self._scopes[scope]
            entry: dict[str, Any] = {
                "frames": {f: {"source": m.source, "sign": m.sign} for f, m in sorted(frames.items())}
            }
            b_open, b_close = self._buttons.get(scope, (None, None))
            if b_open is not None or b_close is not None:
                entry["buttons"] = {"open": b_open, "close": b_close}
            data[scope] = entry
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2) + "\n")

    # ---- which map applies -------------------------------------------------

    def _scope_for(self, arm: str) -> str:
        return arm if arm in self._scopes and arm != self.SHARED else self.SHARED

    def for_arm(self, arm: str, frame: str = "world") -> AxisMap:
        """The map this arm uses in this frame, with the scope's buttons attached.

        A frame that has never been tuned is **seeded from the world map** rather
        than started blank — see `AxisMap.seeded_from_world()` for why that makes the
        tool frame feel identical to world at the home pose.
        """
        scope = self._scope_for(arm)
        frames = self._scopes.setdefault(scope, {})
        if frame not in frames:
            world = frames.get("world") or AxisMap()
            frames[frame] = world.copy() if frame == "world" else world.seeded_from_world()
        m = frames[frame]
        m.button_open, m.button_close = self._buttons.get(scope, (None, None))
        return m

    def is_shared(self, arm: str) -> bool:
        return arm not in self._scopes or arm == self.SHARED

    def scope_note(self, arm: str) -> str:
        if self.is_shared(arm):
            return "SHARED — edits here affect BOTH arms"
        return f"{arm} ONLY — arm-specific, the other arm keeps the shared map"

    # ---- editing -----------------------------------------------------------

    def set(self, arm: str, m: AxisMap, frame: str = "world") -> None:
        scope = self._scope_for(arm)
        self._scopes.setdefault(scope, {})[frame] = m
        # Buttons belong to the scope, not to one frame, so lift them back out.
        self._buttons[scope] = (m.button_open, m.button_close)

    def fork(self, arm: str) -> None:
        """Give `arm` its own maps, copied from whatever it uses today."""
        if arm in self._scopes and arm != self.SHARED:
            return
        src = self._scopes.get(self.SHARED, {})
        self._scopes[arm] = {f: m.copy() for f, m in src.items()}
        self._buttons[arm] = self._buttons.get(self.SHARED, (None, None))

    def unfork(self, arm: str) -> None:
        if arm != self.SHARED:
            self._scopes.pop(arm, None)
            self._buttons.pop(arm, None)

    def tuned_frames(self, arm: str) -> list[str]:
        """Which frames this arm has an explicit map for. Useful for reporting."""
        return sorted(self._scopes.get(self._scope_for(arm), {}))

    def _wiring(self) -> dict[str, dict[str, tuple[list[int], list[int]]]]:
        """Just the source/sign per scope per frame.

        ⚠️ Deliberately excludes the button fields on the AxisMap objects.
        `for_arm()` stamps the scope's buttons onto the map it hands back, as a
        convenience for callers — so those fields are a transient view, not state
        the scope owns, and comparing them made a store unequal to its own reloaded
        self depending on which arms had been read.
        """
        return {sc: {f: (list(m.source), list(m.sign)) for f, m in fr.items()}
                for sc, fr in self._scopes.items()}

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AxisMapStore):
            return False
        live = lambda b: {k: v for k, v in b.items() if v != (None, None)}  # noqa: E731
        return self._wiring() == other._wiring() and live(self._buttons) == live(other._buttons)

    def copy(self) -> AxisMapStore:
        return AxisMapStore(
            scopes={s: {f: m.copy() for f, m in fr.items()} for s, fr in self._scopes.items()},
            buttons=dict(self._buttons),
        )


def axes_readout(axes: Any) -> str:
    """A live readout of all six puck axes, for teaching and for verifying.

    The operator has to be able to see what the device reports, because the
    alternative is trusting a label — and no gesture-to-axis label in this repo has
    ever been measured. Zero axes print as `·` so the ones in play stand out.
    """
    parts = []
    for name, v in zip(PUCK_AXES, np.asarray(axes, dtype=float)):
        parts.append(f"{name} {'·' * 5}" if v == 0.0 else f"{name} {v:+.2f}")
    return "  ".join(parts)
