"""One arm follows the other, joint for joint.

⭐ Julien's idea, 2026-08-11: *"be able to move one of the arms in the guide mode and
have the second arm just mirror the exact movements with zero latency."*

**Why this is the right first two-arm feature, ahead of bimanual teleop.** It needs
no inverse kinematics, no second SpaceMouse, and nothing cartesian at all — the
leader's *measured* joint angles are commanded straight to the follower. So it
exercises the whole two-arm process (two CAN buses, two robots, one 100 Hz loop, one
shutdown path) while every genuinely risky part is absent. That makes it the natural
shakedown for the `ArmSession` refactor rather than a reward for finishing it.

COPY vs MIRROR — and why COPY is the default
---------------------------------------------
Julien, asked directly: *"Both options, mirroring and copying should be possible, but
standard or unchanged default should be copy. They are currently next to each
other."*

Two arms **side by side** face the same way, so the follower should reproduce the
leader's angles unchanged — that is `copy`. Two arms **facing each other** are
reflected, and reproducing a motion then means negating the joints that turn about
the mirror plane — that is `mirror`. Getting this wrong does not damage anything, but
the follower moves opposite to expectation, which is alarming the first time.

⛔ THE HAZARD, AND IT IS THE WHOLE DESIGN PROBLEM
--------------------------------------------------
**The two arms will not be in the same pose when you start.** Commanding the follower
straight to the leader's angles would make it *jump* across that gap at whatever
speed the rate limiter allows — a large, fast, unasked-for motion, which is precisely
the class of event this codebase exists to prevent.

So engagement is staged, and `following` is never entered until the gap is closed:

    ALIGNING  -> the follower ramps to the leader's current pose at a bounded speed,
                 using the same trajectory helper PARK uses (already tested)
    FOLLOWING -> the gap is small; the follower now tracks the leader continuously

⚠️ And the leader must be **held still during ALIGNING**. If it is being hand-guided
while the follower is still catching up, the target keeps moving and the gap may
never close — so alignment reports its progress and gives up rather than chasing
forever, exactly like PARK's stall detector.
"""

from __future__ import annotations

from typing import Any

import numpy as np

N_ARM = 6

# ⭐ Which joints get negated in `mirror`. Joints that rotate about the vertical or
# twist the wrist reverse when the arms face each other; the pitching joints, which
# move in the sagittal plane, do not.
#
# ⚠️ MEASURED FROM GEOMETRY, NOT VERIFIED ON HARDWARE. From FINDINGS §1 the joints
# are: 1 base_yaw, 2 shoulder_pitch, 3 elbow_pitch, 4 forearm_pitch, 5 wrist_roll,
# 6 gripper_twist. Reflecting through a vertical plane between two facing arms
# negates the yaw and the two roll/twist joints and leaves the three pitches alone.
# **This is a prediction. Try it with the arms clear and expect to adjust it.**
MIRROR_SIGNS = np.array([-1.0, 1.0, 1.0, 1.0, -1.0, -1.0])

DEFAULT_ALIGN_SPEED = 0.30      # rad/s per joint while closing the initial gap
# ⭐ The follower is rate-limited while FOLLOWING too, not only while aligning.
# 1.0 rad/s matches SafeRobot's own cap, so this never becomes the binding limit —
# anything faster would simply be clipped one layer down. See `step()` for why a
# single limit throughout matters.
DEFAULT_FOLLOW_SPEED = 1.0
DEFAULT_ENGAGE_TOLERANCE = 0.05  # rad — below this, following starts
DEFAULT_MAX_GAP = 0.35          # rad — above this while following, stop: something is wrong


def follower_target(leader_q: Any, mode: str = "copy") -> np.ndarray:
    """The follower's joint targets for the leader's measured pose.

    `copy` reproduces the angles unchanged — correct for arms standing side by side.
    `mirror` negates the joints that reverse under reflection — for arms that face
    each other. Only the six arm joints are touched; the gripper is deliberately not
    mirrored, because a gripper has no handedness and copying its opening is what
    anybody would expect.
    """
    q = np.asarray(leader_q, dtype=float)
    out = q.copy()
    n = min(N_ARM, len(q))
    if mode == "mirror":
        out[:n] = q[:n] * MIRROR_SIGNS[:n]
    elif mode != "copy":
        raise ValueError(f"mode must be 'copy' or 'mirror', got {mode!r}")
    return out


def gap(follower_q: Any, target_q: Any) -> float:
    """Worst per-joint disagreement, in radians. The number engagement is gated on."""
    f = np.asarray(follower_q, dtype=float)
    t = np.asarray(target_q, dtype=float)
    n = min(len(f), len(t), N_ARM)
    return float(np.max(np.abs(t[:n] - f[:n]))) if n else 0.0


class MirrorLink:
    """Tracks the two-stage engagement and produces one command per cycle.

    ⛔ Deliberately has **no** robot handle, no CAN, and no I/O of any kind. It takes
    measured positions in and returns a command out, so the whole engagement
    behaviour — the part that could produce a sudden motion — is testable without an
    arm. Everything in this repo that skipped that step failed on first contact with
    the hardware.
    """

    def __init__(
        self,
        mode: str = "copy",
        align_speed: float = DEFAULT_ALIGN_SPEED,
        follow_speed: float = DEFAULT_FOLLOW_SPEED,
        engage_tolerance: float = DEFAULT_ENGAGE_TOLERANCE,
        max_gap: float = DEFAULT_MAX_GAP,
    ):
        if mode not in ("copy", "mirror"):
            raise ValueError(f"mode must be 'copy' or 'mirror', got {mode!r}")
        self.mode = mode
        self.align_speed = align_speed
        self.follow_speed = follow_speed
        self.engage_tolerance = engage_tolerance
        self.max_gap = max_gap
        self.state = "aligning"
        self.command: np.ndarray | None = None
        self.stop_reason: str | None = None

    def step(self, leader_q: Any, follower_q: Any, dt: float) -> np.ndarray | None:
        """One cycle. Returns the follower's command, or None if it must not move.

        None means "send nothing this cycle" and is returned only once the link has
        stopped — never as a transient. A follower that is sometimes commanded and
        sometimes not would drift under its own 400 ms timeout.
        """
        target = follower_target(leader_q, self.mode)
        measured = np.asarray(follower_q, dtype=float)
        if self.command is None:
            self.command = measured.copy()

        g = gap(measured, target)

        if self.state == "stopped":
            return None

        if self.state == "following" and g > self.max_gap:
            # ⛔ Stop rather than chase. A gap this large while following means the
            # follower is not tracking — blocked, at a joint limit, or faulted — and
            # continuing would keep commanding a position it cannot reach, which is
            # how a motor ends up held against a stop.
            self.state = "stopped"
            self.stop_reason = (
                f"follower fell {g:.3f} rad behind the leader (limit {self.max_gap}). "
                "It is blocked, at a joint limit, or faulted."
            )
            return None

        if self.state == "aligning" and g <= self.engage_tolerance:
            self.state = "following"

        # ⛔⭐ ONE RATE LIMIT, APPLIED IN BOTH STATES — and this is the fix for a real
        # defect the tests caught. An earlier version ramped only while aligning and
        # then, on the cycle it switched to following, assigned the target directly.
        # That handover was itself a jump of up to `engage_tolerance`: 0.05 rad in a
        # single cycle is **5 rad/s**, seventeen times the align speed it had just
        # been so careful to respect. A discontinuity hidden exactly at the moment a
        # guard hands over is the shape of bug this whole file exists to avoid.
        #
        # Limiting both states removes the transition entirely: the command is a
        # continuous trajectory throughout, and only the allowed rate changes.
        #
        # ⚠️ `self.command` keeps its OWN length. Taking the target's length instead
        # silently dropped the gripper element when a 7-DoF follower tracked a 6-DoF
        # leader — the same length-mismatch class that dropped a raised arm once.
        rate = self.align_speed if self.state == "aligning" else self.follow_speed
        step = rate * dt
        n = min(len(self.command), len(target))
        delta = target[:n] - self.command[:n]
        self.command[:n] = self.command[:n] + np.clip(delta, -step, step)
        return self.command

    def status(self, leader_q: Any, follower_q: Any) -> str:
        g = gap(follower_q, follower_target(leader_q, self.mode))
        if self.state == "aligning":
            return f"ALIGNING — {g:.3f} rad to close, following starts under {self.engage_tolerance}"
        if self.state == "following":
            return f"FOLLOWING ({self.mode}) — tracking {g:.3f} rad behind"
        return f"STOPPED — {self.stop_reason}"
