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
never close — so alignment reports its progress and gives up after
`ALIGN_STALL_SECONDS` of no improvement, the same patience PARK's stall detector uses.

⛔ **That last sentence was FALSE from 2026-08-11 until 2026-08-14.** It described an
intention; the gap check only ever ran in the `following` state, so an alignment
really could chase forever. It is implemented now. **A design note written in the
present tense reads afterwards as a description of the code**, and nothing raises.
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

#: ⛔⭐⭐⭐ PER-JOINT GAP MULTIPLIERS, BECAUSE ONE THRESHOLD FOR ALL SIX JOINTS IS WRONG.
#:
#: `max_gap` asks "has the follower lost the plot?", and losing the plot means the arm is
#: swinging somewhere it should not be. **How far the arm actually swings for one radian of
#: error is wildly different per joint**, measured on the shipped model across four poses
#: taken from Julien's own 2026-08-17 logs:
#:
#: | joint | | tip metres per radian | how dangerous |
#: |---|---|---|---|
#: | 1 | base_yaw | 0.333 | ⛔ high |
#: | 2 | shoulder_pitch | 0.390 | ⛔ high |
#: | 3 | elbow_pitch | 0.418 | ⛔ **the worst** |
#: | 4 | forearm_pitch | 0.169 | ⚠️ half |
#: | 5 | wrist_roll | 0.100 | ⭐ a quarter |
#: | 6 | gripper_twist | 0.051 | ⭐ **an eighth** |
#:
#: ⛔ **This is why he had to keep raising `--mirror-gap`.** Every stop in his logs was on
#: joint 5 or joint 6 — the two that barely move the tip — and the only way to tolerate a
#: flicked wrist was to raise the threshold for the shoulder too. On 2026-08-17 he reached
#: `--mirror-gap 1.335`, which allows the ELBOW to lag by 1.3 rad. At 0.418 m/rad that is
#: **56 cm of tip error**, on a limit whose whole purpose is noticing when the arm has gone
#: somewhere wrong.
#:
#: ⭐ Scaled as `1 / sensitivity`, normalised to the worst joint, and **capped at 4x**.
#:
#: ⚠️⚠️ THE CAP IS THERE FOR A REASON THE MEASUREMENT DOES NOT CAPTURE. Tip POSITION is the
#: right basis for a danger limit, and it is the wrong basis for task accuracy: 1.4 rad on the
#: gripper twist is the gripper rotated **80° from where it should be**, which ruins a grasp
#: while moving the tip almost nowhere. So the wrist gets more rope than the shoulder, and not
#: as much more as pure tip-displacement would allow.
GAP_WEIGHTS = (1.26, 1.07, 1.00, 2.48, 4.00, 4.00)

#: ⛔⭐⭐⭐ HOW FAST THE STANDING OFFSET IS CORRECTED, in 1/seconds. **0.0 means OFF.**
#:
#: Julien, 2026-08-17: *"I can move the mirrored robot about in a maybe two centimetre
#: diameter sphere around the position it should actually be at… when I try to pick up
#: something from the table, sometimes my guiding robot is already moving into the table
#: whilst my mirror robot isn't even far enough down to pick up the object. Why is it not
#: millimetre perfect?"*
#:
#: ⭐⭐ THE ANSWER, AND IT IS BOTH A MOTOR PROPERTY AND A SOFTWARE OMISSION.
#:
#: The follower is position-controlled: its motors push toward the commanded angle with a
#: force proportional to how far away they are. So it settles where that force balances
#: gravity and friction, which is always SHORT of the command. That residual is the constant
#: term in this repo's own measured law (docs/ROADMAP.md §8.2 item 11): **0.04 to 0.10 rad of
#: error even at zero speed.** Stiffness is a motor and gain property and no command changes
#: it.
#:
#: ⛔ **What was missing in software is that nothing ever noticed.** `follower_target()`
#: copies the leader's measured angles and the command converges to exactly that, so the
#: follower ends up at `leader − droop` forever and no part of the loop reads the difference
#: back. **But the difference is measured every single cycle**, which is what makes it
#: correctable above the SDK.
#:
#: ⭐ So: accumulate the remaining error into a small bias and add it to the command, until
#: the follower actually arrives. That is an integral term, and a standing offset under
#: constant load is exactly what integral action is for.
#:
#: ⚠️ Measured on his own hardware: **0.024 rad of joint error is 11 mm at the tip** in the
#: extended pose his log shows him reaching with, so his 2 cm sphere is precisely what those
#: numbers predict rather than an impression.
DEFAULT_CATCHUP = 0.0

#: ⛔⭐⭐ THE HARD CLAMP ON THAT BIAS, in radians. **This is the safety property of the whole
#: idea**, and it is small on purpose.
#:
#: An integral term winds up: if the follower is BLOCKED, the error never closes, the bias
#: grows without limit, and the moment the block clears the arm lurches by however much it
#: accumulated. That is the same family as the stale cached variable that snapped this arm on
#: 2026-08-10 (`SafeRobot`'s docstring).
#:
#: ⭐ 0.06 rad is about 25 mm at the tip, which comfortably covers a droop measured at 0.012
#: to 0.024 rad while being far too small to throw anything. ⚠️ It also sits well under
#: `SafeRobot`'s 0.25 rad following-error clip, so the bias can never be the thing that trips
#: that limit.
DEFAULT_MAX_BIAS = 0.06

#: ⭐ Only correct while the leader is moving slower than this, in rad/s. **This is what
#: makes the term target the STANDING offset rather than the moving one.**
#:
#: While the leader is moving, part of the error is honest dynamic lag that will disappear on
#: its own, and integrating it would push the follower past the leader on every stop. ⭐ And
#: the case Julien actually cares about — lining the gripper up with something on the table —
#: is slow by nature, so a slow-only correction helps exactly when it matters and does
#: nothing during a fast sweep.
DEFAULT_CATCHUP_BELOW = 0.25
#: ⭐ Below this the follower is not moving at all, so a gap means it is blocked rather than
#: slow. Same order as the gripper stall threshold, and for the same reason: a velocity this
#: small is indistinguishable from encoder noise.
STUCK_SPEED = 0.05              # rad/s
#: ⭐ How long ALIGNING may make no progress before it gives up, in seconds. Same patience as
#: PARK's stall detector, and for the same reason: an alignment that cannot converge is a
#: leader that keeps moving, or a follower that cannot move, and neither improves by waiting.
ALIGN_STALL_SECONDS = 4.0


def pick_pair(arm_names: list[str], selected: list[str]) -> tuple[str, str]:
    """Which arm leads and which follows. Returns `(leader, follower)`.

        pick_pair(["B", "G"], ["B"])   ->  ("B", "G")
        pick_pair(["B", "G"], ["G"])   ->  ("G", "B")

    ⭐ THE SELECTED ARM LEADS, because the operator selects the arm they are about to put
    their hands on. Everything else in the session already works that way: `a` aims the
    mode keys at the arm you are about to change.

    ⛔ REFUSES WHEN BOTH ARE SELECTED, rather than picking the first. "Both arms lead" has
    no meaning, and guessing would engage a motion on whichever arm happened to be first
    in `--arms` — a moving arm chosen by the order of a command-line flag.

    Raises `ValueError` with a message written for the person at the keyboard; the caller
    prints it as a hint.
    """
    if len(arm_names) != 2:
        raise ValueError(
            f"mirror needs exactly two arms and this session has {len(arm_names)}. "
            "Start it with --arms B,G.")
    if len(selected) != 1:
        raise ValueError(
            "select ONE arm to lead, with a. That arm is the one you hand-guide; the "
            "other one follows it.")
    leader = selected[0]
    if leader not in arm_names:
        raise ValueError(f"arm {leader} is not in this session ({', '.join(arm_names)})")
    follower = next(name for name in arm_names if name != leader)
    return leader, follower


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


def worst_scaled_joint(follower_q: Any, target_q: Any,
                       max_gap: float) -> tuple[int, float, float]:
    """`(joint index, its gap, its own limit)` for the joint closest to its OWN threshold.

    ⭐ "Closest to its own threshold" rather than "largest gap", which are different questions
    once the thresholds differ. A 0.9 rad wrist error against a 1.4 rad wrist limit is further
    from stopping than a 0.4 rad elbow error against a 0.35 rad elbow limit, even though the
    wrist number is bigger. Reporting the largest raw gap would name the wrong joint.
    """
    f = np.asarray(follower_q, dtype=float)
    t = np.asarray(target_q, dtype=float)
    n = min(len(f), len(t), N_ARM, len(GAP_WEIGHTS))
    if not n:
        return (0, 0.0, max_gap)
    worst, worst_ratio = 0, -1.0
    for j in range(n):
        limit = max_gap * GAP_WEIGHTS[j]
        ratio = abs(t[j] - f[j]) / limit if limit > 0 else float("inf")
        if ratio > worst_ratio:
            worst, worst_ratio = j, ratio
    return (worst, float(abs(t[worst] - f[worst])), max_gap * GAP_WEIGHTS[worst])


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
        align_stall_seconds: float = ALIGN_STALL_SECONDS,
        catchup: float = DEFAULT_CATCHUP,
        max_bias: float = DEFAULT_MAX_BIAS,
        catchup_below: float = DEFAULT_CATCHUP_BELOW,
    ):
        if mode not in ("copy", "mirror"):
            raise ValueError(f"mode must be 'copy' or 'mirror', got {mode!r}")
        self.mode = mode
        self.align_speed = align_speed
        self.follow_speed = follow_speed
        self.engage_tolerance = engage_tolerance
        self.max_gap = max_gap
        self.align_stall_seconds = align_stall_seconds
        self.catchup = catchup
        self.max_bias = max_bias
        self.catchup_below = catchup_below
        #: The accumulated per-joint correction. ⛔ Reset whenever the link (re)starts, because
        #: carrying it across an engagement is carrying stale control state, which is the exact
        #: class of bug that snapped this arm once.
        self.bias: np.ndarray | None = None
        self.state = "aligning"
        self.command: np.ndarray | None = None
        self.stop_reason: str | None = None
        #: ⭐ The measured explanation, kept SEPARATE from `stop_reason` so the caller can put
        #: them on two lines. One long line gets truncated by `StatusLine.say()` when a live
        #: block is on screen, and Julien's first high-speed run lost the end of the sentence
        #: to an ellipsis — the half that named the cause.
        self.stop_detail: str | None = None
        # ⭐⭐ WHAT THE STOP MEASURED, so the message can name a cause instead of listing
        # three. `stop_joint` is the index of the joint that opened the gap, and
        # `stop_leader_speed` is how fast the LEADER was moving that joint when it happened.
        #
        # ⛔ WHY THIS EXISTS. On 2026-08-14 the first mirror run stopped twice, and the
        # message read *"It is blocked, at a joint limit, or faulted."* **None of those was
        # true.** Julien was hand-guiding the leader's wrist faster than the follower is
        # allowed to move, so the gap opened on one joint. The message named three causes and
        # missed the only one that had occurred — the same defect as the speed throttle that
        # blamed the reach limit at a comfortable pose ([FINDINGS §41.2](../docs/FINDINGS.md)).
        self.stop_joint: int | None = None
        self.stop_gap: float = 0.0
        self.stop_leader_speed: float | None = None
        #: ⭐⭐ THE FOLLOWER'S OWN MEASURED SPEED, and it is what turns two possible causes
        #: into three distinguishable ones. Added 2026-08-14 after the message got it wrong a
        #: second time: at `--max-speed 5` it said *"blocked, at a joint limit, or faulted"*
        #: and Julien's answer was *"the robot was never blocked by anything. It just, like,
        #: didn't kind of catch up at high speeds."* He was right, and the reason is one layer
        #: down: `SafeRobot` clips every command to **0.25 rad from the measured position**, so
        #: the follower's command can never run further ahead than that however high
        #: `max_speed` goes. Past a certain leader speed the follower is tracking as hard as
        #: it can and STILL losing ground, which is neither of the causes the message named.
        self.stop_follower_speed: float | None = None
        self.stop_cause: str | None = None      # "follow_limit" · "tracking" · "stuck"
        #: ⛔⭐⭐ ALIGNING NOW GIVES UP, AND THE DOCSTRING CLAIMED IT ALREADY DID. The module
        #: header has said since 2026-08-11 that *"alignment reports its progress and gives up
        #: rather than chasing forever, exactly like PARK's stall detector"* — and the gap check
        #: only ever ran in the `following` state. So a leader that kept moving during ALIGNING
        #: had the follower chasing it indefinitely, with nothing to stop it. **The sentence was
        #: written as a design intention and read afterwards as a description**, which is the
        #: [FINDINGS §0](../docs/FINDINGS.md) shape: a confident, plausible, wrong statement
        #: that raised nothing.
        self._align_best = float("inf")
        self._align_since = 0.0
        self._elapsed = 0.0
        self._prev_leader: np.ndarray | None = None
        self._prev_follower: np.ndarray | None = None
        self._leader_speed = np.zeros(N_ARM)
        self._follower_speed = np.zeros(N_ARM)

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
            self.bias = np.zeros(len(measured))

        # ⭐ How fast the leader is moving each joint, smoothed a little so one noisy sample
        # cannot decide the diagnosis. Measured here because it is the number that tells a
        # follower falling behind (the leader is faster than the limit) apart from a follower
        # that cannot move (blocked, at a limit, faulted).
        lead = np.asarray(leader_q, dtype=float)
        if self._prev_leader is not None and dt > 0:
            n_lead = min(N_ARM, len(lead), len(self._prev_leader))
            fresh = np.abs(lead[:n_lead] - self._prev_leader[:n_lead]) / dt
            self._leader_speed[:n_lead] += 0.3 * (fresh - self._leader_speed[:n_lead])
        self._prev_leader = lead.copy()
        # ⭐ The follower's own speed, measured the same way. A follower moving nearly as fast
        # as the leader and still losing ground is at its PHYSICAL limit; one barely moving is
        # blocked. Without this the two look identical from the gap alone.
        if self._prev_follower is not None and dt > 0:
            n_f = min(N_ARM, len(measured), len(self._prev_follower))
            fresh_f = np.abs(measured[:n_f] - self._prev_follower[:n_f]) / dt
            self._follower_speed[:n_f] += 0.3 * (fresh_f - self._follower_speed[:n_f])
        self._prev_follower = measured.copy()

        g = gap(measured, target)

        if self.state == "stopped":
            return None

        # ⭐⭐ PER-JOINT, so a flicked wrist no longer forces the shoulder's limit up with it.
        j_worst, j_gap, j_limit = worst_scaled_joint(measured, target, self.max_gap)

        # ⛔⭐⭐⭐ A STUCK JOINT IS STOPPED ON THE **UNSCALED** LIMIT, AND THIS IS A SAFETY
        # CONSEQUENCE OF THE PER-JOINT SCALING THAT I DID NOT THINK THROUGH FIRST TIME.
        #
        # The scaling exists to tolerate a joint that is LAGGING, and joint 4's 2.48x
        # multiplier is right for that. ⛔ **But it also gave a joint that is not moving at
        # all 2.48x longer to be pushed.** Julien's 2026-08-17 log is the evidence: joint 4
        # stopped at 0.869 rad against its scaled 0.87 limit, having moved **0.01 rad/s**,
        # with `SafeRobot` clipping the command on **1115 cycles** — about 12 seconds of a
        # motor working against something that would not move. The session's hottest readings
        # were 45 and 46 °C, the highest yet.
        #
        # ⭐ The two cases want opposite treatment and the code already tells them apart by
        # the follower's measured speed. So: **lagging gets the generous scaled limit,
        # stalled gets the tight unscaled one.** Tolerating lag was never meant to mean
        # tolerating a stall.
        stalled = (self.state == "following"
                   and j_gap > self.max_gap
                   and float(self._follower_speed[j_worst]) < STUCK_SPEED)

        if self.state == "following" and (j_gap > j_limit or stalled):
            # ⛔ Stop rather than chase. A gap this large while following means the follower
            # is not keeping up, and continuing would keep commanding a position it cannot
            # reach, which is how a motor ends up held against a stop.
            #
            # ⭐⭐ THE MESSAGE NAMES WHAT WAS MEASURED. It reports which joint opened the gap
            # and how fast the leader was moving it, then says which of the two explanations
            # the numbers support. It used to list three possible causes and, on its first
            # real run, all three were wrong.
            self.state = "stopped"
            # ⭐ The joint that actually FIRED, from the per-joint check, rather than the one
            # with the largest raw gap. Those differ once the thresholds differ, and naming
            # the wrong joint is how a diagnosis sends somebody after the wrong flag.
            worst = j_worst
            self.stop_joint = worst
            self.stop_gap = j_gap
            self.stop_leader_speed = float(self._leader_speed[worst])
            self.stop_follower_speed = float(self._follower_speed[worst])
            # ⭐⭐ THREE CAUSES, EACH MEASURED, and the order is the order of certainty.
            #
            # ⛔ A follower that is not moving is blocked whatever the leader was doing, so
            # that check comes first. Then the software follow limit, which is a number this
            # code owns. Only what is left over is the physical one, and calling it that is a
            # claim about the hardware, so it is the last resort rather than the default.
            if self.stop_follower_speed < STUCK_SPEED:
                self.stop_cause = "stuck"
                why = (f"the follower barely moved that joint "
                       f"({self.stop_follower_speed:.2f} rad/s), so it is blocked, at a "
                       "joint limit, or faulted")
            elif self.stop_leader_speed > self.follow_speed:
                self.stop_cause = "follow_limit"
                why = (f"the leader moved it at {self.stop_leader_speed:.2f} rad/s and the "
                       f"follower may only move at {self.follow_speed:.2f}, so it could not "
                       "keep up")
            else:
                self.stop_cause = "tracking"
                why = (f"the leader moved it at {self.stop_leader_speed:.2f} rad/s and the "
                       f"follower managed {self.stop_follower_speed:.2f}, inside its "
                       f"{self.follow_speed:.2f} allowance — so the ARM itself could not "
                       "track that fast, not the software")
            # ⭐⭐ THE JOINT'S OWN LIMIT, and where it came from. Without the multiplier the
            # reader sees "limit 1.40" while `--mirror-gap` says 0.35 and cannot reconcile
            # them, which is exactly the confusion the old single-number message caused in the
            # other direction.
            mult = GAP_WEIGHTS[worst] if worst < len(GAP_WEIGHTS) else 1.0
            if stalled and j_gap <= j_limit:
                # ⭐ Say which limit fired, because "0.42 behind, limit 0.87" reads as a bug.
                self.stop_reason = (
                    f"the follower fell {j_gap:.3f} rad behind on joint {worst + 1} and is "
                    f"NOT MOVING, so it stopped at the tight {self.max_gap:.2f} limit rather "
                    f"than that joint's usual {j_limit:.2f}")
            else:
                scaled = (f" = {self.max_gap:.2f} × {mult:.2f}"
                          if abs(mult - 1.0) > 0.01 else "")
                self.stop_reason = (f"the follower fell {j_gap:.3f} rad behind on joint "
                                    f"{worst + 1} (its limit is {j_limit:.2f}{scaled})")
            self.stop_detail = why
            return None

        # ⛔ ALIGNING gives up rather than chasing. Progress means the gap actually closed;
        # `align_stall_seconds` of no improvement means it never will, because the leader is
        # still being moved or the follower cannot move.
        self._elapsed += dt
        if g < self._align_best - 0.003:
            self._align_best, self._align_since = g, self._elapsed
        if (self.state == "aligning"
                and self._elapsed - self._align_since > self.align_stall_seconds):
            self.state = "stopped"
            self.stop_cause = "align_stalled"
            self.stop_gap = g
            self.stop_reason = (f"the follower could not close the {g:.3f} rad gap to the "
                                f"leader (needs under {self.engage_tolerance})")
            self.stop_detail = (f"no progress for {self.align_stall_seconds:g}s — hold the "
                                "leader STILL while the follower catches up, or check that "
                                "the follower can move")
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
        # ⭐⭐ THE CATCH-UP TERM. Aim slightly PAST the leader so the follower's own droop is
        # taken out. See `DEFAULT_CATCHUP` for why the droop exists and why it is correctable.
        #
        # ⛔ FOUR GUARDS, and each one prevents a specific way this could misbehave:
        #   1. `following` only — during ALIGNING the gap is large by design and the bias
        #      would wind up to its clamp instantly.
        #   2. slow leader only — a moving leader's error is partly honest lag, and
        #      integrating that would push the follower PAST the leader every time it stopped.
        #   3. hard clamp — a blocked follower never closes its error, so without this the
        #      bias grows forever and the arm lurches when the block clears.
        #   4. reset on (re)engage — carrying control state across an engagement is the same
        #      class of bug as the stale `prev_q` that snapped this arm on 2026-08-10.
        aim = target
        if self.catchup > 0.0 and self.state == "following" and self.bias is not None:
            n_b = min(len(self.bias), len(target), len(measured), N_ARM)
            slow = self._leader_speed[:n_b] < self.catchup_below
            err = target[:n_b] - measured[:n_b]
            # ⛔⭐⭐ A FAST LEADER DECAYS THE BIAS RATHER THAN FREEZING IT, and a test found
            # why that matters. The leader's speed estimate is smoothed, so at the START of a
            # fast motion it still reads slow for a few cycles and a little bias accumulates
            # (measured: 0.0011 rad, about 0.5 mm at the tip — harmless once).
            #
            # ⚠️ But a frozen bias never gives that back. **Every slow-to-fast transition
            # would add a little more**, and over a long session of reaching and sweeping it
            # could creep to the 0.06 rad clamp for a reason nobody chose — 25 mm of the
            # follower deliberately sitting past the leader.
            #
            # ⭐ Decaying at the same rate it accumulates makes the term self-correcting:
            # spurious bias bleeds away, and only a sustained slow-and-short condition holds
            # one. That is the difference between an integrator with leak and one without.
            self.bias[:n_b] = np.clip(
                np.where(slow,
                         self.bias[:n_b] + err * self.catchup * dt,
                         self.bias[:n_b] * max(0.0, 1.0 - self.catchup * dt)),
                -self.max_bias, self.max_bias)
            aim = target.copy()
            aim[:n_b] = target[:n_b] + self.bias[:n_b]

        rate = self.align_speed if self.state == "aligning" else self.follow_speed
        step = rate * dt
        n = min(len(self.command), len(aim))
        delta = aim[:n] - self.command[:n]
        self.command[:n] = self.command[:n] + np.clip(delta, -step, step)
        return self.command

    def worst_bias(self) -> float:
        """The largest correction currently applied, in radians. For the status row."""
        return 0.0 if self.bias is None else float(np.max(np.abs(self.bias)))

    def status(self, leader_q: Any, follower_q: Any) -> str:
        target = follower_target(leader_q, self.mode)
        g = gap(follower_q, target)
        if self.state == "aligning":
            return f"ALIGNING — {g:.3f} rad to close, following starts under {self.engage_tolerance}"
        if self.state == "following":
            # ⭐ A WARNING BEFORE IT TRIPS, not only after. Julien's first mirror run stopped
            # twice with no notice: the row said "tracking 0.34 rad behind" one second and
            # the link was gone the next. Past 70% of the limit the row says so, which is
            # enough time to slow the hand down.
            # ⭐ Against the joint's OWN limit, or the row would warn about a wrist that has
            # three quarters of its rope left while staying quiet about an elbow near its own.
            j_w, j_g, j_lim = worst_scaled_joint(follower_q, target, self.max_gap)
            room = (f" ⚠️ joint {j_w + 1} near its {j_lim:.2f} limit"
                    if j_lim > 0 and j_g > 0.7 * j_lim else "")
            # ⭐⭐ SHOW THE CORRECTION WHEN IT IS DOING SOMETHING. Julien's complaint was that
            # the follower sits in a ~2 cm sphere, and this row is where he reads that back.
            # A correction he cannot see is one he cannot tell apart from no correction.
            # ⛔⭐ RADIANS, AND THE FIRST VERSION OF THIS LINE INVENTED MILLIMETRES. It read
            # `worst_bias() * 1000` and called the result "mm-equivalent", which is simply
            # wrong: 0.024 rad is **11 mm** at the tip in a typical pose, not 24. The
            # radians-to-millimetres conversion depends on which joint and on the arm's
            # current pose, so there is no constant to multiply by.
            #
            # ⚠️ A fabricated unit in a status row is worse than no figure at all, because it
            # reads as a measurement. Same family as every "guessing message" this repo has
            # had to replace (docs/FINDINGS.md §58.0).
            fix = ""
            if self.catchup > 0.0:
                fix = f", correcting {self.worst_bias():.3f} rad"
            return f"FOLLOWING ({self.mode}) — tracking {g:.3f} rad behind{fix}{room}"
        return f"STOPPED — {self.stop_reason}"
