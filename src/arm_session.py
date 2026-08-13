"""⭐ ONE ARM'S STATE AND MODE MACHINE, so that N of them can run in one loop.

    from arm_session import ArmSession
    arm = ArmSession(robot, name="B", frame="world")
    arm.enter_teleop()
    verdict = arm.step_park(t=1.0, dt=0.01)

⛔ WHY THIS EXISTS — the blocker for bimanual, stated exactly. `teleop_session.py`
is single-arm all the way through: `robot`, `teleop`, `mode`, `gripper_value`,
`prev_q`, `home_ee`, `park_target`, `guide_ref`, `park_cmd` and the rest are one
arm's state held in **one function's locals**. Two arms cannot exist in that shape,
and ROADMAP step 6 is unambiguous about the alternative: extract, then run N of
them, so single-arm and bimanual are the same code with N=1 or N=2.

⛔ **Why an extraction and not a second `teleop_bimanual.py`.** Duplication has bitten
this repo four times: `src/spacemouse.py` exists because device logic was
copy-pasted and a fix landed in only one copy; the simulator's `twist_from_axes()`
ignored the axis map for the same reason; PARK went around the gripper clamp
because the clamp lived only in the teleop branch; and the quit path carried a
second park loop until 2026-08-12. A second control loop would be the fifth — and
it would be the one driving two arms at once.

⭐⭐ THE DESIGN RULE THAT MAKES THIS TESTABLE: **the class decides, the script
narrates.** No method here prints. They return verdicts and messages, and the
caller displays them. That is the same shape as `ThermalGuard` and
`park_verdict()`, and it is why the agent — which may never touch the hardware —
can still prove the mode machine behaves.

⚠️ WHAT THIS DELIBERATELY DOES **NOT** OWN, and why:

- **Building the robot.** `build_robot()` energises motors and is the single most
  dangerous call in the project; it stays visible in the script, and this class
  takes an already-built handle. That also lets every test below run against a fake.
- **Reading the SpaceMouse.** One puck belongs to one arm, but the device layer and
  its wiggle-assignment already exist and are shared.
- **Key handling.** Which arm a keypress applies to is a *session* question, not an
  arm question — ROADMAP step 6 decides it (`a` selects; driving always applies to
  all arms; mode changes apply to the selected one).
- **IK stepping.** `CartesianTeleop` already owns that; this holds one and calls it.

⚠️ **STATUS, 2026-08-13: built, unit-tested, brought up to date, and STILL NOT wired
into `teleop_session.py`.** That wiring is a separate, reviewable step, deliberately not
done in the same session as writing this. Session 4 is the standing warning — three
changes that passed 34 tests, three dry runs and a simulated IK loop produced three
failures on first hardware contact, one of which dropped 4.3 kg. When it is wired,
ROADMAP §6.1 says how: `--arms B` runs the N-arm code with **N=1** first, so the
restructure is verified against a feel Julien already knows, separately from the
two-arm risk.

⛔⭐⭐ **AND A WARNING WORTH MORE THAN THE CLASS ITSELF: THIS FILE WENT STALE IN ONE
HOUR.** It was committed 2026-08-12 at 14:16 with a park built from a queue of legs and
a per-leg speed ramp. At **15:15 the same day** `teleop_session.py` replaced exactly that
with a single blended `JointPath`, and the commit message says the earlier version *"was
the wrong thing"*. This class then sat for a day modelling a design the script no longer
had, **with all 17 of its tests passing the whole time**, because the tests asserted the
superseded behaviour. It was found by auditing before the restructure rather than by
anything failing.

⚠️ **So: an unwired class is a copy of a design, and a copy drifts.** Whenever
`teleop_session.py` changes how an arm behaves, this file is the second place that change
has to land, and nothing enforces it. **The fix is to finish the wiring** — after that
there is one copy. Until then, diff this against the script's park before trusting it.

⛔ **What this deliberately does NOT own, decided 2026-08-13: recording and playback.**
They look like per-arm state and they are not. `amazon-far/abc` wants 14 states and 14
actions per timestep, **two arms in ONE timeline** (ROADMAP §9.2), so a recorder owned by
an arm cannot produce the target format at all. One session-level recorder samples every
arm each cycle, and one playback cursor drives them all — splitting the cursor per arm
would let the arms drift apart in time, which is the one thing a bimanual demonstration
must not do. Migration map: ROADMAP §6.1.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from motion import EASINGS, Easing, JointPath, easing_factor
from yam_robot import (
    ThermalGuard,
    motor_temperatures,
    park_target_from,
    park_verdict,
)

N_ARM = 6

#: Defaults copied from `teleop_session.py` rather than imported from it. ⛔ A library
#: must not import the script that uses it, and these are the values the script has been
#: tuned against on hardware. They are constructor arguments so a caller can override
#: them, and the script passes its own live knobs in.
PARK_SPEED = 0.40           # rad/s along the path
PARK_RAMP = 0.20            # how much of the move is eased
PARK_BLEND = 0.15           # corner radius, "smooth"
PARK_TOLERANCE = 0.02       # rad — "arrived"
PARK_SETTLED = 0.06         # rad — "as close as it holds itself under load"
PARK_SETTLE_SECONDS = 0.5
PARK_STALL_SECONDS = 4.0
PARK_PROGRESS_EPS = 0.003   # rad of improvement that still counts as progress
MAX_CURSOR_LAG = 0.15       # rad — past this the cursor waits for the arm


@dataclass
class ParkLeg:
    """One waypoint in a run: its slot name and the pose to reach."""

    name: str
    pose: list


@dataclass(frozen=True)
class ParkStep:
    """Everything the caller needs to narrate one cycle of a park. Nothing is printed.

    ⭐ Every field exists because `teleop_session.py` prints it today. Returning them
    instead of printing is the whole reason this class can be proven without an arm.
    """

    verdict: str                 # moving · arrived · settled · blocked
    err: float                   # distance from the FINAL target, MEASURED
    lag: float                   # how far the arm trails the commanded point
    remaining: float             # rad of path still ahead of the cursor
    leg_passed: str | None       # a waypoint the cursor reached on THIS cycle
    next_leg: str | None         # the waypoint after it, for "→ next 3"
    leg_seconds: float           # since the previous waypoint was passed
    total_seconds: float         # since the whole park began
    settling_seconds: float      # since the cursor finished the path


class ArmSession:
    """Everything that is true of **one** arm during a session.

    Modes are the same five the script has always had — `guide`, `teleop`, `hold`,
    `park`, `map` — because they are Julien's mental model and renaming them would
    make every note in FINDINGS harder to follow.
    """

    def __init__(self, robot: Any, name: str, frame: str = "world",
                 gripper_min: float = 0.02, gripper_max: float = 0.98,
                 warn_at: float = 55.0, stop_at: float = 65.0) -> None:
        self.robot = robot
        self.name = name
        self.frame = frame
        self.mode = "hold"

        self.gripper_min, self.gripper_max = gripper_min, gripper_max
        self.gripper_value = 0.0
        self.stall_since: float | None = None

        self.teleop: Any = None
        self.home_ee: Any = None
        self.prev_q = np.zeros(N_ARM)
        self.guide_ref: np.ndarray | None = None

        # ⛔⭐ THE PARK IS ONE BLENDED PATH WITH A CURSOR ALONG IT, and this replaced a
        # queue of separate legs on 2026-08-13. The earlier model drove to each waypoint
        # and stopped dead, which is the thing Julien explicitly did not ask for:
        # *"instead of moving and then jittering ninety degrees to the next side, in a
        # smooth curve it would go to the next point."* `teleop_session.py` changed to
        # `JointPath` on 2026-08-12 at 15:15, one hour after this class was written, and
        # the class was left behind for a day. Audit: ROADMAP §6.1.
        self.park_path: JointPath | None = None
        self.park_s = 0.0                       # arc-length cursor along the path
        self.park_marks: list[tuple[str, float]] = []   # waypoint name → arc length
        self.park_target: np.ndarray | None = None      # the FINAL pose of the run
        self.park_cmd: np.ndarray | None = None
        self.park_best_err = float("inf")
        self.park_progress_t = 0.0
        # ⛔ TWO CLOCKS. `park_leg_t` resets at every waypoint so each leg reports its own
        # duration; `park_start_t` never resets so the arrival line can report the whole
        # park. Sharing one variable printed "PARK reached in 0.0s" after a 4.4 s park,
        # because the last waypoint is passed at the very end. FINDINGS §34.3.
        self.park_leg_t = 0.0
        self.park_start_t = 0.0

        # Live knobs. The script owns the keys that change them; this owns the motion.
        self.park_speed = PARK_SPEED
        self.park_ramp = PARK_RAMP
        self.blend = PARK_BLEND
        self.easing: Easing = EASINGS[3]        # "both", the script's default
        self._smooth = True                     # the caller's --no-smooth, per run

        self.thermal = ThermalGuard(warn_at=warn_at, stop_at=stop_at)

    # ---------------------------------------------------------- liveness ----

    def alive(self) -> bool:
        """Is this arm still actually being commanded?

        ⛔ The single most important check. I2RT's control thread raises and exits on
        a motor fault and tells nobody; without this the loop commands a corpse while
        printing healthy numbers, which it did for 64 seconds on 2026-08-10.

        ⚠️ With N arms this becomes per-arm, and ROADMAP step 6 already ruled on what
        it means: **a fault on one arm stops BOTH.** A chain death on B must not leave
        G uncommanded and sagging.
        """
        chain = getattr(self.robot, "motor_chain", None)
        return bool(chain is not None and getattr(chain, "running", False))

    def read_thermal(self):  # noqa: ANN201
        """One thermal cycle. Returns the guard's verdict; `None` states = blind."""
        chain = getattr(self.robot, "motor_chain", None)
        try:
            states = chain.read_states()
        except Exception:  # noqa: BLE001
            states = None
        if states is None:
            self.stall_since = None
            return self.thermal.update(None), None, None
        temps, hottest, jaw = motor_temperatures(states, N_ARM)
        motor = temps.index(hottest) if hottest is not None else None
        return self.thermal.update(hottest, jaw, motor=motor), hottest, jaw

    # ------------------------------------------------------------- modes ----

    def clamp_gripper(self, value: float) -> float:
        return float(np.clip(value, self.gripper_min, self.gripper_max))

    def resync(self) -> None:
        """⛔ Re-anchor every cached variable to the measured pose.

        A mode change must re-read reality. Never carry cached state across one —
        `prev_q` surviving a hand-guide is what made the arm snap back to a pose from
        minutes earlier the first time GUIDE → TELEOP was tried.
        """
        self.prev_q = np.asarray(self.robot.get_joint_pos(), dtype=float)[:N_ARM]
        if hasattr(self.robot, "resync"):
            self.robot.resync()

    def enter_hold(self) -> None:
        self.resync()
        self.robot.command_joint_pos(np.asarray(self.robot.get_joint_pos(), dtype=float))
        self.mode = "hold"

    def enter_teleop(self, teleop_factory=None) -> None:  # noqa: ANN001
        """Leave zero-gravity and take the jaws exactly where they are.

        ⛔ Do NOT clamp the gripper here. Clamping on entry is a *command to move*,
        and nobody asked for that — an earlier version did, and if the jaws happened
        to sit outside the band the session drove them the moment teleop began.
        """
        self.resync()
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        self.robot.command_joint_pos(q)
        self.gripper_value = float(q[N_ARM]) if len(q) > N_ARM else 0.5
        if teleop_factory is not None:
            self.teleop = teleop_factory(self.frame)
            self.teleop.reset(q[:N_ARM])
            self.home_ee = self.teleop.ee_position().copy()
        self.mode = "teleop"

    def enter_guide(self) -> str | None:
        """Go weightless. Returns a warning string if the API is missing.

        ⛔⭐ UNDERSTAND WHAT THIS RESTS ON. Zero-gravity sets **kp = 0**, so the
        computed gravity compensation is the ONLY thing holding 4.3 kg up — there is
        no position term to absorb an error. Any shortfall in the model is an
        unopposed torque, which is how the arm fell on 2026-08-10. `guide_ref` is
        recorded here precisely so drift is measurable while it happens.
        """
        self.resync()
        self.guide_ref = np.asarray(self.robot.get_joint_pos(), dtype=float)
        self.mode = "guide"
        fn = getattr(self.robot, "enter_gravity_comp_idle", None)
        if callable(fn):
            fn()
            return None
        return "enter_gravity_comp_idle() missing — staying in HOLD (NOT weightless)"

    def guide_drift(self) -> float | None:
        """How far the arm has sunk since it went weightless, in radians."""
        if self.guide_ref is None:
            return None
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        return float(np.max(np.abs(q[:N_ARM] - self.guide_ref[:N_ARM])))

    # -------------------------------------------------------------- park ----

    def begin_path(self, legs: list[ParkLeg], t: float, smooth: bool = True) -> list[str]:
        """Start ONE continuous motion through every leg. Returns any target warnings.

        ⛔ Every waypoint goes through `park_target_from`, so the gripper clamp and the
        6-versus-7-joint reconciliation apply to all of them. A length mismatch on one
        leg once raised mid-park and dropped the arm (FINDINGS §11), and that path
        reaches every leg here, not only the first.

        ⚠️ `smooth=False` is the caller's `--no-smooth`: the path is still blended, and
        only the easing ramp is switched off. Blending is the *shape*; easing is the
        *speed along it*. They are independent axes and Julien wants both adjustable.
        """
        warnings: list[str] = []
        targets = []
        for leg in legs:
            target, warn = park_target_from(self.robot.get_joint_pos(), leg.pose,
                                            gripper_index=N_ARM, clamp=self.clamp_gripper)
            if warn:
                warnings.append(warn)
            targets.append(target)
        start = np.asarray(self.robot.get_joint_pos(), dtype=float)
        self.park_path = JointPath([start, *targets], blend=self.blend)
        self.park_marks = list(zip([leg.name for leg in legs],
                                   self.park_path.arrival_lengths()[1:]))
        self.park_s = 0.0
        self.park_target = targets[-1]
        self.park_cmd = start.copy()
        self.enter_hold()
        self.mode = "park"
        self._smooth = smooth
        self.park_best_err = float(np.max(np.abs(self.park_target - start)))
        self.park_progress_t = t
        self.park_leg_t = t
        self.park_start_t = t
        return warnings

    def step_path(self, t: float, dt: float,
                  tolerance: float = PARK_TOLERANCE,
                  settled: float = PARK_SETTLED,
                  stall_seconds: float = PARK_STALL_SECONDS,
                  settle_seconds: float = PARK_SETTLE_SECONDS,
                  progress_eps: float = PARK_PROGRESS_EPS,
                  max_cursor_lag: float = MAX_CURSOR_LAG) -> ParkStep:
        """Advance the park by one control cycle and report what happened.

        ⛔ Completion is judged from the **measured** pose, never from the command. The
        command always arrives first, so testing it would declare success while the arm
        was still travelling. That was a real bug and it hid for two sessions.

        ⛔⭐ ARRIVAL IS GATED ON THE CURSOR REACHING THE END OF THE PATH, not on the
        error alone. A run like `p 1 2 1` finishes where it started, so the distance to
        the final target is small at t=0 as well — judging on that would declare the
        whole sequence complete before the arm had moved at all.

        ⭐ The cursor waits when the arm falls behind. The trajectory is a *shape*, and a
        command racing ahead while the arm cuts its own corner is not the shape anyone
        chose. Progress means "the cursor moved OR the arm closed the gap": without the
        first half a legitimately slow leg looks stalled, and without the second an arm
        pinned against something never does.
        """
        blocked = ParkStep("blocked", float("inf"), 0.0, 0.0, None, None,
                           t - self.park_leg_t, t - self.park_start_t,
                           t - self.park_leg_t)
        if self.park_path is None or self.park_target is None or self.park_cmd is None:
            return blocked

        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        err = float(np.max(np.abs(self.park_target - q)))
        lag = float(np.max(np.abs(self.park_cmd - q)))
        length = self.park_path.length

        def result(verdict: str, leg_passed: str | None = None,
                   next_leg: str | None = None) -> ParkStep:
            return ParkStep(verdict, err, lag, max(0.0, length - self.park_s),
                            leg_passed, next_leg, t - self.park_leg_t,
                            t - self.park_start_t, t - self.park_leg_t)

        if self.park_s < length:
            advanced = False
            if lag < max_cursor_lag:
                ramp = easing_factor(self.easing, self.park_s, length - self.park_s,
                                     self.park_ramp if self._smooth else 0.0)
                self.park_s = min(length, self.park_s + self.park_speed * ramp * dt)
                advanced = True
            self.park_cmd = self.park_path.point_at(self.park_s)
            self.robot.command_joint_pos(self.park_cmd)

            if advanced or err < self.park_best_err - progress_eps:
                self.park_best_err = min(self.park_best_err, err)
                self.park_progress_t = t
            if t - self.park_progress_t > stall_seconds:
                return result("blocked")
            if self.park_marks and self.park_s >= self.park_marks[0][1]:
                name, _ = self.park_marks.pop(0)
                step = result("moving", leg_passed=name,
                              next_leg=self.park_marks[0][0] if self.park_marks else None)
                self.park_leg_t = t
                return step
            return result("moving")

        verdict = park_verdict(err, t - self.park_progress_t > stall_seconds,
                               tolerance, settled,
                               stopped_briefly=t - self.park_progress_t > settle_seconds)
        return result(verdict)

    def abandon_path(self) -> float:
        """⛔ Leaving PARK abandons the rest of the run, and returns the rad dropped.

        An arm that resumes a queued trajectory after the operator pressed HOLD is doing
        something nobody asked for. Returning the distance rather than a waypoint count
        is deliberate: with one blended path there are no separate legs left to count,
        and "1.8 rad of path abandoned" is what an operator can actually picture.
        """
        left = 0.0 if self.park_path is None else max(0.0, self.park_path.length - self.park_s)
        self.park_path, self.park_marks = None, []
        self.park_s = 0.0
        return left
