"""Per-arm state and mode transitions for the shared operator loop.

ArmSession owns one robot's mode, motion path, teleop solver, puck reader, axis
map, saved poses, and thermal/gripper guards. ArmSelector chooses which sessions
receive a keypress; every arm continues stepping each control cycle. Methods
return results for the application to display rather than printing themselves.

The application owns device acquisition and shutdown. Recording and replay use
one shared timeline for every arm and therefore remain session-level concerns.
ArmSession.step_path executes the same path logic for one or multiple arms.

Both arms and the shared recorder were integrated and exercised on hardware in
the August walkthrough. The history of this extraction, including the earlier
unwired copy that drifted while its tests passed, is in FINDINGS sections 50-54.
Keep tests attached to the implementation the application actually calls.
"""

from __future__ import annotations

#: ⛔⭐⭐ HOW FAR THE JAWS MUST OPEN BEFORE A LATCHED BLOCK IS RELEASED, in normalised jaw
#: units where 0 is closed and 1 is fully open. **3% of full travel.**
#:
#: ⚠️ Chosen to sit far above sensor jitter and far below any deliberate movement. His
#: gripper-step default is 0.02 per keypress and the puck-button rate moves faster than that,
#: so one intentional open press already clears it while a wandering measurement never will.
JAW_CLEAR_MARGIN = 0.03

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

import numpy as np

from yam.motion import EASINGS, Easing, JointPath, easing_factor, plan_gripper_stops
from yam.robot import (
    GraspCheck,
    ThermalGuard,
    check_grasp,
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

#: ⛔ Pushing hard while not moving is the definition of a stall, and stall is the worst
#: thermal case there is: full current, no motion, no cooling. Motor 7 was cooked three
#: times before this guard existed. Values copied from `teleop_session.py`.
GRIPPER_STALL_TORQUE = 1.0    # Nm
GRIPPER_STALL_VEL = 0.05      # rad/s
GRIPPER_STALL_SECONDS = 0.4

#: ⭐⭐ THE JAW PAUSE (ROADMAP §6.6.2, items 3 + 10). A run splits wherever only the jaws
#: move, the arm holds at the split, and the run resumes when the jaws are DONE — which is
#: measured, never timed, because "how long the jaws take" is not a preference. A jaw that
#: stalls on an object counts as done: stopped-on-the-object IS the grab succeeding.
#:
#: ⚠️ All four are in NORMALISED jaw units (0 closed, 1 open) and seconds, because the pause
#: watches `get_joint_pos()[6]`, which is normalised. The raw-rad stall constants above watch
#: a different instrument (torque + raw velocity from the chain read) and stay separate.
JAW_STILL_RATE = 0.3        #: below this, in stroke/s, a cycle counts as "the jaws are still"
JAW_SETTLE_SECONDS = 0.35   #: the jaws must be still this long before the run resumes
JAW_MIN_WAIT = 0.25         #: never resume before this — pre-command stillness must not count
JAW_TIMEOUT_SECONDS = 3.0   #: a jaw still moving after this stops gating the run (it is said, not hidden)


def parse_arms(single: str | None, spec: str | None,
               known: Iterable[str], default: str) -> list[str]:
    """Return ordered arm names from --arm/--arms, or the default when absent.

    Conflicting flags, unknown names, empty entries and duplicate arms raise
    ValueError. Duplicates would create competing controllers on one CAN bus.
    """
    valid = list(known)
    if single is not None and single not in valid:
        raise ValueError(f"unknown arm {single!r} — this rig has {', '.join(valid)}")

    if spec is None:
        return [single or default]

    names = [part.strip() for part in spec.split(",")]
    if any(not name for name in names):
        raise ValueError(
            f"--arms {spec!r} has an empty entry — write it as B, G or B,G")
    unknown = [name for name in names if name not in valid]
    if unknown:
        raise ValueError(f"unknown arm(s) {', '.join(unknown)} — this rig has "
                         f"{', '.join(valid)}")
    seen = [name for i, name in enumerate(names) if name in names[:i]]
    if seen:
        raise ValueError(
            f"--arms {spec!r} names {', '.join(sorted(set(seen)))} more than once. "
            "One arm is one CAN bus, and two sessions of it would command the same "
            "motors twice a cycle with different cached state")
    if single is not None and names != [single]:
        raise ValueError(
            f"--arm {single} and --arms {spec} disagree. Pass one of them: --arm is the "
            "one-arm spelling, --arms takes the list")
    return names


class ArmSelector:
    """Select the arm(s) targeted by mode keys and edits.

    For two arms, cycle B → G → BOTH; one arm keeps its only label. Each arm
    normally follows its own puck. SessionInput also uses this selection to
    route a single shared puck; selecting BOTH deliberately drives both.
    """

    BOTH = "BOTH"

    def __init__(self, names: list[str]) -> None:
        if not names:
            raise ValueError("a session needs at least one arm")
        self._names = list(names)
        #: ⭐ Every arm, then BOTH — so the first press moves to the *other* arm rather
        #: than to BOTH. Aiming at one arm is the safe end of this cycle, so the cheapest
        #: presses stay there.
        self._targets = [*self._names] + ([self.BOTH] if len(self._names) > 1 else [])
        self._at = 0

    @property
    def label(self) -> str:
        """What the status row shows: an arm's name, or `BOTH`."""
        return self._targets[self._at]

    def cycle(self) -> str:
        """Advance to the next target and return its label."""
        self._at = (self._at + 1) % len(self._targets)
        return self.label

    def names(self) -> list[str]:
        """The arms a mode key applies to right now."""
        if self.label == self.BOTH:
            return list(self._names)
        return [self.label]

    def only_one(self) -> bool:
        """True when there is nothing to select between, so `a` has nothing to do."""
        return len(self._targets) == 1


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

    verdict: str                 # moving · jaws · arrived · settled · blocked
    err: float                   # ARM distance from the FINAL target, MEASURED (jaw excluded)
    lag: float                   # how far the ARM trails the commanded point (jaw excluded)
    remaining: float             # rad of path still ahead of the cursor, queued segments included
    leg_passed: str | None       # a waypoint the cursor reached on THIS cycle
    next_leg: str | None         # the waypoint after it, for "→ next 3"
    leg_seconds: float           # since the previous waypoint was passed
    total_seconds: float         # since the whole park began
    settling_seconds: float      # since the cursor finished the path

    # ⭐ The jaw pause (verdict "jaws"): the run is split at a waypoint where only the jaws
    # move, the arm holds, and these narrate the wait. All defaulted so the original
    # nine-field constructions above stay valid.
    jaw_name: str | None = None      # the waypoint whose jaw value is being waited on
    jaw_target: float | None = None  # the commanded (clamped) jaw value, 0 closed to 1 open
    jaw_seconds: float = 0.0         # how long the pause has been running
    jaw_started: bool = False        # True exactly once, on the pause's first cycle
    jaw_done: bool = False           # True exactly once, on the cycle the run resumes
    jaw_timed_out: bool = False      # the pause ended by timeout, not by measured stillness
    grasp: GraspCheck | None = None  # on jaw_done: did the jaws close on something? (item 10)
    jaw_arm_off: float | None = None # on jaw_started: how far the ARM settled from the split
                                     # waypoint before the jaws were allowed to move — the
                                     # number that diagnoses a missed grab (friction floor
                                     # vs a badly taught pose)


class ArmSession:
    """Own one arm's measurements, mode state, map, jaw latch and park execution.

    Controls-mode state lives here; key dispatch and selection live in the
    operator and UI handlers. SessionInput fills puck state. SessionHealth
    uses the thermal/stall methods and reports their decisions. Recording,
    playback and mirror coordination span arms and have session owners.

    The operator still applies per-mode commands and teleop clamps. SafeRobot
    applies its command limits beneath those modes; CartesianTeleop owns the
    workspace constraint. Device lifetimes belong to SessionResources.
    """

    def __init__(self, robot: Any, name: str, frame: str = "world",
                 gripper_min: float = 0.02, gripper_max: float = 0.98,
                 warn_at: float = 55.0, stop_at: float = 65.0,
                 axis_map: Any = None,
                 slots: dict[str, list] | None = None,
                 base_slot: str = "default",
                 reader: Any = None) -> None:
        self.robot = robot
        self.name = name
        self.frame = frame
        self.mode = "hold"

        # Copy the startup map at construction; controls revert and the closing report use it.
        self.axis_map = axis_map
        self.axis_map_at_start = axis_map.copy() if axis_map is not None else None

        # The base pose is the release destination; ordinary task waypoints must not replace it.
        # None means no saved base; the operator then seeds it from the startup pose.
        self.slots: dict[str, list] = dict(slots or {})
        self.base_slot = base_slot
        self.base_pose: list | None = self.slots.get(base_slot)

        # Keep the assigned reader, not handle ownership. SessionResources closes every
        # acquired handle, including when arm construction fails.
        self.reader = reader
        # ⭐ THIS CYCLE'S PUCK DEFLECTION, six axes in [-1, 1], read once per cycle and used
        # by the mode action and by the CONTROLS readout. Per arm because the reader is: two
        # arms are two different hands, and one session-level copy would drive both arms from
        # whichever puck happened to be read last.
        self.raw_axes: list[float] = [0.0] * 6

        # Remember the last axis without a timeout so edits work after the puck centers.
        # last_input_kind disambiguates axis/button edits; buttons_prev detects rising edges.
        self.last_active_axis: int | None = None
        self.last_active_value = 0.0
        self.last_input_kind: str | None = None     # None | "axis" | "button"
        self.learn_button: str | None = None        # None | "open" | "close"
        self.buttons_prev = 0

        self.gripper_min, self.gripper_max = gripper_min, gripper_max
        self.gripper_value = 0.0
        self.stall_since: float | None = None
        # Track consecutive stalls and throttle repeat reports; a latch prevents immediate re-pushing.
        self.stall_count = 0
        self.stall_last_said = 0.0
        self._states: Any = None        # this cycle's chain read, for the stall guard

        self.teleop: Any = None
        self.home_ee: Any = None
        self.prev_q = np.zeros(N_ARM)
        self.guide_ref: np.ndarray | None = None

        # One blended park path with an arc-length cursor; waypoint marks support progress reports.
        self.park_path: JointPath | None = None
        self.park_s = 0.0                       # arc-length cursor along the path
        self.park_marks: list[tuple[str, float]] = []   # waypoint name → arc length
        self.park_target: np.ndarray | None = None      # the FINAL pose of the run
        self.park_cmd: np.ndarray | None = None
        # ⭐⭐ THE JAW PAUSE QUEUE (ROADMAP §6.6.2). `plan_gripper_stops` splits a run
        # wherever only the jaws move; the first segment goes into `park_path` and each
        # later one waits here as (jaw target, waypoint name, its path, its marks). The
        # pause itself lives in `park_jaw`: None means the cursor is driving, a value means
        # the arm is holding at a split while the jaws travel toward it.
        self.park_queue: list[tuple[float, str, JointPath, list[tuple[str, float]]]] = []
        self.park_jaw: float | None = None      # the jaw value being waited on
        self.park_jaw_name: str | None = None
        self.park_jaw_t = 0.0                   # when the pause began
        self.park_jaw_still_t = 0.0             # since when the jaws have not moved
        self.park_jaw_prev: float | None = None # last cycle's measured jaw, for stillness
        # ⭐ The settle-gate before a jaw pause: the cursor finishing is not the arm
        # arriving, and jaws that close while the arm creeps its last millimetres close
        # in the wrong place (his 2026-08-18 grab missed exactly this way). `None` means
        # no gate is open; a value is the best arm lag seen since the gate opened.
        self.park_gate_best: float | None = None
        self.park_gate_t = 0.0                  # when that best last improved
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
        # None means blind, not cold. Keep temperature reporting separate for each arm.
        self.hottest: float | None = None
        self.jaw_temp: float | None = None
        # Cache this cycle for incident reporting without a fresh read on a dead chain.
        # A failed read sets states to None but preserves the last temperatures.
        self.read_error: str | None = None
        self.states: Any = None
        self.temps: Any = None

        # Latch the measured blocked position across cycles. Only opening beyond the
        # clearance margin releases it, so mirror/teleop cannot immediately push again.
        self.jaw_block: float | None = None
        #: ⭐ Set to the block value when the latch CLEARS, so the loop can say so once. A
        #: latch that silently comes and goes is impossible to tell apart from a latch that
        #: never worked, which is precisely the ambiguity his 2026-08-17 log left behind:
        #: three stalls at 0.117, 0.098 and 0.104, and no way to know whether they were three
        #: deliberate squeezes or one latch being cleared twice by noise.
        self.jaw_unblocked_from: float | None = None

    # ------------------------------------------------------------- jaws ----

    def hold_jaw(self, wanted: float) -> float:
        """Apply the jaw-block latch to a requested normalized position.

        With no block, return wanted. With a block, hold the measured stall
        position until wanted exceeds it by JAW_CLEAR_MARGIN (larger is more
        open). Then clear the latch and record jaw_unblocked_from for reporting.
        The margin prevents measurement jitter from restarting repeated stalls;
        holding the measured position preserves the grip instead of dropping it.
        """
        if self.jaw_block is None:
            return wanted
        if wanted > self.jaw_block + JAW_CLEAR_MARGIN:
            self.jaw_unblocked_from = self.jaw_block
            self.jaw_block = None
            return wanted
        return self.jaw_block

    def block_jaw_at(self, where: float) -> None:
        """Latch a jaw block at the measured position where the stall happened."""
        self.jaw_block = float(where)

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

    def read_thermal(self, *, n_arm: int = N_ARM):
        """Cache one chain read and return the guard verdict; unreadable means blind.

        states and _states refer to the same cycle for reporting and the legacy
        stall-helper API. Keep the last temperatures after a failed read.
        """
        try:
            states = self.robot.motor_chain.read_states()
            self.read_error = None
        except Exception as exc:
            states = None
            self.read_error = f"{type(exc).__name__}: {exc}"
        self.states = self._states = states
        if states is None:
            self.stall_since = None
            self.hottest, self.jaw_temp = None, None
            return self.thermal.update(None), None, None
        self.temps, self.hottest, self.jaw_temp = motor_temperatures(states, n_arm)
        motor = self.temps.index(self.hottest) if self.hottest is not None else None
        return self.thermal.update(self.hottest, self.jaw_temp, motor=motor), self.hottest, self.jaw_temp

    def gripper_stall_release(self, t: float, *, n_arm: int = N_ARM,
                              torque: float = GRIPPER_STALL_TORQUE,
                              velocity: float = GRIPPER_STALL_VEL,
                              seconds: float = GRIPPER_STALL_SECONDS) -> float | None:
        """Return a measured jaw position after sustained high torque/low velocity.

        Call read_thermal first in the same cycle. Missing readings, no gripper or
        an existing jaw block reset the timer and return None. SessionHealth
        applies the returned value, latches it and reports the stall. This method
        does not command a robot.
        """
        states = getattr(self, "_states", None)
        if states is None or len(states) <= n_arm:
            self.stall_since = None
            return None
        if self.jaw_block is not None:
            self.stall_since = None
            return None
        jaw = states[n_arm]
        pushing = abs(getattr(jaw, "eff", 0.0)) > torque
        still = abs(getattr(jaw, "vel", 0.0)) < velocity
        if not (pushing and still):
            self.stall_since = None
            self.stall_count = 0
            return None
        if self.stall_since is None:
            self.stall_since = t
            return None
        if t - self.stall_since <= seconds:
            return None
        self.stall_since = None
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        return float(q[n_arm])

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
        fn = getattr(self.robot, "enter_gravity_comp_idle", None)
        if callable(fn):
            fn()
            self.mode = "guide"
            return None
        self.enter_hold()
        return "enter_gravity_comp_idle() missing — staying in HOLD (NOT weightless)"

    def guide_drift(self) -> float | None:
        """How far the arm has sunk since it went weightless, in radians."""
        if self.guide_ref is None:
            return None
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        return float(np.max(np.abs(q[:N_ARM] - self.guide_ref[:N_ARM])))

    # -------------------------------------------------------------- park ----

    def begin_path(self, legs: list[ParkLeg], t: float, smooth: bool = True,
                   mixed_leg_advice: bool = True) -> list[str]:
        """Validate every target, split jaws-only legs, then start the blended run.

        Every waypoint passes park_target_from for joint count and jaw limits.
        Between segments, step_path holds the arm and waits for the jaw movement.
        Mixed arm/jaw legs produce advice rather than a split; mixed_leg_advice
        suppresses that advice only, never target warnings. smooth=False disables
        the speed easing ramp, not geometric blending. Return collected warnings.
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
        poses = [start, *targets]
        names = ["start", *[leg.name for leg in legs]]
        plan = plan_gripper_stops(poses, N_ARM)
        if mixed_leg_advice:
            warnings.extend(plan.warnings)

        def built(seg: list[int]) -> tuple[JointPath, list[tuple[str, float]]]:
            path = JointPath([poses[j] for j in seg], blend=self.blend)
            marks = list(zip([names[j] for j in seg[1:]], path.arrival_lengths()[1:]))
            return path, marks

        self.park_path, self.park_marks = built(plan.segments[0])
        # ⭐ Each queued entry starts at a waypoint whose ARM pose the previous segment
        # already reached, so the jaw value of its FIRST waypoint is what the pause
        # drives the jaws to, and that waypoint's name is what the pause reports.
        self.park_queue = []
        for seg in plan.segments[1:]:
            path, marks = built(seg)
            self.park_queue.append((float(poses[seg[0]][N_ARM]), names[seg[0]], path, marks))
        self.park_jaw, self.park_jaw_name, self.park_jaw_prev = None, None, None
        self.park_gate_best = None

        self.park_s = 0.0
        self.park_target = targets[-1]
        self.park_cmd = start.copy()
        self.enter_hold()
        self.mode = "park"
        self._smooth = smooth
        self.park_best_err = self._arm_err(self.park_target, start)
        self.park_progress_t = t
        self.park_leg_t = t
        self.park_start_t = t
        return warnings

    @staticmethod
    def _arm_err(a: np.ndarray, b: np.ndarray) -> float:
        """Worst ARM joint distance — the jaw is deliberately excluded.

        ⛔ The jaw legitimately sits far from its command whenever it holds an object
        (that is what a successful grab IS), so counting it would stall the cursor and
        fail the arrival verdict on every run that grips anything. Jaw completion has
        its own measured judgement in the jaw phase. Same rule, same reason, as the
        playback's by-index gripper exclusion in `teleop_session.py`.
        """
        return float(np.max(np.abs(a[:N_ARM] - b[:N_ARM])))

    def _command_park(self, cmd: np.ndarray) -> None:
        """Send a park command with the jaw routed through the block latch.

        ⛔⭐ THIS CLOSES A REAL HOLE: the park used to command the saved jaw value raw,
        every cycle, so a park whose pose closes the jaws onto an object would push into
        it at 90 Hz for the rest of the run — the stall guard latched a block and the
        park ignored it, the exact §58.2 shape one layer down. `hold_jaw` honours the
        latch (keep the grip, stop the push) and clears it on a deliberate open.
        """
        if len(cmd) > N_ARM:
            held = self.hold_jaw(float(cmd[N_ARM]))
            if held != float(cmd[N_ARM]):
                cmd = cmd.copy()
                cmd[N_ARM] = held
        self.robot.command_joint_pos(cmd)

    @property
    def park_total_length(self) -> float:
        """Travel still planned: the live segment plus every queued one, in rad."""
        live = self.park_path.length if self.park_path is not None else 0.0
        return live + sum(path.length for _, _, path, _ in self.park_queue)

    @property
    def park_stops(self) -> int:
        """Gripper pauses not yet completed in the current run."""
        return len(self.park_queue)

    def count_gripper_stops(self, legs: list[ParkLeg]) -> int:
        """How many times a run through these legs would pause for the jaws.

        ⭐ For the plan line, BEFORE Enter: a grab should be visible while the sequence
        is still being typed (ROADMAP §6.6.2 item 4). Same clamping and reconciliation
        as `begin_path`, so the count cannot disagree with the run.
        """
        q = self.robot.get_joint_pos()
        poses = [np.asarray(q, dtype=float)]
        for leg in legs:
            target, _ = park_target_from(q, leg.pose,
                                         gripper_index=N_ARM, clamp=self.clamp_gripper)
            poses.append(target)
        return len(plan_gripper_stops(poses, N_ARM).gripper_legs)

    def step_path(self, t: float, dt: float,
                  tolerance: float = PARK_TOLERANCE,
                  settled: float = PARK_SETTLED,
                  stall_seconds: float = PARK_STALL_SECONDS,
                  settle_seconds: float = PARK_SETTLE_SECONDS,
                  progress_eps: float = PARK_PROGRESS_EPS,
                  max_cursor_lag: float = MAX_CURSOR_LAG) -> ParkStep:
        """Advance one park cycle and return a ParkStep with measured progress.

        Arrival requires the path cursor to finish AND the measured arm to arrive;
        a closed loop must not finish immediately because its end is its start.
        The cursor waits for lagging arms. Progress includes either cursor motion
        or a shrinking measured gap. Arm error excludes jaws, so holding an object
        does not prevent arrival.

        At segment boundaries, hold the arm before moving jaws through the block
        latch. Resume after measured stillness and the minimum wait, or a reported
        timeout. The resume step includes a grasp assessment for a closing leg.
        """
        blocked = ParkStep("blocked", float("inf"), 0.0, 0.0, None, None,
                           t - self.park_leg_t, t - self.park_start_t,
                           t - self.park_leg_t)
        if self.park_path is None or self.park_target is None or self.park_cmd is None:
            return blocked

        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        err = self._arm_err(self.park_target, q)
        lag = self._arm_err(self.park_cmd, q)
        length = self.park_path.length
        queued = sum(path.length for _, _, path, _ in self.park_queue)

        def result(verdict: str, leg_passed: str | None = None,
                   next_leg: str | None = None, **jaw: Any) -> ParkStep:
            return ParkStep(verdict, err, lag,
                            max(0.0, length - self.park_s) + queued,
                            leg_passed, next_leg, t - self.park_leg_t,
                            t - self.park_start_t, t - self.park_leg_t, **jaw)

        # ---- the jaw phase: the arm holds, the jaws travel, the wait is measured ----
        if self.park_jaw is not None:
            jaw_now = float(q[N_ARM]) if len(q) > N_ARM else None
            cmd = self.park_cmd.copy()
            if len(cmd) > N_ARM:
                cmd[N_ARM] = self.park_jaw
            self._command_park(cmd)

            moved = (jaw_now is not None and self.park_jaw_prev is not None
                     and abs(jaw_now - self.park_jaw_prev) > JAW_STILL_RATE * max(dt, 1e-9))
            if moved:
                self.park_jaw_still_t = t
            self.park_jaw_prev = jaw_now
            waited = t - self.park_jaw_t
            done = (waited >= JAW_MIN_WAIT
                    and t - self.park_jaw_still_t >= JAW_SETTLE_SECONDS)
            timed_out = not done and waited >= JAW_TIMEOUT_SECONDS
            if not (done or timed_out):
                return result("jaws", jaw_name=self.park_jaw_name,
                              jaw_target=self.park_jaw, jaw_seconds=waited)

            # The pause is over: grade the grasp, swap in the next segment, resume.
            # ⚠️ The step is built BEFORE the pop so its `remaining` still counts the
            # segment about to become current (`length - park_s` is 0 here, the queue
            # carries everything left); `next_leg` peeks at that segment's first mark.
            grasp = None
            if jaw_now is not None:
                grasp = check_grasp(self.park_jaw, jaw_now, settled=done)
            nxt = self.park_queue[0][3]
            step = result("jaws", leg_passed=self.park_jaw_name,
                          next_leg=nxt[0][0] if nxt else None,
                          jaw_name=self.park_jaw_name, jaw_target=self.park_jaw,
                          jaw_seconds=waited, jaw_done=True, jaw_timed_out=timed_out,
                          grasp=grasp)
            _, _, path, marks = self.park_queue.pop(0)
            self.park_path, self.park_marks = path, marks
            self.park_s = 0.0
            self.park_cmd = path.point_at(0.0)
            self.park_jaw, self.park_jaw_name, self.park_jaw_prev = None, None, None
            self.park_leg_t = t
            self.park_progress_t = t
            return step

        if self.park_s < length:
            advanced = False
            if lag < max_cursor_lag:
                ramp = easing_factor(self.easing, self.park_s, length - self.park_s,
                                     self.park_ramp if self._smooth else 0.0)
                self.park_s = min(length, self.park_s + self.park_speed * ramp * dt)
                advanced = True
            self.park_cmd = self.park_path.point_at(self.park_s)
            self._command_park(self.park_cmd)

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

        # ---- the cursor has finished this segment ----
        if self.park_queue:
            # ⭐⭐ LET THE ARM SETTLE BEFORE THE JAWS MOVE. The cursor finishing is not
            # the arm arriving — the arm still trails by its friction floor, and jaws
            # that close while it creeps close in the wrong place. His 2026-08-18 bench
            # pass measured exactly this: the grab missed by millimetres while the run
            # read clean. So the split pose keeps being commanded until the arm is
            # within `tolerance` or has stopped improving for `settle_seconds` — the
            # same two exits the final settle has — and only then do the jaws move.
            # Costs at most about half a second per stop; `jaw_arm_off` reports the
            # offset the arm actually settled at, which is the number that tells a
            # friction-floor miss from a badly taught pose.
            self._command_park(self.park_cmd)
            if self.park_gate_best is None:
                self.park_gate_best, self.park_gate_t = lag, t
            elif lag < self.park_gate_best - progress_eps:
                self.park_gate_best, self.park_gate_t = lag, t
            if lag >= tolerance and t - self.park_gate_t < settle_seconds:
                return result("moving")
            jaw, name, _, _ = self.park_queue[0]
            self.park_jaw, self.park_jaw_name = jaw, name
            self.park_jaw_t, self.park_jaw_still_t = t, t
            self.park_jaw_prev = float(q[N_ARM]) if len(q) > N_ARM else None
            self.park_gate_best = None
            return result("jaws", jaw_name=name, jaw_target=jaw, jaw_started=True,
                          jaw_arm_off=lag)

        verdict = park_verdict(err, t - self.park_progress_t > stall_seconds,
                               tolerance, settled,
                               stopped_briefly=t - self.park_progress_t > settle_seconds)
        if verdict not in ("arrived", "settled", "blocked"):
            # ⭐ THE SETTLE PHASE STILL COMMANDS AND STILL CREDITS PROGRESS — merged from
            # the script on 2026-08-18 (item 23 group ④), which had learned both after
            # this class was written. Re-sending the final point keeps the velocity
            # feedforward decaying to zero instead of freezing at its last value, and
            # crediting an err improvement keeps a slowly-settling arm from being
            # declared blocked at the stall timeout while it is still visibly closing.
            self._command_park(self.park_cmd)
            if err < self.park_best_err - progress_eps:
                self.park_best_err, self.park_progress_t = err, t
        return result(verdict)

    def abandon_path(self) -> float:
        """⛔ Leaving PARK abandons the rest of the run, and returns the rad dropped.

        An arm that resumes a queued trajectory after the operator pressed HOLD is doing
        something nobody asked for. Returning the distance rather than a waypoint count
        is deliberate: with one blended path there are no separate legs left to count,
        and "1.8 rad of path abandoned" is what an operator can actually picture.

        ⭐ Queued segments and a pause in progress are part of the run, so they are
        dropped — and counted — here too. A latched jaw block survives on purpose: it
        describes the object still between the jaws, not the abandoned motion.
        """
        left = 0.0 if self.park_path is None else max(0.0, self.park_path.length - self.park_s)
        left += sum(path.length for _, _, path, _ in self.park_queue)
        self.park_path, self.park_marks = None, []
        self.park_queue = []
        self.park_jaw, self.park_jaw_name, self.park_jaw_prev = None, None, None
        self.park_gate_best = None
        self.park_s = 0.0
        return left
