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

⚠️ **STATUS, 2026-08-12: built and unit-tested, NOT yet wired into
`teleop_session.py`.** That wiring is a separate, reviewable step, deliberately not
done in the same session as writing this. Session 4 is the standing warning — three
changes that passed 34 tests, three dry runs and a simulated IK loop produced three
failures on first hardware contact, one of which dropped 4.3 kg. When it is wired,
ROADMAP step 6 says how: `--arms B` runs the N-arm code with **N=1** first, so the
restructure is verified against a feel Julien already knows, separately from the
two-arm risk.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from yam_robot import (
    ThermalGuard,
    advance_park_command,
    motor_temperatures,
    park_speed_factor,
    park_target_from,
    park_verdict,
)

N_ARM = 6


@dataclass
class ParkLeg:
    """One waypoint in a queued run: its slot name and the pose to reach."""

    name: str
    pose: list


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

        self.park_target: np.ndarray | None = None
        self.park_cmd: np.ndarray | None = None
        self.park_start: np.ndarray | None = None
        self.park_best_err = float("inf")
        self.park_progress_t = 0.0
        self.park_queue: list[ParkLeg] = field(default_factory=list)  # type: ignore[assignment]
        self.park_queue = []
        self.park_speed = 0.40

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

    def begin_park(self, pose, t: float) -> str | None:  # noqa: ANN001
        """Start a leg toward `pose`. Returns a warning from the target builder."""
        target, warn = park_target_from(self.robot.get_joint_pos(), pose,
                                        gripper_index=N_ARM, clamp=self.clamp_gripper)
        self.park_target = target
        self.enter_hold()
        self.mode = "park"
        self.park_cmd = np.asarray(self.robot.get_joint_pos(), dtype=float)
        self.park_start = self.park_cmd.copy()
        self.park_best_err = float(np.max(np.abs(target - self.park_cmd)))
        self.park_progress_t = t
        return warn

    def step_park(self, t: float, dt: float, tolerance: float = 0.02,
                  settled: float = 0.06, stall_seconds: float = 4.0,
                  ramp: float = 0.20) -> tuple[str, float]:
        """Advance the park by one control cycle. Returns `(verdict, error)`.

        Verdicts are `park_verdict`'s: `moving` · `arrived` · `settled` · `blocked`.
        The caller decides what to do — pop the next leg, hold, or ask a human.

        ⛔ Completion is judged from the **measured** pose, never the command. The
        command always arrives first, so testing it would declare success while the
        arm was still travelling. That was a real bug and it hid for two sessions.
        """
        if self.park_target is None or self.park_cmd is None:
            return "blocked", float("inf")
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        err = float(np.max(np.abs(self.park_target - q)))
        verdict = park_verdict(err, t - self.park_progress_t > stall_seconds,
                               tolerance, settled)
        if verdict != "moving":
            return verdict, err
        factor = park_speed_factor(
            float(np.max(np.abs(self.park_cmd - self.park_start))),
            float(np.max(np.abs(self.park_target - self.park_cmd))), ramp)
        self.park_cmd = advance_park_command(self.park_cmd, self.park_target,
                                             self.park_speed * factor * dt)
        self.robot.command_joint_pos(self.park_cmd)
        if err < self.park_best_err - 0.003:
            self.park_best_err, self.park_progress_t = err, t
        return "moving", err

    def next_leg(self, t: float) -> ParkLeg | None:
        """Pop and start the next queued waypoint, or None when the run is done."""
        if not self.park_queue:
            return None
        leg = self.park_queue.pop(0)
        self.begin_park(leg.pose, t)
        return leg

    def abandon_queue(self) -> int:
        """⛔ Leaving PARK abandons the rest of a sequence. An arm that resumes a
        queued trajectory after the operator pressed HOLD is doing something nobody
        asked for. Returns how many were dropped, so the caller can say so."""
        dropped = len(self.park_queue)
        self.park_queue = []
        return dropped
