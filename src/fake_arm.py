"""A fake arm that behaves like the real one OVER TIME, so the session loop can run
with no hardware.

⭐⭐ WHY THIS EXISTS. Three defects in one week reached Julien's hardware because
nothing could run the loop without an arm: a playback cursor that advanced twice per
cycle, a playback that cancelled itself when one arm parked before the other, and a
stale `q` that fed a 7-element snapshot against 14-element targets. **All three are
timing and sequencing bugs, and all three are invisible to a test whose fake robot
teleports to whatever it is commanded.** (docs/FINDINGS.md §58.4 item 2,
docs/ROADMAP.md §8.2 item 30.)

⛔ THE DISTINCTION THAT MAKES THIS DIFFERENT FROM THE EXISTING FAKES.
`scripts/test_park_arms.py` and `scripts/test_status_row.py` already carry a fake
robot, and it is the right fake for what they test: `command_joint_pos` sets the
measured position, so the arm is exactly where it was told, instantly. That is a fake
where **following error cannot exist**, so every bug that lives in the gap between
"commanded" and "measured" is unreachable. This module is the other kind: the arm
lags, and the lag is the point.

⭐⭐ THE LAG IS MEASURED, NOT INVENTED. docs/ROADMAP.md §8.2 item 11 closed on
2026-08-13 with a law fitted to three playbacks and then tested on held-out data:

    following error ≈ 0.04 to 0.10 rad  +  0.033 s × speed

Both halves are reproduced here, and `scripts/test_fake_arm.py` asserts that driving
this fake at a constant speed lands inside that measured band. The two terms are
physically different things and are modelled as different things:

| term | what it is | how it is modelled |
|---|---|---|
| `0.033 s × speed` | a first-order lag; the joint chases its target | `tau = 0.033` s |
| `0.04 to 0.10 rad` | static friction; a joint does not move for a small error | `deadband = 0.05` rad |

⭐ Why that combination reproduces the law exactly, which is worth writing down because
it is the reason the constants are not fudge factors. Following a ramp at speed `v`,
the joint settles where the movement it makes per step equals `v·dt`:

    (|gap| − deadband) · (1 − e^(−dt/tau))  =  v · dt

For a small step that is `(|gap| − deadband) · dt/tau = v · dt`, so
`|gap| = deadband + v·tau`. **A constant term plus a term proportional to speed, which
is the shape the measurement found.**

⛔⭐⭐ WHAT THIS CANNOT DO, AND SAYING SO IS PART OF BUILDING IT.

1. ⛔ **No gravity.** The real arm droops under its own 4.3 kg and the motors hold it
   up; here a joint with no command simply stays put. So this can never test gravity
   compensation, and a `zero_gravity` session means nothing to it.
2. ⛔ **No dynamics.** No inertia, no coupling between joints, no wrist swinging
   because the shoulder moved. Each joint is independent, which the real arm's joints
   are not.
3. ⛔ **The thermal model is a SHAPE, not a calibration.** It warms when it pushes and
   cools when it does not, with a time constant chosen to be watchable in a short test.
   ⚠️ Never read a number off it and compare it to a real motor.
4. ⛔ **Nothing here says anything about feel.** Whether teleop is pleasant to drive,
   whether mirror tracks well enough to be useful, whether a speed is safe in the room
   — hardware only, always.

⭐ So the honest claim is narrow and still worth a lot: **this catches bugs in
sequencing, state machines, cursors, mode transitions and following-error handling,
which is where this week's defects actually lived.**

⭐⭐ IT WRAPS THE REAL `SafeRobot`, ON PURPOSE. `build_fake_robot()` returns
`SafeRobot(FakeArm(...))` using the real class from `src/yam_robot.py`, not a
reimplementation of its two limits. A copy of a safety limit tests the copy. ⚠️ That
also means `SafeRobot`'s internal `time.perf_counter()` is in the loop, so a simulated
robot driven by a simulated clock has one real-time component; `build_fake_robot` says
so where it matters.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

#: Measured on the real arm, docs/ROADMAP.md §8.2 item 11. The speed-proportional half
#: of the following-error law, and it was the same on all six arm joints.
MEASURED_TAU_S = 0.033

#: The constant half of the same law, whose measured range was 0.04 to 0.10 rad. The
#: midpoint is used so a test can assert the whole band rather than one endpoint.
MEASURED_DEADBAND_RAD = 0.05

#: Room temperature as actually read off both arms at rest on 2026-08-15: every motor
#: between 27 and 30 °C. ⚠️ A starting value, not a calibration.
AMBIENT_C = 28.0

#: Seven joints: six arm joints and the gripper, matching the real YAM.
N_JOINTS = 7


@dataclass
class FakeMotorState:
    """One motor's reading. ⭐ The field NAMES matter and are not arbitrary.

    `src/yam_robot.py::motor_temperatures` reads `temp_mos` and `temp_rotor` through
    `getattr(s, ..., 0)`, and `scripts/teleop_session.py` reads `pos` and `eff`. ⛔ A
    `getattr` default means a misspelled field here would read as **zero** rather than
    raising, and a zero temperature silently disarms the thermal guard — which is the
    exact failure `motor_temperatures` has a warning about. So these four names are a
    contract with real code, not a convenience.
    """

    pos: float = 0.0
    vel: float = 0.0
    eff: float = 0.0
    temp_mos: float = AMBIENT_C
    temp_rotor: float = AMBIENT_C


class FakeMotorInterface:
    """The one method the shutdown path calls on a chain's motor interface.

    ⚠️ `motor_off` is recorded rather than acted on, because a simulated motor has no
    torque to remove. What matters is that it was CALLED for every motor, since that is
    the difference between an arm released deliberately and an arm dropped.
    """

    def __init__(self, chain: FakeChain) -> None:
        self._chain = chain

    def motor_off(self, motor_id: int) -> None:
        self._chain.disabled.append(int(motor_id))


class FakeChain:
    """Stands in for `robot.motor_chain`.

    ⭐ Two things the session asks of it, and both are failure paths worth simulating:
    `running` (a chain that dies mid-motion is what happened on 2026-08-14) and
    `read_states()` (whose failure makes the thermal guard blind, a path
    `src/yam_robot.py::ThermalGuard` was specifically rewritten for).
    """

    def __init__(self, arm: FakeArm) -> None:
        self._arm = arm
        self.running = True
        #: ⭐ Set True to simulate an unreadable chain. The guard's blind path is
        #: otherwise unreachable without unplugging hardware mid-session.
        self.blind = False
        self.closed = False
        #: ⭐⭐ THE TEARDOWN PATH NEEDS THESE, AND IT IS THE MOST SAFETY-CRITICAL CODE IN
        #: THE PROJECT. `yam_robot.shutdown_robot()` walks `motor_list` and calls
        #: `motor_interface.motor_off(id)` on each, then reports which ones genuinely
        #: succeeded. ⛔ Every one of those calls sits inside `except Exception: pass`, so
        #: a fake lacking these attributes does not raise — it silently reports **zero
        #: motors disabled**, and "confirmed disabled: []" would be read as a fake being
        #: a fake rather than as the shutdown having failed.
        #:
        #: ⚠️ Providing them means the park-then-disable sequence can be exercised end to
        #: end with no hardware. Until now it had only ever been tested by Julien pressing
        #: Ctrl-C on a live arm, which is a poor way to test the code that decides whether
        #: 4.3 kg is released.
        self.motor_list = [(i, "DM4340" if i <= 3 else "DM4310") for i in range(1, 8)]
        self.motor_interface = FakeMotorInterface(self)
        #: Which motor IDs were actually told to switch off, in order.
        self.disabled: list[int] = []

    def read_states(self) -> list[FakeMotorState]:
        if self.blind:
            raise RuntimeError("simulated: cannot read motor states")
        return self._arm.states()

    def close(self) -> None:
        self.closed = True
        self.running = False


class FakeArm:
    """An arm that follows commands over time, with measured lag and static friction.

    ⚠️ Implements exactly the interface the session uses and nothing more:
    `get_joint_pos`, `command_joint_pos`, `num_dofs`, `close`, `motor_chain`. That list
    was taken by grepping every attribute reached through a robot handle, so it is the
    real surface rather than a guess.
    """

    def __init__(
        self,
        n_joints: int = N_JOINTS,
        *,
        name: str = "B",
        start: Any = None,
        tau: float = MEASURED_TAU_S,
        deadband: float = MEASURED_DEADBAND_RAD,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.name = name
        self.n = n_joints
        self.tau = float(tau)
        self.deadband = float(deadband)
        #: ⭐ Injectable so tests are deterministic. A test that leans on wall-clock
        #: time measures the machine's load as much as the code.
        self.clock = clock or time.perf_counter

        self.q = (np.zeros(n_joints, dtype=float) if start is None
                  else np.asarray(start, dtype=float).copy())
        self.cmd = self.q.copy()
        self.vel = np.zeros(n_joints, dtype=float)
        self.temp = np.full(n_joints, AMBIENT_C, dtype=float)

        self.motor_chain = FakeChain(self)

        #: ⭐ `{joint_index: (low, high)}` — a joint physically cannot leave this range.
        #: This is how an object in the way is simulated: the command keeps climbing,
        #: the joint stops, and the following error grows exactly as it does in the
        #: room. It is what makes the jaw-block and stall paths reachable at all.
        self.blocked: dict[int, tuple[float, float]] = {}

        self.commands: list[np.ndarray] = []
        #: ⭐ Every velocity setpoint received via `command_joint_state` (item 44), so a
        #: test can assert the feedforward plumbing end to end.
        self.commanded_vels: list[np.ndarray] = []
        self.cycles = 0
        self._last_t: float | None = None

    # ---------------------------------------------------------------- the interface

    def num_dofs(self) -> int:
        return self.n

    def get_joint_pos(self) -> np.ndarray:
        return self.q.copy()

    def command_joint_pos(self, q: Any) -> None:
        """Take a command and advance the physics by the time that has really passed.

        ⭐ The physics advances HERE rather than in `get_joint_pos` on purpose. A real
        arm holds the pose it was last commanded to, so reading its position twice with
        no command in between must not move it. Advancing on the read would make an
        idle arm drift, which no real arm does.
        """
        q = np.asarray(q, dtype=float)
        if len(q) != self.n:
            raise ValueError(
                f"commanded {len(q)} joints on a {self.n}-joint arm — this is the "
                f"14-vs-7 shape bug the two-arm recorder hit, so it raises")
        self.cmd = q.copy()
        self.commands.append(q.copy())
        self.cycles += 1

        now = self.clock()
        # ⚠️ Same dt clamp as SafeRobot: a stalled loop must not buy a huge movement
        # budget. First call gets a nominal 10 ms rather than zero, so one isolated
        # command still moves the arm a little instead of nothing at all.
        dt = 0.01 if self._last_t is None else min(0.05, max(1e-4, now - self._last_t))
        self._last_t = now
        self.step(dt)

    def command_joint_state(self, joint_state: dict) -> None:
        """Accept the pos+vel form the real `MotorChainRobot` offers (item 44).

        ⭐ The velocity setpoint is RECORDED so a test can assert the feedforward
        plumbing, and the physics then runs exactly as for a plain position command.
        ⚠️ Deliberately so: how much feedforward actually tightens tracking is a
        property of the real motors, and a constant for it here would be invented
        rather than measured — the exact defect class FINDINGS §33.3 exists for. When
        the real arm is measured with feedforward on, the fitted law belongs here.
        """
        vel = joint_state.get("vel")
        self.commanded_vels.append(
            np.zeros(self.n) if vel is None else np.asarray(vel, dtype=float).copy())
        self.command_joint_pos(joint_state["pos"])

    def close(self) -> None:
        self.motor_chain.close()

    # ------------------------------------------------------------------ the physics

    def step(self, dt: float) -> None:
        """Advance `dt` seconds toward the standing command.

        ⭐ Public and separate from `command_joint_pos` so a test can hold a command
        and let time pass, which is how "the arm is still catching up" is expressed.
        """
        gap = self.cmd - self.q
        alpha = 1.0 - math.exp(-dt / self.tau) if self.tau > 0 else 1.0

        # ⭐ Static friction: below the deadband the joint does not move at all. This
        # is the constant term in the measured law and it is why the fake settles with
        # a residual error instead of converging exactly, which the real arm also does.
        past = np.maximum(np.abs(gap) - self.deadband, 0.0)
        move = np.sign(gap) * past * alpha

        before = self.q.copy()
        self.q = self.q + move

        # ⭐ A blocked joint stops where the obstacle is. The COMMAND is deliberately
        # left alone, so the following error keeps growing exactly as it does when the
        # real arm pushes into something — which is the signal every stall detector and
        # SafeRobot's own lag clip actually key off.
        for j, (lo, hi) in self.blocked.items():
            if 0 <= j < self.n:
                self.q[j] = min(max(self.q[j], lo), hi)

        self.vel = (self.q - before) / dt if dt > 0 else np.zeros(self.n)
        self._heat(dt)

    def _heat(self, dt: float) -> None:
        """⚠️ A SHAPE, NOT A CALIBRATION. Warms while pushing, cools while not.

        ⛔ Do not compare a number from here with a real motor reading. It exists so
        the thermal guard's warn and stop thresholds can be crossed on demand, which
        on hardware means cooking a motor — and motor 7 has already been cooked three
        times (`src/yam_robot.py::gripper_stall_release`).
        """
        # Torque in a position-controlled joint tracks the following error, so the
        # error is the heat source. 30 s time constant: slow enough to look like a
        # motor, fast enough that a test need not run for minutes.
        push = np.abs(self.cmd - self.q)
        target = AMBIENT_C + 120.0 * push
        self.temp += (target - self.temp) * (1.0 - math.exp(-dt / 30.0))

    def states(self) -> list[FakeMotorState]:
        """What `motor_chain.read_states()` hands back."""
        # ⚠️ `eff` is a plausible stand-in, not a torque model: a position-controlled
        # joint's effort is proportional to its following error, so that is what is
        # reported. ⛔ With no gravity here it will read near zero for an arm that on
        # hardware carries 5.9 Nm in joint 3 just to hold itself up.
        gap = self.cmd - self.q
        return [FakeMotorState(pos=float(self.q[i]), vel=float(self.vel[i]),
                               eff=float(20.0 * gap[i]),
                               temp_mos=float(self.temp[i]),
                               temp_rotor=float(self.temp[i] - 2.0))
                for i in range(self.n)]

    # ------------------------------------------------------------- test conveniences

    def block(self, joint: int, low: float, high: float) -> None:
        """Put something in the way of one joint."""
        self.blocked[joint] = (float(low), float(high))

    def kill_chain(self) -> None:
        """Simulate the CAN link going away, which is what took the rig down on
        2026-08-14. ⭐ The arm keeps its pose here; the real one sags, and nothing in
        software can prevent that once the link is gone."""
        self.motor_chain.running = False

    def following_error(self) -> float:
        """Worst |command − measured| across joints, in rad. The number `SafeRobot`'s
        `max_lag` clips and the one every stall message should be quoting."""
        return float(np.max(np.abs(self.cmd - self.q)))


def build_fake_robot(
    arm: str = "B",
    *,
    n_joints: int = N_JOINTS,
    start: Any = None,
    max_speed: float | None = None,
    max_lag: float | None = None,
    clock: Callable[[], float] | None = None,
    tau: float = MEASURED_TAU_S,
    deadband: float = MEASURED_DEADBAND_RAD,
) -> tuple[Any, str]:
    """`(robot, note)`, matching `src/yam_robot.py::build_robot`'s shape exactly.

    ⭐ Returning the same tuple is what lets a session take either one, so the
    simulated path and the real path stay one code path rather than two.

    ⭐⭐ The wrapper is the REAL `SafeRobot`. Its defaults for `max_speed` and
    `max_lag` are used when none are given, so a simulated session is limited by the
    same numbers as a real one, read from the same constants.

    ⚠️⚠️ ONE REAL-TIME COMPONENT SURVIVES A SIMULATED CLOCK. `SafeRobot` reads
    `time.perf_counter()` itself to size its rate-limit budget, and it cannot be
    injected without changing safety code. So with a fake clock the *arm* advances in
    simulated time while *SafeRobot's* budget is computed from wall time. ⛔ For any
    test about the rate limit specifically, drive `FakeArm` directly and construct
    `SafeRobot` around it knowingly, or use the real clock.
    """
    from yam_robot import SAFE_MAX_LAG, SAFE_MAX_SPEED, SafeRobot

    fake = FakeArm(n_joints, name=arm, start=start, clock=clock,
                   tau=tau, deadband=deadband)
    robot = SafeRobot(
        fake,
        max_speed=SAFE_MAX_SPEED if max_speed is None else max_speed,
        max_lag=SAFE_MAX_LAG if max_lag is None else max_lag,
    )
    note = (f"SIMULATED arm {arm} — {n_joints} joints, lag tau={tau:.3f}s, "
            f"deadband={deadband:.2f} rad. No gravity, no dynamics, "
            f"thermal model uncalibrated.")
    return robot, note


class StillPuck:
    """A SpaceMouse that is never touched. The reader interface and nothing more.

    ⭐ `scripts/teleop_session.py` asks a puck for exactly two things: `read()`, which
    returns six axis values, and a `buttons` attribute. That surface was taken by grepping
    every attribute reached through a reader handle, so it is the real one.

    ⚠️ Reporting zero is the honest simulation of nobody's hand on the puck: TELEOP holds
    still. ⛔ So a simulated session can say nothing about driving feel, about the axis map,
    or about whether a speed is comfortable. Those need a real hand on a real device.
    """

    def __init__(self) -> None:
        self.buttons = 0

    def read(self) -> list[float]:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
