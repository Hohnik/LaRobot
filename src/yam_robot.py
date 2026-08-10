"""Build and safely tear down a whole-arm YAM robot on macOS.

Everything that talks to all seven motors goes through here, so the startup and
shutdown rules live in exactly one place.

TWO PROBLEMS THIS SOLVES, both found on real hardware 2026-08-10.

**1. The gripper re-calibrated on every single startup.**
`linear_4310.yml` has `gripper_limits: null`, so `get_yam_robot()` runs
`detect_gripper_limits` each time it is constructed. That routine applies a
constant 0.5 Nm and waits for the position to *stop changing* — i.e. it drives
the jaws into each hard stop and holds them there. Julien: *"they move really
quickly and quite hard… they seem to crash into the ends and then seem to try to
push further."* He is right, and the fix is not to soften a routine that runs
forever — it is to run it **once**, gently, and cache the answer. Supplying
`gripper_limits_override` is what turns the auto-detection off
(`get_robot.py:223-225`).

**2. Teardown raced itself.** `MotorChainRobot.close()` prints *"Robot closed
with all torques set to zero"*, but it only calls `motor_chain.close()`, which
shuts the bus **without disabling the motors** — while the chain's own 250 Hz
control thread is still mid-transaction. The observed result was an exception
storm from a dying thread and no certainty that anything was disabled.
Order matters: **stop the control thread → disable the motors → close the bus.**
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from yam_can import (
    DEFAULT_ARM,
    add_i2rt_to_path,
    chain_channel,
    patch_dm_driver_for_gs_usb,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
GRIPPER_LIMITS_FILE = REPO_ROOT / "config" / "gripper_limits.json"

# 0.5 Nm is I2RT's default and is what Julien watched slam the stops. 0.3 is
# ~60% of it: still enough to reach both ends of a 6.57 rad stroke, noticeably
# gentler on arrival. Only ever used by scripts/calibrate_gripper.py, which runs
# once — not on every startup.
GENTLE_TEST_TORQUE = 0.3


def save_gripper_limits(arm: str, limits: tuple[float, float]) -> Path:
    GRIPPER_LIMITS_FILE.parent.mkdir(parents=True, exist_ok=True)
    data = {}
    if GRIPPER_LIMITS_FILE.exists():
        data = json.loads(GRIPPER_LIMITS_FILE.read_text())
    data[arm] = [float(limits[0]), float(limits[1])]
    GRIPPER_LIMITS_FILE.write_text(json.dumps(data, indent=2) + "\n")
    return GRIPPER_LIMITS_FILE


def load_gripper_limits(arm: str) -> list[float] | None:
    """Measured jaw limits for `arm`, or None if it has never been calibrated.

    When this returns a value, `build_robot` passes it as `gripper_limits_override`
    and the arm starts **silently** — no jaws slamming into stops.
    """
    if not GRIPPER_LIMITS_FILE.exists():
        return None
    try:
        return json.loads(GRIPPER_LIMITS_FILE.read_text()).get(arm)
    except Exception:  # noqa: BLE001
        return None


def build_robot(
    arm: str = DEFAULT_ARM,
    *,
    zero_gravity: bool = False,
    allow_calibration: bool = False,
) -> tuple[Any, str]:
    """Construct the real robot. Returns `(robot, note)`.

    ⛔ This ENERGISES ALL SEVEN MOTORS and starts a 250 Hz control loop.

    `zero_gravity=False` (default) commands the arm to hold the pose it is
    already in — success looks like nothing happening. `zero_gravity=True` makes
    it back-drivable for hand-guiding, which is a genuinely different and looser
    physical state and should be asked for explicitly.

    `allow_calibration=True` permits the jaw-limit detection to run. Off by
    default precisely so it cannot happen by accident: it is a real motion into
    both mechanical stops, and once `config/gripper_limits.json` exists it never
    needs to happen again.
    """
    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()

    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType

    kwargs: dict = dict(
        channel=chain_channel(arm),
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=zero_gravity,
        sim=False,
    )

    saved = load_gripper_limits(arm)
    if saved is not None:
        kwargs["gripper_limits_override"] = saved
        note = f"using saved jaw limits {saved} — no calibration, jaws will not move"
    elif allow_calibration:
        note = "⚠️ NO saved jaw limits — the jaws WILL be driven into both stops to find them"
    else:
        raise RuntimeError(
            f"No saved gripper limits for {arm!r} and calibration is not allowed.\n"
            "  Run this once:  uv run scripts/calibrate_gripper.py --yes\n"
            "  It calibrates gently, saves the result, and every later start is silent."
        )
    robot = get_yam_robot(**kwargs)
    # ⭐ Everything above this line is I2RT's; everything that touches the robot
    # from here on goes through the rate limiter. See SafeRobot for why.
    return SafeRobot(robot), note


class SafeRobot:
    """A rate limiter that sits BELOW all control logic. Wraps any I2RT robot.

    ⭐ WHY THIS EXISTS — Julien, 2026-08-10, after the arm snapped:

        *"maybe there should be some safety mode where specific high speed
        movements just aren't possible… it would have to go over a more low
        level control before the output actually gets sent, because I'm guessing
        the actual snapping mistake wasn't an actual control that you sent, it
        was a misprogramming. So any type of safety would have to go lower than
        that."*

    He is exactly right, and the diagnosis was correct: the snap was not a
    commanded motion, it was a **stale cached variable** (`prev_q` was not reset
    when TELEOP was re-entered, so the first command after hand-guiding aimed at
    the pose from minutes earlier). No amount of care *inside* the teleop loop
    protects against a bug *in* the teleop loop. The guard has to be somewhere
    the buggy code cannot reach around.

    So every command passes through two independent limits here:

    1. **Rate limit on the command itself** — the commanded position may not move
       more than `max_speed · dt` per call, whatever it is asked for. A caller
       that suddenly demands a pose one radian away gets a ramp, not a jump.
    2. **Following-error limit** — the command may never run more than
       `max_lag` away from the *measured* position. This is the one that makes it
       genuinely low-level: it is anchored to physical reality rather than to any
       internal state, so it holds even if every variable above it is wrong.

    ⚠️ Why not clamp against the measured position alone (which would be simpler
    and stricter): the PD term is `kp · (command − measured)`, so a tight
    following-error limit is also a torque limit. Squeeze it too hard and the arm
    cannot overcome its own friction and goes sluggish. Two loose limits that
    each catch a different failure beat one tight limit that also breaks normal
    operation.

    This cannot prevent a *slow* wrong motion — nothing at this level can know
    that a direction is wrong. It bounds how fast anything can go wrong, which
    is what turns "dangerous" into "catchable".
    """

    def __init__(self, robot: Any, max_speed: float = 1.0, max_lag: float = 0.25):
        self._robot = robot
        self.max_speed = max_speed   # rad/s, per joint
        self.max_lag = max_lag       # rad, command vs measured
        self._last_cmd: Any = None
        self._last_t: float | None = None
        self.limited_cycles = 0      # how often a limit actually bit

    def __getattr__(self, name: str) -> Any:
        # Everything not overridden passes straight through.
        return getattr(self._robot, name)

    def command_joint_pos(self, q: Any) -> None:
        import numpy as np

        q = np.asarray(q, dtype=float)
        now = time.perf_counter()
        # Cap dt so a stalled loop cannot buy itself a huge movement budget.
        dt = 0.02 if self._last_t is None else min(0.05, max(1e-3, now - self._last_t))
        self._last_t = now

        measured = np.asarray(self._robot.get_joint_pos(), dtype=float)
        if self._last_cmd is None or len(self._last_cmd) != len(q):
            self._last_cmd = measured.copy()

        budget = self.max_speed * dt
        limited = self._last_cmd + np.clip(q - self._last_cmd, -budget, budget)
        limited = np.clip(limited, measured - self.max_lag, measured + self.max_lag)

        if not np.allclose(limited, q, atol=1e-6):
            self.limited_cycles += 1

        self._last_cmd = limited
        self._robot.command_joint_pos(limited)

    def resync(self) -> None:
        """Forget the command history and re-anchor to the measured position.

        ⛔ Call this on EVERY mode transition. The rate limiter is stateful, and
        stale state is exactly what caused the incident it exists to prevent —
        it would be absurd for the guard to carry the same class of bug.
        """
        import numpy as np

        self._last_cmd = np.asarray(self._robot.get_joint_pos(), dtype=float)
        self._last_t = None


def shutdown_robot(robot: Any) -> list[int]:
    """Stop cleanly and return the motor IDs actually confirmed disabled.

    ⛔ Order is the whole point.

    1. **Stop the control thread.** It runs at 250 Hz and will otherwise be
       mid-`set_control` when the bus closes underneath it, which produced a
       thread-death traceback on the first real run.
    2. **Disable the motors, while the bus is still open.** `close()` does not do
       this, despite announcing that it has.
    3. **Then** close.

    Returns the IDs whose `motor_off` genuinely succeeded — not a fixed list.
    Reporting "all motors disabled" without checking is the same class of lie
    this codebase has produced all day.
    """
    disabled: list[int] = []
    robot = getattr(robot, "_robot", robot)   # unwrap SafeRobot if present
    chain = getattr(robot, "motor_chain", None)

    if chain is not None:
        try:
            chain.running = False       # ask the control loop to stop
            time.sleep(0.15)            # let it finish the transaction in flight
        except Exception:  # noqa: BLE001, S110
            pass

        try:
            for motor_id, _ in chain.motor_list:
                try:
                    chain.motor_interface.motor_off(motor_id)
                    disabled.append(motor_id)
                except Exception:  # noqa: BLE001, S110
                    pass
        except Exception:  # noqa: BLE001, S110
            pass

    try:
        robot.close()
    except Exception:  # noqa: BLE001, S110
        pass
    return disabled
