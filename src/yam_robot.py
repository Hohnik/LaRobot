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
    return get_yam_robot(**kwargs), note


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
