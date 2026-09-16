"""Construct, constrain and shut down YAM robots using the platform CAN backend.

Startup reuses verified gripper calibration or requires explicit calibration
permission. Failures after acquisition attempt cleanup before propagating.
SafeRobot applies rate/following-error limits beneath application modes.
Shutdown requests a stopped control thread, then disables motors before closing
the bus; its returned IDs report software confirmations, not physical proof.

Historical incidents and superseded explanations are preserved in
`docs/archive/robot-source-notes.md`; current cleanup evidence is in CLEANUP.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from yam.can import (
    DEFAULT_ARM,
    add_i2rt_to_path,
    chain_channel,
    patch_dm_driver_for_gs_usb,
)

from yam import REPO_ROOT  # ⛔ anchored ONCE in yam/__init__.py, never per file
GRIPPER_LIMITS_FILE = REPO_ROOT / "config" / "gripper_limits.json"

# Default software command-rate cap, in rad/s; every mode still passes SafeRobot.
# Changing a mode-level requested speed cannot bypass this lower-layer cap.
SAFE_MAX_SPEED = 1.0

# Maximum command lead over measured position, in radians. With position
# control this also bounds the error driving proportional torque; it is not a speed.
SAFE_MAX_LAG = 0.25

# Feedforward gain stays at or below 1.0. Higher gains produced measured jitter
# in the archived FINDINGS §68.6 experiments. Keep settings.LIVE_BOUNDS in sync.
VEL_FF_CEILING = 1.0


# 0.5 Nm is I2RT's default and is what Julien watched slam the stops. 0.3 is
# ~60% of it: still enough to reach both ends of a 6.57 rad stroke, noticeably
# gentler on arrival. Only ever used by scripts/calibrate_gripper.py, which runs
# once — not on every startup.
GENTLE_TEST_TORQUE = 0.3

# Physical gripper mass remains on the arm even when motor 7 is uncontrolled.
# The no-gripper dynamics model omits it, so build_robot supplies this end mass.
GRIPPER_MASS_KG = 0.695


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


TWO_PI = 6.283185307179586


def frame_correct_gripper_limits(saved: list[float], raw_pos: float, margin: float = 0.3) -> list[float] | None:
    """Express saved jaw limits in the wrap frame I2RT will use at startup.

    Find the unique shift in {0, +2π, -2π} whose range brackets raw_pos within
    margin; return None for no fit OR ambiguous fits. Preserve endpoint order
    (open/closed sense), then compensate I2RT's one-turn position wrap beyond
    ±π. Use this before constructing the normalized gripper, not after.
    """
    import math

    lo, hi = min(saved), max(saved)

    # Require one wrap placement, not the first match. Wider travel plus margin
    # can make two ±2π placements plausible; refuse that ambiguity.
    fits = [k for k in (0.0, TWO_PI, -TWO_PI)
            if lo + k - margin <= raw_pos <= hi + k + margin]
    if len(fits) > 1:
        return None
    base = [v + fits[0] for v in saved] if fits else None
    if base is None:
        return None

    # (b) apply the wrap correction the runtime is about to apply to the position
    shift = -TWO_PI if raw_pos > math.pi else (TWO_PI if raw_pos < -math.pi else 0.0)
    return [v + shift for v in base]


def reconcile_gripper_limits(saved: list[float], raw_pos: float, margin: float = 0.3) -> list[float] | None:
    """Shift limits into the raw reading's frame without I2RT startup wrapping.

    Return the uniquely matching {0, +2π, -2π} shift, preserving endpoint order.
    Return None when no placement or multiple placements fit within margin.
    Unlike frame_correct_gripper_limits, do not apply the subsequent I2RT wrap.
    """
    lo, hi = min(saved), max(saved)
    # ⛔ Same refusal as `frame_correct_gripper_limits`: if two shifts both bracket the
    # measured position, the choice would be a jaw SCALE decided by list order. See the
    # comment there for the arithmetic and for the measured headroom.
    fits = [shift for shift in (0.0, TWO_PI, -TWO_PI)
            if lo + shift - margin <= raw_pos <= hi + shift + margin]
    if len(fits) != 1:
        return None
    return [saved[0] + fits[0], saved[1] + fits[0]]


def advance_park_command(command: Any, target: Any, step: float) -> Any:
    """Advance each command coordinate toward target by at most step radians.

    This advances the previous COMMAND, not the measured pose. Re-anchoring
    each cycle to a lagging measurement would prevent steady progress.
    The caller supplies arrays of matching shape and a nonnegative step.
    """
    import numpy as np

    command = np.asarray(command, dtype=float)
    target = np.asarray(target, dtype=float)
    return command + np.clip(target - command, -step, step)


def park_target_from(
    measured: Any,
    saved: Any,
    gripper_index: int | None = None,
    clamp: Any = None,
) -> tuple[Any, str | None]:
    """Reconcile a saved pose with the current robot shape and clamp its jaw.

    Start from measured, replace shared coordinates with saved values, and
    leave extra live joints at their measured position. Return (target, warning)
    for mismatched lengths rather than failing halfway through a park. Apply
    the supplied jaw clamp only when that coordinate exists.
    """
    import numpy as np

    measured = np.asarray(measured, dtype=float)
    saved = np.asarray(saved, dtype=float)
    target = measured.copy()

    shared = min(len(measured), len(saved))
    target[:shared] = saved[:shared]

    warning = None
    if len(saved) != len(measured):
        warning = (
            f"the saved park pose has {len(saved)} joints and this robot has "
            f"{len(measured)} — parking the {shared} they share and leaving the rest "
            f"where they are"
        )

    if gripper_index is not None and clamp is not None and len(target) > gripper_index:
        target[gripper_index] = clamp(float(target[gripper_index]))

    return target, warning


def park_speed_factor(travelled: float, remaining: float, ramp: float,
                      floor: float = 0.15) -> float:
    """Return a trapezoidal speed factor using joint-space path distance.

    A nonpositive ramp disables easing. Otherwise ramp up with travelled and
    down with remaining, bounded below by floor and above by 1. The nonzero
    floor lets a path begin and finish without asymptotically stalling.
    """
    if ramp <= 0:
        return 1.0
    return max(floor, min(1.0, travelled / ramp, remaining / ramp))


def resolve_park_legs(wanted: list[str], base: list | None, slots: dict[str, list],
                      ) -> tuple[list[tuple[str, list]], list[str]]:
    """Resolve typed slot names into (ordered copied poses, missing names).

    Slot "0" means the base pose; other names use slots. Preserve repetitions
    and order for sequences such as 1,2,1. Report missing/empty poses separately.
    """
    legs: list[tuple[str, list]] = []
    missing: list[str] = []
    for name in wanted:
        pose = base if name == "0" else slots.get(name)
        if pose:
            legs.append((name, list(pose)))
        else:
            missing.append(name)
    return legs, missing


def park_verdict(err: float, stopped_improving: bool, tolerance: float,
                 settled_band: float, stopped_briefly: bool | None = None) -> str:
    """Classify measured arm error as arrived, settled, blocked or moving.

    Below tolerance: arrived. Below settled_band after a brief lack of
    improvement: settled (the friction floor). Otherwise a long lack of
    improvement means blocked. Legacy callers omitting stopped_briefly use
    the longer stopped_improving flag for both decisions. The caller must gate
    this on completion of the intended path, especially for a closed loop.
    """
    if stopped_briefly is None:
        stopped_briefly = stopped_improving
    if err < tolerance:
        return "arrived"
    if err < settled_band and stopped_briefly:
        return "settled"
    if stopped_improving:
        return "blocked"
    return "moving"


def park_slots(data: dict, arm: str) -> dict[str, list]:
    """Read an arm's saved poses from current or legacy JSON.

    A legacy list becomes the "default" base slot. A dictionary contributes
    only nonempty list values; an absent arm has no slots.
    """
    entry = data.get(arm)
    if entry is None:
        return {}
    if isinstance(entry, list):
        return {"default": entry}                      # the pre-slots format
    return {k: v for k, v in entry.items() if isinstance(v, list) and v}


def with_park_slot(data: dict, arm: str, slot: str, pose: list) -> dict:
    """`data` with `pose` stored in `slot` for `arm`, migrating the legacy shape.

    Returns a new dict rather than mutating, so a caller can compare before writing —
    the axis-map file was once overwritten with mangled values, and the lesson taken
    from it was to make "did this actually change?" answerable.
    """
    updated = dict(data)
    slots = dict(park_slots(data, arm))
    slots[slot] = list(pose)
    updated[arm] = slots
    return updated


def motor_temperatures(states: Any, gripper_index: int) -> tuple[list[float], float | None, float | None]:
    """Return (per-motor temperatures, hottest, jaw) from chain states.

    For each motor use the larger MOS/rotor reading; missing sensor attributes
    are treated as zero. An empty chain yields no hottest/jaw value. The caller
    must handle a failed chain read separately; this helper does not detect it.
    """
    temps = [float(max(getattr(s, "temp_mos", 0) or 0, getattr(s, "temp_rotor", 0) or 0))
             for s in states]
    hottest = max(temps) if temps else None
    jaw = temps[gripper_index] if len(temps) > gripper_index else None
    return temps, hottest, jaw


@dataclass
class ThermalVerdict:
    """What the guard wants done this cycle. Both fields may be None."""

    stop_reason: str | None = None
    warning: str | None = None


class ThermalGuard:
    """Track thermal warnings, maxima and stops, including unreadable cycles.

    None increments the blind counter and eventually requests a stop. A valid
    read resets blindness. Hot warnings rearm only below warn_at minus the
    hysteresis margin. Reaching stop_at requests stopping immediately. The
    caller executes the stop; this object never commands motors.
    Blind-stop text estimates seconds at 100 Hz; the guard itself counts cycles.
    """

    def __init__(self, warn_at: float = 55.0, stop_at: float = 65.0,
                 blind_cycles: int = 100, rearm_below: float = 3.0) -> None:
        self.warn_at = warn_at
        self.stop_at = stop_at
        # 100 cycles is 1 s at the 100 Hz control rate. A single dropped read is a
        # bus hiccup and must not end a session; a second of silence means the
        # instrument is gone, not noisy.
        self.blind_cycles = blind_cycles
        # Hysteresis, so a motor sitting exactly on the warn line does not print a
        # warning every cycle and bury the rest of the readout.
        self.rearm_below = rearm_below
        self.blind = 0
        self.max_seen = 0.0
        self.max_jaw_seen = 0.0
        self._warned_blind = False
        self._warned_hot = False

    def update(self, hottest: float | None, jaw: float | None = None,
               motor: int | None = None) -> ThermalVerdict:
        """Observe one cycle. `hottest=None` means **the read failed**, not 0 °C."""
        if hottest is None:
            self.blind += 1
            if self.blind >= self.blind_cycles:
                return ThermalVerdict(stop_reason=(
                    f"motor temperatures have been unreadable for {self.blind} cycles "
                    f"({self.blind / 100:.1f}s). The thermal guard is the only thing "
                    "between the gripper and a stall burn, so this stops rather than "
                    "running blind"))
            if not self._warned_blind:
                self._warned_blind = True
                return ThermalVerdict(warning=(
                    "cannot read motor temperatures — the thermal guard is BLIND. "
                    f"Stopping if this lasts {self.blind_cycles} cycles"))
            return ThermalVerdict()

        if self._warned_blind:
            self._warned_blind = False
            self.blind = 0
            recovered = ThermalVerdict(warning="motor temperatures readable again")
        else:
            self.blind = 0
            recovered = ThermalVerdict()

        self.max_seen = max(self.max_seen, hottest)
        if jaw is not None:
            self.max_jaw_seen = max(self.max_jaw_seen, jaw)

        if hottest >= self.stop_at:
            where = f"motor {motor + 1}" if motor is not None else "a motor"
            return ThermalVerdict(stop_reason=(
                f"{where} reached {hottest:.0f}°C (limit {self.stop_at:.0f}°C) — "
                "stopping before the firmware trips"))
        if hottest >= self.warn_at:
            if not self._warned_hot:
                self._warned_hot = True
                where = f"motor {motor + 1}" if motor is not None else "a motor"
                return ThermalVerdict(warning=(
                    f"{where} is at {hottest:.0f}°C, past the {self.warn_at:.0f}°C "
                    f"warning line. It stops at {self.stop_at:.0f}°C"))
        elif hottest < self.warn_at - self.rearm_below:
            self._warned_hot = False
        return recovered


def read_raw_gripper_position(arm: str) -> float | None:
    """Raw jaw motor position, read the same way the robot will read it."""
    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()
    from i2rt.motor_drivers.dm_driver import ControlMode, DMSingleMotorCanInterface, MotorType

    iface = DMSingleMotorCanInterface(
        control_mode=ControlMode.MIT, channel=chain_channel(arm), name="jawread"
    )
    try:
        info = iface.motor_on(7, MotorType.DM4310)
        return float(info.position)
    except Exception:  # noqa: BLE001
        return None
    finally:
        try:
            iface.motor_off(7)
        except Exception:  # noqa: BLE001, S110
            pass
        iface.close()


def _close_failed_build(robot: Any, failure: BaseException, n_motors: int) -> None:
    """Release a handle that cannot be returned, preserving the startup error."""
    try:
        disabled = shutdown_robot(robot)
        missing = sorted(set(range(1, n_motors + 1)) - set(disabled))
        if missing:
            failure.add_note(f"Could not confirm motors {missing} disabled after startup failed.")
    except Exception as cleanup_error:  # noqa: BLE001
        failure.add_note(f"Could not confirm robot shutdown: {cleanup_error}")


def build_robot(
    arm: str = DEFAULT_ARM,
    *,
    zero_gravity: bool = False,
    allow_calibration: bool = False,
    with_gripper: bool = True,
    max_speed: float = SAFE_MAX_SPEED,
    max_lag: float = SAFE_MAX_LAG,
) -> tuple[Any, str]:
    """Return (rate-limited robot, startup note) after validating the gripper.

    With a gripper, require saved limits unless allow_calibration is explicit;
    reconcile startup wrapping and verify the normalized position after build.
    The explicit no-gripper path uses six controlled joints but retains the
    physical gripper mass in the gravity model. Neither path changes calibration
    values on disk. Failures after get_yam_robot returns attempt motor shutdown;
    failures internal to that constructor remain its responsibility.
    """
    add_i2rt_to_path()
    patch_dm_driver_for_gs_usb()

    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType

    if not with_gripper:
        # Explicit six-joint operation leaves motor 7 uncontrolled. Retain the physical
        # gripper mass in the gravity model; omitting it under-compensates the loaded arm.
        kwargs: dict = dict(
            channel=chain_channel(arm),
            arm_type=ArmType.YAM,
            gripper_type=GripperType.NO_GRIPPER,
            zero_gravity_mode=zero_gravity,
            ee_mass=GRIPPER_MASS_KG,
            sim=False,
        )
        robot = get_yam_robot(**kwargs)
        try:
            wrapped = SafeRobot(robot, max_speed=max_speed, max_lag=max_lag)
        except BaseException as failure:
            _close_failed_build(robot, failure, 6)
            raise
        return wrapped, (
            f"gripper NOT controlled (6 DoF) — motor 7 is left free, and the gravity model "
            f"carries ee_mass={GRIPPER_MASS_KG} kg so the arm still holds itself "
            f"(~0.19 Nm residual at the elbow)."
        )

    kwargs: dict = dict(
        channel=chain_channel(arm),
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=zero_gravity,
        sim=False,
    )

    saved = load_gripper_limits(arm)
    if saved is not None:
        # ⛔ Verify the saved limits describe the frame the jaws are ACTUALLY in
        # before handing them to a layer that will force-clip every command into
        # them. See reconcile_gripper_limits() for why this is not optional.
        raw = read_raw_gripper_position(arm)
        if raw is not None:
            fixed = frame_correct_gripper_limits(saved, raw)
            if fixed is None:
                raise RuntimeError(
                    f"⛔ STALE GRIPPER LIMITS — refusing to start.\n"
                    f"   The jaws are at {raw:+.3f} rad; the saved range is "
                    f"[{min(saved):+.3f}, {max(saved):+.3f}], and no ±2π shift reconciles them.\n"
                    f"   MotorChainRobot force-clips every gripper command into that range whatever\n"
                    f"   the jaws are doing, so continuing would drive the gripper into a stop and\n"
                    f"   cook it -- which is exactly what happened on 2026-08-10.\n"
                    f"   Re-measure:  uv run scripts/calibrate_gripper.py --yes --arm {arm}"
                )
            if fixed != saved:
                note_shift = f" (shifted by {fixed[0] - saved[0]:+.3f} rad to match this session's frame)"
                saved = fixed
            else:
                note_shift = ""
        else:
            note_shift = " (could not verify against the jaws — read failed)"
        kwargs["gripper_limits_override"] = saved
        note = f"jaw limits {[round(v, 3) for v in saved]} verified against the jaws{note_shift}"
    elif allow_calibration:
        note = "⚠️ NO saved jaw limits — the jaws WILL be driven into both stops to find them"
    else:
        # Name the arm in calibration instructions so the operator cannot recalibrate
        # a different arm by following a default.
        raise RuntimeError(
            f"No saved gripper limits for {arm!r} and calibration is not allowed.\n"
            f"  Run this once:  uv run scripts/calibrate_gripper.py --yes --arm {arm}\n"
            f"  It calibrates gently, saves the result, and every later start is silent.\n"
            f"  Or start without the gripper:  --no-gripper"
        )
    robot = get_yam_robot(**kwargs)

    # The builder owns the enabled handle until it can return a verified wrapper.
    # A failed read is not evidence that the gripper frame is safe to command.
    try:
        norm = float(robot.get_joint_pos()[6])
        if not (-0.02 <= norm <= 1.02):
            raise RuntimeError(
                f"GRIPPER FRAME CHECK FAILED: normalised jaw position {norm:.3f}; "
                "expected a value within [0,1]. Recalibrate before using the gripper."
            )
        wrapped = SafeRobot(robot, max_speed=max_speed, max_lag=max_lag)
        note += f"; jaws normalise to {norm:.3f} ✓"
        return wrapped, note
    except BaseException as failure:
        # Includes interruption between acquisition and handing ownership to the caller.
        _close_failed_build(robot, failure, 7)
        raise



class SafeRobot:
    """Apply command-rate and measured-following-error limits to every mode.

    Advance from the previous command within max_speed * dt, then clip against
    the current measured pose ± max_lag. dt is bounded to 1–50 ms (20 ms for the
    first command). Other robot attributes delegate to the wrapped handle.

    Optional velocity feedforward derives from limited commands, caps its gain
    at VEL_FF_CEILING, suppresses jaw velocity and clears velocity pointing past
    the command. Missing command_joint_state falls back to position-only once
    reported. Call resync on mode transitions to discard stale command/velocity
    history. This wrapper is not a collision checker or a workspace planner.
    """

    def __init__(self, robot: Any, max_speed: float = SAFE_MAX_SPEED,
                 max_lag: float = SAFE_MAX_LAG):
        self._robot = robot
        self.max_speed = max_speed   # rad/s, per joint
        self.max_lag = max_lag       # rad, command vs measured
        # Velocity feedforward defaults off; resync clears its history on mode changes.
        self.vel_ff = 0.0
        self._ff_warned = False
        self._ff_prev: Any = None    # last SENT velocity setpoint, for the smoothing
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

        prev_cmd = self._last_cmd
        self._last_cmd = limited

        # Derive velocity from limited command changes, capped by VEL_FF_CEILING.
        ff = min(float(self.vel_ff), VEL_FF_CEILING)
        if ff <= 0.0:
            self._robot.command_joint_pos(limited)
            return
        vel = (limited - prev_cmd) / dt * ff
        # ⛔ The jaw (index 6 in this stack) NEVER gets feedforward. On a jaw squeezing an
        # object the extra torque pushes harder into it, which is how motor 7 was cooked.
        vel[6:] = 0.0
        # ⭐ Smooth the setpoint over ~2 cycles. The derivative of a 90 Hz stepped command
        # carries step noise, and at release this decays the push over a few cycles
        # instead of cutting it in one — the small "latent movement" he felt below gain 1
        # was the unsmoothed cut. A tuning constant, not a measured one — verify on arm.
        if self._ff_prev is None or len(self._ff_prev) != len(vel):
            self._ff_prev = vel.copy()
        else:
            self._ff_prev = 0.5 * vel + 0.5 * self._ff_prev
        # Clear velocity pointing beyond the position command, including smoothing
        # memory, so feedforward cannot keep pushing an overshot joint.
        past = np.sign(self._ff_prev) * np.sign(limited - measured) < 0.0
        self._ff_prev[past] = 0.0
        vel = self._ff_prev.copy()
        send = getattr(self._robot, "command_joint_state", None)
        if send is None:
            # ⚠️ Said ONCE rather than silently dropped: a set gain that quietly does
            # nothing is the fails-by-lying pattern (FINDINGS §0). The arm still moves,
            # position-only, exactly as with vel_ff = 0.
            if not self._ff_warned:
                self._ff_warned = True
                print("⚠️  vel_ff is set but this robot has no command_joint_state — "
                      "feedforward is OFF for it, position-only commands continue.")
            self._robot.command_joint_pos(limited)
            return
        send({"pos": limited, "vel": vel})

    def resync(self) -> None:
        """Forget the command history and re-anchor to the measured position.

        ⛔ Call this on EVERY mode transition. The rate limiter is stateful, and
        stale state is exactly what caused the incident it exists to prevent —
        it would be absurd for the guard to carry the same class of bug.
        """
        import numpy as np

        self._last_cmd = np.asarray(self._robot.get_joint_pos(), dtype=float)
        # ⛔ The smoothed feedforward is state too: carrying it across a mode change
        # would push the first command of the new mode with the OLD mode's speed —
        # the park-spasm family (FINDINGS §66.0), one layer down.
        self._ff_prev = None
        self._last_t = None


def shutdown_robot(robot: Any) -> list[int]:
    """Attempt thread stop, motor disable and bus close; return confirmed IDs.

    Unwrap SafeRobot, set chain.running=False and allow 150 ms for its loop to
    stop before issuing each motor_off. Continue after individual failures and
    attempt robot.close even if chain cleanup failed. Only successful motor_off
    calls contribute IDs; the caller must report missing confirmations.
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


@dataclass(frozen=True)
class GraspCheck:
    """Whether the jaws are holding something, judged from where they stopped closing."""

    holding: bool
    gap: float                  # how far from the commanded position they stopped, 0-1 of stroke
    confident: bool             # False when the question cannot be answered from this input
    why: str


def check_grasp(commanded: float, measured: float, settled: bool,
                closed_enough: float = 0.25, hold_threshold: float = 0.03) -> GraspCheck:
    """Infer a grasp from normalized jaw position after a closing leg settles.

    If motion is unsettled or the command was not sufficiently closed, return
    an uncertain assessment. Otherwise measured-commanded above hold_threshold
    means something prevented full closure; below it means empty closure.
    This is a geometric heuristic, not an object/force sensor.
    """
    if not settled:
        return GraspCheck(False, 0.0, False, "the jaws had not finished moving")
    if commanded > closed_enough:
        return GraspCheck(False, 0.0, False,
                          f"the jaws were commanded to {commanded:.2f}, not closed, "
                          "so their position says nothing about an object")
    gap = measured - commanded
    if gap > hold_threshold:
        return GraspCheck(True, gap, True,
                          f"the jaws stopped {gap:.3f} of the stroke short of closed, "
                          "so something is between them")
    return GraspCheck(False, max(0.0, gap), True,
                      "the jaws closed onto themselves, so they are empty")
