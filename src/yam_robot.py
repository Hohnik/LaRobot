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
from dataclasses import dataclass
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

# ⛔⭐ THE MASS THAT MADE THE ARM FALL, 2026-08-10.
#
# `GripperType.NO_GRIPPER` does not merely leave motor 7 unenabled — it swaps the
# *dynamics model* used for gravity compensation. The bare arm XML gives its
# terminal `gripper` body `mass="1e-6"` (yam.xml:38), and the real mass is merged
# in from the gripper XML, which NO_GRIPPER replaces with a stub. Summing
# linear_4310.xml's inertials: body 0.553219 + two fingers at 0.0710042 =
# **0.695 kg**, at the very end of the arm.
#
# Measured consequence, in simulation, at the saved park pose:
#
#     gravity torque WITH gripper    [-0.00, -4.81, 6.34, 1.34, -0.07, -0.00] Nm
#     gravity torque WITHOUT gripper [-0.00, -2.67, 3.88, 0.49, -0.00,  0.00] Nm
#     joint 3 (elbow_pitch) short by  +2.47 Nm  =  39% of what it needs
#
# In `zero_gravity_mode=True` the constructor sets **kp = 0** and commands zero
# torque (motor_chain_robot.py:241), so `motor_torques = gravity_comp` alone
# (:366). There is no position term to take up a shortfall — the missing 2.47 Nm
# is an unopposed torque pulling the elbow down, and the arm folds forward. That
# is exactly what happened: GUIDE mode was entered with --no-gripper and the arm
# sank while the status line reported a calm 35 °C for 33 seconds.
#
# Passing `ee_mass` restores it: worst residual falls 2.465 -> 0.188 Nm (3% of the
# elbow's requirement), verified in simulation.
# ⚠️ `ee_inertia` is NOT usable — the SDK writes an `ipos` attribute that MuJoCo
# rejects ("Schema violation: unrecognized attribute: 'ipos'", should be `pos`).
# That is a bug in the vendored tree, so only the mass can be corrected, and the
# 0.188 Nm residual is the centre-of-mass offset we cannot express.
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
    """Express saved jaw limits in the frame `get_yam_robot()` will actually use.

    ⭐ THIS IS THE REAL FIX. An earlier version compared the saved limits against
    the RAW motor position and, finding them consistent, passed them through
    unchanged — which was wrong, because the robot does not use the raw position.

    Two shifts are involved and they are easy to conflate:

    **(a) The calibration frame.** `calibrate_gripper.py` records limits through a
    bare `DMChainCanInterface`, which applies no wrap correction. If the jaws were
    somewhere different when it ran, the recorded numbers can be a whole 2π out.

    **(b) The runtime frame.** `get_yam_robot()` adds ±2π to `motor_offset` at
    every construction, chosen from the motor's momentary position
    (`get_robot.py:268-274`), and every reported position is then `raw − offset`.
    **The limits are NOT given the same treatment**, so unless we apply it here,
    positions and limits live in frames 2π apart.

    Worked example, measured 2026-08-10:

        jaws raw                6.3235
        6.3235 > π  ⇒ runtime reports  6.3235 − 2π = 0.0403
        saved limits            [6.481, 1.231]        (un-shifted)
        normalised = (0.0403 − 6.481) / (1.231 − 6.481) = 1.227   ← outside [0,1]

    A normalised position outside [0,1] is clipped back to the nearest limit by
    `motor_chain_robot.py:390`, which commands the motor into a stop it is already
    past. That is what cooked motor 7 three times.

    With the runtime shift applied the limits become [0.198, −5.052] and the same
    position normalises to **0.030** — comfortably inside. As a cross-check, that
    range is what the very first calibration of the day measured independently:
    [0.0704, −5.0528].

    Returns None only if no ±2π placement brackets the jaws at all, which means
    the jaws really are outside their measured travel and a re-calibration is due.
    """
    import math

    lo, hi = min(saved), max(saved)

    # (a) put the recorded range in the same wrap frame as the raw reading
    base = None
    for k in (0.0, TWO_PI, -TWO_PI):
        if lo + k - margin <= raw_pos <= hi + k + margin:
            base = [v + k for v in saved]
            break
    if base is None:
        return None

    # (b) apply the wrap correction the runtime is about to apply to the position
    shift = -TWO_PI if raw_pos > math.pi else (TWO_PI if raw_pos < -math.pi else 0.0)
    return [v + shift for v in base]


def reconcile_gripper_limits(saved: list[float], raw_pos: float, margin: float = 0.3) -> list[float] | None:
    """Shift saved jaw limits into the frame the jaws are actually in, or give up.

    ⛔ THIS IS THE FIX FOR THE WORST BUG OF 2026-08-10, and the mechanism is worth
    understanding because it will recur anywhere raw motor positions are cached.

    `get_yam_robot()` applies a **±2π wrap correction at every construction**,
    chosen from wherever the motor happens to be sitting at that instant
    (`get_robot.py:268-274`). `calibrate_gripper.py` builds a `DMChainCanInterface`
    directly and gets **no** such correction. So the limits are written in one
    coordinate frame and read in another, and whether they happen to agree depends
    on the jaws' position when each ran.

    When they disagree, the consequence is not a wrong number — it is a cooked
    motor. `motor_chain_robot.py:390` force-clips every gripper command into
    `[min(limits), max(limits)]` **regardless of where the jaws are**. With the
    jaws at −1.380 and the range at [+1.231, +6.481], the gripper was commanded
    2.6 rad away and held there against a mechanical stop. 43 °C → 65 °C in five
    seconds.

    So: try the saved range shifted by 0, +2π and −2π. If one brackets the
    measured position, that is the same physical range expressed in this session's
    frame — return it, and nothing needs to move. If none does, the limits are
    genuinely stale and **None** is returned so the caller re-measures.

    ⚠️ Never "warn and continue" here. That is precisely what was done, and it is
    what burned the motor.
    """
    lo, hi = min(saved), max(saved)
    for shift in (0.0, TWO_PI, -TWO_PI):
        if lo + shift - margin <= raw_pos <= hi + shift + margin:
            return [saved[0] + shift, saved[1] + shift]
    return None


def advance_park_command(command: Any, target: Any, step: float) -> Any:
    """One cycle of a park trajectory: move the COMMAND toward the target.

    ⛔⭐ THE BUG THIS FIXES, and it matters because it is completely invisible.

    PARK used to command `measured + clip(target - measured, ±step)` — it
    re-anchored to where the arm actually was, every single cycle. The commanded
    position was therefore never more than **one step** ahead of reality:
    `0.4 rad/s × 0.01 s = 0.004 rad`, about **0.23°**.

    A position controller makes torque from the *error* between command and
    measurement. Capping that error at 0.23° caps the torque at `kp × 0.004`, which
    does not overcome static friction plus 4.3 kg of arm. So the arm does not move;
    because it does not move the measurement does not change; and because the
    measurement does not change the next cycle commands the same 0.23° offset.
    **A treadmill.** It printed "parking… 1.2 rad to go" indefinitely while the
    number barely moved, raised nothing, and read as a controller that was merely
    slow. Julien reported PARK broken twice before this was found.

    TELEOP never had the bug, and that contrast is the proof: it integrates from
    `prev_q`, the **previously commanded** target, never from the measurement. When
    the arm lags, its command keeps advancing, the error grows, and the torque grows
    with it until the joint moves.

    This does the same. The command runs ahead of the arm as far as it needs to, and
    `SafeRobot.max_lag` (0.25 rad) is what stops it running away — exactly the right
    place for that guard, and already there.

    ⚠️ Completion must therefore be judged from the **measured** position, not from
    this command. The command arrives first, always.
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
    """Build a PARK target from a saved pose that may not match this robot's shape.

    Returns `(target, warning_or_None)`. Pure, so it can be tested without an arm —
    which is the point, because the bug it fixes could only be found by reading.

    ⛔ TWO REAL DEFECTS LIVE HERE, both found on 2026-08-10 by reading the code.

    **1. A length mismatch used to drop the arm.** `config/park_pose.json` holds 7
    joints. Run with `--no-gripper` and the robot has 6, so the old
    `park_target - measured` raised `ValueError` — and that exception escaped the
    control loop, skipped the "the arm is HOLDING, press g or d" consent flow, and
    fell into `finally`, which **disables the motors**. A raised arm sags. And
    `--no-gripper` is precisely the escape hatch the gripper instructions tell you
    to fall back to, so the fallback was the broken path. Symmetrically, a pose
    saved *in* a no-gripper session has 6 entries and broke a later 7-DoF session.
    Fixed by starting from the measured pose and overlaying only the joints the
    saved pose actually carries: never invent a target for a joint we know nothing
    about.

    **2. PARK was the one path that bypassed the gripper clamp.** It commanded the
    saved jaw value directly, so a pose saved with the jaws resting on a mechanical
    stop would drive them back onto that stop and *hold* them there. Holding a
    position at a stop is stall torque — full current, no motion, no cooling — which
    is exactly how motor 7 cooked three times (FINDINGS §4, rule 1: never command
    the gripper to hold at a hard stop). `clamp` is applied here so no caller can
    forget it.
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
    """How much of the park speed to use right now — a trapezoidal ramp, in [floor, 1].

    Ease in over the first `ramp` radians, cruise, ease out over the last `ramp`. On a
    move shorter than `2 * ramp` the profile degenerates to a triangle, which is the
    correct thing for a short hop rather than a special case.

    ⭐ WHY THIS IS A SEPARATE FUNCTION AND NOT A CHANGE TO `advance_park_command`.
    That function is confirmed on hardware, has 15 tests, and its one job — *advance
    the COMMAND, never the measurement* — cost two sessions to get right. Smoothing
    does not need to touch it: this returns a scale factor and the caller multiplies
    its step. The trajectory integrator stays exactly as it is.

    ⚠️ AND THAT SHAPE REMOVES THE RISK I ORIGINALLY WORRIED ABOUT. The plan in
    ROADMAP 6.5 said easing should be opt-in because *"a deceleration bug shows up as
    overshoot — the arm arriving somewhere it was not aimed"*. That is true of a new
    integrator carrying velocity state. It is **not** true here: `advance_park_command`
    is `command + clip(target - command, -step, step)`, so a step is already bounded
    by the distance that remains. **Scaling that step DOWN cannot overshoot**, only
    slow down. Checked by reading it rather than assumed, which is why this is on by
    default with `--no-smooth` as the escape hatch.

    ⚠️ `floor` matters: without it the factor reaches zero at both ends and the arm
    creeps for ever, which the stall detector would eventually — and wrongly — call
    an obstruction.
    """
    if ramp <= 0:
        return 1.0
    return max(floor, min(1.0, travelled / ramp, remaining / ramp))


def resolve_park_legs(wanted: list[str], base: list | None, slots: dict[str, list],
                      ) -> tuple[list[tuple[str, list]], list[str]]:
    """Turn typed digits into `(legs, missing)` — the poses to visit, in order.

    `"0"` always means the **base** pose, whatever it is called in the file. Anything
    else is a waypoint. A digit with nothing saved behind it lands in `missing` and is
    **skipped**, never substituted.

    ⛔ Why skipped and not substituted: the tempting alternative — fall back to the
    base when a waypoint is empty — would send the arm somewhere the operator did not
    ask for, in the middle of a sequence they are watching. A pose the arm moves to is
    never a default.

    Duplicates are kept: `p 1 2 1 Enter` visits slot 1, slot 2, then slot 1 again,
    which is the obvious reading and makes a there-and-back trivial to type.
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
                 settled_band: float) -> str:
    """`"arrived"` · `"settled"` · `"blocked"` · `"moving"` — has the park finished?

    ⛔⭐ THE KNIFE EDGE THIS REMOVES, seen on hardware 2026-08-12. In one session the
    interleaved park reported **"PARK reached (0.020 rad off)"** and in the next the
    Ctrl-C park reported **"PARK STALLED — 0.021 rad still to go"**. Same arm, same
    pose, same code — and `PARK_TOLERANCE` is **0.02**. The arm was landing either
    side of the threshold by a thousandth of a radian.

    **That is not a fault, it is the noise floor.** A position-controlled arm holding
    itself against gravity has a steady-state error: the controller settles where its
    stiffness balances the load, a fraction of a degree short of the commanded pose.
    `park_target_from`'s own comment says so — *"it must allow for a position
    controller's steady-state error"* — and 0.02 rad (1.1°) turns out to sit right at
    it rather than safely above it.

    ⭐ **The fix is not a bigger number.** Simply loosening the tolerance would also
    make a genuinely obstructed arm look parked. What actually separates the two
    cases is **how far away it stopped**:

    - stopped improving and *close* → the controller has arrived as near as it can.
      That is `"settled"`, and it is a success.
    - stopped improving and *far* → something is in the way, or the pose is
      unreachable. That is `"blocked"`, and it must never be treated as arrival —
      it is the case that protects a hand or a clamp in the arm's path.

    So the threshold that was doing two jobs is split into two thresholds, each doing
    one. `tolerance` means "arrived cleanly"; `settled_band` means "close enough that
    the remaining error is the controller, not an obstruction".
    """
    if err < tolerance:
        return "arrived"
    if not stopped_improving:
        return "moving"
    return "settled" if err < settled_band else "blocked"


def park_slots(data: dict, arm: str) -> dict[str, list]:
    """Every saved pose for `arm`, keyed by slot name.

    ⭐ Julien's design, 2026-08-12: *"it would also make sense to have more options to
    save more positions … hit s and then a number every time we wanna save a position,
    and then hitting p and then the number would park to that position."*

    ⚠️ **Accepts the legacy shape.** `config/park_pose.json` used to be
    `{"B": [q0, q1, …]}` — one pose per arm, no slots — and that file is *measured
    calibration* that exists on the rig right now. A format change that silently
    dropped it would cost bench time to recreate, so a bare list is read as the
    `default` slot and keeps working exactly as before. `q p d` is unaffected.
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
    """`(temps, hottest, jaw)` from a chain's state list. Pure, so it can be tested.

    Each motor reports two temperatures — MOSFET and rotor — and the one that
    matters is whichever is higher, so they are combined per motor rather than
    picked between.

    ⚠️ `hottest` is **None** for an empty list, never 0.0. A temperature of zero is
    a plausible-looking reading that no motor in a warm room ever produces, and
    inventing one is how a thermal guard gets quietly disarmed — see `ThermalGuard`.

    ⭐ The gripper is returned SEPARATELY and is deliberately not folded into
    `hottest`. Motors 2 and 3 carry the arm's 4.3 kg and sit at 41-42 °C in normal
    equilibrium while an idle motor 7 is 31-36 °C, so a gripper climbing 33 → 41 °C
    is completely hidden behind the shoulder in a `max()`. Watching motor 7 plateau
    is the actual test of the 2π frame fix, and a test that cannot see the thing it
    tests is not a test (FINDINGS §0).
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
    """Turns motor temperatures into a decision — **including when it cannot read them.**

    ⛔⭐ THE DEFECT THIS REPLACES, found by reading on 2026-08-12 and never yet
    triggered on hardware, which is the only reason it is not in FINDINGS §0's table.

    The session used to read temperatures inside a bare `try`, and its handler was:

        except Exception:
            temps, hottest, jaw_temp = [], 0.0, None

    So **any** failure of `chain.read_states()` — a CAN hiccup, a decode error, a
    short state list — set the hottest motor to **0 °C**. The comparison
    `if hottest >= TEMP_STOP` then could not fire, thermal protection was gone for
    that cycle, and the status line printed a calm `hottest 0°C`. Nothing warned.
    If the read failed persistently the session would run to completion with no
    thermal guard at all, reassuring the operator the whole way.

    That is three of this repo's own rules at once: it warns-and-continues past a
    hazard (working contract rule 4), it is a guard with a path straight around it
    (rule 7), and it fails by lying rather than by crashing (FINDINGS §0). Motor 7
    has been cooked three times on this rig; the thermal guard is not decoration.

    **So: a failed read is not a temperature.** Blindness is reported the first time
    it happens and becomes a *stop* if it persists — because a session that cannot
    see temperatures has lost the thing standing between the gripper and a stall
    burn, and continuing is a decision nobody made deliberately.

    ⭐ It also actually issues the warning the session has always advertised.
    `TEMP_WARN = 55.0` was printed in the startup plan — *"temperature : warn 55°C,
    stop 65°C"* — and, verified by an exhaustive grep on 2026-08-12, **was used
    nowhere else in the codebase.** The session promised a warning it could not
    give. Same defect class as the refusal that named the wrong arm (FINDINGS §16):
    the text is right, the behaviour is absent, and only a user at the bench finds
    out.
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


def build_robot(
    arm: str = DEFAULT_ARM,
    *,
    zero_gravity: bool = False,
    allow_calibration: bool = False,
    with_gripper: bool = True,
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

    if not with_gripper:
        # ⛔⭐ DEFAULT: DO NOT CONTROL THE GRIPPER. This is the only reliable fix
        # for the failure that cooked motor 7 three separate times on 2026-08-10.
        #
        # The jaws end up PHYSICALLY OUTSIDE the calibrated range -- measured, the
        # normalised position read **1.186**, i.e. 18.6% beyond "fully open".
        # `command_joint_pos` clips that back to 1.0, which maps to the end stop,
        # so the motor is commanded into a stop it is already past and pushes at
        # 7.71 Nm indefinitely. Worse, it is self-reinforcing: 7.7 Nm shoves the
        # jaws even further beyond the 0.3 Nm calibration's idea of the limit, so
        # every cycle makes the next one worse.
        #
        # No clamp above this layer can help, because the vendor's clip happens
        # BELOW it (`motor_chain_robot.py:390`). Releasing the command to the
        # measured value does not help either -- 1.186 clips to 1.0 all the same.
        #
        # With NO_GRIPPER the motor is never enabled and never commanded. Its own
        # 400 ms timeout leaves it damped and free. The six arm joints are
        # entirely unaffected, so teleop works completely.
        #
        # Re-enabling gripper control needs the calibration reworked to measure
        # limits at the torque the RUNTIME uses, not at 0.3 Nm. See docs/FINDINGS.md.
        #
        # ⛔ `ee_mass` IS NOT OPTIONAL HERE. NO_GRIPPER swaps the gravity-compensation
        # model as well as dropping motor 7, and without this the elbow is short by
        # 39% of its holding torque — which in GUIDE mode (kp=0) drops the arm. See
        # GRIPPER_MASS_KG above for the measurement and the incident.
        kwargs: dict = dict(
            channel=chain_channel(arm),
            arm_type=ArmType.YAM,
            gripper_type=GripperType.NO_GRIPPER,
            zero_gravity_mode=zero_gravity,
            ee_mass=GRIPPER_MASS_KG,
            sim=False,
        )
        return SafeRobot(get_yam_robot(**kwargs)), (
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
        # ⛔ THE ARM NAME MUST BE IN THE COMMAND. This message used to read
        # "uv run scripts/calibrate_gripper.py --yes" with no --arm, so following it
        # literally would re-calibrate B — driving the WRONG arm's jaws into both
        # mechanical stops — while the arm you were actually trying to start stayed
        # uncalibrated and the same refusal came back. Julien hit exactly this on
        # G's first run. A remediation message that names the wrong target is
        # worse than no message: it converts a clean refusal into a wrong action.
        raise RuntimeError(
            f"No saved gripper limits for {arm!r} and calibration is not allowed.\n"
            f"  Run this once:  uv run scripts/calibrate_gripper.py --yes --arm {arm}\n"
            f"  It calibrates gently, saves the result, and every later start is silent.\n"
            f"  Or start without the gripper:  --no-gripper"
        )
    robot = get_yam_robot(**kwargs)

    # ⛔ VERIFY, do not trust. frame_correct_gripper_limits() predicts the wrap the
    # runtime will apply; this checks the prediction against what the runtime
    # actually reports, BEFORE any control loop starts. A normalised gripper
    # position outside [0,1] gets clipped onto a limit by motor_chain_robot.py:390
    # and the motor then pushes into a stop indefinitely — the failure that cooked
    # motor 7 three times. Better to refuse to start than to discover it thermally.
    try:
        norm = float(robot.get_joint_pos()[6])
        if not (-0.02 <= norm <= 1.02):
            for mid in (1, 2, 3, 4, 5, 6, 7):
                try:
                    robot.motor_chain.motor_interface.motor_off(mid)
                except Exception:  # noqa: BLE001, S110
                    pass
            try:
                robot.close()
            except Exception:  # noqa: BLE001, S110
                pass
            raise RuntimeError(
                f"⛔ GRIPPER FRAME CHECK FAILED — shut down before the control loop ran.\n"
                f"   The runtime reports a normalised jaw position of {norm:.3f}; it must be within [0,1].\n"
                f"   Anything outside is clipped onto a mechanical stop and held there, which is what\n"
                f"   cooked motor 7 on 2026-08-10. Limits passed were {[round(v, 3) for v in saved]}.\n"
                f"   Re-measure:  uv run scripts/calibrate_gripper.py --yes --arm {arm}\n"
                f"   Or run without it:  add --no-gripper to the session."
            )
        note += f"; jaws normalise to {norm:.3f} ✓"
    except RuntimeError:
        raise
    except Exception:  # noqa: BLE001, S110
        pass

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
