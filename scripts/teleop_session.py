#!/usr/bin/env python3
"""⭐ Interactive arm session: switch between hand-guiding and SpaceMouse at will.

    uv run scripts/teleop_session.py            # dry run: explains the keys
    uv run scripts/teleop_session.py --yes      # LIVE

⛔ MOVES THE WHOLE ARM. Desk clear, hand near the power.

    KEY   MODE
    g     GUIDE   — zero gravity. The arm is weightless; push it where you like.
    t     TELEOP  — the SpaceMouse drives the end effector.
    h     HOLD    — the arm holds its current pose. The safe idle.
    p     PARK    — slowly drive back to the saved park pose.
    s     save the current pose as the park pose.
    x/y/z flip that axis of the SpaceMouse mapping (saved on exit).
    +/-   faster / slower.
    ?     print this again.
    q     QUIT — goes to HOLD first and asks; it never just releases the arm.

WHY IT IS ONE SESSION AND NOT A SEQUENCE OF SCRIPTS
---------------------------------------------------
Julien, after the first run: *"it shouldn't be in phases. It should be more like
going forward and backward. I should be able to control when the weightless or
SpaceMouse-controlled things are happening."* He is right, and there is a safety
argument for it beyond convenience: **zero gravity cancels the arm's weight, so
the instant any process stops commanding, the weight is back.** Every gap between
scripts is a moment when a raised arm sags. One long-lived session with mode
switches has no gaps.

FOUR REAL FAILURES FROM THE PREVIOUS RUN, ALL FIXED HERE
--------------------------------------------------------
1. **The control thread died and the loop carried on regardless.** Motor 7 hit an
   over-temperature fault at t≈24 s; I2RT's control thread raised and exited. The
   teleop loop kept solving IK and kept calling `command_joint_pos` into a dead
   robot for another **64 seconds**, printing plausible EE numbers the whole time
   while the arm did nothing. Julien saw exactly that. **Now every cycle checks
   the chain is alive and stops the instant it is not.** A loop that cannot tell
   whether its commands are arriving is worse than one that crashes.
2. **The arm drooped when the thread died.** With no commands arriving, the
   motors' own 400 ms timeout damps them — so the arm sank slowly under gravity
   rather than holding. Now the session detects the death immediately and says so
   loudly, so a human can catch it while it is still a slow sag.
3. **Motor over-temperature came as a surprise.** Temperatures are now read every
   cycle from the chain, shown live, warned at 55 °C and stopped at 65 °C — below
   the firmware's own trip, so the session ends in a controlled way instead of
   the thread dying underneath it.
4. **Quitting released the arm on a timer.** A 5 s countdown is not consent. Now
   `q` moves to HOLD and waits for an explicit second key.

ON RECOVERING A DROOPED ARM — yes, and nothing is lost
------------------------------------------------------
The encoders report true joint positions at all times; that is why a hand-twist
of the gripper this morning read back exactly. When the arm drooped, the system
did **not** lose track of where it was — it lost the loop that was commanding it.
So PARK is safe: it reads the true current pose, then interpolates slowly to the
saved one. It is not dead reckoning and it cannot be "miscalibrated" by a droop.
⚠️ What it cannot know is what is now in the way, so it moves slowly and can be
stopped with any key.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "third_party" / "i2rt"))
from keyboard import KeyReader  # noqa: E402
from spacemouse import (  # noqa: E402
    TwistReader,
    countdown_hands_off,
    open_device,
    pick_device_by_wiggle,
)
from teleop import CartesianTeleop  # noqa: E402
from yam_can import ARM_SERIALS, DEFAULT_ARM, YAM_JOINTS  # noqa: E402
from yam_robot import build_robot, load_gripper_limits, shutdown_robot  # noqa: E402

CONTROL_HZ = 100.0
N_ARM = 6

# Faster than the first run, which Julien found "very slow". Still well short of
# what the hardware can do — this is a human-in-the-loop speed, not a limit.
LINEAR_SCALE = 0.12    # m/s at full deflection  (was 0.04)
ANGULAR_SCALE = 0.60   # rad/s at full deflection (was 0.25)

WORKSPACE_BOX = 0.30
MAX_JOINT_STEP = 0.015     # rad/cycle ≈ 1.5 rad/s at 100 Hz
JOINT_LIMIT_MARGIN = 0.08
TEMP_WARN = 55.0
TEMP_STOP = 65.0
PARK_SPEED = 0.25          # rad/s per joint when driving to the park pose

# ⛔ NEVER command the gripper to 0.0 or 1.0. Those are the mechanical stops, and
# holding a position AT a stop is stall torque: full current, no motion, no
# cooling. That is what cooked motor 7 twice on 2026-08-10 -- the arm was simply
# told to "hold where you are" while the jaws happened to be resting on a stop.
# Keeping the command inside this band means the jaws are always free to move,
# so a hold command costs almost no torque.
GRIPPER_MIN = 0.15
GRIPPER_MAX = 0.85
GRIPPER_STEP = 0.02        # per keypress

MAP_FILE = REPO / "config" / "spacemouse_map.json"
PARK_FILE = REPO / "config" / "park_pose.json"

HELP = """
  g GUIDE (weightless)   t TELEOP (spacemouse)   h HOLD   p PARK   s save park
  x/y/z flip axis        +/- speed               ?  help   q QUIT (asks first)
  o/c  open / close the gripper (TELEOP mode only)
"""


def load_json(path: Path, default):  # noqa: ANN001, ANN201
    try:
        return json.loads(path.read_text()) if path.exists() else default
    except Exception:  # noqa: BLE001
        return default


def save_json(path: Path, data) -> None:  # noqa: ANN001
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def chain_alive(robot) -> bool:  # noqa: ANN001
    """Is the robot still actually being commanded?

    ⛔ The single most important check in this file. I2RT's control thread raises
    and exits on a motor fault; nothing tells the caller. Without this, the loop
    keeps issuing commands into a corpse and reporting healthy-looking numbers,
    which is what happened for 64 s on 2026-08-10.
    """
    chain = getattr(robot, "motor_chain", None)
    if chain is None:
        return False
    return bool(getattr(chain, "running", False))


def main() -> int:  # noqa: PLR0915
    ap = argparse.ArgumentParser(description="Interactive YAM session: guide, teleop, park.")
    ap.add_argument("--yes", action="store_true", help="actually energise the arm")
    ap.add_argument("--arm", default=DEFAULT_ARM, choices=sorted(ARM_SERIALS))
    ap.add_argument("--start-mode", default="guide", choices=["guide", "hold", "teleop"])
    ap.add_argument("--rotation", action="store_true", help="map puck rotation to wrist rotation")
    ap.add_argument("--linear-scale", type=float, default=LINEAR_SCALE)
    ap.add_argument("--box", type=float, default=WORKSPACE_BOX)
    args = ap.parse_args()

    axis_map = load_json(MAP_FILE, {"sign": [1, 1, 1, 1, 1, 1]})
    sign = np.array(axis_map.get("sign", [1] * 6), dtype=float)
    park = load_json(PARK_FILE, {}).get(args.arm)

    print("=== plan ===")
    print(f"  ARM         : {args.arm}  (serial {ARM_SERIALS[args.arm]})")
    print(f"  jaw limits  : {load_gripper_limits(args.arm) or 'NONE — run calibrate_gripper.py first'}")
    print(f"  start mode  : {args.start_mode}")
    print(f"  speed       : {args.linear_scale} m/s linear, "
          f"{ANGULAR_SCALE if args.rotation else 0} rad/s angular")
    print(f"  axis signs  : {sign.astype(int).tolist()}  (x y z roll pitch yaw)")
    print(f"  park pose   : {np.round(park, 3).tolist() if park else 'none saved — press s to set one'}")
    print(f"  workspace   : ±{args.box} m box, re-centred whenever TELEOP is entered")
    print(f"  temperature : warn {TEMP_WARN}°C, stop {TEMP_STOP}°C")
    print(HELP)

    if not args.yes:
        print("DRY RUN — nothing transmitted, nothing energised. Re-run with --yes.")
        return 0

    info = pick_device_by_wiggle()
    if info is None:
        print("No SpaceMouse found (or none was moved).")
        return 1
    countdown_hands_off(3)
    handle = open_device(info)
    handle.set_nonblocking(True)
    reader = TwistReader(handle)

    robot = None
    mode = args.start_mode
    stop_reason: str | None = None
    teleop: CartesianTeleop | None = None
    home_ee = None
    gripper_value = 0.0
    park_target = None
    max_temp_seen = 0.0

    try:
        print("building robot — enables all 7 motors, starts the control loop …")
        robot, note = build_robot(args.arm, zero_gravity=(mode == "guide"))
        print(f"  {note}\n")
        chain = robot.motor_chain

        # ⛔ Stale-limit check. A power cycle can shift the gripper motor's position
        # reference, which leaves config/gripper_limits.json describing a range the
        # jaws are no longer inside. The normalised value then falls outside [0,1],
        # every hold command pushes toward a stop, and the motor stalls and cooks.
        # Measured after the 2026-08-10 power cycle: raw jaw position +1.6691 rad
        # against saved limits +0.0704 … -5.0528. Outside, by a lot.
        try:
            raw_jaw = chain.read_states()[N_ARM].pos
            lims = load_gripper_limits(args.arm)
            if lims is not None:
                lo_r, hi_r = min(lims), max(lims)
                if not (lo_r - 0.2 <= raw_jaw <= hi_r + 0.2):
                    print(f"⚠️  STALE GRIPPER LIMITS: jaws read {raw_jaw:+.3f} rad but the saved range is "
                          f"[{lo_r:+.3f}, {hi_r:+.3f}].")
                    print("   A power cycle shifts the position reference. Re-run:")
                    print("     uv run scripts/calibrate_gripper.py --yes")
                    print("   Continuing, but the gripper command is clamped so it cannot stall.\n")
        except Exception:  # noqa: BLE001, S110
            pass

        def clamp_gripper(v: float) -> float:
            return float(np.clip(v, GRIPPER_MIN, GRIPPER_MAX))

        def enter_teleop() -> None:
            nonlocal teleop, home_ee, gripper_value
            q = np.asarray(robot.get_joint_pos(), dtype=float)
            robot.command_joint_pos(q)          # leaves zero-gravity mode
            gripper_value = clamp_gripper(float(q[N_ARM]) if len(q) > N_ARM else 0.5)
            teleop = CartesianTeleop()
            teleop.reset(q[:N_ARM])
            home_ee = teleop.ee_position().copy()

        def enter_hold() -> None:
            robot.command_joint_pos(np.asarray(robot.get_joint_pos(), dtype=float))

        def enter_guide() -> None:
            # command_joint_pos put us in PD; this returns to weightless.
            for name in ("enable_gravity_comp", "set_zero_gravity_mode", "zero_gravity"):
                fn = getattr(robot, name, None)
                if callable(fn):
                    try:
                        fn()
                        return
                    except Exception:  # noqa: BLE001, S110
                        pass
            print("  ⚠️  could not re-enter zero gravity; staying in HOLD")

        if mode == "teleop":
            enter_teleop()
        elif mode == "hold":
            enter_hold()

        dt = 1.0 / CONTROL_HZ
        prev_q = np.asarray(robot.get_joint_pos(), dtype=float)[:N_ARM]
        t0 = time.perf_counter()
        next_report = 1.0

        with KeyReader() as keys:
            if not keys.enabled:
                print("⚠️  stdin is not a terminal — keys will not work. Ctrl-C still does.\n")
            print(f"⭐ MODE: {mode.upper()}\n")

            while True:
                loop_start = time.perf_counter()
                t = loop_start - t0

                # ---- 1. is the robot still there? -------------------------
                if not chain_alive(robot):
                    stop_reason = (
                        "the motor chain STOPPED — I2RT's control thread exited, almost "
                        "certainly on a motor fault. Commands are no longer reaching the arm."
                    )
                    break

                # ---- 2. temperatures --------------------------------------
                try:
                    states = chain.read_states()
                    temps = [max(getattr(s, "temp_mos", 0) or 0, getattr(s, "temp_rotor", 0) or 0)
                             for s in states]
                    hottest = max(temps)
                    max_temp_seen = max(max_temp_seen, hottest)
                    if hottest >= TEMP_STOP:
                        stop_reason = (
                            f"motor {temps.index(hottest) + 1} reached {hottest:.0f}°C "
                            f"(limit {TEMP_STOP}°C) — stopping before the firmware trips"
                        )
                        break
                except Exception:  # noqa: BLE001
                    temps, hottest = [], 0.0

                # ---- 3. keys ----------------------------------------------
                for k in keys.drain():
                    if k == "q":
                        stop_reason = "quit requested"
                    elif k == "g" and mode != "guide":
                        mode = "guide"; enter_guide(); print("\n⭐ MODE: GUIDE — arm is weightless\n")
                    elif k == "t" and mode != "teleop":
                        mode = "teleop"; enter_teleop(); print("\n⭐ MODE: TELEOP — SpaceMouse drives\n")
                    elif k == "h" and mode != "hold":
                        mode = "hold"; enter_hold(); print("\n⭐ MODE: HOLD\n")
                    elif k == "s":
                        q = np.asarray(robot.get_joint_pos(), dtype=float)
                        data = load_json(PARK_FILE, {}); data[args.arm] = q.tolist()
                        save_json(PARK_FILE, data); park = q.tolist()
                        print(f"\n  park pose saved: {np.round(q[:N_ARM], 3)}\n")
                    elif k == "p":
                        if park is None:
                            print("\n  no park pose saved — press s first\n")
                        else:
                            mode = "park"
                            park_target = np.asarray(park, dtype=float)
                            enter_hold()
                            print("\n⭐ MODE: PARK — driving slowly to the saved pose. Any key stops.\n")
                    elif k == "o" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value + GRIPPER_STEP)
                    elif k == "c" and mode == "teleop":
                        gripper_value = clamp_gripper(gripper_value - GRIPPER_STEP)
                    elif k in "xyz":
                        idx = "xyz".index(k)
                        sign[idx] *= -1
                        print(f"\n  axis {k} flipped → signs {sign.astype(int).tolist()}\n")
                    elif k == "+":
                        args.linear_scale *= 1.25
                        print(f"\n  speed → {args.linear_scale:.3f} m/s\n")
                    elif k == "-":
                        args.linear_scale /= 1.25
                        print(f"\n  speed → {args.linear_scale:.3f} m/s\n")
                    elif k == "?":
                        print(HELP)
                    elif mode == "park":
                        mode = "hold"; enter_hold(); print("\n⭐ PARK cancelled → HOLD\n")
                if stop_reason:
                    break

                # ---- 4. act on the mode -----------------------------------
                if mode == "teleop" and teleop is not None:
                    axes = np.array(reader.read(), dtype=float) * sign
                    twist = np.array([
                        axes[0] * args.linear_scale, axes[1] * args.linear_scale, axes[2] * args.linear_scale,
                        axes[3] * ANGULAR_SCALE if args.rotation else 0.0,
                        axes[4] * ANGULAR_SCALE if args.rotation else 0.0,
                        axes[5] * ANGULAR_SCALE if args.rotation else 0.0,
                    ])
                    q_target = teleop.step(twist, dt)

                    ee = teleop.ee_position()
                    if np.any(np.abs(ee - home_ee) > args.box):
                        import mink  # noqa: PLC0415
                        teleop.target = mink.SE3.from_rotation_and_translation(
                            rotation=teleop.target.rotation(),
                            translation=np.clip(ee, home_ee - args.box, home_ee + args.box),
                        )

                    step = q_target - prev_q
                    q_target = prev_q + np.clip(step, -MAX_JOINT_STEP, MAX_JOINT_STEP)

                    lo = np.array([YAM_JOINTS[i][1] for i in range(1, N_ARM + 1)]) + JOINT_LIMIT_MARGIN
                    hi = np.array([YAM_JOINTS[i][2] for i in range(1, N_ARM + 1)]) - JOINT_LIMIT_MARGIN
                    q_target = np.clip(q_target, lo, hi)

                    full = np.zeros(robot.num_dofs())
                    full[:N_ARM] = q_target
                    if robot.num_dofs() > N_ARM:
                        full[N_ARM] = clamp_gripper(gripper_value)
                    robot.command_joint_pos(full)
                    prev_q = q_target.copy()

                elif mode == "park" and park_target is not None:
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    delta = park_target - q
                    stepmax = PARK_SPEED * dt
                    if np.max(np.abs(delta)) < 0.01:
                        mode = "hold"; enter_hold()
                        print("\n⭐ PARK reached → HOLD\n")
                    else:
                        robot.command_joint_pos(q + np.clip(delta, -stepmax, stepmax))

                # ---- 5. report --------------------------------------------
                if t >= next_report:
                    next_report += 1.0
                    q = np.asarray(robot.get_joint_pos(), dtype=float)
                    extra = ""
                    if mode == "teleop" and teleop is not None:
                        extra = f"  EE {np.round(teleop.ee_position(), 3)}"
                    print(f"\r[{mode.upper():6}] t={t:6.1f}s  hottest {hottest:4.0f}°C"
                          f"  q {np.round(q[:N_ARM], 2)}{extra}   ", end="", flush=True)

                time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

            # ---- controlled shutdown -----------------------------------------
            print(f"\n\n⛔ stopping: {stop_reason}")
            if chain_alive(robot):
                enter_hold()
                print("\nThe arm is HOLDING its pose. It will not be released until you choose.")
                print("   g = go weightless so you can park it by hand")
                print("   d = disable now (⚠️ a raised arm will sag)")
                while True:
                    k = keys.get()
                    if k == "g":
                        enter_guide()
                        print("\n⭐ weightless — park the arm, then press d to disable.")
                    elif k == "d":
                        break
                    time.sleep(0.05)
                    if not chain_alive(robot):
                        print("\n⚠️  the chain died while waiting — disabling now.")
                        break
            else:
                print("⚠️  the chain is already dead, so the arm is NOT being commanded.")
                print("   It will be sagging under gravity. Support it now if it is raised.")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    except Exception as exc:  # noqa: BLE001
        print(f"\n⛔ {type(exc).__name__}: {exc}")
    finally:
        try:
            handle.close()
        except Exception:  # noqa: BLE001, S110
            pass
        save_json(MAP_FILE, {"sign": sign.astype(int).tolist()})
        if robot is not None:
            disabled = shutdown_robot(robot)
            print(f"\nmotors confirmed disabled: {disabled}")

    print(f"\nhottest motor seen this session: {max_temp_seen:.0f}°C")
    print(f"axis signs saved: {sign.astype(int).tolist()} → {MAP_FILE.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
