"""Cartesian teleoperation: a 6-DoF twist in, joint targets out.

This is the piece `docs/Setup-Plan.md` §4.2 says we have to build ourselves, and
the single largest deviation from the papers. ABC/ENPIRE/RoboTTT all teleoperate
with GELLO leader arms, which are passive copies of the robot and therefore hand
over **joint angles** directly. A SpaceMouse hands over a **cartesian twist** of
the end effector instead, and there is no fixed mapping between the two — it
depends on the arm's current configuration. Computing it is inverse kinematics:

    twist  →  integrate into a target EE pose  →  IK  →  joint targets  →  arm

⭐ Deliberately robot-agnostic. It talks to anything exposing I2RT's `Robot`
interface, which `get_yam_robot(sim=True)` and `get_yam_robot(sim=False)` both
do. The same code therefore drives the simulated arm and the real one, and
moving between them is a flag rather than a rewrite — so a sign error costs
nothing the first time it happens.

WHY THE TARGET POSE IS INTEGRATED, NOT COMMANDED DIRECTLY
---------------------------------------------------------
A SpaceMouse reports *deflection*, i.e. a velocity command, not a position.
Releasing it returns to zero, which must mean "stop", not "go home". So the
target pose is state that we integrate the twist into: push and the target
drifts, release and it stays where it was. This also makes a deadman trivial —
zero deflection integrates nothing.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mink
import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
I2RT_MODELS = REPO_ROOT / "third_party" / "i2rt" / "i2rt" / "robot_models" / "arm"

# The 4310 gripper variant, matching the measured hardware (README §6.0), and the
# only YAM model that ships an end-effector site for IK to aim at.
DEFAULT_MODEL = I2RT_MODELS / "yam" / "v1" / "yam_linear_4310_d405.xml"
DEFAULT_EE_SITE = "tcp_site"

N_ARM_JOINTS = 6  # joints 1-6; the gripper is commanded separately, not by IK

# ⭐ The frames a twist can be expressed in. `world` is the original behaviour and
# stays the default; the others exist because Julien wants to drive while watching a
# wrist camera, and *"push forward"* then means forward IN THE IMAGE, which turns
# with the wrist. See `CartesianTeleop._twist_to_world()`.
FRAMES = {
    "world": None,              # base frame: +X out from the base, +Y left, +Z up
    "tool": ("tcp_site", "site"),
    "camera": ("camera", "body"),
}


class CartesianTeleop:
    """Integrates a twist into an EE pose and solves IK for joint targets.

    Costs are the tuning surface. `position_cost` and `orientation_cost` trade
    translation accuracy against rotation accuracy; `posture_cost` is a weak pull
    toward a reference configuration that resolves the redundancy and, more
    importantly, keeps the solver away from singular configurations where joint
    velocities blow up.
    """

    def __init__(
        self,
        model_path: Path = DEFAULT_MODEL,
        ee_site: str = DEFAULT_EE_SITE,
        position_cost: float = 1.0,
        # ⛔⭐ 0.05, NOT 0.5. Lowered 10x on 2026-08-11 after measuring, and the
        # measurement is counter-intuitive enough to be worth keeping:
        #
        #   pos:ori    pure-roll tool wander    rotation achieved
        #   1.0:0.5          0.443 m                  7.9 deg     <- the old default
        #   1.0:0.2          0.034 m                129.5 deg
        #   1.0:0.05         0.002 m                134.6 deg     <- now
        #   1.0:0.01         0.000 m                 18.2 deg
        #
        # The old default was the WORST OF BOTH: it wandered 44 cm *and* achieved
        # the least rotation. A higher orientation cost produced LESS rotation,
        # because the effort went into satisfying an unreachable orientation by
        # translating, which drags the arm into a configuration that can rotate
        # even less. Verified at three different starting poses; small rotations
        # are unaffected and translation reach is unchanged (0.319 -> 0.320 m).
        #
        # The priority this encodes, in one line: **never sacrifice where the tool
        # IS to chase where it POINTS.** A wrist that cannot turn should simply not
        # turn — it should not drag the whole arm across the desk.
        orientation_cost: float = 0.05,
        posture_cost: float = 1e-2,
        lm_damping: float = 1.0,
        solver: str = "daqp",
        damping: float = 1e-3,
        max_lead_m: float = 0.05,
        max_lead_rad: float = 0.25,
        frame: str = "world",
        max_joint_rate: float = 0.9,
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.ee_site = ee_site
        self.solver = solver
        self.damping = damping

        self.configuration = mink.Configuration(self.model)
        self.ee_task = mink.FrameTask(
            frame_name=ee_site,
            frame_type="site",
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping,
        )
        self.posture_task = mink.PostureTask(self.model, cost=posture_cost)
        self.tasks = [self.ee_task, self.posture_task]

        # Joint limits enforced inside the QP, so the solver never proposes a
        # velocity that would drive a joint past its stop. Cheaper and safer than
        # clamping afterwards, which would silently distort the requested motion.
        try:
            self.limits: list = [mink.ConfigurationLimit(self.model)]
        except Exception:  # noqa: BLE001
            self.limits = []

        # ⛔⭐ ANTI-WINDUP. How far the integrated goal may run ahead of the pose the
        # arm has actually achieved. This is `SafeRobot.max_lag` one layer up, and it
        # exists for a measured reason — see `step()`.
        self.max_lead_m = max_lead_m
        self.max_lead_rad = max_lead_rad

        # ⭐ WHICH FRAME THE PUCK'S TWIST IS EXPRESSED IN. See `_twist_to_world()`.
        if frame not in FRAMES:
            raise ValueError(f"frame must be one of {sorted(FRAMES)}, got {frame!r}")
        self.frame = frame

        # ⭐ Slow the puck down when the arm cannot keep up. See `_apply_speed_scale`.
        # 0.9 rad/s sits just under SafeRobot's 1.0 cap, so the twist is reined in
        # BEFORE the rate limiter has to intervene — the limiter stays a guard rather
        # than becoming part of normal operation.
        self.max_joint_rate = max_joint_rate
        self.speed_scale = 1.0
        #: ⭐ The joint rate the last IK step ASKED for, before throttling — the measured
        #: quantity behind any SLOWED message. The old message asserted "near the reach
        #: limit" as the cause and was wrong on a comfortable pose (FINDINGS §41.2);
        #: showing this number instead lets the operator tell a singular pose (spikes
        #: only when extended) from an over-eager linear speed (high everywhere).
        self.requested_rate = 0.0

        self.target: mink.SE3 | None = None

    def reset(self, q_arm: np.ndarray) -> None:  # noqa: D401
        """Seed the IK state from the arm's *measured* joint positions.

        Always seed from reality rather than from zero: the IK's notion of where
        the arm is has to match where it actually is, or the very first solve
        commands a jump.
        """
        q = np.zeros(self.model.nq)
        n = min(len(q_arm), N_ARM_JOINTS)
        q[:n] = np.asarray(q_arm)[:n]
        self.configuration.update(q)
        self.speed_scale = 1.0        # a mode change must not inherit a throttle
        self.requested_rate = 0.0
        self.posture_task.set_target_from_configuration(self.configuration)
        self.target = self.configuration.get_transform_frame_to_world(self.ee_site, "site")

    def step(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """Advance by one control cycle. Returns joint targets for joints 1-6.

        `twist` is [vx, vy, vz, wx, wy, wz] — linear m/s and angular rad/s,
        expressed in the world frame.
        """
        if self.target is None:
            raise RuntimeError("reset() must be called with the arm's measured joint positions first")

        twist = np.asarray(twist, dtype=float)
        twist = self._twist_to_world(twist) * self.speed_scale
        lin, ang = twist[:3] * dt, twist[3:] * dt

        # World-frame integration: rotation pre-multiplies, so a twist means the
        # same thing regardless of how the gripper happens to be oriented. Body
        # frame would be more natural to a hand holding the puck, and is a
        # deliberate later choice — not something to leave ambiguous now.
        self.target = mink.SE3.from_rotation_and_translation(
            rotation=mink.SO3.exp(ang).multiply(self.target.rotation()),
            translation=self.target.translation() + lin,
        )
        self._limit_lead()
        self.ee_task.set_target(self.target)

        vel = mink.solve_ik(
            self.configuration,
            self.tasks,
            dt,
            self.solver,
            damping=self.damping,
            limits=self.limits,
        )
        q_before = np.array(self.configuration.q[:N_ARM_JOINTS], dtype=float)
        self.configuration.integrate_inplace(vel, dt)
        q_after = np.array(self.configuration.q[:N_ARM_JOINTS], dtype=float)
        self._apply_speed_scale(q_after - q_before, dt)
        return q_after

    def _apply_speed_scale(self, joint_step: np.ndarray, dt: float) -> None:
        """Throttle the puck when the arm physically cannot follow it.

        ⛔⭐ THE PROBLEM, measured 2026-08-11. Julien: *"at high speeds the arm takes
        longer to follow the path that it's been told to move… I can only really
        control it at speeds of less than half a meter per second."*

        Pushing +X at 0.25 m/s from the home pose, watching the Jacobian's smallest
        singular value (`sigma_min`, which measures how close the arm is to a
        configuration where some direction of motion becomes unreachable):

            cycle  20   joint 0.68 rad/s   moved 0.05 m   sigma_min 0.170
            cycle 100   joint 1.34 rad/s   moved 0.25 m   sigma_min 0.121
            cycle 140   joint 2.93 rad/s   moved 0.35 m   sigma_min 0.048
            cycle 180   joint 0.12 rad/s   moved 0.38 m   sigma_min 0.005   (stalled)

        **The requested joint speed is not constant — it escalates as the arm
        extends.** The same 0.25 m/s at the tip costs 0.68 rad/s in the middle of the
        workspace and 2.93 rad/s near full reach, because as `sigma_min` collapses
        the arm must move its joints ever faster to produce the same tip motion.
        `SafeRobot` caps commands at 1.0 rad/s, so beyond that point the command is
        throttled, the arm falls behind, and it feels like latency.

        ⚠️ So this is **not really a speed problem**. Speed only decides how quickly
        you reach the part of the workspace where it happens. Raising the cap would
        not fix it either — it would just move the wall, at the cost of the guard
        that makes a wrong motion catchable, on a rig with no e-stop.

        **The fix is to ask for less.** If the last cycle wanted more joint speed
        than allowed, scale the twist by exactly the ratio; because tip speed and
        joint speed are locally proportional, that lands on the allowed rate in one
        step. Recovery is deliberately slower than reduction (5% per cycle, so ~0.2 s
        to return to full) — reacting instantly in both directions would oscillate at
        the boundary, which would feel worse than the lag it replaces.

        The result is that the arm slows down smoothly near its limits instead of
        silently lagging, and `speed_scale` is reported so the operator can see it
        happening rather than wonder why the arm feels heavy.
        """
        if dt <= 0:
            return
        requested = float(np.max(np.abs(joint_step))) / dt
        self.requested_rate = requested
        if requested > self.max_joint_rate:
            self.speed_scale = max(0.05, self.speed_scale * self.max_joint_rate / requested)
        elif self.speed_scale < 1.0:
            self.speed_scale = min(1.0, self.speed_scale * 1.05)

    def _twist_to_world(self, twist: np.ndarray) -> np.ndarray:
        """Re-express the puck's twist in the world frame, from whichever frame it
        was meant in. `world` returns it untouched.

        ⭐ WHY THIS EXISTS. Julien, 2026-08-11, wanting to mount a webcam on the arm:
        *"I can try to learn to control the arm from the point of view of the camera
        to get the tilts right and stuff."*

        World-frame control means "forward" is a fixed direction on the desk, however
        the wrist happens to be turned. That is the right default when you are looking
        AT the arm — it is predictable, and a wrong sign only nudges. But it is the
        wrong thing entirely when you are looking THROUGH a camera on the wrist:
        there, "push forward" means forward *in the image*, and the image turns with
        the wrist. `teleop.py` flagged this from the start as *"a deliberate later
        choice — not something to leave ambiguous now"*. This is that choice, made.

        The maths is small: a twist meant in frame F is `R_wf @ v` and `R_wf @ ω` in
        world coordinates, where `R_wf` is F's orientation in the world. Integration
        stays world-frame, so the anti-windup and the workspace box are untouched.

        ⚠️ **`camera` uses the MODELLED D405 mount** — the MJCF puts it on the flange
        at a 25° cant with `+Z` along the optical axis (ROS/OpenCV convention). That
        is correct for the real wrist cameras when they arrive and **wrong for a
        webcam cable-tied on by hand**, whose mounting transform nobody has measured.
        For the C920 stand-in use **`tool`**, mount the camera roughly looking the way
        the gripper points, and dial the rest out with the axis map. Using `camera`
        for an unmeasured mount would be inventing a transform, which is the single
        most repeated failure in FINDINGS.
        """
        spec = FRAMES[self.frame]
        if spec is None:
            return twist
        name, kind = spec
        rot = self.configuration.get_transform_frame_to_world(name, kind).rotation().as_matrix()
        out = np.empty(6)
        out[:3] = rot @ twist[:3]
        out[3:] = rot @ twist[3:]
        return out

    FRAME_NOTES = {
            "world": "WORLD — fixed to the desk; 'forward' does not turn with the wrist",
            "tool": "TOOL — attached to the gripper; 'forward' is where the gripper points",
            "camera": "CAMERA — the MODELLED D405 optical frame (⚠️ wrong for a hand-mounted webcam)",
    }

    def frame_note(self) -> str:
        """One line describing what the puck's directions currently mean."""
        return self.FRAME_NOTES[self.frame]

    def _limit_lead(self) -> None:
        """Stop the integrated goal running away from the pose actually achieved.

        ⛔⭐ THE BUG THIS FIXES — Julien, 2026-08-11: *"the inverse kinematics being
        weird and not working as intended, specifically when the robot gets into
        weird positions, and then it starts moving very, very incoherently."*

        `step()` advances `self.target` by the twist **unconditionally**. It never
        asks whether the arm followed. Measured in simulation, commanding pure roll
        at 0.6 rad/s from the park pose:

            t=4 s   target-vs-achieved gap 0.0004 m   tool point moved 0.000 m
            t=8 s   target-vs-achieved gap 0.238  m   tool point moved 0.238 m
            t=12 s                                    tool point moved 0.290 m

        **A pure rotation command moved the tool point 41 cm.** The chain is:

        1. A wrist joint hits its limit — the tight ones are ±1.5708. (Confirmed:
           the IK-vs-command gap pins at exactly 0.0800 rad, which *is*
           `JOINT_LIMIT_MARGIN`, so a joint is clamped at the margin while the IK
           believes it is at the true limit.)
        2. The orientation goal keeps integrating anyway, so it runs arbitrarily
           far past anything reachable.
        3. The QP now holds an impossible orientation target, and because
           `position_cost` (1.0) and `orientation_cost` (0.5) are traded against
           each other, it starts **moving the tool point to partially satisfy the
           unreachable rotation**.
        4. The workspace box then re-clamps translation, which fights the
           orientation task — hence the oscillation in the measurement above.

        So the arm does something the operator never asked for, in a direction that
        has no relation to the puck. That is exactly "incoherent".

        ⭐ The fix is the same idea as `SafeRobot.max_lag`, one layer up: a goal may
        only lead reality by a bounded amount. Translation and rotation are limited
        separately because they fail independently — the workspace box already
        happened to bound translation, which is why only rotation misbehaved.

        ⚠️ This deliberately does NOT try to detect singularities or predict
        reachability. It does not need to: whatever the reason the arm cannot
        follow — joint limit, singularity, rate limiter, a stalled motor — the
        symptom is the same, an unclosable gap, and bounding the gap bounds every
        one of those cases with no model of why.
        """
        if self.target is None:
            return
        achieved = self.configuration.get_transform_frame_to_world(self.ee_site, "site")

        lin = self.target.translation() - achieved.translation()
        dist = float(np.linalg.norm(lin))
        if dist > self.max_lead_m:
            lin = lin * (self.max_lead_m / dist)

        # Rotational lead, as an axis-angle in the world frame.
        d_rot = self.target.rotation().multiply(achieved.rotation().inverse())
        log = d_rot.log()
        angle = float(np.linalg.norm(log))
        if angle > self.max_lead_rad:
            d_rot = mink.SO3.exp(log * (self.max_lead_rad / angle))

        self.target = mink.SE3.from_rotation_and_translation(
            rotation=d_rot.multiply(achieved.rotation()),
            translation=achieved.translation() + lin,
        )

    def lead(self) -> tuple[float, float]:
        """How far the goal is currently ahead of the achieved pose: (metres, rad).

        Exposed so the session can show it. A goal that sits pinned at the limit is
        the signature of an arm that cannot follow, and that is worth seeing rather
        than inferring from the arm behaving oddly.
        """
        if self.target is None:
            return 0.0, 0.0
        achieved = self.configuration.get_transform_frame_to_world(self.ee_site, "site")
        d = float(np.linalg.norm(self.target.translation() - achieved.translation()))
        a = float(np.linalg.norm(self.target.rotation().multiply(achieved.rotation().inverse()).log()))
        return d, a

    def ee_position(self) -> np.ndarray:
        """Where the IK believes the end effector currently is."""
        return self.configuration.get_transform_frame_to_world(self.ee_site, "site").translation()


#: ⭐ THE WORKSPACE LIMIT, replacing a ±0.30 m cube on 2026-08-14. Julien's decision,
#: after the cube was measured stopping him at 71% of the arm's reach.
#:
#: `REACH_LIMIT` is a distance from the arm's base. The old cube re-centred on wherever
#: TELEOP was entered, so the wall sat somewhere different every session and nothing on
#: screen said where. His words, 2026-08-13: *"it stops moving in the direction I want it
#: to move even though the arm hasn't even close to fully extended."* Measured: he was
#: stopped 0.524 m from the base against a reachable 0.738 m (FINDINGS §41.1).
#:
#: ⛔⭐⭐ `FLOOR_LIMIT` EXISTS BECAUSE THE CUBE WAS QUIETLY PROVIDING ONE. A cube centred
#: on a tip at z = 0.475 bounded the tip above z = 0.175. **A bare sphere has no floor at
#: all**, and this arm can put its tip at **z = −0.377**, which is below its own base. So
#: swapping the cube for a radius alone would have removed real protection while looking
#: like a pure improvement. Working-contract rule 4: never continue past a hazard you have
#: correctly identified.
#:
#: ⛔⭐⭐ THE FLOOR IS 0.0 — EXACTLY THE BASE PLANE — AND IT TOOK TWO CORRECTIONS FROM
#: JULIEN TO GET THERE. Both were right and both are worth keeping, because the pair of
#: them defines what this number is allowed to be.
#:
#: **It shipped at +0.05 m.** He caught it before it ran: *"the bottom floor five
#: centimeter thing… sounds problematic because then I can't really pick anything up from
#: the table anymore."* A floor above the desk stops the tip short of everything lying on
#: it, and picking things up off the desk is what the rig is for.
#:
#: **I then over-corrected to −0.10 m**, reasoning that the floor should only bound a gross
#: plunge. He caught that too: *"ten centimeter below doesn't make any sense because then
#: it's still gonna crash into the table. So maybe do, like, one millimeter above or
#: something… or just do exactly on the base."*
#:
#: ⭐⭐ **He is right both times, and the two objections bracket the answer exactly.** Too
#: high forbids the task. Too low permits driving into the desk. **The base plane is the
#: only defensible value**, because the arm is bolted to the desk, so the desk is at or
#: just below z = 0 and the tip reaching z = 0 is a tip touching the desk.
#:
#: ⚠️ **It is his choice and it is meant to be tried**: *"we can test around with it
#: later."* If a flat object turns out to need a few mm below, `--floor -0.005` is one flag.
#: ⛔ Do NOT raise it above 0 again. That is the mistake this comment exists to prevent.
#:
#: ⭐ What it still protects against: this arm can otherwise put its tip at **z = −0.377 m**,
#: well below its own base. Every park pose sits at z ≥ 0.174, so the limit never
#: interferes with `q p d`. ⚠️ Measuring the true desk height remains open (ROADMAP §8.4).
REACH_LIMIT = 0.60          # m from the base
FLOOR_LIMIT = 0.0           # m — the base plane, which is the desk to within a plate


#: ⛔⭐⭐ HOW FAR PAST THE STARTING POSE TO OPEN A WIDENED LIMIT, and it must not be zero.
#:
#: `effective_limits()` widens a limit to include the pose TELEOP started from. The first
#: version widened it to *exactly* that distance, which left an arm starting outside
#: sitting precisely on the wall — so the clamp fired on every single cycle.
#:
#: That is a knife edge, and this repo has been cut by one before: the park stall check
#: passed at 0.020 and stalled at 0.021 against a 0.02 tolerance (FINDINGS §26).
#:
#: ⛔ **And clamping every cycle has a known consequence.** A position clamp fights the
#: orientation task in the QP, which is written down in `_limit_lead`'s own notes: *"the
#: workspace box then re-clamps translation, which fights the orientation task — hence the
#: oscillation."* Measured: commanding pure roll from a folded pose moved the tool point
#: **0.178 m** with the limit on the wall, against under 0.002 m with room to spare.
#:
#: ⭐ 0.05 m because that is `max_lead_m`, the distance the goal is already allowed to run
#: ahead of the arm. A limit closer than one lead-length sits inside the controller's own
#: slack and will chatter by construction.
LIMIT_WIDEN_MARGIN = 0.05


def effective_limits(home_ee: np.ndarray | None, reach: float,
                     floor: float) -> tuple[float, float]:
    """Widen the limits, if needed, so they contain the pose TELEOP started from.

    ⛔⭐⭐ WHY THIS FUNCTION EXISTS AT ALL, and it is the one real hazard in swapping a
    moving limit for a fixed one.

    The old cube re-centred on the arm at the moment TELEOP began, so the arm was always
    at the exact centre and **the cube could never be entered from outside.** A fixed
    limit can be. If the arm is already at 0.65 m when `t` is pressed, clamping to a
    0.60 m sphere would command it 5 cm inward **the instant TELEOP starts**, with
    nobody having asked for it. An unrequested move at mode entry is the shape of
    several defects in this repo already.

    So the limits open just far enough to include the starting pose, and stay there for
    that session. ⚠️ **Deliberately no decay back to the nominal limit as the arm comes
    in.** A limit that moves during a session is what was wrong with the cube, and
    trading one moving wall for another would be a poor exchange for the small gain.

    ⭐ It needs no new state: `home_ee` is already the tip position at TELEOP entry, so
    the widening is derived rather than remembered. That also keeps the field earning
    its place after the cube stopped using it.

    Returns `(reach, floor)` unchanged whenever the starting pose is already inside,
    which is every pose the arm actually rests in — the furthest park pose on record
    puts the tip 0.433 m out and 0.306 m up.
    """
    if home_ee is None:
        return reach, floor
    start = np.asarray(home_ee, dtype=float)
    dist, height = float(np.linalg.norm(start)), float(start[2])
    # ⚠️ The margin applies only when the limit actually has to open. An arm starting
    # inside keeps the configured limits exactly, which is every normal session.
    out = reach if dist <= reach else dist + LIMIT_WIDEN_MARGIN
    low = floor if height >= floor else height - LIMIT_WIDEN_MARGIN
    return out, low


def clamp_to_workspace(ee: np.ndarray, reach: float, floor: float) -> np.ndarray:
    """Pull a tip position back inside the reach sphere and above the floor.

    Returns the position unchanged when it is already inside. Pass the limits through
    `effective_limits()` first, or an arm that started outside will be yanked.

    ⚠️ This clamps position only. It says nothing about speed. `_apply_speed_scale`
    handles the arm slowing near awkward configurations, and the two are independent —
    FINDINGS §41.2 is what happened when a message confused one for the other.
    """
    out = np.asarray(ee, dtype=float).copy()

    dist = float(np.linalg.norm(out))
    if dist > reach and dist > 0.0:
        out *= reach / dist

    if out[2] < floor:
        out[2] = floor

    return out


def workspace_room(ee: np.ndarray, reach: float, floor: float) -> tuple[float, float]:
    """How much room is left: (metres of reach used, metres above the floor).

    ⭐ Reported on the status line every second. The cube was invisible, which is why
    hitting it read as the arm refusing to move rather than as a limit being reached.
    """
    ee = np.asarray(ee, dtype=float)
    return float(np.linalg.norm(ee)), float(ee[2] - floor)


def scripted_twist(t: float, speed: float = 0.04) -> np.ndarray:
    """A slow horizontal circle, for validating IK with no input device involved.

    Debugging IK and debugging a HID decoder at the same time is how you end up
    unable to tell which one is wrong.
    """
    return np.array([speed * np.cos(t * 0.8), speed * np.sin(t * 0.8), 0.0, 0.0, 0.0, 0.0])
