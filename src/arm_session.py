"""⭐ ONE ARM'S STATE AND MODE MACHINE, so that N of them can run in one loop.

    from arm_session import ArmSession, ArmSelector, parse_arms
    names = parse_arms(args.arm, args.arms, ARM_SERIALS, "B")   # ["B"] or ["B", "G"]
    arm = ArmSession(robot, name=names[0], frame="world", axis_map=…, slots=…, reader=…)
    step = arm.step_path(t=1.0, dt=0.01)        # ⚠️ step_PATH. This example said
                                                # `step_park` until 2026-08-14, and no
                                                # method by that name has ever existed.

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
- **The SpaceMouse DEVICE layer** — enumerating, the wiggle assignment, opening and
  closing the handle. ⚠️ **The `TwistReader` itself IS here** since 2026-08-14: with two
  arms, "which puck" is exactly as per-arm as "which robot", and a session-level reader is
  how both arms end up following one hand.
- **Key handling.** Which arm a keypress applies to is a *session* question, not an
  arm question — ROADMAP step 6 decides it (`a` selects; driving always applies to
  all arms; mode changes apply to the selected one).
- **IK stepping.** `CartesianTeleop` already owns that; this holds one and calls it.

✅✅ **STATUS, 2026-08-14 (night): STEP 2 IS COMPLETE AND TWO ARMS RUN FROM ONE LOOP.**
The 247-reference move landed as five commits ([FINDINGS §50](../docs/FINDINGS.md)); Julien
drove every mode on it ([§51](../docs/FINDINGS.md)); sixteen further commits made everything
below this class per arm and added `--arms`, the `a` selector and one status row per arm
([§52](../docs/FINDINGS.md), [§53](../docs/FINDINGS.md), [§54](../docs/FINDINGS.md)).

⭐ **This class now holds 34 per-arm fields**: the robot and its puck, the mode, the axis map
and its start-of-session copy, the control frame, the base pose and saved slots, the eleven
park fields, the CONTROLS memory and button edge, the thermal guard with this cycle's
temperatures, the last chain read, and this cycle's puck deflection.
`uv run scripts/check_restructure.py` proves none of it survives as a local in the script,
and it makes eight checks — **every one of which caught something real** ([§54.6](../docs/FINDINGS.md)).

⬜ **What has NOT happened: two arms have never run on the hardware.** That is ROADMAP §6.1
step 3 and it is Julien's, because building a robot sends setpoints. The procedure is
[FINDINGS §54.7](../docs/FINDINGS.md).

⚠️ **Three things refuse with two arms connected, all on purpose:** `--start-mode guide`,
`w`/`l` (the recorder is single-arm until ABC's two-arm format exists), and `m` while BOTH is
selected.

⛔ **THIS DOCSTRING SAID *"STILL NOT wired"* UNTIL 2026-08-14, which was true when written
on 08-13 and wrong the next day.** It is the same staleness pattern as the paragraph below
and as the six instances in [FINDINGS §33.3](../docs/FINDINGS.md): a written claim about
live state is a cache with no invalidation. ⭐ The remedy that works here is not writing it
more carefully — it is that `uv run scripts/check_restructure.py` recomputes the real
answer, so prefer running it over trusting any sentence in this header.

⛔⭐⭐ **AND A WARNING WORTH MORE THAN THE CLASS ITSELF: THIS FILE WENT STALE IN ONE
HOUR, WHILE UNWIRED.** It was committed 2026-08-12 at 14:16 with a park built from a queue
of legs and a per-leg speed ramp. At **15:15 the same day** `teleop_session.py` replaced
exactly that with a single blended `JointPath`, and the commit message says the earlier
version *"was the wrong thing"*. This class then sat for a day modelling a design the
script no longer had, **with all 17 of its tests passing the whole time**, because the
tests asserted the superseded behaviour. It was found by auditing before the restructure
rather than by anything failing.

⚠️ **The lesson, kept because it explains the shape of the work: an unwired class is a
copy of a design, and a copy drifts.** The fix was to finish the wiring, and that is done
— the script no longer holds a second copy of this state. ⛔ **What it still holds is a
second copy of the park LOOP**: `step_path()` here and the `mode == "park"` branch there
implement the same motion, and only the branch runs. Diff them before trusting this one,
and see [ROADMAP §6.1](../docs/ROADMAP.md) for where they are meant to collapse.

⛔ **What this deliberately does NOT own, decided 2026-08-13: recording and playback.**
They look like per-arm state and they are not. `amazon-far/abc` wants 14 states and 14
actions per timestep, **two arms in ONE timeline** (ROADMAP §9.2), so a recorder owned by
an arm cannot produce the target format at all. One session-level recorder samples every
arm each cycle, and one playback cursor drives them all — splitting the cursor per arm
would let the arms drift apart in time, which is the one thing a bimanual demonstration
must not do. Migration map: ROADMAP §6.1.
"""

from __future__ import annotations

from collections.abc import Iterable
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

#: ⛔ Pushing hard while not moving is the definition of a stall, and stall is the worst
#: thermal case there is: full current, no motion, no cooling. Motor 7 was cooked three
#: times before this guard existed. Values copied from `teleop_session.py`.
GRIPPER_STALL_TORQUE = 1.0    # Nm
GRIPPER_STALL_VEL = 0.05      # rad/s
GRIPPER_STALL_SECONDS = 0.4


def parse_arms(single: str | None, spec: str | None,
               known: Iterable[str], default: str) -> list[str]:
    """Turn `--arm` and `--arms` into the ordered list of arms a session drives.

        parse_arms(None, None,  ARM_SERIALS, "B")   -> ["B"]
        parse_arms(None, "B,G", ARM_SERIALS, "B")   -> ["B", "G"]
        parse_arms("G",  None,  ARM_SERIALS, "B")   -> ["G"]

    ⭐ WHY BOTH FLAGS EXIST. `--arm` is the spelling every other script here uses
    (`ping_motors.py`, `identify_arm.py`, `check_arms_match.py`), it is in every
    document, and it is what Julien types. `--arms` is the N-arm spelling ROADMAP §6.1
    step 2 asks for. **They are two spellings of one idea, not two ideas** — the same
    relationship `ö`/`ä` have to `[`/`]`, and the reason is the same: a working command
    must not stop working because the code grew a more general form.

    ⛔ THEY MUST AGREE. `--arm B --arms G` is refused rather than resolved by a
    precedence rule nobody would remember. This repo has already paid for the other
    approach: `--arm arm1` was deleted rather than aliased, because a flag that keeps
    working while its meaning has moved underneath is worse than one that fails loudly
    (`src/yam_can.py`, and the same call again for `--box`).

    ⛔ AND NO ARM MAY APPEAR TWICE. `--arms B,B` would build two `ArmSession` objects
    over one CAN bus, each with its own cached `prev_q`, both commanding the same seven
    motors every cycle. The last write of each cycle would win and nothing would raise,
    so the arm would follow a blend of two controllers. That is the FINDINGS §0 defect
    class exactly: a confident, plausible, wrong answer with no exception.

    Raises `ValueError` with a message written for the person at the keyboard. The
    caller turns it into `argparse`'s own error, so it prints like any other bad flag.
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
    """Which arm a MODE key applies to. `a` cycles it: B → G → BOTH → B.

        sel = ArmSelector(["B", "G"])
        sel.label            # "B"
        sel.cycle()          # "G"
        sel.cycle()          # "BOTH"
        sel.names()          # ["B", "G"]

    ⛔⭐ WHY MODE KEYS NEED A SELECTOR AT ALL, and it is a safety argument rather than a
    convenience one. ROADMAP §6 decided it: a global `g` would put **8.6 kg** weightless
    in one keypress, and GUIDE is the mode where an error in the dynamics model becomes a
    *falling* arm rather than a droop ([FINDINGS §11.1](../docs/FINDINGS.md)). So a mode
    change is aimed at one arm unless the operator has deliberately selected BOTH.

    ⭐ **Driving is NOT selected.** Each arm follows its own puck, continuously, always —
    that is the whole point of two arms. Only mode changes and edits are aimed. Julien's
    own words for the goal are in ROADMAP §6, and the split is in its decision table.

    ⚠️ **With one arm there is nothing to cycle**, and `cycle()` says so by returning the
    same label rather than inventing a BOTH that means the same as B. A key that appears
    to do something while doing nothing is the `b` defect again
    ([FINDINGS §17.1](../docs/FINDINGS.md)).
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

    Modes keep the script's own names — `guide`, `teleop`, `hold`, `park` — because they
    are Julien's mental model and renaming them would make every note in FINDINGS harder
    to follow.

    ⚠️ **`map` is deliberately absent, and this docstring used to claim it was here.**
    CONTROLS (`m`) is an interactive wizard: it asks the operator to move one axis at a
    time and waits for answers. That is a *session* activity, like key handling, so it
    stays in the script along with `last_active_axis`. **The old wording said "the same
    five modes" and the class only ever had four**, which is the kind of small untruth
    that makes a reader trust the rest of the file less. Found by the diff in
    [FINDINGS §36.5](../docs/FINDINGS.md).

    ⛔ **Still NOT here, and each is a decision rather than an oversight** — see
    [ROADMAP §6.1](../docs/ROADMAP.md):

    - **The teleop per-cycle clamp and the joint-limit clamp.** They belong here and are
      not here yet, and the argument is working-contract rule 7: *what path reaches the
      hazard without passing through the guard?* Today they live only in the teleop
      branch, and PARK already went around the gripper clamp once for exactly that reason
      (FINDINGS §9). Moving them into this class's single command path would close that
      whole class of defect. **Not done here**, because it changes what gets commanded
      and that deserves its own reviewable step.
    - **The workspace box.** A cartesian idea, so it stays with `CartesianTeleop`.
    - **Recording and playback.** They span both arms; see the module docstring.
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

        # ⭐⭐ WHICH PUCK DIRECTION DRIVES WHICH MOTION, for THIS arm in THIS frame.
        #
        # ⚠️ Passed to the constructor rather than assigned afterwards, and that is a
        # deliberate contrast with `mode`. `mode` cannot be a constructor argument, because
        # `build_robot()` reads it to decide zero-gravity and runs before the robot exists —
        # so the script has to hand it over on the next line, and a forgotten handover would
        # have been silent ([FINDINGS §50.2](../docs/FINDINGS.md)). The map has no such
        # constraint: `AxisMapStore` can be read before anything is built. **So it is
        # impossible to forget rather than merely tested for.**
        #
        # ⭐ `axis_map_at_start` is copied HERE rather than by the caller, because the two
        # must be taken at the same instant. `0` in CONTROLS reverts to it, and the closing
        # summary reports "was:" from it, so a copy taken a few lines later would quietly be
        # a copy of a different map.
        self.axis_map = axis_map
        self.axis_map_at_start = axis_map.copy() if axis_map is not None else None

        # ⭐⭐ THIS ARM'S SAVED POSES, AND THE ONE Ctrl-C GOES TO.
        #
        # ⛔ THE BASE POSE AND THE WAYPOINTS ARE DIFFERENT THINGS, and it is a safety
        # requirement rather than a preference. Julien, 2026-08-12: *"the control-c park to
        # disable needs to always go back to the stable parking save. If I save a new
        # parking option it shouldn't go back to that and then disable."* Ctrl-C parks and
        # then RELEASES the motors, so the pose it chooses must be one that is safe to let
        # go in. A waypoint saved mid-task, with the arm extended over the desk holding
        # something, is exactly what that must never be.
        #
        # ⭐ Called `base_pose` and not `park`, deliberately: this file already has eleven
        # `park_*` fields describing the motion in progress, and a `park` beside them would
        # read as one of them. The old session local was named `park`, which is why
        # `check_restructure.py` carries it in RETIRED_LOCALS.
        #
        # ⚠️ `None` is a real state, not a missing value: it means no pose has ever been
        # saved for this arm. The script then defaults it to wherever the arm was when the
        # session started, which it can only do after the robot exists.
        self.slots: dict[str, list] = dict(slots or {})
        self.base_slot = base_slot
        self.base_pose: list | None = self.slots.get(base_slot)

        # ⭐⭐ THE PUCK THAT DRIVES THIS ARM. One `TwistReader`, already opened and bound to
        # one physical SpaceMouse by the wiggle assignment.
        #
        # ⚠️ THE MODULE DOCSTRING ABOVE USED TO SAY THIS CLASS DELIBERATELY DOES NOT OWN
        # "reading the SpaceMouse", and that is still half true. The *device layer* stays
        # shared and outside: enumerating, the wiggle assignment, `open_device`, and closing
        # the handle on the way out. **What belongs to the arm is the one reader it is
        # driven by**, because with two arms "which puck" is exactly as per-arm as "which
        # robot" — and a session-level reader is how both arms end up following one hand.
        #
        # ⛔ The HANDLE is deliberately NOT here. It has to be closed even when
        # `build_robot()` failed and no `ArmSession` was ever created, so the script keeps
        # its own dict of handles for teardown. Two references to one object, not two
        # copies of state.
        self.reader = reader
        # ⭐ THIS CYCLE'S PUCK DEFLECTION, six axes in [-1, 1], read once per cycle and used
        # by the mode action and by the CONTROLS readout. Per arm because the reader is: two
        # arms are two different hands, and one session-level copy would drive both arms from
        # whichever puck happened to be read last.
        self.raw_axes: list[float] = [0.0] * 6

        # ⭐⭐ WHAT THE CONTROLS WIZARD REMEMBERS ABOUT *THIS* PUCK, and it is per arm for
        # the same reason the reader is: two pucks have two "controls you just used".
        #
        # ⛔ `last_active_axis` has NO TIMEOUT on purpose. In CONTROLS, `f` and `1`-`6` act
        # on "the control you just used", and that has to still be remembered after the puck
        # has sprung back to centre and the operator's hand has left it.
        #
        # ⭐ `last_input_kind` exists so that ONE key means one thing: `f` reverses whichever
        # control was last used, an axis by flipping its sign or a button by swapping
        # open/close.
        #
        # ⚠️ `buttons_prev` is what makes a press an EDGE rather than a state. Without it a
        # held button would re-fire its action every cycle at 100 Hz.
        #
        # ⚠️ The module docstring says key HANDLING stays in the script, and it still does:
        # which arm a keypress is aimed at is a session question, answered by `ArmSelector`.
        # What lives here is the state a key acts ON.
        self.last_active_axis: int | None = None
        self.last_active_value = 0.0
        self.last_input_kind: str | None = None     # None | "axis" | "button"
        self.learn_button: str | None = None        # None | "open" | "close"
        self.buttons_prev = 0

        self.gripper_min, self.gripper_max = gripper_min, gripper_max
        self.gripper_value = 0.0
        self.stall_since: float | None = None
        #: ⛔⭐⭐ HOW MANY TIMES THE JAWS HAVE STALLED IN A ROW, and when it was last said out
        #: loud. Julien, 2026-08-15: *"the gripper arm print was way too often, and it happened
        #: because I was pushing on the leader arm gripper and the follower was picking
        #: something up, so it pushed too far in."*
        #:
        #: ⚠️ THE GUARD IS WORKING; the REPORTING is not. In MIRROR the follower's jaw command
        #: is the leader's measured jaw position, re-sent every cycle. So the guard releases the
        #: jaws, the next cycle commands them back onto the object, and 0.4 s later it fires
        #: again — for as long as the operator squeezes the leader. Twenty identical lines in
        #: ten seconds is how a real warning gets trained into background noise, which
        #: [FINDINGS §0](../docs/FINDINGS.md) is a catalogue of.
        self.stall_count = 0
        self.stall_last_said = 0.0
        self._states: Any = None        # this cycle's chain read, for the stall guard

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
        # ⛔⭐ THIS CYCLE'S READING, AND `None` MEANS BLIND RATHER THAN COLD. The status
        # row prints `??°C ⚠️BLIND` for `None`, never a number: a fabricated 0 °C is what
        # made a disarmed thermal guard look healthy on screen (FINDINGS §24.1), and the
        # readout is the only place a human would have noticed.
        #
        # ⭐ Per arm, because the row is per arm. As a session-level pair these were one
        # arm's temperatures painted on whichever row happened to be drawn — the shape of
        # error that would hide one arm's gripper behind the other arm's shoulder.
        self.hottest: float | None = None
        self.jaw_temp: float | None = None
        # ⛔⭐ THE LAST CHAIN READ THIS ARM MANAGED, kept because the incident record needs
        # it AFTER the chain has died. On 2026-08-14 the arm fell, the CAN link went away,
        # and every value describing that instant was lost — the gravity torques had to be
        # recovered by simulating joint angles the arm had already measured and thrown away
        # ([FINDINGS §45](../docs/FINDINGS.md)). A fresh read on a dead chain raises; the
        # last good reading is what actually describes the failure.
        #
        # ⚠️ `None` means the read failed, exactly like `hottest`. Never an empty list: an
        # empty list would read as "seven motors reporting nothing", which is a different
        # and much calmer claim than "I could not ask".
        self.states: Any = None
        self.temps: Any = None

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
            self.stall_since = None      # a stall cannot be judged if it cannot be seen
            self._states = None
            self.hottest, self.jaw_temp = None, None
            return self.thermal.update(None), None, None
        self._states = states
        temps, hottest, jaw = motor_temperatures(states, N_ARM)
        motor = temps.index(hottest) if hottest is not None else None
        self.hottest, self.jaw_temp = hottest, jaw
        return self.thermal.update(hottest, jaw, motor=motor), hottest, jaw

    def gripper_stall_release(self, t: float) -> float | None:
        """Is the gripper pushing hard without moving? Returns a jaw value to back off to.

        ⛔⭐ WHY THIS EXISTS: motor 7 was cooked three times. Pushing at full current
        while not moving is the worst thermal case there is — full current, no motion, no
        cooling — and the jaws reach it whenever they are commanded past whatever they are
        holding. The release is to the **measured** jaw position, so the command stops
        fighting the object and the motor stops heating.

        ⭐ It returns a value instead of applying one, because *the class decides and the
        script narrates*: the caller sets `gripper_value` and prints the warning. Returning
        `None` means there is nothing to do.

        ⚠️ It needs `read_thermal()` to have run this cycle, because the torque and
        velocity come from the same chain read. Calling it without one is not an error; it
        simply reports nothing, which is the same "cannot see it, cannot judge it" rule the
        thermal guard uses.

        ⛔ **This was missing from this class for a day**, while `teleop_session.py` had it
        the whole time and this file even carried the `stall_since` variable with nothing
        writing to it. Found by a systematic diff rather than by anything failing.
        FINDINGS §36.5.
        """
        states = getattr(self, "_states", None)
        if states is None or len(states) <= N_ARM:
            self.stall_since = None
            return None
        jaw = states[N_ARM]
        pushing = abs(getattr(jaw, "eff", 0.0)) > GRIPPER_STALL_TORQUE
        still = abs(getattr(jaw, "vel", 0.0)) < GRIPPER_STALL_VEL
        if not (pushing and still):
            self.stall_since = None
            return None
        if self.stall_since is None:
            self.stall_since = t
            return None
        if t - self.stall_since <= GRIPPER_STALL_SECONDS:
            return None
        self.stall_since = None
        q = np.asarray(self.robot.get_joint_pos(), dtype=float)
        return float(q[N_ARM])

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
