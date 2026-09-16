"""Mirror pair selection, confirmation and run diagnostics.

MirrorLink retains the motion algorithm. The operator still reads poses, steps
that link and commands the follower through its existing jaw/robot constraints.
"""
from __future__ import annotations

import numpy as np

from yam.can import YAM_JOINTS
from yam.mirror import MirrorLink, pick_pair


class MirrorSession:
    def __init__(self, *, align_speed: float, n_arm: int, emit=print, hint=lambda text: None):
        self.align_speed = align_speed
        self.n_arm = n_arm
        self.emit = emit
        self.hint = hint
        self.link: MirrorLink | None = None
        self.leader = self.follower = None
        self.clipped_at = 0

    def preview(self, arms, selected, mode: str) -> bool:
        """Choose the pair and print the plan; never change modes or command joints."""
        try:
            lead_name, follow_name = pick_pair(
                [one.name for one in arms], selected)
        except ValueError as exc:
            self.hint(str(exc))
            return False
        self.leader = next(o for o in arms if o.name == lead_name)
        self.follower = next(o for o in arms if o.name == follow_name)
        start_gap = float(np.max(np.abs(
            np.asarray(self.follower.robot.get_joint_pos(), dtype=float)[:self.n_arm]
            - np.asarray(self.leader.robot.get_joint_pos(), dtype=float)[:self.n_arm])))
        self.emit(f"\n⭐ MIRROR: arm {lead_name} LEADS, arm {follow_name} FOLLOWS "
              f"({mode}).")
        self.emit(f"     arm {follow_name} will first close a {start_gap:.2f} rad "
              f"gap at {self.align_speed} rad/s, then track continuously.")
        self.emit(f"     ⚠️ HOLD ARM {lead_name} STILL until it says FOLLOWING, and "
              f"keep the space around arm {follow_name} clear.")
        self.emit("     ⛔ NOTHING CHECKS FOR THE ARMS COLLIDING. No arm knows "
              "where the other one is.")
        self.emit("     Enter engages · i switches copy/mirror · any other key "
              "cancels\n")
        return True

    def confirm(self, k: str, *, mode: str, catchup: float,
                max_gap: float, max_speed: float) -> tuple[bool, str]:
        """Return (keep_prompt, preferred_mode); only acceptance engages the follower."""
        if k == "i":
            mode = "mirror" if mode == "copy" else "copy"
            self.emit(f"     ⭐ now {mode.upper()}: "
                  + ("the follower reproduces the leader's angles unchanged, "
                     "for arms side by side" if mode == "copy" else
                     "the follower negates the joints that reverse under "
                     "reflection, for arms FACING each other")
                  + "\n     Enter engages · i switches again · any other "
                    "key cancels\n")
            return True, mode
        if k in ("\r", "\n", " ") and self.follower is not None:
            self.follower.enter_hold()
            self.follower.mode = "mirror"
            self.link = MirrorLink(
                mode=mode, align_speed=self.align_speed,
                catchup=catchup,
                follow_speed=getattr(self.follower.robot, "max_speed",
                                     max_speed),
                max_gap=max_gap)
            self.clipped_at = getattr(self.follower.robot,
                                        "limited_cycles", 0)
            self.emit(f"\n▶  MIRROR engaged: arm {self.follower.name} is "
                  f"following arm {self.leader.name}. "
                  "Press h, t, g or i to stop it.\n")
        else:
            self.leader = self.follower = None
            self.hint("")
            self.emit("\n  mirror cancelled.\n")
        return False, mode

    def clear(self) -> None:
        self.link = None

    def turn_off(self, arms) -> None:
        self.clear()
        for one in arms:
            if one.mode == "mirror":
                one.enter_hold()
        self.hint("")
        self.emit("\n  ⭐ MIRROR off — the follower is HOLDING.\n")

    def observe_modes(self, arms) -> None:
        if self.link is not None and not any(one.mode == "mirror" for one in arms):
            self.clear()
            self.hint("")
            self.emit("  ⭐ MIRROR off — the follower left the mode.\n")

    def report_stop(self, one, max_speed: float) -> None:
        """Report the link's measured reason before the application clears it."""
        joint_name = ""
        if self.link.stop_joint is not None:
            joint_name = YAM_JOINTS.get(
                self.link.stop_joint + 1, ("joint",))[0]
        self.emit(f"\n⛔ MIRROR STOPPED — {self.link.stop_reason}"
              + (f", {joint_name}" if joint_name else ""))
        self.emit(f"     {self.link.stop_detail}")
        clipped = getattr(one.robot, "limited_cycles", 0) - self.clipped_at
        if clipped > 0:
            self.emit(f"     ⚠️ SafeRobot held the command back on {clipped} "
                  f"cycle(s) (its "
                  f"{getattr(one.robot, 'max_lag', 0.25):.2f} rad "
                  "following-error limit).")
        if self.link.stop_cause == "follow_limit":
            suggest = max(3.0, round(self.link.stop_leader_speed
                                     * 1.5 + 0.4, 1))
            self.emit(f"     ⭐ TWO WAYS OUT, and the first is the limit "
                  f"that actually fired:")
            self.emit(f"       1. `--mirror-gap {self.link.max_gap * 2:.2f}`"
                  f"  (now {self.link.max_gap:.2f}) — how far behind "
                  f"is TOLERATED before stopping.")
            self.emit(f"       2. `--max-speed {suggest:g}`"
                  f"  (now {max_speed:.2f}) — how fast the follower "
                  f"may move. You moved the leader at "
                  f"{self.link.stop_leader_speed:.2f} rad/s.")
            self.emit(f"     ⚠️ `--max-lag` does NOT affect this stop.")
        elif self.link.stop_cause == "tracking":
            self.emit("     ⭐ More `--max-speed` will NOT help: the arm, not "
                  "the software, is the limit.")
            self.emit(f"     Either guide the leader more slowly, or loosen "
                  f"the tolerance with --mirror-gap "
                  f"{self.link.max_gap * 2:.2f}.")
        self.emit("     Press i then Enter to engage it again.\n")
