"""Per-cycle liveness, temperature and gripper-stall checks.

ArmSession retains measurements and guard state. This service reports a typed
session stop; it never bypasses the application's controlled-stop/cleanup path.
"""
from __future__ import annotations

from yam.lifecycle import StopCause, StopRequest


class SessionHealth:
    def __init__(self, *, n_arm: int, stall_torque: float, stall_velocity: float,
                 stall_seconds: float, emit=print):
        self.n_arm = n_arm
        self.stall_torque = stall_torque
        self.stall_velocity = stall_velocity
        self.stall_seconds = stall_seconds
        self.emit = emit

    def check(self, arms, now: float, *, stop: StopRequest | None = None) -> StopRequest | None:
        # Read failures feed the thermal guard; they never imply a safe temperature.
        # Check all thermal guards, retaining the last stop as the operator did.
        for one in arms:
            if not one.alive():
                stop = StopRequest(
                    StopCause.FAULT,
                    f"arm {one.name}: the motor chain STOPPED — I2RT's control "
                    "thread exited, almost certainly on a motor fault. Commands "
                    "are no longer reaching the arm.")
        if stop:
            return stop

        for one in arms:
            verdict, _, _ = one.read_thermal(n_arm=self.n_arm)
            read_error = one.read_error
            if one.states is not None:
                if one.jaw_unblocked_from is not None:
                    self.emit(f"\n  ⭐ arm {one.name} jaws opened past the block at "
                          f"{one.jaw_unblocked_from:.3f} — free to close again.\n")
                    one.jaw_unblocked_from = None
                measured_jaw = one.gripper_stall_release(
                    now, n_arm=self.n_arm, torque=self.stall_torque,
                    velocity=self.stall_velocity, seconds=self.stall_seconds)
                if measured_jaw is not None:
                    jaw = one.states[self.n_arm]
                    one.gripper_value = measured_jaw
                    one.block_jaw_at(measured_jaw)
                    one.stall_since = None
                    one.stall_count += 1
                    if (one.stall_count == 1
                            or now - one.stall_last_said > 5.0):
                        one.stall_last_said = now
                        extra = ("" if one.stall_count == 1 else
                                 f" ({one.stall_count} times now)")
                        self.emit(f"\n⚠️  ARM {one.name} GRIPPER STALLED{extra} "
                              f"({jaw.eff:+.2f} Nm, not moving) — released to "
                              f"{measured_jaw:.3f} so it stops pushing.")
                        if one.stall_count > 1:
                            self.emit("     Something is holding the jaws and the "
                                  "command keeps pushing past it. In MIRROR that "
                                  "is the leader's jaws being squeezed while the "
                                  "follower already has hold of something.\n")
                        else:
                            self.emit("")

            if verdict.warning:
                detail = f"  ({read_error})" if read_error else ""
                self.emit(f"\n⚠️  arm {one.name}: {verdict.warning}{detail}\n")
            if verdict.stop_reason:
                stop = StopRequest(StopCause.FAULT, f"arm {one.name}: {verdict.stop_reason}")
        return stop
