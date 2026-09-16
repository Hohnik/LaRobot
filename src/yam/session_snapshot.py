"""Detached values for terminal status, acquired at the application's display boundary.

Read each arm pose once per capture. These sequential reads are not a simultaneous
hardware snapshot; they do not replace the control loop's command/health reads.
Acquisition errors propagate to the application's existing fault cleanup.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from yam.teleop import effective_limits, workspace_room

if TYPE_CHECKING:
    from yam.mirror_session import MirrorSession
    from yam.session import ArmSession


@dataclass(frozen=True)
class TeleopStatus:
    ee: tuple[float, ...]
    reach_limit: float
    radius: float
    floor_clearance: float
    lead_m: float
    lead_rad: float
    max_lead_m: float
    max_lead_rad: float
    speed_scale: float
    requested_rate: float
    max_joint_rate: float


@dataclass(frozen=True)
class ArmStatus:
    name: str
    mode: str
    frame: str
    q: tuple[float, ...]
    hottest: float | None
    jaw_temp: float | None
    guide_ref: tuple[float, ...] | None
    teleop: TeleopStatus | None
    note: str = ""


def capture_arm_status(one: ArmSession, reach: float, floor: float) -> ArmStatus:
    """Copy measured joints, cached temperatures and solver diagnostics; never command."""
    q = tuple(float(v) for v in one.robot.get_joint_pos())
    teleop = None
    if one.mode == "teleop" and one.teleop is not None:
        solver = one.teleop
        ee = tuple(float(v) for v in solver.ee_position())
        lim_r, lim_f = effective_limits(one.home_ee, reach, floor)
        radius, clearance = workspace_room(ee, lim_r, lim_f)
        lead_m, lead_rad = solver.lead()
        teleop = TeleopStatus(
            ee, lim_r, radius, clearance, lead_m, lead_rad,
            solver.max_lead_m, solver.max_lead_rad, solver.speed_scale,
            solver.requested_rate, solver.max_joint_rate)
    return ArmStatus(
        one.name, one.mode, one.frame, q, one.hottest, one.jaw_temp,
        None if one.guide_ref is None else tuple(float(v) for v in one.guide_ref),
        teleop)


def capture_status(arms: list[ArmSession], reach: float, floor: float,
                   mirror: MirrorSession | None = None) -> list[ArmStatus]:
    """Capture rows in arm order; reuse those same poses for mirror diagnostics."""
    rows = [capture_arm_status(one, reach, floor) for one in arms]
    if mirror is not None and mirror.link is not None:
        leader = next(row for row in rows if row.name == mirror.leader.name)
        rows = [replace(row, note=mirror.link.status(leader.q, row.q))
                if row.mode == "mirror" else row for row in rows]
    return rows
