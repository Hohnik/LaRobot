"""Pure terminal formatting of detached status and playback measurements."""
from __future__ import annotations

import numpy as np

from yam.can import YAM_JOINTS
from yam.session import N_ARM
from yam.session_snapshot import ArmStatus


def flat_joint_names(arm_names: list[str], per_arm: int, total: int) -> list[str]:
    """Joint labels in flattened sample order, clipped to the tracked count."""
    return [f"{arm} {YAM_JOINTS.get(j + 1, ('joint',))[0]}"
            for arm in arm_names
            for j in range(per_arm)][:total]


def tracking_table(rows: list, names: list[str], hold_rad: float,
                   quiet: float = 0.01) -> list[str]:
    """Format requested replay speeds and measured target-minus-encoder lag.

    Name quiet joints separately; keep their list short enough for the terminal."""
    out, unmoved = [], []
    for i, worst, at_speed, top, lag_top in rows:
        label = names[i] if i < len(names) else f"joint {i}"
        if top < quiet:
            unmoved.append(label)
            continue
        out.append(f"       {label:<18} worst lag {worst:.3f} rad at "
                   f"{at_speed:5.2f} rad/s · asked for up to {top:5.2f} rad/s "
                   f"with {lag_top:.3f} rad of lag")
    if unmoved:
        if len(unmoved) == len(rows):
            out.append(f"       ⚠️ ALL {len(rows)} joints moved slower than {quiet} rad/s, "
                       f"so there is no table — nothing actually moved.")
        else:
            shown = ", ".join(unmoved[:2])
            more = f" +{len(unmoved) - 2} more" if len(unmoved) > 2 else ""
            out.append(f"       ⚠️ {len(unmoved)} of {len(rows)} joints too slow to rate "
                       f"(<{quiet} rad/s): {shown}{more}")
    return out


def status_row(one: ArmStatus, lead: str) -> str:
    """Format detached status values; never read devices or change session state."""
    q = np.asarray(one.q, dtype=float)
    extra = ""
    if one.mode == "teleop" and one.teleop is not None:
        extra = f"  EE {np.round(one.teleop.ee, 3)}"
        lim_r = one.teleop.reach_limit
        ee_now = one.teleop.ee
        out, up = one.teleop.radius, one.teleop.floor_clearance
        extra += f"  reach {out:.2f}/{lim_r:.2f}m"
        if out > 0.9 * lim_r:
            extra += " ⚠️ AT THE EDGE"
        if up < 0.10:
            extra += f"  ⚠️ {up * 100:.0f}cm above the floor (z={ee_now[2]:+.2f})"
        lead_m, lead_r = one.teleop.lead_m, one.teleop.lead_rad
        if lead_m > 0.8 * one.teleop.max_lead_m or lead_r > 0.8 * one.teleop.max_lead_rad:
            extra += f"  ⚠️ STUCK lead {lead_m * 100:.0f}cm/{np.degrees(lead_r):.0f}°"
        if one.teleop.speed_scale < 0.95:
            extra += (f"  ⚠️ SLOWED to {one.teleop.speed_scale * 100:.0f}% (joints asked "
                      f"for {one.teleop.requested_rate:.1f} rad/s, cap "
                      f"{one.teleop.max_joint_rate:.1f})")
    therm = (f"hottest {one.hottest:4.0f}°C" if one.hottest is not None
             else "hottest   ??°C ⚠️BLIND")
    if one.jaw_temp is not None:
        therm += f"  jaw {one.jaw_temp:4.0f}°C"
    if one.mode == "guide" and one.guide_ref is not None:
        sank = float(np.max(np.abs(q[:N_ARM] - one.guide_ref[:N_ARM])))
        extra = f"  drift {sank:5.3f} rad ({np.degrees(sank):4.1f}°){extra}"
    if one.note:
        extra = f"  {one.note}{extra}"
    if one.mode == "teleop":
        label = f"TELEOP/{one.frame[0]}"
    else:
        label = "CONTROLS" if one.mode == "map" else one.mode.upper()
    return (f"[{one.name} {label:8}]{lead}  {therm}"
            f"  q {np.round(q[:N_ARM], 2)}{extra}   ")

