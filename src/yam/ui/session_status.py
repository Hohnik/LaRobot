"""Read and format per-arm status and playback tracking measurements.

No function in this module commands a robot. The application owns when to read
and display rows, and catches read/render failures in its status reporting path.
"""
from __future__ import annotations

import numpy as np

from yam.can import YAM_JOINTS
from yam.session import ArmSession, N_ARM
from yam.teleop import effective_limits, workspace_room


def flat_joint_names(arm_names: list[str], per_arm: int, total: int) -> list[str]:
    """Joint labels in flattened sample order, including each arm name.
    
    Clip to the tracked joint count; names must match printed and saved tables.
    """
    return [f"{arm} {YAM_JOINTS.get(j + 1, ('joint',))[0]}"
            for arm in arm_names
            for j in range(per_arm)][:total]


def tracking_table(rows: list, names: list[str], hold_rad: float,
                   quiet: float = 0.01) -> list[str]:
    """The per-joint tracking table as lines. Pure, so it can be tested.

    `rows` is `TrackingLog.rows()`: `(index, worst lag, speed then, top speed, lag then)`.

    ⚠️⭐ THE SPEEDS ARE REQUESTS, THE LAGS ARE MEASUREMENTS, and the old wording hid that. Both speed columns come from the replay's own target sequence, sampled before `SafeRobot` applies `max_speed`. So a row may legitimately name a speed higher than the session's own cap: that is the recording asking and the clamp refusing. The lag columns are `target − measured`, read from the encoders, and they are real.

    ⚠️⭐ A JOINT THAT BARELY MOVED IS NAMED, NOT SILENTLY DROPPED. Leaving it out of the
    table is right, because a joint that never moved says nothing about tracking. But a
    table that quietly omits rows reads as complete: his 2026-08-17 log printed **12 rows
    for a 14-joint recording** and nothing said so. The two missing ones were the grippers,
    which he had not touched, and a reader counting rows would reasonably conclude the
    recorder had lost two joints.
    """
    out, unmoved = [], []
    for i, worst, at_speed, top, lag_top in rows:
        label = names[i] if i < len(names) else f"joint {i}"
        if top < quiet:
            unmoved.append(label)
            continue
        # ⛔ "asked for", NOT "top speed". Both speeds in this row are the speed the
        # RECORDING requested, measured on the replay's own target before `SafeRobot`
        # sees it. On 2026-08-19 a row read "top speed 1.83 rad/s" in a session whose own
        # banner said max-speed caps every joint at 1.00 rad/s, which reads as the cap
        # having leaked. It had not: the recording asked for 1.83 and the clamp refused,
        # which is the clamp working (FINDINGS §76). The LAG figures are real measurements
        # against the encoders; only the speeds are requests.
        out.append(f"       {label:<18} worst lag {worst:.3f} rad at "
                   f"{at_speed:5.2f} rad/s · asked for up to {top:5.2f} rad/s "
                   f"with {lag_top:.3f} rad of lag")
    if unmoved:
        # ⛔⭐⭐ THE LINE MUST STAY SHORT, AND ONLY RUNNING IT SHOWED WHY. The first version
        # listed every unmoved joint. In a simulated playback where nothing moved that was
        # all fourteen names on one line, and `src/yam/ui/screen.py`'s painter **truncated it with
        # an ellipsis** — the reader saw "B base_yaw, B shoulder_pit…" and nothing more. A
        # note whose whole job is to say WHICH joints are missing, cut off before it says
        # so, is worse than no note, because it looks answered.
        #
        # ⚠️ Reading the code could not have found this. The note was correct; the terminal
        # ate it. It took an actual `--sim` run.
        if len(unmoved) == len(rows):
            out.append(f"       ⚠️ ALL {len(rows)} joints moved slower than {quiet} rad/s, "
                       f"so there is no table — nothing actually moved.")
        else:
            # ⚠️ TWO names, and the wording is short on purpose. Measured against the
            # painter: four names plus the old longer phrasing came to 145 characters and
            # was still being cut. His real case is two still grippers out of fourteen,
            # which fits in full.
            shown = ", ".join(unmoved[:2])
            more = f" +{len(unmoved) - 2} more" if len(unmoved) > 2 else ""
            out.append(f"       ⚠️ {len(unmoved)} of {len(rows)} joints too slow to rate "
                       f"(<{quiet} rad/s): {shown}{more}")
    return out


def status_row(one: ArmSession, lead: str, reach: float, floor: float,
               note: str = "") -> str:
    """Format one arm's mode, measured temperatures, pose and tracking warnings.
    
    The caller supplies shared-session text in lead on the first row only.
    This function reads state and must never command a robot. Its caller handles
    read/render failures so display errors do not terminate the control loop.
    """
    q = np.asarray(one.robot.get_joint_pos(), dtype=float)
    extra = ""
    if one.mode == "teleop" and one.teleop is not None:
        extra = f"  EE {np.round(one.teleop.ee_position(), 3)}"
        # ⭐⭐ SHOW THE WORKSPACE WALL, because it used to be invisible. Julien,
        # 2026-08-13: *"it stops moving in the direction I want it to move even though the
        # arm hasn't even close to fully extended."* Showing it settled the cause in one
        # session, and the limit is now a fixed sphere around the base plus a floor rather
        # than a cube that moved every time TELEOP was entered. FINDINGS §41.1 and §43.
        lim_r, lim_f = effective_limits(one.home_ee, reach, floor)
        ee_now = one.teleop.ee_position()
        out, up = workspace_room(ee_now, lim_r, lim_f)
        extra += f"  reach {out:.2f}/{lim_r:.2f}m"
        if out > 0.9 * lim_r:
            extra += " ⚠️ AT THE EDGE"
        # ⚠️ The floor is only worth screen space when it is close. With the floor at the
        # base plane this starts warning around z = 0, which is roughly desk height and
        # exactly where a pick happens. So it reads as "you are down at the desk" rather
        # than as an alarm.
        if up < 0.10:
            extra += f"  ⚠️ {up * 100:.0f}cm above the floor (z={ee_now[2]:+.2f})"
        # ⭐ How far the goal is running ahead of the pose actually achieved. Pinned at the
        # limit = the arm cannot follow (joint limit, singularity, something in the way),
        # which used to present only as the arm behaving strangely. See
        # CartesianTeleop._limit_lead().
        lead_m, lead_r = one.teleop.lead()
        if lead_m > 0.8 * one.teleop.max_lead_m or lead_r > 0.8 * one.teleop.max_lead_rad:
            extra += f"  ⚠️ STUCK lead {lead_m * 100:.0f}cm/{np.degrees(lead_r):.0f}°"
        # ⭐ Say WHY the arm feels slow — with the MEASURED numbers, never a guessed cause
        # (item 21, closed 2026-08-18). The old wording asserted "near the reach limit"
        # while the arm stood in a comfortable pose (FINDINGS §41.2). What the throttle
        # actually measures is the joint rate the IK asked for versus the cap, so that
        # pair is what prints. Reading it: spikes only when extended = a singular pose;
        # high everywhere = the linear speed is set faster than the joints can serve.
        if one.teleop.speed_scale < 0.95:
            extra += (f"  ⚠️ SLOWED to {one.teleop.speed_scale * 100:.0f}% (joints asked "
                      f"for {one.teleop.requested_rate:.1f} rad/s, cap "
                      f"{one.teleop.max_joint_rate:.1f})")
    # ⭐ `jaw` is shown separately from `hottest` on purpose. Watching this number plateau
    # is the actual test of the 2π gripper frame fix; watching `hottest` is not, because
    # the shoulder sits hotter than the gripper all session.
    # ⛔ "??" when the read failed, never a number. A fabricated 0 °C is exactly what made
    # a disarmed thermal guard look healthy on screen (FINDINGS §24.1), and the readout is
    # the only place a human would have noticed.
    therm = (f"hottest {one.hottest:4.0f}°C" if one.hottest is not None
             else "hottest   ??°C ⚠️BLIND")
    if one.jaw_temp is not None:
        therm += f"  jaw {one.jaw_temp:4.0f}°C"
    # ⭐ GUIDE reports DRIFT from where it went weightless. On 2026-08-10 the arm sank to
    # its own stops over ~33 s while this line calmly read "hottest 35°C" — gravity
    # compensation was 39% short at the elbow (FINDINGS §11) and nothing on screen was
    # measuring the one quantity that was going wrong. The cause is fixed; the instrument
    # should exist anyway. A readout must show what can fail, not what looks calm.
    if one.mode == "guide" and one.guide_ref is not None:
        sank = float(np.max(np.abs(q[:N_ARM] - one.guide_ref[:N_ARM])))
        extra = f"  drift {sank:5.3f} rad ({np.degrees(sank):4.1f}°){extra}"
    # ⭐ `note` is whatever the session knows about this arm that the arm does not: today
    # only the mirror link's state, which is a relationship between two arms and therefore
    # cannot live on either.
    if note:
        extra = f"  {note}{extra}"
    # ⭐ TELEOP carries its control frame in the bracket (item 28, closed 2026-08-18):
    # `v` aims at ONE arm, so two arms can be driven in different frames at once and
    # nothing on screen said which was which. `TELEOP/w`·`/t`·`/c` = world·tool·camera,
    # exactly 8 characters like CONTROLS, so the columns stay aligned.
    if one.mode == "teleop":
        label = f"TELEOP/{one.frame[0]}"
    else:
        label = "CONTROLS" if one.mode == "map" else one.mode.upper()
    return (f"[{one.name} {label:8}]{lead}  {therm}"
            f"  q {np.round(q[:N_ARM], 2)}{extra}   ")

