#!/usr/bin/env python3
"""Drive a full `--sim` session end to end and check it does what it claims to.

    uv run checks/drive_sim_session.py

⭐⭐ WHY THIS IS IN THE REPO. It caught a crash that **616 unit tests could not**, hours
after it was written. The save handler read a local (`replace_slot`) that is assigned only
inside the overwrite-guard branch, so the FIRST save of any session raised
`UnboundLocalError` and took the whole session down.

⛔ `tests/test_save_slot.py` has 12 tests of that exact decision and every one passed,
because they call the pure function directly and hand it that argument. **The defect was in
the CALL SITE.** Extracting a decision into a testable function does not test the code that
calls it, and nothing else in this repo runs the 3000-line loop from end to end.

⭐ What it exercises in one run: building two arms, the arm selector, TELEOP on both,
recording 14 joints, the save prompt, replaying a REAL recording on simulated arms, the
staged park with one arm waiting for the other, the tracking table, and `q q` parking and
disabling all 14 motors.

⚠️ It needs a pseudo-terminal, because the session reads keys from a terminal in raw mode
and a plain pipe will not do. ⚠️ Its keypresses are on fixed delays, so a very loaded
machine can drift; a failing check is worth re-running once before believing it.

⛔ It only ever touches `recordings/sim/`, never a real recording.
"""

import os
import pathlib
import pty
import re
import select
import subprocess
import sys
import time

REPO = str(pathlib.Path(__file__).resolve().parent.parent)

# (delay before typing, keys). Delays are generous: the session prints a lot and some
# steps wait for the simulated arm to settle.
SCRIPT = [
    (4.0, ""),          # let both arms build
    (1.5, "n"),         # the SETTINGS screen
    (1.0, "1"),         # pick max_speed
    (0.8, "+"),         # raise it
    (0.8, "+"),
    (0.8, "\x1b[B"),    # down arrow — the key a person reaches for at a list
    (0.8, "\x1b[A"),    # and back up
    (0.8, "3"),         # pick max_lag
    (0.8, "+"),         # raise it
    (0.8, "9"),         # pick vel_ff — the live feedforward path (item 44)
    (0.8, "+"),         # 0 -> 0.25, pushed onto the live SafeRobots
    (0.8, "0"),         # back to how the session started
    (1.0, "h"),         # leave SETTINGS
    (1.0, "a"),         # B -> G
    (0.5, "a"),         # G -> BOTH
    (1.0, "t"),         # TELEOP on both
    (1.5, "w"),         # start recording
    (2.5, "w"),         # stop recording
    (1.0, "8"),         # save to slot 8 (goes to recordings/sim/)
    (2.0, "l"),         # play...
    (1.0, "0"),         # ...the synthetic two-arm recording this driver writes itself
    (1.0, "\r"),        # confirm -> parks both arms, then plays
    (16.0, ""),         # let the park and the 5.2s playback run
    # ⭐ COMPOSITE RUN (ROADMAP §6.6.1a): pose 1 → play take 8 → pose 1 again. Slot 1 is
    # a real saved waypoint on arm B (G skips it with a note); take 8 is the two-arm sim
    # recording made above. Three legs, two handover kinds, one completion line.
    (2.0, "p"),
    (0.8, "1"),
    (0.8, "w"),         # arms "next digit is a take"
    (0.8, "8"),
    (0.8, "1"),
    (0.8, "\r"),        # first Enter shows the plan (3 entries = confirm step)
    (0.8, "\r"),        # second Enter runs it
    (30.0, ""),         # park → take → park, all three legs
    (1.0, "q"),         # quit
    (1.5, "q"),         # park + disable
    (16.0, ""),         # let the park finish
]


def write_synthetic_take(path: pathlib.Path) -> None:
    """A two-arm recording with real movement in BOTH arms, written from scratch.

    Both arms have to move, and faster than the tracking table's 0.01 rad/s floor, because
    the checks assert that the table names rows for arm B *and* arm G — the defect that
    produced those rows anonymously is [FINDINGS §60.1](../docs/FINDINGS.md). The shape is a
    slow sine so the simulated arm can follow it: ~0.3 rad of travel over 3 s is 0.2-0.6
    rad/s per joint, well inside every clamp and well above the floor.

    ⚠️ Stamped `simulated: true` and written under `recordings/sim/`, the same two independent
    marks a sim take gets when the session saves one ([FINDINGS §60.2](../docs/FINDINGS.md)),
    so this file can never be mistaken for a demonstration.
    """
    import math
    import sys as _sys

    _sys.path.insert(0, str(pathlib.Path(REPO) / "src"))
    from yam.recording import Trajectory

    take = Trajectory(meta={
        "arms": ["B", "G"], "joints_per_arm": 7,
        "method": "sim:synthetic (written by drive_sim_session, not a demonstration)",
        "simulated": True,
        "why": "this driver must not depend on rig-local recordings — FINDINGS §75.1",
    })
    hz, seconds = 90.0, 3.0
    for i in range(int(hz * seconds) + 1):
        t_s = i / hz
        wave = 0.3 * math.sin(2 * math.pi * t_s / seconds)
        b = [wave * (1 + j * 0.1) for j in range(6)] + [0.3]
        g = [-wave * (1 + j * 0.1) for j in range(6)] + [0.5]
        take.append(t_s, b + g)
    path.parent.mkdir(parents=True, exist_ok=True)
    take.save(path)
    print(f"wrote a synthetic two-arm take to {path.relative_to(pathlib.Path(REPO))} "
          f"({take.duration:.1f}s, {len(take)} samples, peak "
          f"{take.max_joint_speed():.2f} rad/s)")


def main() -> int:
    # ⭐ Clear this driver's OWN artifact first. Leaving it triggers the session's
    # overwrite guard, which asks for a confirming keypress this script does not send, and
    # then every downstream check fails for a reason that has nothing to do with the code.
    # It only ever removes recordings/sim/, never a real recording.
    stale = pathlib.Path(REPO) / "recordings" / "sim" / "8.json"
    if stale.is_file():
        stale.unlink()
        print(f"cleared {stale}")

    # ⛔⭐ WHY THIS DRIVER WRITES ITS OWN RECORDING (FINDINGS §75.1). It used to play slot 7,
    # one of Julien's real hand-guided takes — and `recordings/` is gitignored, so on the
    # Linux PC's fresh clone slot 7 did not exist. The playback then had nothing to move, and
    # two checks failed with a message about unnamed table rows that pointed nowhere near the
    # cause. **A checker that depends on data it does not create goes blind on a new machine**,
    # which is the same class of defect as a checker that validates nothing (§70.8).
    #
    # Slot 0 is used because it is the one slot his rig leaves free, and because a sim
    # recording SHADOWS a real one of the same number (`slot_for_reading`) — writing slot 7
    # here would have quietly replaced the real take on the Mac and removed the very thing
    # that made this check interesting there.
    write_synthetic_take(pathlib.Path(REPO) / "recordings" / "sim" / "0.json")

    master, slave = pty.openpty()
    # ⭐ Anything on THIS script's command line is passed through to the session, so any
    # new session flag can be sim-driven end to end without editing this file:
    #     uv run checks/drive_sim_session.py --vel-ff 0.5
    proc = subprocess.Popen(
        ["uv", "run", "apps/teleop_session.py", "--sim", "--yes",
         "--arms", "B,G", "--start-mode", "hold", *sys.argv[1:]],
        cwd=REPO, stdin=slave, stdout=slave, stderr=slave, close_fds=True,
    )
    os.close(slave)
    out = []

    def drain(seconds):
        end = time.time() + seconds
        while time.time() < end:
            r, _, _ = select.select([master], [], [], 0.2)
            if r:
                try:
                    chunk = os.read(master, 65536)
                except OSError:
                    return
                if not chunk:
                    return
                out.append(chunk.decode("utf-8", "replace"))

    for delay, keys in SCRIPT:
        drain(delay)
        # The session can finish before the script runs out of keys. Writing to a pty whose
        # child has exited raises EIO, which looks like a crash in the session and is not.
        if keys and proc.poll() is None:
            try:
                os.write(master, keys.encode())
            except OSError:
                break
    drain(6.0)

    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    text = "".join(out)
    # Strip the terminal control codes the status painter emits.
    clean = re.sub(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b[=>]|\r", "", text)
    clean = clean.replace("\x1b", "")
    path = str(pathlib.Path(REPO) / "recordings" / "sim" / "last_drive.log")
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write(clean)

    print(f"exit code: {proc.returncode}   log: {path}   {len(clean)} chars\n")
    # The claims a working --sim must support, in the order they should happen.
    checks = [
        ("simulated pucks announced", r"SIMULATED PUCKS"),
        ("the SETTINGS screen opens", r"SETTINGS — the speed and safety limits"),
        ("a setting can be raised live", r"max_speed 1\.000 → 1\.500"),
        ("a change shows as ONE line, not the whole screen",
         r"▸ max_speed 1\.500 → 2\.000"),
        ("the ladder gives ROUND numbers", r"▸ max_speed 1\.500 → 2\.000"),
        ("up/down arrows move the selection", r"▸ max_lag 0\.250"),
        ("0 reverts to the session start", r"back to the values this session started"),
        ("leaving says nothing was written", r"nothing was written to the file"),
        ("both arms built as SIMULATED", r"SIMULATED arm B"),
        ("arm G built too", r"SIMULATED arm G"),
        ("says it has no gravity", r"No gravity"),
        ("teleop entered on both", r"TELEOP on B\+G"),
        ("recording says 14 joints", r"RECORDING.*14 joints"),
        ("recording saved to the SIM folder", r"recording 8 saved"),
        ("park to the start pose", r"PARK →"),
        ("waited for the slower arm", r"waiting for"),
        ("PLAYBACK ran", r"PLAYING"),
        ("playback finished", r"PLAYBACK finished"),
        ("tracking table printed", r"how well each joint kept up"),
        ("rows name their arm", r"\bB base_yaw\b"),
        ("a recording is listed from the sim folder", r"saved: 0\(sim\), 8\(sim\)"),
        ("arm G's rows are named too", r"\bG base_yaw\b"),
        ("no anonymous rows", r"^(?!.*\bjoint  +worst lag)"),
        ("composite run announced with its leg count", r"COMPOSITE RUN: 3 leg"),
        ("the take leg parked to the recording's start", r"start pose in recording 8"),
        ("the queue narrated between legs", r"composite: \d+ leg\(s\) still queued"),
        ("composite completed", r"COMPOSITE RUN complete — all 3 leg"),
        ("both arms parked", r"arm G PARKED"),
        ("motors reported disabled", r"motors confirmed disabled: \[1, 2, 3, 4, 5, 6, 7\]"),
        # ⭐ The worst single pass, added 2026-08-20. It is printed by the shutdown path, so
        # this is the only check in the repo that proves the instrument is wired into the real
        # loop rather than only into its own unit tests. ⚠️ The NUMBER here is meaningless: a
        # simulated arm answers in microseconds (FINDINGS §76.12). Only the wiring is checked.
        ("the worst loop pass is reported", r"loop: worst pass \d+\.\d+ ms at t="),
    ]
    # ⛔⭐ FINDINGS §72.1: the composite's take leg once started PLAYING while an arm still
    # had 1.28 rad of park left — a pose-leg arrival was credited to the take leg armed in
    # the same event, and the old checks could not see it (the run still "completed").
    # These two measure the CONSEQUENCE, which is the §0 rule: every replay arm must
    # actually ARRIVE (two "PARK reached" between the take-park announcements and PLAYING),
    # and the start-pose guard must never fire in a healthy run.
    checks.append(("every arm ARRIVED before the take played", None))
    checks.append(("no playback was refused off its start pose", None))
    bad = 0
    for label, pattern in checks:
        if label == "no anonymous rows":
            hit = not re.search(r"^ +joint +worst lag", clean, re.M)
        elif label == "every arm ARRIVED before the take played":
            marks = list(re.finditer(r"start pose in recording 8", clean))
            hit = False
            if marks:
                tail = clean[marks[-1].end():]
                play = tail.find("PLAYING")
                hit = play >= 0 and tail[:play].count("PARK reached") >= 2
        elif label == "no playback was refused off its start pose":
            hit = "NOT playing" not in clean
        else:
            hit = re.search(pattern, clean, re.M | re.S)
        ok = bool(hit)
        bad += not ok
        print(f"  {'✓' if ok else '⛔'} {label}")
    print(f"\n{len(checks) - bad}/{len(checks)} checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
