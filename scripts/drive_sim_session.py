#!/usr/bin/env python3
"""Drive a full `--sim` session end to end and check it does what it claims to.

    uv run scripts/drive_sim_session.py

⭐⭐ WHY THIS IS IN THE REPO. It caught a crash that **616 unit tests could not**, hours
after it was written. The save handler read a local (`replace_slot`) that is assigned only
inside the overwrite-guard branch, so the FIRST save of any session raised
`UnboundLocalError` and took the whole session down.

⛔ `scripts/test_save_slot.py` has 12 tests of that exact decision and every one passed,
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
    (0.8, "0"),         # back to how the session started
    (1.0, "h"),         # leave SETTINGS
    (1.0, "a"),         # B -> G
    (0.5, "a"),         # G -> BOTH
    (1.0, "t"),         # TELEOP on both
    (1.5, "w"),         # start recording
    (2.5, "w"),         # stop recording
    (1.0, "8"),         # save to slot 8 (goes to recordings/sim/)
    (2.0, "l"),         # play...
    (1.0, "7"),         # ...HIS REAL two-arm recording, on simulated arms
    (1.0, "\r"),        # confirm -> parks both arms, then plays
    (16.0, ""),         # let the park and the 5.2s playback run
    (1.0, "q"),         # quit
    (1.5, "q"),         # park + disable
    (16.0, ""),         # let the park finish
]


def main() -> int:
    # ⭐ Clear this driver's OWN artifact first. Leaving it triggers the session's
    # overwrite guard, which asks for a confirming keypress this script does not send, and
    # then every downstream check fails for a reason that has nothing to do with the code.
    # It only ever removes recordings/sim/, never a real recording.
    stale = pathlib.Path(REPO) / "recordings" / "sim" / "8.json"
    if stale.is_file():
        stale.unlink()
        print(f"cleared {stale}")

    master, slave = pty.openpty()
    proc = subprocess.Popen(
        ["uv", "run", "scripts/teleop_session.py", "--sim", "--yes",
         "--arms", "B,G", "--start-mode", "hold"],
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
        ("real recording found from sim", r"saved: 8\(sim\)"),
        ("arm G's rows are named too", r"\bG base_yaw\b"),
        ("no anonymous rows", r"^(?!.*\bjoint  +worst lag)"),
        ("both arms parked", r"arm G PARKED"),
        ("motors reported disabled", r"motors confirmed disabled: \[1, 2, 3, 4, 5, 6, 7\]"),
    ]
    bad = 0
    for label, pattern in checks:
        hit = re.search(pattern, clean, re.M | re.S)
        if label == "no anonymous rows":
            hit = not re.search(r"^ +joint +worst lag", clean, re.M)
        ok = bool(hit)
        bad += not ok
        print(f"  {'✓' if ok else '⛔'} {label}")
    print(f"\n{len(checks) - bad}/{len(checks)} checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
