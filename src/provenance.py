"""Which code, and when — the two facts every measurement here has to carry.

⭐ WHY THIS IS ITS OWN MODULE

Provenance has paid for itself twice in one day on this rig, before any dataset
exists (FINDINGS §33.2, §34.7). Both times a written number turned out to
describe a file that had since been overwritten, and the *only* reason the
mismatch was detectable was that the file carried the commit it was made under
and the time it was made.

Three separate things now record it — a recording, a playback's tracking log, and
the motor-register baseline — which is the point at which a copied helper starts
to drift. This repo has been bitten by copy-paste three times (``src/spacemouse.py``
exists because a device fix landed in only one of two copies), so the third caller
gets a module instead of a third copy.

⚠️ ``scripts/teleop_session.py`` still carries its own ``git_commit`` and
``dt_now``. They are identical to these. They collapse into this module during
the ``ArmSession`` restructure, which rewrites that file anyway — doing it now
would edit the script Julien is about to test, for no gain (ROADMAP §6.1 step 1).
"""

from __future__ import annotations

import subprocess
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def git_commit() -> str:
    """Short hash of the code that is running, or ``"unknown"``.

    Julien's requirement, 2026-08-12: *"being able to reproduce everything and
    connect it to other research papers."* Which version of the code produced a
    measurement is free to record now and unrecoverable later.

    ⚠️ Never raises. A missing git is not a reason to lose a measurement, and a
    helper that can throw inside a control loop is a hazard rather than a record.
    """
    try:
        out = subprocess.run(
            ["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        return out.stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001
        return "unknown"


def dt_now() -> str:
    """Wall-clock time as text, for the record.

    Local time with the offset attached, because a human reads it and because a
    bare naive timestamp is what made two 2026-08-13 measurements ambiguous
    about which of two runs three hours apart they described.
    """
    return datetime.now().astimezone().isoformat(timespec="seconds")
