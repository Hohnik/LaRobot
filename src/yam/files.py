"""Listing files on disk without picking up the operating system's own litter.

⛔⭐⭐ WHY THIS EXISTS, and it produced four separate wrong answers in one session on 2026-08-19 ([FINDINGS §76](../../docs/FINDINGS.md)).

macOS stores a file's extended attributes in a **sidecar file named `._<original>`** whenever it writes to a filesystem that cannot hold those attributes itself. Copy `recordings/` from the Mac to a USB stick and on to the Linux station and every single file arrives with a twin. The station's copy had **813 of them**. They are not readable as anything: `._5.json` is binary, so `json.loads` on it raises `UnicodeDecodeError`.

⛔ Every plain `folder.glob("*.json")` in this repo picked them up, and each one failed differently:

- `checks/check_recordings.py` crashed outright on `._5.json` in a second pass that had no error guard.
- The session's playback menu offered `._5` as a recording you could play.
- `glob("*.jpg")` matched `._000123.jpg`, so all three of slot 5's camera counts came out at **exactly double** what the recording claimed, and the checker declared the frames foreign to the recording. That was a false alarm about real data.
- Any `._X.md` in `docs/` would break `check_links`, `check_flags` and `test_unwrap` the same way, because they read every match as text.

⭐ THE RULE. A file whose name starts with `.` is never this repo's data. `.DS_Store`, `._anything`, editor swap files and `.gitignore` are all the same category: present on disk, invisible to the project. So every listing of the project's OWN files goes through here, and the sidecars are reported once rather than silently skipped, because an operator whose copy brought 813 pieces of litter should be told.

⚠️ This is deliberately NOT a filter on `/dev/video*` or any other system path. Those are not the project's files and they have their own naming rules (`yam/platform.py`).
"""

from __future__ import annotations

from pathlib import Path

__all__ = ["is_os_litter", "is_mac_sidecar", "listing", "sidecars"]


def is_mac_sidecar(path: Path | str) -> bool:
    """True for a macOS AppleDouble twin, `._<original>`. These specifically mislead counts."""
    return Path(path).name.startswith("._")


def is_os_litter(path: Path | str) -> bool:
    """True for anything the operating system left behind rather than this project writing it.

    Any name starting with a dot: `._5.json`, `.DS_Store`, `.gitignore`, `.#swapfile`.
    """
    return Path(path).name.startswith(".")


def listing(folder: Path | str, pattern: str) -> list[Path]:
    """`sorted(folder.glob(pattern))` with the operating system's litter removed.

    ⭐ Use this instead of `glob` for every listing of files this project wrote. Returns an empty list when the folder does not exist, which is what every caller wanted anyway.
    """
    folder = Path(folder)
    if not folder.is_dir():
        return []
    return sorted(p for p in folder.glob(pattern) if not is_os_litter(p))


def sidecars(folder: Path | str, pattern: str = "*") -> list[Path]:
    """The macOS sidecars a listing just skipped, recursively. For telling the operator.

    ⭐ Recursive on purpose: the counts that went wrong were inside `frames/<slot>/<camera>/`, three levels down from where anyone was looking.
    """
    folder = Path(folder)
    if not folder.is_dir():
        return []
    return sorted(p for p in folder.rglob(pattern) if is_mac_sidecar(p))
