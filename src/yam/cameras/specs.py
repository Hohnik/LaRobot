"""Parse the camera arguments a human actually types, and refuse the one combination that would poison data.

⭐ WHY THE FLATTENER EXISTS (FINDINGS §71.1): the 2026-08-19 bench pass LOST its bandwidth measurement to argument format. The his-list said `--indices <the three indices>`, Julien typed `--indices 0, 1, 2` and then `--indices 0 1 2` — both natural spellings, both dictated through speech-to-text he never sees — and argparse refused both, because the flag took ONE comma-joined string. The fix is not a better error message; it is accepting every spelling a person would produce. `flatten_tokens` makes `0 1 2`, `0,1,2` and `0, 1, 2` identical.

⛔ `sim_camera_error` is ROADMAP §8.2 item 48's trap ②, as a pure function so a test can hold it: a `--sim` session must refuse `--cameras`. A simulated take that carries real photographs is a mislabelling engine — the sim stamp says "nothing here happened" while the images show the real bench, and a dataset tool that trusts either one alone is deceived.
"""

from __future__ import annotations

from typing import Sequence

__all__ = ["flatten_tokens", "parse_indices", "camera_dir_name", "sim_camera_error"]


def flatten_tokens(tokens: Sequence[str]) -> list[str]:
    """Every comma- or space-separated CLI spelling, as one flat list.

    `["0,", "1,", "2"]` (argparse's view of `--indices 0, 1, 2`) → `["0", "1", "2"]`. Empty pieces vanish, so a trailing comma costs nothing.
    """
    return [piece.strip() for tok in tokens for piece in str(tok).split(",") if piece.strip()]


def parse_indices(tokens: Sequence[str]) -> list[int]:
    """Flattened tokens as camera indices, refusing loudly on anything non-numeric."""
    out: list[int] = []
    for piece in flatten_tokens(tokens):
        if not piece.lstrip("-").isdigit():
            raise ValueError(
                f"{piece!r} is not a camera index — give plain numbers, "
                "space- or comma-separated: --indices 0 1 2"
            )
        out.append(int(piece))
    return out


def camera_dir_name(spec: str) -> str:
    """A camera spec as a filesystem-safe directory name.

    `d405:255323071773` → `d405-255323071773` · `c920` → `c920` · a raw index `2` → `cam2`. The name lands in `recordings/frames/<slot>/<name>/` and inside every episode's metadata, so it must never contain a path separator and a bare number must say what it is.
    """
    clean = "".join(ch if ch.isalnum() else "-" for ch in spec.strip()).strip("-")
    return f"cam{clean}" if clean.isdigit() else clean


def sim_camera_error(sim: bool, cameras: str | None) -> str | None:
    """The refusal text when `--sim` and `--cameras` meet, or None when the combination is fine."""
    if sim and cameras:
        return ("--sim refuses --cameras: a simulated take with real photographs in it is a "
                "mislabelling engine — the sim stamp and the images would contradict each "
                "other, and training-data tools believe whichever one they read. Record "
                "camera frames in a real session only.")
    return None
