"""Read-only inventory of interrupted saves and retained camera directories.

Presence and parseability do not prove that a JSON file and images belong together.
Never infer permission to delete from an absent recording or follow manifest paths.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

from yam.recording import Trajectory


def trajectory_summary(path: Path) -> dict:
    if path.is_symlink():
        return {"state": "symlink_skipped"}
    if not path.exists():
        return {"state": "absent"}
    try:
        take = Trajectory.load(path)
        return {"state": "readable", "samples": len(take),
                "seconds": take.duration, "joints": take.n_joints,
                "camera_reference": (take.meta.get("cameras") or {}).get("dir")}
    except Exception as exc:
        return {"state": "unreadable", "error": f"{type(exc).__name__}: {exc}"}


def frame_summary(path: Path) -> dict:
    """Count retained images/indexes without decoding images or following symlinks."""
    if path.is_symlink():
        return {"state": "symlink_skipped"}
    if not path.exists():
        return {"state": "absent"}
    if not path.is_dir():
        return {"state": "not_a_directory"}
    counts = {"state": "present", "images": 0, "indexes": 0, "symlinks_skipped": 0}
    errors = []
    for directory, folders, files in os.walk(path, followlinks=False,
                                             onerror=lambda exc: errors.append(str(exc))):
        for name in folders + files:
            item = Path(directory) / name
            if item.is_symlink():
                counts["symlinks_skipped"] += 1
            elif not name.startswith(".") and name in files:
                counts["images"] += item.suffix.lower() in {".jpg", ".jpeg", ".png"}
                counts["indexes"] += name == "index.json"
    if errors:
        counts.update(state="partially_readable", errors=errors)
    return counts


def inspect_recovery(root: Path) -> dict:
    """Inspect one recording root. Call separately for recordings/sim."""
    root = Path(root)
    if not root.is_dir():
        raise ValueError(f"Recording directory does not exist: {root}")
    published = {p.name: trajectory_summary(p) for p in sorted(root.glob("*.json"))
                 if not p.name.startswith(".")}
    # Match only local frame directory references; never traverse JSON-supplied paths.
    claims = {summary.get("camera_reference") for summary in published.values()
              if isinstance(summary.get("camera_reference"), str)}
    retained = []
    frames = root / "frames"
    if frames.is_symlink():
        retained.append({"path": "frames", "state": "symlink_skipped"})
    elif frames.is_dir():
        for child in sorted(frames.iterdir()):
            if child.name.startswith("."):
                continue
            relative = child.relative_to(root).as_posix()
            if relative not in claims or child.name.startswith("pending_"):
                retained.append({"path": relative, **frame_summary(child)})
    saves = []
    for stage in sorted(root.glob(".save-*")):
        entry = {"path": stage.name}
        if stage.is_symlink() or not stage.is_dir():
            entry["error"] = "Not an ordinary directory; skipped"
            saves.append(entry)
            continue
        try:
            manifest = stage / "recovery.json"
            if manifest.is_symlink():
                raise ValueError("recovery.json is a symlink; skipped")
            data = json.loads(manifest.read_text())
            slot = data["slot"]
            if not isinstance(slot, str) or len(slot) != 1 or slot not in "0123456789":
                raise ValueError("invalid slot in recovery.json")
            entry.update(slot=slot, pending_frames_recorded=data.get("pending_frames"),
                         published=trajectory_summary(root / f"{slot}.json"),
                         slot_frames=frame_summary(frames / slot)
                         if not frames.is_symlink() else {"state": "symlink_skipped"})
        except Exception as exc:
            entry["error"] = f"{type(exc).__name__}: {exc}"
        entry.update(candidate=trajectory_summary(stage / "new.json"),
                     previous=trajectory_summary(stage / "previous.json"),
                     previous_frames=frame_summary(stage / "previous.frames"))
        saves.append(entry)
    unreadable = [name for name, value in published.items() if value["state"] != "readable"]
    return {"directory": str(root), "published": published,
            "retained_frames": retained, "interrupted_saves": saves,
            "needs_inspection": bool(retained or saves or unreadable),
            "limits": "Read-only snapshot; stop recording before recovery. Images alone do not "
                      "recover in-memory joint samples. Readable JSON and frame counts do not "
                      "prove pairing, completeness or durability. Listing evidence never "
                      "authorizes deletion."}
