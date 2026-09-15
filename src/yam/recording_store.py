"""Publish a recording and its frames together, with rollback on write failures.

The slot JSON is the publication point: it is absent while frame directories are
swapped. A stopped process can leave a recovery directory, but cannot publish old
JSON alongside new frames. Recovery directories are retained if rollback fails.
Only one process may write a given recordings directory at a time.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import tempfile
from typing import Any

from yam.provenance import dt_now, git_commit
from yam.recording import Trajectory


@dataclass(frozen=True)
class SavedTake:
    path: Path
    has_frames: bool
    warning: str = ""


def save_take(take: Trajectory, recordings_dir: Path, slot: str, *,
              pending_frames: Path | None = None,
              frame_report: dict[str, Any] | None = None) -> SavedTake:
    """Save an operator-confirmed slot, preserving both versions on failure.

    Serialize before touching the old slot. Move its JSON out of view before
    changing frames, and publish the new JSON last. On failure restore frames
    before restoring old JSON. Never delete the caller's pending frames on error.
    """
    if len(slot) != 1 or slot not in '0123456789':
        raise ValueError('A recording slot must be one digit from 0 to 9')
    if (pending_frames is None) != (frame_report is None):
        raise ValueError('Pending frames and their report must be supplied together')
    if frame_report is not None:
        reports = frame_report.get('per_camera', {})
        if not reports or any(report.get('flushed') is not True for report in reports.values()):
            raise ValueError('Every camera writer must finish before saving its frames')
    root = Path(recordings_dir)
    root.mkdir(parents=True, exist_ok=True)
    unresolved = list(root.glob(f'.save-{slot}-*'))
    if unresolved:
        raise RuntimeError(f'Recover the previous save before replacing slot {slot}: {unresolved[0]}')
    path, frames = root / f'{slot}.json', root / 'frames' / slot
    candidate = copy.copy(take)
    candidate.meta = copy.deepcopy(take.meta)
    if pending_frames is not None:
        candidate.meta['cameras'] = {
            'dir': f'frames/{slot}', 'mono0_ns': frame_report['mono0_ns'],
            'per_camera': {
                camera: {key: report[key] for key in ('written', 'dropped', 'write_errors')}
                for camera, report in frame_report['per_camera'].items()},
        }
    else:
        candidate.meta.pop('cameras', None)
    candidate.meta.update(commit=git_commit(), recorded_at=dt_now())
    stage = Path(tempfile.mkdtemp(prefix=f'.save-{slot}-', dir=root))
    old_json, old_frames = stage / 'previous.json', stage / 'previous.frames'
    moved_json = moved_old_frames = moved_new_frames = False
    try:
        # This file also records how to recover after process termination.
        (stage / 'recovery.json').write_text(json.dumps({
            'slot': slot, 'pending_frames': str(pending_frames) if pending_frames else None,
            'procedure': 'If slot JSON exists, inspect it before cleanup. Otherwise move new slot frames back to pending_frames, restore previous.frames to frames/slot, then previous.json to slot.json. Keep new.json to recover the new take.',
        }, indent=2))
        candidate.save(stage / 'new.json')
        if path.exists():
            path.rename(old_json)
            moved_json = True
        if frames.exists():
            frames.rename(old_frames)
            moved_old_frames = True
        if pending_frames is not None:
            frames.parent.mkdir(parents=True, exist_ok=True)
            Path(pending_frames).rename(frames)
            moved_new_frames = True
        (stage / 'new.json').replace(path)
    except BaseException as failure:
        try:
            if moved_new_frames:
                frames.rename(pending_frames)
            if moved_old_frames:
                old_frames.rename(frames)
            if moved_json:
                old_json.rename(path)
        except BaseException as rollback:
            failure.add_note(f'Save rollback failed: {rollback}. Both versions are retained in {stage}. Recover this directory before using the slot.')
            raise failure
        try:
            shutil.rmtree(stage)
        except OSError as cleanup:
            failure.add_note(f'Could not remove save staging directory {stage}: {cleanup}')
        raise
    take.meta = candidate.meta
    warning = ''
    try:
        shutil.rmtree(stage)
    except OSError as cleanup:
        warning = f'Recording saved, but old-slot cleanup failed at {stage}: {cleanup}'
    return SavedTake(path, pending_frames is not None, warning)
