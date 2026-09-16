"""Record synthetic joints/images through the actual asynchronous recording owner.

Run from the repository: .venv-teleop/bin/python examples/record_take.py
Creates and retains a new temporary directory; user recording slots are untouched.
No camera or motor is opened. Times and images below are explicitly synthetic.
"""
from collections.abc import Callable
import json
from pathlib import Path
import tempfile
import time

import numpy as np

from yam.cameras.frame import Frame
from yam.recording import Trajectory
from yam.recording_session import RecordingSession


def wait_for(recording: RecordingSession, done: Callable[[], bool]) -> None:
    """Console-demo wait. The live operator polls while continuing control cycles."""
    deadline = time.monotonic() + 5.0
    while True:
        recording.poll()
        if recording.frame_error or recording.save_error:
            raise RuntimeError(recording.frame_error or str(recording.save_error))
        if done():
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Recording work is incomplete; output directory retained")
        time.sleep(0.005)


def main() -> None:
    root = Path(tempfile.mkdtemp(prefix="yam-recording-example-"))
    print(f"Synthetic example output (retained): {root}", flush=True)
    recording = RecordingSession()
    mono0 = 10_000_000_000  # synthetic nanosecond origin, shared by joints and frames
    modes = ["B:teleop"]
    try:
        recording.start(10.0, meta={"simulated": True, "example": True,
                                   "arms": ["B"], "joints_per_arm": 7}, modes=modes)
        recording.start_frames(root / "pending", ["demo"], mono0)
        for index in range(6):
            t = 10.0 + index / 10
            joints = [index * 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
            recording.sample(t, joints, modes)
            # Frame.rgb is a legacy name: the encoder expects OpenCV BGR pixels.
            bgr = np.full((48, 64, 3), (index * 30, 50, 180), dtype=np.uint8)
            frame = Frame("demo", index + 1, None, mono0 + index * 100_000_000, bgr)
            recording.offer_frames({"demo": frame})

        take = recording.freeze()
        recording.sample(99.0, joints, modes)  # ignored: the take is already frozen
        print(f"Frozen: {len(take)} samples, {take.duration:.1f} s")
        wait_for(recording, lambda: recording.ready)
        recording.request_save(root, "1")
        wait_for(recording, lambda: recording.saved is not None)
        saved = recording.saved
        if saved is None:
            raise RuntimeError("Save completed without a published take")
        loaded = Trajectory.load(saved.path)
        index = json.loads((root / "frames/1/demo/index.json").read_text())
        print(f"Reloaded: {len(loaded)} samples, {loaded.duration:.1f} s")
        print(f"Frames written: {index['written']}; dropped: {index['dropped']}; "
              f"errors: {index['write_errors']}")
        print(f"Saved take: {saved.path}")
        if saved.warning:
            print(saved.warning)
    except BaseException as failure:
        # Keep the original failure and unfinished files for inspection.
        try:
            recording.shutdown()
        except Exception as cleanup:
            failure.add_note(f"Recording cleanup also failed: {cleanup}")
        raise
    else:
        recording.shutdown()


if __name__ == "__main__":
    main()
