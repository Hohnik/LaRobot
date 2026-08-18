"""One camera, one background thread, newest frame wins.

⭐ `FrameGrabber` moved here VERBATIM from `apps/camera_view.py` on 2026-08-19 (ROADMAP §8.2 item 6), where it is hardware-confirmed; the app imports it back, so there is exactly one copy. The only additions are the store-time host stamp and `newest_stamped()` — every call the app already made (`newest()`, `capture_fps()`, `pixel_format()`, `stop()`) is unchanged.
"""

from __future__ import annotations

import threading
import time


def fourcc_name(code: int) -> str:
    """The four letters of a pixel format code, as the camera reports it.

    ⭐⭐ WHY THIS IS ON SCREEN. Julien's D405 produced a clean photograph at 640x480 and at 1280x720, and diagonal coloured bands at **848x480**, on 2026-08-13. Smooth diagonal banding with cycling colour is the signature of **16-bit data being read as 8-bit BGR triplets**: 848 pixels of 16-bit is 1696 bytes a row, read as 848x3 it wants 2544, and the mismatch shears every row sideways while the byte pairs cycle through blue, green and red.

    ⚠️ The depth question that inference raised was later SETTLED by pixel statistics: every D405 mode over UVC on macOS is an ordinary colour photograph (FINDINGS §63.0). The format display stays because "what is the camera actually delivering" is a question every capture session should be able to answer.
    """
    if code <= 0:
        return "?"
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)).strip() or "?"


class FrameGrabber:
    """Reads the camera in a background thread and keeps only the newest frame.

    ⛔⭐ THE BUG THIS REPLACES, because it is a good lesson and it was mine.

    The previous version tried to avoid stale frames by "draining the queue":

        cap.grab()                    # cheap, no decode
        for _ in range(4):            # drain whatever else is waiting
            if not cap.grab(): break
        ok, frame = cap.retrieve()    # decode only the newest

    That is correct on Linux/V4L2, where `grab()` returns immediately when no frame is waiting. **On macOS/AVFoundation `grab()` BLOCKS until the next frame arrives.** So the loop did not drain a queue — it *waited for five more frames*. At 30 fps that is 5 x 33 ms = 167 ms per displayed frame, i.e. **6 fps**. Julien measured 5.

    ⚠️ And the reason it survived: `--probe` measured with `cap.read()` while the viewer used the drain loop, so the probe reported a healthy 30 fps for code the viewer never ran. **A measurement that does not exercise the real path measures nothing.** `--measure` now runs through this exact class.

    The right way to get "newest frame, no waiting" on a blocking backend is to move the blocking somewhere it does not matter: a thread reads continuously at the camera's own rate, and each new frame simply overwrites the last. The control or display loop then takes whatever is currently there and never blocks, so it sees the most recent frame captured and old frames are dropped by being overwritten rather than by being read and thrown away.
    """

    def __init__(self, cap):  # noqa: ANN001
        self._cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._stamp_ns = 0
        self._running = True
        self._captured = 0
        self._t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._seq += 1
                self._captured += 1
                # ⭐ Stamped at STORE time, under the same lock as the frame it stamps —
                # the closest observable to the exposure this backend offers, and the
                # host half of LaRobot's dual-timestamp record (yam.cameras.frame).
                self._stamp_ns = time.monotonic_ns()

    def newest(self):  # noqa: ANN201
        """The most recent frame and its sequence number. Never blocks."""
        with self._lock:
            return self._frame, self._seq

    def newest_stamped(self):  # noqa: ANN201
        """`(frame, seq, host_stamp_ns)` — the stamp taken when the frame was stored.

        ⚠️ A separate method rather than a change to `newest()`, on purpose: every
        hardware-confirmed call site in `apps/camera_view.py` unpacks a 2-tuple, and
        widening a confirmed API is how a working viewer breaks in a capture change.
        """
        with self._lock:
            return self._frame, self._seq, self._stamp_ns

    def capture_fps(self) -> float:
        dt = time.perf_counter() - self._t0
        return self._captured / dt if dt > 0 else 0.0

    def pixel_format(self) -> str:
        """The four-letter format the camera is delivering, or "?" if it will not say."""
        import cv2  # noqa: PLC0415 — lazy, so fakes and tests never touch OpenCV

        try:
            return fourcc_name(int(self._cap.get(cv2.CAP_PROP_FOURCC)))
        except Exception:  # noqa: BLE001
            return "?"

    def stop(self) -> None:
        """⚠️ Always call this. A daemon thread holding the camera open keeps the device busy for the next process, and Julien asked specifically that every test be quittable."""
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()
