"""Discover camera names and resolve OpenCV indices without using list order.

macOS mode probes distinguish models; identical cameras still require a physical
port/lens confirmation. Hints order the search and do not prove identity. Linux
resolution delegates to the kernel's by-id metadata in cameras.identity.

Historical measurements and discovery failures are recorded in docs/FINDINGS.md,
sections 21, 63, 67 and 70-76. No discovery or capture runs at module import.
"""
from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

MAX_PROBE_INDEX = 6
REPO_ROOT = Path(__file__).resolve().parents[3]

@dataclass(frozen=True)
class MacCamera:
    """Camera identity and advertised sizes/rates from macOS. Empty modes mean unknown."""

    name: str
    model_id: str
    unique_id: str
    modes: frozenset = frozenset()

    mode_rates: frozenset = frozenset()

    @property
    def usb(self) -> str | None:
        """`vid:pid` in hex, or None for a camera that is not USB.

        ⚠️ macOS writes the IDs in **decimal** inside the model string:
        `UVC Camera VendorID_32902 ProductID_2907` is `8086:0b5b`. Every datasheet,
        `ioreg` dump and USB tool speaks hex, so convert once here rather than
        leaving two number bases loose in the codebase.
        """
        vid = pid = None
        for token in self.model_id.split():
            if token.startswith("VendorID_"):
                vid = token.removeprefix("VendorID_")
            elif token.startswith("ProductID_"):
                pid = token.removeprefix("ProductID_")
        if not (vid and pid and vid.isdigit() and pid.isdigit()):
            return None
        return f"{int(vid):04x}:{int(pid):04x}"

    @property
    def short(self) -> str:
        """A name that fits on a status line without pushing the numbers off it."""
        tidy = " ".join(self.name.replace("(R)", "").replace("(TM)", "").split())
        return SHORT_NAMES.get(self.usb or "", tidy)


# Friendly names for the hardware actually on this rig. Keyed by USB id because
# that is stable — the marketing string is not (`HD Pro Webcam C920` is one of
# several names Logitech ships for the same camera).
SHORT_NAMES = {
    "8086:0b5b": "RealSense D405 (depth)",
    "046d:08e5": "C920 webcam",
}

_MAC_CAMERA_CACHE: list[MacCamera] | None = None


def _av_cameras() -> list[MacCamera]:
    """Enumerate AVFoundation devices and supported modes; return [] if bindings are absent."""
    try:
        import AVFoundation as AV  # noqa: PLC0415
        import CoreMedia as CM  # noqa: PLC0415
    except ImportError:
        return []
    cams = []
    for dev in AV.AVCaptureDevice.devicesWithMediaType_(AV.AVMediaTypeVideo) or []:
        rates: dict[tuple[int, int], float] = {}
        for fmt in dev.formats() or []:
            dims = CM.CMVideoFormatDescriptionGetDimensions(fmt.formatDescription())
            key = (int(dims.width), int(dims.height))
            best = max((float(r.maxFrameRate())
                        for r in fmt.videoSupportedFrameRateRanges() or []), default=0.0)
            # The same size can appear several times with different pixel formats;
            # keep the fastest, because that is what the mode can actually do.
            rates[key] = max(rates.get(key, 0.0), best)
        cams.append(MacCamera(str(dev.localizedName()), str(dev.modelID() or ""),
                              str(dev.uniqueID() or ""), frozenset(rates),
                              frozenset((w, h, f) for (w, h), f in rates.items())))
    return cams


def _profiler_cameras() -> list[MacCamera]:
    """Names only, from `system_profiler`. The fallback when AVFoundation is absent."""
    try:
        out = subprocess.run(["system_profiler", "-json", "SPCameraDataType"],
                             capture_output=True, text=True, timeout=30, check=False)
        return [MacCamera(e.get("_name", "?"), e.get("spcamera_model-id", ""),
                          e.get("spcamera_unique-id", ""))
                for e in json.loads(out.stdout).get("SPCameraDataType", [])]
    except (OSError, ValueError, subprocess.SubprocessError):
        return []          # not macOS, or the tool changed its output shape


def mac_cameras(refresh: bool = False) -> list[MacCamera]:
    """Return cached macOS device descriptions. Enumeration order never determines OpenCV indices."""
    global _MAC_CAMERA_CACHE  # noqa: PLW0603
    if _MAC_CAMERA_CACHE is None or refresh:
        _MAC_CAMERA_CACHE = _av_cameras() or _profiler_cameras()
    return _MAC_CAMERA_CACHE


@dataclass
class ProbeResult:
    """Observed index response. Width/height use a real frame when available; reported fps may be unreliable."""

    index: int
    ok: bool
    width: int
    height: int
    fps: float
    mean: float
    mono: bool | None
    settle: float = 0.0     # seconds until the camera produced a usable frame


def frame_is_mono(frame) -> bool | None:  # noqa: ANN001
    """Test channel equality; return None for missing or uniform frames. This does not establish depth or camera identity."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return None
    if float(frame.std()) < 1.0:
        return None      # uniform: black, blown out, or a lens cap. It cannot say.
    return bool(np.array_equal(frame[..., 0], frame[..., 1])
                and np.array_equal(frame[..., 1], frame[..., 2]))


def probe_indices(read_frames: bool = True, limit: int | None = None,
                  settle: float = 1.5) -> list[ProbeResult]:
    """Probe every index independently, waiting up to settle seconds for nonuniform pixels. Closed indices do not end the scan."""
    results: list[ProbeResult] = []
    for idx in range(limit if limit is not None else MAX_PROBE_INDEX):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame, waited = False, None, 0.0
        if read_frames:
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < settle:
                got, candidate = cap.read()
                if got and candidate is not None:
                    ok, frame = True, candidate
                    if float(candidate.std()) >= 1.0:
                        break        # real content, not a warm-up frame
            waited = time.perf_counter() - t0

        shape = frame.shape[:2] if ok and frame is not None else None
        results.append(ProbeResult(
            index=idx,
            ok=bool(ok),
            width=int(shape[1]) if shape else int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(shape[0]) if shape else int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fps=float(cap.get(cv2.CAP_PROP_FPS)),
            mean=float(np.mean(frame)) if ok and frame is not None else float("nan"),
            mono=frame_is_mono(frame) if ok else None,
            settle=waited,
        ))
        cap.release()
    return results


def discriminating_mode(cam: MacCamera, others: list[MacCamera]) -> tuple[int, int] | None:
    """Smallest mode unique to this device among the supplied others, or None for indistinguishable cameras."""
    shared: set = set()
    for other in others:
        shared |= set(other.modes)
    unique = set(cam.modes) - shared
    return min(unique, key=lambda wh: wh[0] * wh[1]) if unique else None


def model_discriminating_mode(cam: MacCamera, cams: list[MacCamera]) -> tuple[int, int] | None:
    """Smallest mode unique to this model. Same-model twins may share it; this cannot detect swapped twins."""
    same_model = {c.unique_id for c in cams
                  if (c.usb or c.name) == (cam.usb or cam.name)}
    foreign: set = set()
    for other in cams:
        if other.unique_id not in same_model:
            foreign |= set(other.modes)
    unique = set(cam.modes) - foreign
    return min(unique, key=lambda wh: wh[0] * wh[1]) if unique else None


def identify_indices(cams: list[MacCamera], limit: int | None = None,
                     ) -> tuple[list[tuple[int, MacCamera | None]], list[str]]:
    """Match index configuration responses to unique modes; report ambiguity rather than assigning by enumeration order."""
    notes: list[str] = []
    if not cams:
        notes.append("⚠️  macOS reported no cameras at all — nothing to identify against.")
        return [], notes

    questions: dict[str, tuple[int, int]] = {}
    for cam in cams:
        mode = discriminating_mode(cam, [c for c in cams if c.unique_id != cam.unique_id])
        if mode is None:
            notes.append(f"⚠️  {cam.short} shares every mode with another camera, so it "
                         "cannot be identified by measurement. Tell it apart by covering "
                         "it and watching which index goes dark.")
        else:
            questions[cam.unique_id] = mode
    if not questions:
        notes.append("⛔ no camera has a mode of its own — identification is impossible "
                     "here. Use --index and confirm by looking at the picture.")
        return [], notes

    by_uid = {c.unique_id: c for c in cams}
    found: list[tuple[int, MacCamera | None]] = []
    claimed: dict[str, int] = {}
    for idx in range(limit if limit is not None else len(cams) + 1):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        hits = []
        for uid, (want_w, want_h) in questions.items():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
            got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                   int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if got == (want_w, want_h):
                hits.append(uid)
        cap.release()

        if len(hits) == 1 and hits[0] not in claimed:
            cam = by_uid[hits[0]]
            w, h = questions[hits[0]]
            claimed[hits[0]] = idx
            found.append((idx, cam))
            notes.append(f"✅ index {idx} delivered {w}x{h}, which only {cam.short} offers.")
        elif len(hits) > 1:
            found.append((idx, None))
            notes.append(f"⛔ index {idx} matched {len(hits)} cameras at once "
                         f"({', '.join(by_uid[u].short for u in hits)}) — ambiguous, so it "
                         "is left unnamed.")
        elif hits:
            found.append((idx, None))
            notes.append(f"⛔ index {idx} claims to be {by_uid[hits[0]].short}, which index "
                         f"{claimed[hits[0]]} already answered for. Two indices cannot be "
                         "one camera — both are left unnamed.")
        else:
            found.append((idx, None))
            notes.append(f"⚠️  index {idx} matched no camera's own mode — unidentified.")

    missing = [c.short for c in cams if c.unique_id not in claimed]
    if missing:
        notes.append(f"⚠️  never found on any index: {', '.join(missing)}. A camera macOS "
                     "lists but OpenCV cannot open is normal for Continuity when the phone "
                     "is asleep.")
    return found, notes


class CameraLookupError(Exception):
    """Raised when `--camera` cannot be resolved to exactly one index."""


HINT_FILE = REPO_ROOT / "config" / "camera_index_hint.json"


def _load_hints() -> dict:
    try:
        return json.loads(HINT_FILE.read_text())
    except (OSError, ValueError):
        return {}


def _save_hint(unique_id: str, index: int) -> None:
    hints = _load_hints()
    if hints.get(unique_id) == index:
        return
    hints[unique_id] = index
    try:
        HINT_FILE.parent.mkdir(parents=True, exist_ok=True)
        HINT_FILE.write_text(json.dumps(hints, indent=2) + "\n")
    except OSError:
        pass          # a hint that cannot be written just makes the next run slower


def hinted_index(unique_id: str) -> int | None:
    """Remembered index for a uniqueID. A hint requires verification or physical confirmation for identical cameras."""
    idx = _load_hints().get(unique_id)
    return int(idx) if isinstance(idx, int) else None


def find_camera_index(cam: MacCamera, others: list[MacCamera],
                      limit: int = MAX_PROBE_INDEX) -> tuple[int | None, list[str], Any | None]:
    """Probe one camera, trying its hint first. Return (index, notes, open capture); the caller owns the returned capture. Refuse indistinguishable devices."""
    notes: list[str] = []
    mode = discriminating_mode(cam, others)
    if mode is None:
        return None, [f"⛔ {cam.short} shares every capture mode with another camera, so "
                      "it cannot be identified by measurement. Use --index, and confirm "
                      "by covering one of them."], None
    want_w, want_h = mode
    hint = _load_hints().get(cam.unique_id)
    order = ([hint] if hint is not None else []) + [i for i in range(limit) if i != hint]

    started = time.perf_counter()
    for idx in order:
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got == (want_w, want_h):
            took = time.perf_counter() - started
            where = ("where it was last time" if idx == hint
                     else f"which only it offers (the hint said {hint})" if hint is not None
                     else "which only it offers")
            notes.append(f"✅ {cam.short} is index {idx} — it answered {want_w}x{want_h}, "
                         f"{where}. {took:.1f}s.")
            _save_hint(cam.unique_id, idx)

            return idx, notes, cap
        cap.release()
    notes.append(f"⛔ no index delivered {want_w}x{want_h}, so {cam.short} is not "
                 "openable right now. Continuity cameras drop off when the phone sleeps.")
    return None, notes, None


# Words worth accepting that do not appear in the macOS name string. `d405` is the
# obvious one: the device calls itself "Depth Camera 405".
ALIASES = {"d405": "405", "realsense": "realsense", "intel": "realsense",
           "c920": "c920", "logitech": "c920", "webcam": "webcam",
           "iphone": "iphone", "builtin": "macbook", "internal": "macbook"}


def _normalise(text: str) -> str:
    return "".join(ch for ch in text.lower() if ch.isalnum())


def resolve_camera(spec: str, cams: list[MacCamera] | None = None,
                   identified: list[tuple[int, MacCamera | None]] | None = None,
                   ) -> tuple[int, MacCamera | None, Any | None]:
    """Resolve a numeric index or unambiguous name. Return (index, description, optional open capture), owned by the caller. Injected descriptions avoid device enumeration."""

    from yam.platform import IS_LINUX, read_v4l_cameras  # noqa: PLC0415

    if IS_LINUX and cams is None and identified is None:
        from yam.cameras.identity import linux_camera_for_spec  # noqa: PLC0415

        try:
            found = linux_camera_for_spec(spec, read_v4l_cameras())
        except ValueError as e:
            raise CameraLookupError(str(e)) from e
        return found.index, MacCamera(found.model, "", found.by_id), None

    cams = mac_cameras() if cams is None else cams
    if not cams:
        raise CameraLookupError(
            "macOS reported no cameras, so --camera cannot be resolved.\n"
            "  Use --index N instead, and --list to see what is there.")

    if spec.isdigit():
        idx = int(spec)
        if identified is None:
            identified, _ = identify_indices(cams)
        for got_idx, cam in identified:
            if got_idx == idx:
                return idx, cam, None
        return idx, None, None

    want = _normalise(spec)
    want = _normalise(ALIASES.get(want, want)) if want in ALIASES else want
    matches = [c for c in cams
               if want in _normalise(c.name) or want in _normalise(c.usb or "")]
    if not matches:
        listing = "\n".join(f"    {c.short}" for c in cams)
        raise CameraLookupError(f"no camera matches {spec!r}. macOS lists:\n{listing}")
    if len(matches) > 1:
        listing = "\n".join(f"    {c.short}" for c in matches)
        raise CameraLookupError(f"{spec!r} matches more than one camera:\n{listing}\n"
                                "  Be more specific, or use --index.")

    cam = matches[0]
    # ⭐ When the tests inject an identification, honour it. Otherwise ask about THIS
    # camera only — see find_camera_index for why the full scan was the wrong tool
    # for a question about one device.
    if identified is not None:
        for idx, got in identified:
            if got is not None and got.unique_id == cam.unique_id:
                return idx, cam, None
        raise CameraLookupError(
            f"macOS lists {cam.short}, but no index answered for it.\n"
            "  Run --list to see what each index actually shows, then use --index N.")

    others = [c for c in cams if c.unique_id != cam.unique_id]
    idx, notes, cap = find_camera_index(cam, others)
    for note in notes:
        print(f"  {note}")
    if idx is None:
        raise CameraLookupError(
            f"could not find {cam.short} on any camera index.\n"
            "  Run --list to see what each index actually shows, then use --index N.")
    return idx, cam, cap


