#!/usr/bin/env python3
"""Is the D405 giving us a photograph or packed DEPTH? Decides from the pixel data.

    uv run apps/probe_camera_pixels.py --index 0

⭐⭐ WHY THIS EXISTS, and why it is not `camera_view.py --probe`. docs/ROADMAP.md §8.2 item
16 has been open since docs/FINDINGS.md §8: **if what macOS hands us over plain UVC is the
D405's depth stream, then depth is available with no SDK at all**, which would be worth a
great deal. The obvious way to check is to ask the camera for its pixel format.

⛔ **That route is closed.** Julien ran `--probe` on 2026-08-17 and the codec column read
`ÿÿÿÿ` for every single mode: `CAP_PROP_FOURCC` returns **-1** through macOS's AVFoundation
backend, so the format is not readable and no amount of re-running changes it
(docs/FINDINGS.md §62.5).

⭐⭐ SO ASK THE PIXELS INSTEAD. A 16-bit depth frame has statistics no photograph has, and
this reports four of them per mode:

| signature | what it means |
|---|---|
| ⭐ **many EXACT zeros** | depth's strongest tell. Stereo matching fails on plain surfaces and returns 0, so a depth frame is full of holes. A photograph of a real room has almost no pure-black pixels |
| ⭐ **all three channels identical** | greyscale widened to three channels, which is what a depth or infrared frame looks like once OpenCV has made it BGR |
| ⭐ **one channel smooth, another noisy** | a 16-bit value split across two 8-bit channels. The high byte varies slowly across a surface and the low byte varies wildly |
| ⚠️ **plausible colour statistics** | three channels that differ from each other but correlate, with no zero spike — an ordinary photograph |

⛔⭐ IT ALSO TESTS 848x480 DIRECTLY, which is the mode item 16 is actually about and which
**`--probe` never swept** — its resolution list is a fixed set that does not include it.

⚠️⚠️ THIS IS INFERENCE FROM STATISTICS, NOT A FORMAT READ. It can say "this behaves like
depth" and it cannot say "this IS a 16-bit depth buffer". ⭐ What it CAN do is tell Julien
whether the question is worth pursuing with `librealsense`, which needs `sudo` on macOS
(docs/FINDINGS.md §28) and is the reason nobody has gone that way yet.

⛔⭐⭐ THE AGENT CANNOT RUN THIS. macOS grants camera access per parent application, so
Julien's terminal has it and an agent's shell does not, and no agent can grant it
(docs/FINDINGS.md §61.3). **Written to be handed over, not executed here.**

⭐ It saves every frame it captures as a `.npy` next to a JSON of the numbers, because on
2026-08-13 a measurement of this kind existed only as a paste into a chat window
(docs/FINDINGS.md §34.4).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

#: ⭐ The three sizes his camera actually delivers, plus **848x480**, which is what item 16
#: is about and which nothing has ever requested. ⚠️ Larger requests collapse to 1280x720 on
#: this machine, so asking for more is not informative.
SIZES = [(848, 480), (1280, 720), (640, 480), (424, 240)]

#: How many frames to throw away before measuring. ⚠️ The D405's first frames after a mode
#: change are not representative; `camera_view.py` waits out the same warm-up.
WARMUP = 8

#: A pixel is "exactly zero" only if every channel is zero. ⭐ Depth returns 0 for "no
#: reading", and that is the signature this leans on hardest.
ZERO_SPIKE_FRACTION = 0.02          # 2% of the frame being pure zero is already suspicious


def stats_for(frame: np.ndarray) -> dict:
    """Everything worth knowing about one captured frame, as plain numbers."""
    out: dict = {
        "shape": list(frame.shape),
        "dtype": str(frame.dtype),
        "min": int(frame.min()),
        "max": int(frame.max()),
        "mean": round(float(frame.mean()), 3),
    }
    if frame.ndim != 3 or frame.shape[2] < 3:
        # ⭐ A single-channel frame is itself a strong signal: OpenCV usually hands back BGR.
        out["channels"] = 1 if frame.ndim == 2 else int(frame.shape[2])
        out["zero_fraction"] = round(float((frame == 0).mean()), 5)
        return out

    b, g, r = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
    out["channels"] = int(frame.shape[2])
    # ⭐ Pure-zero PIXELS, not zero samples: a black pixel needs all three channels at zero.
    out["zero_fraction"] = round(float(np.all(frame[:, :, :3] == 0, axis=2).mean()), 5)
    out["channels_identical"] = bool(np.array_equal(b, g) and np.array_equal(g, r))
    out["per_channel"] = {}
    for name, ch in (("b", b), ("g", g), ("r", r)):
        # ⭐ The mean absolute difference between neighbouring pixels is the "noisiness"
        # measure that separates a high byte from a low byte. A smooth surface gives a tiny
        # value on the high byte and a large one on the low byte.
        horizontal = np.abs(np.diff(ch.astype(np.int16), axis=1)).mean()
        out["per_channel"][name] = {
            "mean": round(float(ch.mean()), 2),
            "std": round(float(ch.std()), 2),
            "neighbour_diff": round(float(horizontal), 3),
        }
    diffs = [out["per_channel"][c]["neighbour_diff"] for c in ("b", "g", "r")]
    # ⛔⭐⭐ A PERFECTLY SMOOTH CHANNEL IS THE STRONGEST FORM OF THIS SIGNAL, AND THE FIRST
    # VERSION THREW IT AWAY. It guarded the division with `if min(diffs) > 0 else None`, and
    # a synthetic 16-bit-split frame — a high byte constant along each row beside a random
    # low byte — has a smoothest value of **exactly 0**. So the check returned None and the
    # verdict came back "looks like an ordinary photograph" for the very case it exists to
    # catch.
    #
    # ⚠️ Found by feeding it three synthetic frames before it ever saw a camera. **A guard
    # added to avoid a crash silently became a blind spot**, which is the same shape as the
    # placeholder rule that disarmed `check_flags.py` (docs/FINDINGS.md §59.1).
    out["smoothest_neighbour_diff"] = round(min(diffs), 4)
    if min(diffs) <= 0.01 and max(diffs) >= 1.0:
        out["roughest_over_smoothest"] = float("inf")
    elif min(diffs) > 0:
        out["roughest_over_smoothest"] = round(max(diffs) / min(diffs), 2)
    else:
        out["roughest_over_smoothest"] = None
    return out


def verdict_for(st: dict) -> list[str]:
    """The reading those numbers support. ⚠️ Every line says what it is inferring from."""
    said = []
    zero = st.get("zero_fraction", 0.0)
    if zero >= ZERO_SPIKE_FRACTION:
        said.append(f"⭐ DEPTH-LIKE: {zero * 100:.1f}% of pixels are EXACTLY zero. Stereo "
                    f"matching returns 0 where it fails, and a photograph of a real room "
                    f"has almost no pure-black pixels.")
    if st.get("channels_identical"):
        said.append("⭐ DEPTH-OR-INFRARED-LIKE: all three channels are IDENTICAL, so this is "
                    "a greyscale image widened to BGR rather than a colour picture.")
    ratio = st.get("roughest_over_smoothest")
    if ratio == float("inf"):
        said.append(f"⭐⭐ PACKED-16-BIT-LIKE, strongly: one channel is COMPLETELY SMOOTH "
                    f"between neighbouring pixels while another averages "
                    f"{st['per_channel']['g']['neighbour_diff']:.1f} steps. A high byte "
                    f"beside its own low byte looks exactly like that.")
    elif ratio is not None and ratio >= 4.0:
        said.append(f"⭐ PACKED-16-BIT-LIKE: one channel is {ratio:.1f}x rougher than "
                    f"another between neighbouring pixels. That is what a high byte and a "
                    f"low byte of one 16-bit number look like side by side.")
    if st.get("channels") == 1:
        said.append("⭐ SINGLE CHANNEL, which OpenCV rarely returns for a webcam.")
    if not said:
        said.append("⚠️ LOOKS LIKE AN ORDINARY PHOTOGRAPH: three channels that differ, no "
                    "spike of exact zeros, no rough/smooth split. Nothing here suggests "
                    "depth data.")
    return said


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # ⚠️⭐ INDEX, NOT NAME, AND THAT IS DELIBERATE. `camera_view.find_camera_index` takes a
    # resolved `MacCamera` object and a list of the others, not a name string, so a `--camera
    # d405` path here would have to duplicate that whole resolution. ⛔ The first version
    # called it with a string inside a `try`, which meant `--camera` could never work and
    # said so only as a caught exception. **One honest flag beats a broken convenient one.**
    ap.add_argument("--index", type=int, default=0,
                    help="camera index (default 0). ⭐ Get it from "
                         "`uv run apps/camera_view.py --list`, which identifies each "
                         "index by MEASUREMENT — on 2026-08-17 the D405 was index 0")
    ap.add_argument("--out", default="recordings/camera_pixels",
                    help="where to write the frames and the numbers")
    args = ap.parse_args()

    import cv2

    index = args.index

    out_dir = REPO / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nPIXEL PROBE — camera index {index}. Reads frames only; nothing is energised.")
    print(f"Frames and numbers go to {args.out}/\n")

    results = {}
    for want_w, want_h in SIZES:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            print(f"  ⛔ {want_w}x{want_h}: could not open the camera.\n"
                  f"     On macOS the first run needs camera access granted to THIS "
                  f"terminal.")
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, want_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want_h)
        frame = None
        for _ in range(WARMUP):
            ok, f = cap.read()
            if ok and f is not None:
                frame = f
        cap.release()

        label = f"{want_w}x{want_h}"
        if frame is None:
            print(f"  ⛔ {label}: no frame came back.\n")
            continue

        st = stats_for(frame)
        got = f"{st['shape'][1]}x{st['shape'][0]}"
        st["requested"] = label
        st["delivered"] = got
        results[label] = st

        np.save(out_dir / f"frame_{label}.npy", frame)
        note = "" if got == label else f"  ⚠️ asked {label}, GOT {got}"
        print(f"  {label:>10} → {got:<10} {st['dtype']}, "
              f"{st['channels']} channel(s), zeros {st['zero_fraction'] * 100:5.2f}%{note}")
        for line in verdict_for(st):
            print(f"             {line}")
        print()

    (out_dir / "pixel_stats.json").write_text(json.dumps(results, indent=2) + "\n")

    print("""  ⚠️⚠️ HOW MUCH THIS CAN CLAIM.
     These are STATISTICS, not a format read. The tool can say "this behaves like depth"
     and it cannot say "this IS a 16-bit depth buffer" — OpenCV on macOS will not report
     the pixel format at all, which is why we are looking at the data instead.

  ⭐ WHAT TO DO WITH THE ANSWER.
     If several modes read DEPTH-LIKE, the question is worth pursuing with librealsense,
     which needs sudo on macOS (docs/FINDINGS.md §28) and is the reason nobody has gone
     that way yet. If everything reads like a photograph, ROADMAP item 16 should be closed
     as answered NO, and the D405 is a plain colour camera as far as this stack is
     concerned.

  ⭐ Every frame was saved as .npy beside pixel_stats.json, so the numbers outlive this
     terminal. On 2026-08-13 a measurement of this kind existed only as a chat paste.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
