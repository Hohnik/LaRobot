#!/usr/bin/env python3
"""Tests for the depth-or-photograph analysis, `scripts/probe_camera_pixels.py`. No camera.

    uv run scripts/test_camera_pixels.py

⭐⭐ WHY THESE EXIST AND WHY THEY ARE SYNTHETIC. The tool they test can never be run by an
agent: macOS grants camera access per parent application, so Julien's terminal has it and an
agent's shell does not (docs/FINDINGS.md §61.3). **So the only way to know the analysis works
before handing it over is to feed it frames whose true nature is known by construction.**

⛔ It found a real blind spot on the first run. The tool guarded a division with
`if min(diffs) > 0 else None`, and a 16-bit-split frame — a smooth high byte beside a random
low byte — has a smoothest value of **exactly zero**. So the check returned None and the
verdict read "looks like an ordinary photograph" **for the very case it exists to catch**.

⚠️ A guard added to avoid a crash silently became a blind spot. Same shape as the placeholder
rule that disarmed `check_flags.py` (docs/FINDINGS.md §59.1), and both were invisible until
something deliberately wrong was fed in.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

from probe_camera_pixels import stats_for, verdict_for  # noqa: E402

RNG = np.random.default_rng(20260817)
H, W = 240, 424


def photograph() -> np.ndarray:
    """Three channels that differ but correlate, no exact zeros. An ordinary picture."""
    base = RNG.integers(40, 200, (H, W), dtype=np.uint8)
    return np.stack([np.clip(base.astype(int) + d, 1, 255).astype(np.uint8)
                     for d in (0, 12, -9)], axis=2)


def depth_with_holes(fraction: float = 0.2) -> np.ndarray:
    """Greyscale with a spike of exact zeros, which is what failed stereo matching gives."""
    d = RNG.integers(1, 255, (H, W), dtype=np.uint8)
    d[RNG.random((H, W)) < fraction] = 0
    return np.stack([d, d, d], axis=2)


def packed_16bit() -> np.ndarray:
    """A smooth high byte beside a random low byte, which is one 16-bit number split up."""
    hi = np.repeat(np.arange(H, dtype=np.uint8)[:, None], W, axis=1)
    lo = RNG.integers(0, 255, (H, W), dtype=np.uint8)
    return np.stack([hi, lo, hi], axis=2)


def said(frame: np.ndarray) -> str:
    return " ".join(verdict_for(stats_for(frame)))


# --------------------------------------------------------------- the three cases


def test_a_PHOTOGRAPH_is_called_a_photograph() -> None:
    """⚠️ The false-positive direction matters as much as the other. A tool that calls every
    frame depth-like would send Julien down a `sudo librealsense` path for nothing."""
    out = said(photograph())
    assert "ORDINARY PHOTOGRAPH" in out, out
    assert "DEPTH-LIKE" not in out


def test_DEPTH_WITH_HOLES_is_caught_by_its_zero_spike() -> None:
    """⭐ The strongest signal. Stereo matching returns 0 where it fails, and a photograph of
    a real room has almost no pure-black pixels."""
    out = said(depth_with_holes())
    assert "DEPTH-LIKE" in out, out
    # ⚠️ The MEASURED percentage, parsed rather than string-matched. The first version
    # asserted `"20." in out` for a frame built with a 0.2 probability, and the draw came out
    # at 19.8%. **A test that hardcodes the output of a random process is a flaky test**, and
    # the tool was right while the assertion was wrong.
    import re
    m = re.search(r"([\d.]+)% of pixels", out)
    assert m, f"it should quote the measured percentage: {out}"
    assert 15.0 < float(m.group(1)) < 25.0, f"quoted {m.group(1)}% for a ~20% frame"


def test_a_PACKED_16_BIT_frame_is_caught_even_when_one_channel_is_PERFECTLY_smooth() -> None:
    """⛔⭐⭐ THE BLIND SPOT. The first version reported this as an ordinary photograph,
    because a completely smooth channel made the denominator zero and the check was skipped.
    A smooth channel is the STRONGEST form of the signal, not a reason to give up."""
    out = said(packed_16bit())
    assert "PACKED-16-BIT-LIKE" in out, out
    assert "ORDINARY PHOTOGRAPH" not in out


def test_a_partly_smooth_channel_is_caught_by_the_RATIO() -> None:
    """⭐ The ordinary version of the same signal, where the high byte varies a little."""
    hi = np.repeat(np.arange(H, dtype=np.uint8)[:, None], W, axis=1)
    hi = (hi + (RNG.random((H, W)) < 0.02)).astype(np.uint8)   # a little noise
    lo = RNG.integers(0, 255, (H, W), dtype=np.uint8)
    out = said(np.stack([hi, lo, hi], axis=2))
    assert "PACKED-16-BIT-LIKE" in out, out


# ------------------------------------------------------------------- the numbers


def test_the_zero_fraction_counts_PIXELS_not_samples() -> None:
    """⛔ A black pixel needs all three channels at zero. Counting zero SAMPLES would call a
    photograph with a saturated blue sky "20% zero" because one channel bottomed out."""
    frame = photograph()
    frame[:, :, 0] = 0                     # one channel entirely zero
    assert stats_for(frame)["zero_fraction"] == 0.0
    assert "DEPTH-LIKE" not in said(frame)


def test_identical_channels_are_detected() -> None:
    st = stats_for(depth_with_holes(fraction=0.0))
    assert st["channels_identical"] is True
    assert stats_for(photograph())["channels_identical"] is False


def test_a_SINGLE_CHANNEL_frame_does_not_crash_and_is_flagged() -> None:
    """⚠️ OpenCV rarely returns one channel for a webcam, so if it does that is itself
    information — and the stats code must not index a third axis that is not there."""
    grey = RNG.integers(0, 255, (H, W), dtype=np.uint8)
    st = stats_for(grey)
    assert st["channels"] == 1
    assert "SINGLE CHANNEL" in " ".join(verdict_for(st))


def test_an_all_black_frame_reads_as_depth_like_rather_than_crashing() -> None:
    """⚠️ A camera that has not warmed up returns black. That must not divide by zero, and
    saying "depth-like" for it is acceptable: 100% zeros IS the signature, and the operator
    is told the percentage so an all-black frame is obvious."""
    out = said(np.zeros((H, W, 3), dtype=np.uint8))
    assert "DEPTH-LIKE" in out
    assert "100.0%" in out, out


def test_a_flat_grey_frame_does_not_claim_16_bit() -> None:
    """⛔ Every channel perfectly smooth is a lens cap, not a packed 16-bit number. The
    ratio test needs one channel ROUGH as well as one smooth, which is why it checks both
    ends rather than only the denominator."""
    flat = np.full((H, W, 3), 128, dtype=np.uint8)
    out = said(flat)
    assert "PACKED-16-BIT-LIKE" not in out, out


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except Exception as exc:  # noqa: BLE001
            failed.append(name)
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
