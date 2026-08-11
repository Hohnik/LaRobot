#!/usr/bin/env python3
"""Tests for the terminal renderer's geometry. No camera, no window, no hardware.

    uv run scripts/test_camera_render.py

⛔ WHY THESE EXIST. The agent **cannot** test the camera at all — macOS grants camera
access per application, and the permission Julien granted covers his terminal, not
the process the agent's shell runs under. Every agent-side capture returns
`not authorized to capture video` regardless of the code.

So the only defence against shipping another broken viewer is to make the parts that
*can* be tested pure functions of an image, and test those. The stretch bug Julien
caught in a screenshot — `render_ansi` squashing a 16:9 frame into whatever shape the
terminal happened to be — was exactly such a part, and would have been caught here.
"""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

import camera_view as C  # noqa: E402


class FakeTerminal:
    """Pretend the terminal is a given size, for the duration of a block."""

    def __init__(self, cols: int, rows: int):
        self.size = os.terminal_size((cols, rows))

    def __enter__(self):  # noqa: ANN204
        self._real = shutil.get_terminal_size
        shutil.get_terminal_size = lambda fallback=(80, 24): self.size
        return self

    def __exit__(self, *exc: object) -> None:
        shutil.get_terminal_size = self._real


def displayed_aspect(cols: int, rows: int) -> float:
    """What the grid actually looks like on screen.

    A monospace cell is ~2x taller than wide, and the half-block trick stacks two
    pixels per cell — so one rendered pixel is roughly square and the picture's
    on-screen aspect is `cols / (2 * rows)`.
    """
    return cols / (2 * rows)


def test_the_picture_keeps_its_aspect_ratio() -> None:
    """⭐ THE REGRESSION Julien caught in a screenshot: 16:9 came out stretched."""
    for term in ((80, 40), (200, 50), (60, 60), (120, 24)):
        for aspect in (16 / 9, 4 / 3, 1.0):
            with FakeTerminal(*term):
                cols, rows = C.terminal_grid(aspect)
            got = displayed_aspect(cols, rows)
            assert abs(got - aspect) / aspect < 0.10, (
                f"terminal {term}, source aspect {aspect:.2f}: grid {cols}x{rows} "
                f"displays as {got:.2f} — that is a {abs(got - aspect) / aspect:.0%} stretch"
            )


def test_the_grid_always_fits_inside_the_terminal() -> None:
    """Overflowing wraps every line and destroys the picture entirely."""
    for term in ((80, 40), (200, 50), (40, 12), (300, 100)):
        for aspect in (16 / 9, 4 / 3, 0.5):
            with FakeTerminal(*term):
                cols, rows = C.terminal_grid(aspect)
            assert cols <= term[0], f"{cols} columns in a {term[0]}-column terminal"
            assert rows <= term[1], f"{rows} rows in a {term[1]}-row terminal"


def test_scale_shrinks_the_view_but_keeps_its_shape() -> None:
    """Julien asked for "a small view that is just accurate to the aspect ratio"."""
    with FakeTerminal(160, 50):
        big = C.terminal_grid(16 / 9, scale=1.0)
        small = C.terminal_grid(16 / 9, scale=0.4)
    assert small[0] < big[0] and small[1] < big[1], (small, big)
    assert abs(displayed_aspect(*small) - 16 / 9) / (16 / 9) < 0.10


def test_scale_is_clamped_to_something_usable() -> None:
    with FakeTerminal(80, 40):
        tiny = C.terminal_grid(16 / 9, scale=0.01)
    assert tiny[0] >= 20 and tiny[1] >= 6, f"scale 0.01 produced an unusable {tiny}"


def test_render_fills_exactly_the_grid_it_was_given() -> None:
    img = np.zeros((240, 320, 3), np.uint8)
    img[:, :] = (10, 200, 30)
    out = C.render_ansi(img, 40, 12)
    lines = [ln for ln in out.split("\n") if ln]
    assert len(lines) == 12, f"asked for 12 rows, got {len(lines)}"
    assert lines[0].count(C.UPPER_HALF) == 40, f"asked for 40 columns, got {lines[0].count(C.UPPER_HALF)}"


def test_render_reproduces_the_colours_it_was_given() -> None:
    """A uniform image must render as that colour, not something averaged wrong."""
    img = np.zeros((100, 100, 3), np.uint8)
    img[:, :] = (0, 0, 255)                       # BGR red
    out = C.render_ansi(img, 10, 5)
    assert "38;2;255;0;0" in out, "pure red did not survive the BGR->RGB conversion"


def test_render_emits_a_colour_code_only_when_the_colour_changes() -> None:
    """The run-length optimisation, without which a full frame is ~200 KB."""
    flat = np.zeros((100, 100, 3), np.uint8)
    flat[:, :] = (40, 90, 160)
    noisy = np.random.default_rng(0).integers(0, 255, (100, 100, 3), dtype=np.uint8)
    assert len(C.render_ansi(flat, 60, 20)) < len(C.render_ansi(noisy, 60, 20)) / 5, (
        "a flat image should compress to a small fraction of a noisy one"
    )


def test_every_offered_size_is_a_real_camera_mode() -> None:
    """⚠️ A UVC camera silently substitutes the nearest supported mode for anything
    it does not have. Julien saw 424x240 become 640x360. These are all genuine C920
    modes, so what is asked for is what arrives."""
    c920_modes = {
        (160, 90), (160, 120), (176, 144), (320, 180), (320, 240), (352, 288),
        (432, 240), (640, 360), (640, 480), (800, 448), (800, 600), (864, 480),
        (960, 540), (1024, 576), (1280, 720), (1600, 896), (1920, 1080),
    }
    for size in C.SIZES:
        assert tuple(size) in c920_modes, f"{size} is not a C920 mode; it will be substituted"


def test_the_size_list_spans_small_to_large() -> None:
    """Small sizes exist for latency, large for picture quality."""
    assert min(h for _, h in C.SIZES) <= 240, "no low-latency option offered"
    assert max(h for _, h in C.SIZES) >= 720, "no high-quality option offered"
    assert C.SIZES == sorted(C.SIZES), "keys 1-6 should step upward in size"


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
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




# ------------------------------------------------------ terminal detection ----


class FakeEnv:
    def __init__(self, **kw):
        self.kw = kw

    def __enter__(self):  # noqa: ANN204
        self._saved = {k: os.environ.get(k) for k in
                       ("TERM_PROGRAM", "TERM", "KITTY_WINDOW_ID")}
        for k in self._saved:
            os.environ.pop(k, None)
        os.environ.update({k: v for k, v in self.kw.items() if v is not None})
        return self

    def __exit__(self, *exc: object) -> None:
        for k, v in self._saved.items():
            os.environ.pop(k, None)
            if v is not None:
                os.environ[k] = v


def test_image_capable_terminals_are_detected() -> None:
    for prog, expect in (("iTerm.app", "iterm"), ("WezTerm", "iterm"),
                         ("vscode", "iterm"), ("WarpTerminal", "iterm")):
        with FakeEnv(TERM_PROGRAM=prog):
            mode, why = C.detect_term_mode()
        assert mode == expect, f"{prog} -> {mode}, expected {expect}"
        assert prog.lower() in why.lower()


def test_kitty_and_ghostty_are_detected() -> None:
    with FakeEnv(TERM="xterm-kitty"):
        assert C.detect_term_mode()[0] == "kitty"
    with FakeEnv(KITTY_WINDOW_ID="3"):
        assert C.detect_term_mode()[0] == "kitty"
    with FakeEnv(TERM="xterm-ghostty"):
        assert C.detect_term_mode()[0] == "kitty"


def test_a_terminal_without_images_says_so_rather_than_failing_silently() -> None:
    """⛔ The whole reason `b` looked broken: a silent fallback is indistinguishable
    from a broken feature."""
    with FakeEnv(TERM_PROGRAM="Apple_Terminal"):
        mode, why = C.detect_term_mode()
    assert mode == "blocks"
    assert "no image protocol" in why, why
    with FakeEnv():
        mode, why = C.detect_term_mode()
    assert mode == "blocks"
    assert "cannot be detected" in why and "--term-mode" in why


def test_the_draw_mode_key_cycles_rather_than_toggles() -> None:
    """⛔ THE BUG. `b` used to toggle between "blocks" and whatever was detected —
    so in an undetected terminal both sides were "blocks" and it did nothing. A
    three-way cycle can never be a no-op."""
    order = ["blocks", "iterm", "kitty"]
    seen = set()
    mode = "blocks"
    for _ in range(len(order)):
        mode = order[(order.index(mode) + 1) % len(order)]
        seen.add(mode)
    assert seen == set(order), f"cycling did not reach every mode: {seen}"


def test_both_image_renderers_produce_a_payload() -> None:
    img = np.zeros((90, 160, 3), np.uint8)
    img[:, :] = (30, 120, 200)
    iterm = C.render_iterm(img, 40, 12)
    kitty = C.render_kitty(img, 40, 12)
    assert iterm.startswith("\x1b]1337;File=inline=1") and iterm.endswith("\x07")
    assert "width=40" in iterm and "height=12" in iterm
    assert kitty.startswith("\x1b_G") and kitty.endswith("\x1b\\")
    assert "c=40,r=12" in kitty


def test_kitty_chunks_are_within_the_protocol_limit() -> None:
    """The protocol requires <=4096 base64 bytes per chunk, and m=1 on all but the
    last. A frame large enough to need several chunks is the case that breaks."""
    big = np.random.default_rng(1).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    out = C.render_kitty(big, 80, 24)
    chunks = [c for c in out.split("\x1b_G") if c]
    assert len(chunks) > 1, "a 720p frame should need multiple chunks"
    for chunk in chunks:
        payload = chunk.split(";", 1)[1].rsplit("\x1b\\", 1)[0]
        assert len(payload) <= 4096, f"chunk of {len(payload)} exceeds the 4096 limit"
    assert chunks[-1].split(";", 1)[0].endswith("m=0"), "the final chunk must set m=0"


if __name__ == "__main__":
    sys.exit(main())
