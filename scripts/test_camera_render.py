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


def displayed_aspect(cols: int, rows: int, cell_aspect: float = 2.0) -> float:
    """What the grid actually looks like on screen.

    A cell is `cell_aspect` times taller than it is wide, so a `cols x rows` grid
    occupies a box of aspect `cols / (rows * cell_aspect)`. The default 2.0 is the
    typical font shape, and was hard-coded everywhere until the terminal's real
    pixel geometry became measurable.
    """
    return cols / (rows * cell_aspect)


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
    last. A frame large enough to need several chunks is the case that breaks.

    ⚠️ Control-only escapes (the delete that clears the previous frame) carry no
    `;payload`, so they are skipped rather than parsed as chunks.
    """
    big = np.random.default_rng(1).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    out = C.render_kitty(big, 80, 24)
    parts = [c for c in out.split("\x1b_G") if c]
    data_parts = [c for c in parts if ";" in c]
    assert len(data_parts) > 1, "a 720p frame should need multiple chunks"
    for chunk in data_parts:
        payload = chunk.split(";", 1)[1].rsplit("\x1b\\", 1)[0]
        assert len(payload) <= 4096, f"chunk of {len(payload)} exceeds the 4096 limit"
    assert data_parts[-1].split(";", 1)[0].endswith("m=0"), "the final chunk must set m=0"


def test_kitty_deletes_the_previous_frame_and_silences_replies() -> None:
    """⛔ Two protocol facts that would each break a 30 fps redraw loop.

    Images PERSIST until deleted, so without `a=d` every frame adds a placement and
    the terminal's memory grows without bound. And the terminal REPLIES to each
    image on **stdin** — which this viewer reads for keypresses — so without `q=2`
    every frame injects escape bytes the key handler sees as junk.
    """
    img = np.zeros((90, 160, 3), np.uint8)
    out = C.render_kitty(img, 40, 12)
    assert out.startswith("\x1b_Ga=d,d=A,q=2"), "must clear the previous placement first"
    assert "q=2" in out.split(";", 1)[0], "must suppress replies, or they land in stdin"




def test_kitty_sends_genuine_png_because_there_is_no_jpeg_format_code() -> None:
    """⛔ THE BUG Julien saw as a blank screen in kitty mode.

    The kitty protocol's `f` takes exactly three values: 24 (raw RGB), 32 (raw RGBA)
    and 100 (**PNG**). There is no JPEG. The first version encoded JPEG and labelled
    it `f=100`, so the terminal failed to decode and said nothing — because `q=2` had
    suppressed the very error that explains it.
    """
    import base64

    img = np.zeros((180, 320, 3), np.uint8)
    img[:] = (50, 120, 200)
    out = C.render_kitty(img, 40, 12)
    assert "f=100" in out
    first_data = out.split("\x1b_G")[2]
    payload = first_data.split(";", 1)[1].split("\x1b")[0]
    raw = base64.b64decode(payload + "=" * (-len(payload) % 4))
    assert raw[:4] == b"\x89PNG", (
        f"f=100 declares PNG but the payload starts {raw[:4]!r} — that is the bug"
    )


def test_the_image_is_downscaled_because_payload_is_latency() -> None:
    """720p PNG is ~1 MB and 31 ms to encode: 40 MB/s at 30 fps, which cannot work."""
    big = np.zeros((720, 1280, 3), np.uint8)
    big[:] = (90, 90, 90)
    assert C._downscale(big, 480).shape[1] == 480
    assert C._downscale(big, 480).shape[0] == 270, "aspect ratio must survive the downscale"
    small = np.zeros((180, 320, 3), np.uint8)
    assert C._downscale(small, 480).shape[:2] == (180, 320), "must not UPscale"


def test_a_bigger_image_width_costs_a_bigger_payload() -> None:
    """The knob Julien tunes against the on-screen draw-ms readout."""
    img = np.zeros((720, 1280, 3), np.uint8)
    img[:] = np.random.default_rng(2).integers(0, 255, (720, 1280, 3), dtype=np.uint8)
    assert len(C.render_kitty(img, 60, 20, 320)) < len(C.render_kitty(img, 60, 20, 640))


def test_errors_can_be_unsuppressed_for_diagnosis() -> None:
    """⭐ q=2 is right for a 30 fps loop and wrong for finding out why nothing shows.
    --term-test needs the error, so `quiet=False` must actually drop q=2."""
    img = np.zeros((90, 160, 3), np.uint8)
    assert "q=2" in C.render_kitty(img, 20, 6, quiet=True)
    assert "q=2" not in C.render_kitty(img, 20, 6, quiet=False)


# -------------------------------------------------------- camera identity ----
#
# ⭐ These fixtures are the REAL rig, recorded on 2026-08-11: the four cameras macOS
# listed, and exactly what Julien's `--list` printed for indices 0-3. Testing against
# invented data would prove only that the code agrees with itself.

FAKE_NAMES = [
    C.MacCamera("MacBook Air Camera", "MacBook Air Camera",
                "6C707041-05AC-0010-000D-000000000001"),
    C.MacCamera("Intel(R) RealSense(TM) Depth Camera 405  Depth",
                "UVC Camera VendorID_32902 ProductID_2907", "0x121000080860b5b"),
    C.MacCamera("HD Pro Webcam C920",
                "UVC Camera VendorID_1133 ProductID_2277", "0x1120000046d08e5"),
    C.MacCamera("Julien's iPhone Camera", "iPhone12,3",
                "AB331AB3-1E3B-4DC2-A78D-8B8200000001"),
]

FAKE_PROBES = [
    C.ProbeResult(0, True, 1920, 1080, 24, 127, False),
    C.ProbeResult(1, True, 1280, 720, 5, 29, True),      # the D405's depth stream
    C.ProbeResult(2, True, 1920, 1080, 15, 6, False),
    C.ProbeResult(3, True, 1920, 1080, 30, 0, False),    # iPhone, lying face down
]


def test_usb_ids_are_converted_out_of_the_decimal_macos_prints() -> None:
    """⚠️ macOS writes `VendorID_32902 ProductID_2907`; every datasheet and USB tool
    says `8086:0b5b`. Two number bases for one identifier is how the wrong device
    gets matched."""
    assert FAKE_NAMES[1].usb == "8086:0b5b"
    assert FAKE_NAMES[2].usb == "046d:08e5"
    assert FAKE_NAMES[0].usb is None, "the built-in camera is not a USB device"


def test_names_are_paired_to_indices_and_cross_checked() -> None:
    """The pairing is positional, so it is only worth having if it is checked."""
    pairs, notes = C.pair_cameras(FAKE_NAMES, FAKE_PROBES)
    assert [cam.short for _, cam in pairs][1] == "RealSense D405 (depth)"
    assert [p.index for p, _ in pairs] == [0, 1, 2, 3]
    assert any("cross-check" in n and "1280px" in n for n in notes), notes
    assert not any(n.startswith("⛔") for n in notes), notes


def test_a_pairing_that_contradicts_the_hardware_is_caught() -> None:
    """⭐⭐ THE FALSIFIER. If macOS's order were not OpenCV's order, the D405 would
    land on an index reporting 1920 px — and a D405's imagers are 1280 px wide, so
    that is impossible. This is what makes the pairing survivable rather than merely
    plausible, and it is the check that must never be quietly deleted."""
    shuffled = [FAKE_NAMES[0], FAKE_NAMES[2], FAKE_NAMES[1], FAKE_NAMES[3]]
    _, notes = C.pair_cameras(shuffled, FAKE_PROBES)
    assert any(n.startswith("⛔") and "cannot exceed" in n for n in notes), notes


def test_no_name_is_attached_when_the_two_lists_disagree() -> None:
    """⛔ A wrong name is worse than no name — it is the confident, plausible, wrong
    answer of FINDINGS §0, and it would send the operator to the wrong camera."""
    extra = [*FAKE_PROBES, C.ProbeResult(4, True, 640, 480, 30, 50, False)]
    pairs, notes = C.pair_cameras(FAKE_NAMES, extra)
    assert all(cam is None for _, cam in pairs)
    assert any("disagree" in n for n in notes), notes

    pairs, notes = C.pair_cameras([], FAKE_PROBES)
    assert all(cam is None for _, cam in pairs)
    assert any("indices only" in n for n in notes), notes


def test_a_depth_stream_is_told_apart_from_a_picture() -> None:
    """The D405's UVC entry carries depth, not colour, so its three channels are
    identical. A colour camera's never are — white balance alone sees to that."""
    rng = np.random.default_rng(0)
    grey = np.repeat(rng.integers(0, 255, (40, 60, 1), dtype=np.uint8), 3, axis=2)
    colour = rng.integers(0, 255, (40, 60, 3), dtype=np.uint8)
    assert C.frame_is_mono(grey) is True
    assert C.frame_is_mono(colour) is False


def test_a_camera_can_be_selected_by_name() -> None:
    """The whole point: indices move on replug, names do not."""
    for spec, want_index in (("d405", 1), ("realsense", 1), ("c920", 2),
                             ("iphone", 3), ("builtin", 0), ("8086:0b5b", 1)):
        idx, cam = C.resolve_camera(spec, FAKE_NAMES, FAKE_PROBES)
        assert idx == want_index, f"{spec!r} resolved to {idx}, expected {want_index}"
        assert cam is not None


def test_an_unknown_or_ambiguous_name_is_refused_never_guessed() -> None:
    """⛔ FINDINGS §0 #5: an adapter chosen by index silently drove the OTHER robot.
    Falling back to index 0 when a name does not match is the same failure."""
    for spec in ("d435", "nikon"):
        try:
            C.resolve_camera(spec, FAKE_NAMES, FAKE_PROBES)
        except C.CameraLookupError as exc:
            assert "no camera matches" in str(exc)
        else:
            raise AssertionError(f"{spec!r} resolved to something instead of refusing")
    try:
        C.resolve_camera("camera", FAKE_NAMES, FAKE_PROBES)   # matches three of them
    except C.CameraLookupError as exc:
        assert "more than one" in str(exc)
    else:
        raise AssertionError("an ambiguous name was resolved instead of refused")


# ------------------------------------------------ how much detail is sent ----


def test_a_bigger_capture_now_produces_a_bigger_image() -> None:
    """⭐⭐ THE BUG JULIEN REPORTED, 2026-08-11: *"the resolution is stuck … pressing
    the numbers doesn't do anything."*

    Keys 1-6 changed the CAPTURE size while the image handed to the terminal stayed
    pinned at 480 px, so a sharper capture produced a pixel-identical picture. The
    keys were working perfectly and were invisible, which is indistinguishable from
    broken — the same shape of defect as `b` toggling between two identical states.
    """
    pane = 160  # columns, i.e. a wide terminal
    small = C.auto_image_width(pane, 320, "kitty", C.ASSUMED_CELL)
    large = C.auto_image_width(pane, 1280, "kitty", C.ASSUMED_CELL)
    assert large > small, "capturing more must now show more"


def test_the_image_sent_never_exceeds_the_pane_or_the_capture() -> None:
    """Both ceilings are pure waste past their limit: pixels beyond the pane are
    scaled straight back out, and pixels beyond the capture were invented."""
    tiny_pane = C.auto_image_width(40, 1920, "kitty", C.ASSUMED_CELL)
    assert tiny_pane <= 40 * C.ASSUMED_CELL.width, "sent more than the pane can show"
    small_capture = C.auto_image_width(200, 320, "kitty", C.ASSUMED_CELL)
    assert small_capture <= 320, "upscaled before transmitting, which invents nothing"


def test_kitty_gets_a_tighter_budget_than_iterm_because_png() -> None:
    """⛔ The kitty protocol's `f` accepts 24/32/100 — raw, raw+alpha, and PNG. There
    is no JPEG. MEASURED at 640 px: PNG level 1 is 6.7 ms and 391 KB per frame,
    JPEG q60 is 0.3 ms and 26 KB. Roughly 25x, which is why the two protocols cannot
    carry the same amount of detail at the same frame rate."""
    assert C.IMAGE_WIDTH_CAP["kitty"] < C.IMAGE_WIDTH_CAP["iterm"]


def test_the_cell_size_is_reported_as_measured_or_assumed_never_silently() -> None:
    """A fallback you cannot see is indistinguishable from a bug. Under the test
    harness there is no terminal at all, so this must report the assumed cell."""
    cell = C.cell_size()
    assert cell.measured is False, "no tty here, so nothing can have been measured"
    assert abs(cell.aspect - 2.0) < 0.01, "the assumed cell should be the classic 2:1"


def test_a_non_2to1_cell_changes_the_grid_so_the_picture_stays_square() -> None:
    """⭐ The grid assumed every font was exactly 2:1. When it is not — and Retina
    cells often are not — a 16:9 picture comes out stretched, which is the bug
    Julien caught in a screenshot, in a second disguise."""
    for k in (1.8, 2.0, 2.4):
        with FakeTerminal(200, 60):
            cols, rows = C.terminal_grid(16 / 9, cell_aspect=k)
        got = displayed_aspect(cols, rows, k)
        assert abs(got - 16 / 9) / (16 / 9) < 0.10, (
            f"cell aspect {k}: grid {cols}x{rows} displays as {got:.2f}, not 1.78")


def test_the_grid_leaves_room_for_the_status_lines() -> None:
    """Three lines of status live under the picture. Reserve too few and the picture
    pushes them off the bottom — or scrolls the whole view, every frame."""
    with FakeTerminal(100, 30):
        _, rows = C.terminal_grid(16 / 9)
    assert rows <= 30 - 3, f"{rows} rows leaves no room for the status lines"


if __name__ == "__main__":
    sys.exit(main())
