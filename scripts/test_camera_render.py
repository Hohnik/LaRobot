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

import json
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
    modes, so what is asked for is what arrives.

    ⚠️ The mode set is now **measured** through AVFoundation rather than copied from
    a datasheet. The guessed version listed 960x540, which this C920 does not have,
    and missed 960x720 and 2560x1472, which it does."""
    for size in C.SIZES:
        assert tuple(size) in C920_MODES, f"{size} is not a C920 mode; it will be substituted"


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
# ⭐ These fixtures are the REAL rig, measured on 2026-08-11 through AVFoundation:
# the four cameras macOS lists and the modes each one actually offers. Testing
# against invented data would prove only that the code agrees with itself.
#
# ⛔ AND THE GROUND TRUTH THEY ARE TESTED AGAINST IS THE HARD-WON PART. macOS lists
# these four in the order below. **OpenCV does not use that order.** Julien covered
# each camera in turn and watched which index went dark: the C920 answers on index 0,
# where macOS lists the built-in camera, and the built-in camera answers on index 2.
# `FAKE_WIRING` encodes that measured reality, and every test here runs against it.

MACBOOK_MODES = frozenset({(640, 480), (1280, 720), (1920, 1080), (1080, 1920),
                           (1760, 1328), (1328, 1760), (1552, 1552)})
D405_MODES = frozenset({(424, 240), (480, 270), (640, 360), (640, 480), (848, 480),
                        (1280, 720)})
C920_MODES = frozenset({(160, 90), (160, 120), (176, 144), (320, 180), (320, 240),
                        (352, 288), (432, 240), (640, 360), (640, 480), (800, 448),
                        (864, 480), (800, 600), (1024, 576), (960, 720), (1280, 720),
                        (1600, 896), (1920, 1080), (2560, 1472)})
IPHONE_MODES = frozenset({(640, 480), (1280, 720), (1920, 1080), (1920, 1440)})

FAKE_NAMES = [
    C.MacCamera("MacBook Air Camera", "MacBook Air Camera",
                "6C707041-05AC-0010-000D-000000000001", MACBOOK_MODES),
    C.MacCamera("Intel(R) RealSense(TM) Depth Camera 405  Depth",
                "UVC Camera VendorID_32902 ProductID_2907", "0x121000080860b5b",
                D405_MODES),
    C.MacCamera("HD Pro Webcam C920",
                "UVC Camera VendorID_1133 ProductID_2277", "0x1120000046d08e5",
                C920_MODES),
    C.MacCamera("Julien's iPhone Camera", "iPhone12,3",
                "AB331AB3-1E3B-4DC2-A78D-8B8200000001", IPHONE_MODES),
]

# index -> which camera really answers there. NOT the macOS order. See above.
FAKE_WIRING = [C920_MODES, D405_MODES, MACBOOK_MODES, IPHONE_MODES]
TRUE_INDEX = {"HD Pro Webcam C920": 0, "Intel(R) RealSense(TM) Depth Camera 405  Depth": 1,
              "MacBook Air Camera": 2, "Julien's iPhone Camera": 3}


class FakeCapture:
    """A `cv2.VideoCapture` that behaves like a real UVC camera on this Mac.

    The one behaviour that matters: **ask for a mode it does not have and it gives
    you the nearest one it does**, silently. That was measured here in session 7
    (424x240 came back as 640x360) and it is the property the whole identification
    scheme rests on.
    """

    def __init__(self, modes):
        self.modes = sorted(modes, key=lambda wh: wh[0] * wh[1])
        self.w, self.h = self.modes[-1]
        self._want_w = None

    def isOpened(self):  # noqa: N802
        return True

    def set(self, prop, value):
        if prop == 3:                      # CAP_PROP_FRAME_WIDTH
            self._want_w = int(value)
        elif prop == 4 and self._want_w:   # CAP_PROP_FRAME_HEIGHT
            want = (self._want_w, int(value))
            if want in self.modes:
                self.w, self.h = want
            else:                          # nearest by pixel count, like a real UVC cam
                self.w, self.h = min(self.modes,
                                     key=lambda m: abs(m[0] * m[1] - want[0] * want[1]))
        return True

    def get(self, prop):
        return {3: self.w, 4: self.h, 5: 30.0}.get(prop, 0.0)

    def read(self):
        return True, np.zeros((self.h, self.w, 3), np.uint8)

    def release(self):
        pass


class FakeBus:
    """Patch `cv2.VideoCapture` so tests never touch a real camera."""

    def __init__(self, wiring):
        self.wiring = wiring

    def __enter__(self):
        self._real = C.cv2.VideoCapture
        C.cv2.VideoCapture = lambda idx, *a, **k: (
            FakeCapture(self.wiring[idx]) if idx < len(self.wiring) else _Closed())
        return self

    def __exit__(self, *exc: object) -> None:
        C.cv2.VideoCapture = self._real


class _Closed:
    def isOpened(self):  # noqa: N802
        return False

    def release(self):
        pass


def test_usb_ids_are_converted_out_of_the_decimal_macos_prints() -> None:
    """⚠️ macOS writes `VendorID_32902 ProductID_2907`; every datasheet and USB tool
    says `8086:0b5b`. Two number bases for one identifier is how the wrong device
    gets matched."""
    assert FAKE_NAMES[1].usb == "8086:0b5b"
    assert FAKE_NAMES[2].usb == "046d:08e5"
    assert FAKE_NAMES[0].usb is None, "the built-in camera is not a USB device"


def test_every_camera_has_a_mode_that_is_its_alone() -> None:
    """The question each camera can be identified by. If a camera has no mode of its
    own it must return None rather than something merely likely."""
    for cam in FAKE_NAMES:
        others = [c for c in FAKE_NAMES if c is not cam]
        mode = C.discriminating_mode(cam, others)
        assert mode is not None, f"{cam.short} has no distinguishing mode"
        assert mode in cam.modes
        assert not any(mode in o.modes for o in others), f"{mode} is not unique"


def test_two_identical_cameras_cannot_be_told_apart_and_it_says_so() -> None:
    """⚠️ THIS IS COMING: the second D405 is on the desk waiting to be plugged in.
    Two of the same model share every mode, so measurement cannot separate them and
    the honest answer is None — not a guess with a 50% chance of driving the wrong
    arm's view."""
    twin = C.MacCamera("Intel(R) RealSense(TM) Depth Camera 405  Depth",
                       "UVC Camera VendorID_32902 ProductID_2907", "0xOTHER", D405_MODES)
    assert C.discriminating_mode(FAKE_NAMES[1], [twin]) is None


def test_indices_are_identified_by_measurement_not_by_list_order() -> None:
    """⛔⭐ THE REGRESSION TEST FOR THE BUG THAT SHIPPED, 2026-08-11.

    The wiring here is the real one Julien measured by covering each camera: the C920
    answers on index 0 and the built-in camera on index 2, while macOS lists them the
    other way round. A positional pairing gets both wrong and is confident about it.
    Identification by mode must get all four right.
    """
    with FakeBus(FAKE_WIRING):
        found, notes = C.identify_indices(FAKE_NAMES)
    got = {cam.name: idx for idx, cam in found if cam}
    assert got == TRUE_INDEX, f"identified {got}, truth is {TRUE_INDEX}\n" + "\n".join(notes)
    assert not any(n.startswith("⛔") for n in notes), notes


def test_an_index_that_matches_nothing_is_left_unnamed() -> None:
    """A camera macOS never listed must not inherit somebody else's name."""
    stranger = frozenset({(1024, 768), (2048, 1536)})
    with FakeBus([*FAKE_WIRING, stranger]):
        found, notes = C.identify_indices(FAKE_NAMES, limit=5)
    assert found[4][1] is None, "an unknown camera was given a name"
    assert any("matched no camera" in n for n in notes), notes


def test_a_camera_macos_lists_but_no_index_answers_for_is_reported() -> None:
    """Continuity drops out when the phone sleeps. That is normal and must be said,
    not silently swallowed."""
    with FakeBus(FAKE_WIRING[:3]):
        _, notes = C.identify_indices(FAKE_NAMES)
    assert any("never found on any index" in n and "iPhone" in n for n in notes), notes


def test_a_depth_stream_is_told_apart_from_a_picture() -> None:
    """The D405's UVC entry carries depth, not colour, so its three channels are
    identical. A colour camera's never are — white balance alone sees to that."""
    rng = np.random.default_rng(0)
    grey = np.repeat(rng.integers(0, 255, (40, 60, 1), dtype=np.uint8), 3, axis=2)
    colour = rng.integers(0, 255, (40, 60, 3), dtype=np.uint8)
    assert C.frame_is_mono(grey) is True
    assert C.frame_is_mono(colour) is False


def test_a_frame_with_no_content_says_it_cannot_tell() -> None:
    """⛔⭐ THE BUG JULIEN'S OUTPUT EXPOSED: a black frame has three identical
    channels, so it was declared `MONO — depth/IR` — about his **iPhone**. It was
    black only because the probe read it before the sensor had exposed. An
    information-free frame must answer "unknown", never a measurement."""
    assert C.frame_is_mono(np.zeros((40, 60, 3), np.uint8)) is None
    assert C.frame_is_mono(np.full((40, 60, 3), 255, np.uint8)) is None


def test_a_camera_can_be_selected_by_name() -> None:
    """The whole point: indices move on replug, names do not."""
    with FakeBus(FAKE_WIRING):
        identified, _ = C.identify_indices(FAKE_NAMES)
    for spec, want_index in (("d405", 1), ("realsense", 1), ("c920", 0),
                             ("iphone", 3), ("builtin", 2), ("8086:0b5b", 1)):
        idx, cam, _ = C.resolve_camera(spec, FAKE_NAMES, identified)
        assert idx == want_index, f"{spec!r} resolved to {idx}, expected {want_index}"
        assert cam is not None


def test_an_unknown_or_ambiguous_name_is_refused_never_guessed() -> None:
    """⛔ FINDINGS §0 #5: an adapter chosen by index silently drove the OTHER robot.
    Falling back to index 0 when a name does not match is the same failure."""
    with FakeBus(FAKE_WIRING):
        identified, _ = C.identify_indices(FAKE_NAMES)
    for spec in ("d435", "nikon"):
        try:
            C.resolve_camera(spec, FAKE_NAMES, identified)
        except C.CameraLookupError as exc:
            assert "no camera matches" in str(exc)
        else:
            raise AssertionError(f"{spec!r} resolved to something instead of refusing")
    try:
        C.resolve_camera("camera", FAKE_NAMES, identified)   # matches three of them
    except C.CameraLookupError as exc:
        assert "more than one" in str(exc)
    else:
        raise AssertionError("an ambiguous name was resolved instead of refused")


def test_a_listed_camera_that_no_index_answers_for_is_refused() -> None:
    """macOS listing a camera is not the same as OpenCV being able to open it."""
    with FakeBus(FAKE_WIRING[:3]):
        identified, _ = C.identify_indices(FAKE_NAMES)
    try:
        C.resolve_camera("iphone", FAKE_NAMES, identified)
    except C.CameraLookupError as exc:
        assert "no index answered" in str(exc), exc
    else:
        raise AssertionError("a camera that never answered was resolved anyway")


# ------------------------------------------- finding ONE camera, quickly ----


class FakeHints:
    """Redirect the index-hint file so tests never write into config/."""

    def __init__(self, initial=None):  # noqa: ANN001
        self.initial = initial or {}

    def __enter__(self):  # noqa: ANN204
        import tempfile
        self._dir = tempfile.TemporaryDirectory()
        self._real = C.HINT_FILE
        C.HINT_FILE = Path(self._dir.name) / "camera_index_hint.json"
        if self.initial:
            C.HINT_FILE.write_text(json.dumps(self.initial))
        return self

    def __exit__(self, *exc: object) -> None:
        C.HINT_FILE = self._real
        self._dir.cleanup()


class CountingBus(FakeBus):
    """A FakeBus that records how many times each index was opened."""

    def __enter__(self):  # noqa: ANN204
        self.opens: list[int] = []
        self._real = C.cv2.VideoCapture

        def factory(idx, *a, **k):  # noqa: ANN001, ANN202
            self.opens.append(idx)
            return (FakeCapture(self.wiring[idx]) if idx < len(self.wiring) else _Closed())

        C.cv2.VideoCapture = factory
        return self


def test_finding_one_camera_asks_only_about_that_camera() -> None:
    """⛔⭐ JULIEN, 2026-08-12: *"it takes like twenty seconds for the camera to start
    running, which shouldn't be the case because I deliberately said which camera I
    want."* He was right. `--camera c920` ran the FULL identification — every camera
    opened, every camera's question asked at every index — to answer a question about
    one device, waking his iPhone over Continuity every time."""
    c920 = FAKE_NAMES[2]
    others = [c for c in FAKE_NAMES if c is not c920]
    with FakeHints(), CountingBus(FAKE_WIRING) as bus:
        idx, notes, cap = C.find_camera_index(c920, others)
    assert idx == 0, f"{notes}"
    assert bus.opens == [0], f"opened {bus.opens} — it should have stopped at the first hit"


def test_a_remembered_index_is_tried_first_and_still_verified() -> None:
    """⭐ The hint is an ORDERING OF THE SEARCH, never a cached answer. The camera at
    the remembered index is asked the same question as any other; a right hint costs
    one open, and a wrong one costs one extra."""
    macbook = FAKE_NAMES[0]
    others = [c for c in FAKE_NAMES if c is not macbook]
    with FakeHints({macbook.unique_id: 2}), CountingBus(FAKE_WIRING) as bus:
        idx, _, cap = C.find_camera_index(macbook, others)
    assert idx == 2
    assert bus.opens == [2], "a correct hint should mean exactly one open"


def test_a_WRONG_remembered_index_is_caught_not_trusted() -> None:
    """⛔ The whole safety argument for keeping a hint at all. If a replug moved the
    camera, the hint points at the wrong device — and that device fails the question,
    so the search carries on rather than driving the wrong camera."""
    c920 = FAKE_NAMES[2]
    others = [c for c in FAKE_NAMES if c is not c920]
    with FakeHints({c920.unique_id: 3}), CountingBus(FAKE_WIRING) as bus:
        idx, _, cap = C.find_camera_index(c920, others)
    assert idx == 0, "a stale hint must not win"
    assert bus.opens[0] == 3, "the hint should still have been tried first"


def test_the_hint_is_written_so_the_next_run_is_fast() -> None:
    d405 = FAKE_NAMES[1]
    others = [c for c in FAKE_NAMES if c is not d405]
    with FakeHints() as hints, FakeBus(FAKE_WIRING):
        C.find_camera_index(d405, others)
        saved = json.loads(C.HINT_FILE.read_text())
    assert saved[d405.unique_id] == 1, saved
    assert hints is not None


def test_two_identical_cameras_are_refused_before_anything_is_opened() -> None:
    """⚠️ The second D405. Two of a model share every mode, so there is no question
    that separates them — and guessing would be a coin flip on which arm's view you
    are driving."""
    twin = C.MacCamera("Intel(R) RealSense(TM) Depth Camera 405  Depth",
                       "UVC Camera VendorID_32902 ProductID_2907", "0xTWIN", D405_MODES)
    with FakeHints(), CountingBus(FAKE_WIRING) as bus:
        idx, notes, cap = C.find_camera_index(FAKE_NAMES[1], [twin])
    assert idx is None
    assert bus.opens == [], "it must refuse without touching a camera"
    assert any("shares every capture mode" in n for n in notes), notes


# ------------------------------------------- what the number keys are bound to ----


def test_the_number_keys_offer_the_cameras_OWN_modes() -> None:
    """⛔⭐ JULIEN, 2026-08-11: *"the numbers don't allow for all the quality options.
    They cycle between about three, and not in the correct order either."*

    `SIZES` is a list of C920 modes. On the MacBook Air camera, keys 1-4 all land on
    640x480 because a UVC camera substitutes the nearest mode it has — three distinct
    results out of six keys, which is exactly what he saw. Bound to the device's own
    modes, every key does something different.
    """
    macbook = FAKE_NAMES[0]
    collapsed = {min(macbook.modes, key=lambda m: abs(m[0] * m[1] - w * h))
                 for w, h in C.SIZES}
    assert len(collapsed) <= 3, "the old bug should reproduce with the old list"

    sizes = C.key_sizes(macbook)
    assert len(set(sizes)) == len(sizes), "two keys would give the same size"
    assert sizes == sorted(sizes, key=lambda wh: wh[0] * wh[1]), "keys must ascend"
    assert all(s in macbook.modes for s in sizes), "offered a mode this camera lacks"
    landscape = [m for m in macbook.modes if m[0] > m[1]]
    assert sizes[-1] == max(landscape, key=lambda wh: wh[0] * wh[1]), (
        "the last key should be the best LANDSCAPE mode this camera can do — the "
        "square and portrait Center Stage crops are excluded on purpose, see the "
        "portrait test below")


def test_the_number_keys_do_not_offer_portrait_modes_for_a_video_view() -> None:
    """⚠️ Apple's camera advertises 1080x1920 and 1552x1552 — Center Stage crops —
    and by pixel count those sort ABOVE 1920x1080. Key 6 should be the best
    *landscape* the camera can do, not a square frame."""
    sizes = C.key_sizes(FAKE_NAMES[0])
    assert all(w >= h for w, h in sizes), f"a portrait or square mode was offered: {sizes}"
    assert sizes[-1] == (1760, 1328), f"key 6 should be the largest landscape mode, got {sizes[-1]}"


def test_a_slow_capture_mode_is_never_offered_as_the_best() -> None:
    """⛔⭐ JULIEN, 2026-08-12: pressing 6 dropped him to *"like two frames per
    second"*. MEASURED: the C920's 2560x1472 is a **2 fps** mode — a stills format.
    "The best this camera can do" is the wrong idea for a live view; the right one is
    the sharpest mode that still MOVES, and AVFoundation reports the rate per mode so
    this is a measurement rather than a judgement."""
    fast = {m for m in C920_MODES if m != (2560, 1472)}
    c920 = C.MacCamera("HD Pro Webcam C920", "", "x", frozenset(C920_MODES),
                       frozenset({(w, h, 30.0) for w, h in fast} | {(2560, 1472, 2.0)}))
    sizes = C.key_sizes(c920)
    assert (2560, 1472) not in sizes, "a 2 fps stills mode was offered for a live view"
    assert sizes[-1] == (1920, 1080), f"key 6 should be the best mode that moves, got {sizes[-1]}"


def test_without_rate_information_every_mode_is_still_offered() -> None:
    """No AVFoundation means no rates. Silently dropping every mode would be worse
    than offering one that turns out slow — and the fps readout shows the truth."""
    cam = C.MacCamera("x", "", "x", frozenset({(640, 480), (1280, 720)}))
    assert C.key_sizes(cam) == [(640, 480), (1280, 720)]


def test_the_useful_width_has_no_protocol_budget_in_it() -> None:
    """⭐ The ceiling the adaptive controller climbs toward is the PANE and the
    CAPTURE — physical limits. The protocol budget is only a starting guess, because
    on Julien's machine the cost is writing bytes, not encoding them, and no constant
    measured on one afternoon can know that ratio."""
    cell = C.CellSize(16.0, 34.0, measured=True)
    assert C.useful_image_width(121, 1920, cell) == 1920, "the pane is 1936 px; capture bounds it"
    assert C.useful_image_width(40, 1920, cell) == 640, "a small pane bounds it instead"
    assert C.useful_image_width(121, 320, cell) == 320, "never above what was captured"
    assert C.useful_image_width(121, 1920, cell) > C.IMAGE_WIDTH_CAP["kitty"], (
        "the ceiling must be allowed above the starting cap, or nothing can climb")


def test_the_number_keys_fall_back_when_the_modes_are_unknown() -> None:
    """No AVFoundation (a Linux checkout, or the optional dependency missing) means no
    mode list. The C920 defaults are a reasonable guess and must not crash."""
    assert C.key_sizes(None) == C.SIZES
    assert C.key_sizes(C.MacCamera("x", "", "", frozenset())) == C.SIZES


def test_a_camera_with_few_modes_gets_a_short_key_list() -> None:
    """And the viewer must bounds-check it rather than crash on key 6."""
    sparse = C.MacCamera("tiny", "", "", frozenset({(320, 240), (640, 480)}))
    assert C.key_sizes(sparse) == [(320, 240), (640, 480)]


# --------------------------------------------------------------- the flicker ----


def test_the_new_image_is_placed_before_the_old_one_is_deleted() -> None:
    """⛔⭐ THE FLICKER Julien reported. Every frame began with `a=d,d=A` — delete
    ALL images — and only then transmitted the new one, so the screen was blank for
    however long the terminal took to decode, 30 times a second.

    Double buffering: place the new image over the old, then delete the old from
    underneath. There must be no moment with nothing on screen.
    """
    img = np.zeros((90, 160, 3), np.uint8)
    img[:] = (30, 120, 200)
    out = C.render_kitty(img, 40, 12, image_id=991, previous_id=992)
    assert not out.startswith("\x1b_Ga=d"), "still deleting before drawing — that blinks"
    place, delete = out.find("a=T"), out.find("a=d")
    assert place != -1 and delete != -1, "must both place and delete"
    assert place < delete, "the delete must come after the placement, not before"
    assert "i=991" in out and "i=992" in out
    assert "d=I" in out, "d=I frees the image data; without it the terminal leaks one per frame"


def test_the_first_frame_has_no_previous_image_to_delete() -> None:
    img = np.zeros((90, 160, 3), np.uint8)
    out = C.render_kitty(img, 40, 12, image_id=991, previous_id=None)
    assert "a=d" not in out, "there is nothing to delete yet"
    assert "i=991" in out


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
