"""Terminal image encoding, geometry and adaptive display sizing.

This module owns no camera devices. It reads terminal geometry on request and
encodes supplied images. Historical tuning measurements are archived in
docs/archive/camera-source-notes.md; constants remain unchanged by extraction.
"""
from __future__ import annotations

import base64
import fcntl
import os
import shutil
import struct
import sys
import termios
from dataclasses import dataclass

import cv2
import numpy as np

UPPER_HALF = "\u2580"   # ▀ — foreground paints the top pixel, background the bottom


def render_ansi(frame, cols: int, rows: int) -> str:
    """Render a BGR frame as upper-half blocks with foreground/background colors."""
    small = cv2.resize(frame, (cols, rows * 2), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
    out: list[str] = []
    for y in range(rows):
        top, bot = rgb[2 * y], rgb[2 * y + 1]
        last_fg = last_bg = None
        for x in range(cols):
            fg = (int(top[x][0]), int(top[x][1]), int(top[x][2]))
            bg = (int(bot[x][0]), int(bot[x][1]), int(bot[x][2]))
            if fg != last_fg:
                out.append(f"\x1b[38;2;{fg[0]};{fg[1]};{fg[2]}m")
                last_fg = fg
            if bg != last_bg:
                out.append(f"\x1b[48;2;{bg[0]};{bg[1]};{bg[2]}m")
                last_bg = bg
            out.append(UPPER_HALF)
        out.append("\x1b[0m\n")
    return "".join(out)


@dataclass(frozen=True)
class CellSize:
    """Terminal cell dimensions in pixels, with an explicit measured/fallback marker."""

    width: float
    height: float
    measured: bool

    @property
    def aspect(self) -> float:
        """Height divided by width. ~2 for most fonts, but not exactly, and the
        difference is a visible stretch."""
        return self.height / self.width


# Used when the terminal will not say. A 2:1 cell is the usual shape and the value
# the geometry always silently assumed before it was measurable.
ASSUMED_CELL = CellSize(8.0, 16.0, measured=False)


def cell_size() -> CellSize:
    """Read cell geometry from terminal descriptors; return ASSUMED_CELL when unavailable."""
    for stream in (sys.stdout, sys.stdin, sys.stderr):
        try:
            buf = fcntl.ioctl(stream.fileno(), termios.TIOCGWINSZ, b"\0" * 8)
        except (OSError, ValueError, AttributeError):
            continue
        rows, cols, xpix, ypix = struct.unpack("HHHH", buf)
        if rows and cols and xpix and ypix:
            return CellSize(xpix / cols, ypix / rows, measured=True)
    return ASSUMED_CELL



IMAGE_WIDTH_CAP = {"kitty": 720, "iterm": 1280}


def auto_image_width(cols: int, capture_width: int, mode: str, cell: CellSize) -> int:
    """Choose an initial image width bounded by the terminal, capture and protocol cap."""
    box = int(cols * cell.width)
    return max(64, min(box, capture_width, IMAGE_WIDTH_CAP.get(mode, 720)))


def useful_image_width(cols: int, capture_width: int, cell: CellSize) -> int:
    """Maximum useful image width within both capture resolution and terminal pixels."""
    return max(64, min(int(cols * cell.width), capture_width))


# ⭐ How many consecutive over-budget frames it takes to shrink the image. Two, so a single
# hiccup cannot ratchet the picture down for the rest of the session.
SHRINK_AFTER = 2


def tune_image_width(width: float, draw_ms: float, target_ms: float, over: int,
                     ceiling: int, floor: float = 240.0) -> tuple[float, int]:
    """Adapt width to draw time. Two slow frames trigger shrinking; return width and slow-frame count."""
    if draw_ms > target_ms:
        over += 1
        if over >= SHRINK_AFTER:
            return max(floor, width * 0.93), 0
        return width, over
    if draw_ms < 0.85 * target_ms and width < ceiling:
        return min(float(ceiling), width * 1.06), 0
    return width, 0


def terminal_grid(frame_aspect: float, scale: float = 1.0, margin_rows: int = 4,
                  cell_aspect: float | None = None) -> tuple[int, int]:
    """Fit an aspect-preserving image grid within the terminal, reserving margin_rows for status."""
    k = cell_aspect if cell_aspect and cell_aspect > 0 else ASSUMED_CELL.aspect
    size = shutil.get_terminal_size(fallback=(100, 30))
    max_cols = max(20, int(size.columns * scale))
    max_rows = max(6, int((size.lines - margin_rows) * scale))
    rows = min(max_rows, int(max_cols / (k * frame_aspect)))
    rows = max(6, rows)
    cols = min(max_cols, int(k * rows * frame_aspect))
    return max(20, cols), rows



IMAGE_TERMINALS = {
    "iTerm.app": "iterm",
    "WezTerm": "iterm",
    "vscode": "iterm",        # VS Code implements iTerm2's inline-image escape
    "Hyper": "iterm",
    "Tabby": "iterm",
    "ghostty": "kitty",
    "kitty": "kitty",
    "rio": "kitty",
    "konsole": "kitty",
    "warp": "iterm",
    "WarpTerminal": "iterm",
}


def detect_term_mode() -> tuple[str, str]:
    """Select an image protocol from terminal environment markers and explain the fallback."""
    prog = os.environ.get("TERM_PROGRAM", "")
    term = os.environ.get("TERM", "")
    if os.environ.get("KITTY_WINDOW_ID") or term == "xterm-kitty":
        return "kitty", "kitty graphics protocol detected"
    if term.startswith("xterm-ghostty") or prog == "ghostty":
        return "kitty", "Ghostty detected (speaks the kitty graphics protocol)"
    for key, mode in IMAGE_TERMINALS.items():
        if prog and key.lower() in prog.lower():
            return mode, f"{prog} supports inline images ({mode} protocol)"
    if prog:
        return "blocks", f"{prog} reports no image protocol — coloured text only"
    return "blocks", ("no TERM_PROGRAM set, so image support cannot be detected — "
                      "coloured text only. Force with --term-mode iterm or kitty to try anyway")


def term_diagnosis() -> str:
    """Everything relevant about this terminal, for pasting into a conversation."""
    keys = ("TERM_PROGRAM", "TERM_PROGRAM_VERSION", "TERM", "COLORTERM",
            "KITTY_WINDOW_ID", "WEZTERM_PANE", "LC_TERMINAL")
    lines = ["terminal environment:"]
    for k in keys:
        lines.append(f"    {k:22s} {os.environ.get(k, '(unset)')}")
    mode, why = detect_term_mode()
    lines += ["", f"  -> best mode: {mode}", f"     because   : {why}", "",
              "  If your terminal DOES support images and was not detected, force it:",
              "      uv run apps/camera_view.py --term --term-mode iterm",
              "      uv run apps/camera_view.py --term --term-mode kitty",
              "  and tell the agent which one worked so the detection list can be fixed."]
    return "\n".join(lines)


def _downscale(frame, max_width: int):  # noqa: ANN001, ANN201
    """Downscale to max_width without enlarging small images or changing aspect ratio."""
    h, w = frame.shape[:2]
    if w <= max_width:
        return frame
    return cv2.resize(frame, (max_width, max(1, round(h * max_width / w))),
                      interpolation=cv2.INTER_AREA)


def render_kitty(frame, cols: int, rows: int, max_width: int = 480, quiet: bool = True,
                 image_id: int | None = None, previous_id: int | None = None) -> str:
    """Encode PNG chunks in the kitty protocol. Draw the replacement before deleting the previous image ID."""
    small = _downscale(frame, max_width)
    ok, buf = cv2.imencode(".png", small, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
    if not ok:
        return ""
    data = base64.b64encode(buf.tobytes()).decode("ascii")
    chunks = [data[i:i + 4096] for i in range(0, len(data), 4096)]
    q = "q=2," if quiet else ""
    out: list[str] = []
    if image_id is None:
        # One-shot use (--term-test): nothing follows, so clear first and be simple.
        out.append(f"\x1b_Ga=d,d=A,{q}".rstrip(",") + "\x1b\\")
    ident = f"i={image_id}," if image_id is not None else ""
    for i, chunk in enumerate(chunks):
        first, last = i == 0, i == len(chunks) - 1
        ctrl = f"a=T,f=100,{ident}c={cols},r={rows},{q}" if first else ""
        out.append(f"\x1b_G{ctrl}m={0 if last else 1};{chunk}\x1b\\")
    if image_id is not None and previous_id is not None:
        # ⭐ Delete the OLD image only after the new one is on screen. See the
        # docstring: deleting first is what makes the picture blink.
        out.append(f"\x1b_Ga=d,d=I,i={previous_id},{q}".rstrip(",") + "\x1b\\")
    return "".join(out)





def render_iterm(frame, cols: int, rows: int, max_width: int = 480, quality: int = 60) -> str:
    """Encode a JPEG image in the iTerm inline-image protocol, sized in terminal cells."""
    ok, buf = cv2.imencode(".jpg", _downscale(frame, max_width),
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return ""
    b64 = base64.b64encode(buf.tobytes()).decode("ascii")
    return (f"\x1b]1337;File=inline=1;width={cols};height={rows};"
            f"preserveAspectRatio=1;doNotMoveCursor=0:{b64}\x07")


