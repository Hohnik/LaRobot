#!/usr/bin/env python3
"""Put each paragraph on ONE line, so the renderer decides where lines break.

    uv run scripts/unwrap_markdown.py --check docs/*.md    # report, change nothing
    uv run scripts/unwrap_markdown.py docs/HANDOFF.md      # rewrite in place

⛔ WHY. Julien, 2026-08-12: *"your note taking style and your markdown writing style
does not need manual enters where they're not useful. Writing in a single line and then
letting the markdown renderer sensibly make its own enters is much better, because it
allows for variable sizings."*

He is right, and the reason is that a hard-wrapped paragraph is wrapped for **one**
window width. Every other width then reads badly: too narrow and each source line wraps
again, giving alternating long and short lines; too wide and the text sits in a thin
column with a wasted margin. On a phone, which is where he reads the Obsidian copy of
his other repo, hard wrapping is at its worst. A single line per paragraph has no
opinion about width, so every reader gets a sensible one.

⭐ THE SAFETY PROPERTY, AND IT IS WHY THIS IS A SCRIPT AND NOT AN AGENT. Unwrapping must
change **whitespace only**. So this tool compares the non-whitespace content of the file
before and after, character by character, and refuses to write if a single character
moved. An agent rewriting prose cannot make that promise, and this repo has ~250 KB of
documentation whose exact wording is the product it sells.

⚠️ WHAT IT DELIBERATELY DOES NOT TOUCH, because a newline is load-bearing in all of
these: fenced code blocks · indented (4-space) code blocks · table rows, where one row
per line IS the syntax · headings · horizontal rules · blank lines, which separate
paragraphs · lines ending in two spaces or a backslash, which are deliberate hard breaks
in Markdown. Blockquotes are unwrapped by recursion: strip the `> `, unwrap the inside,
put the `> ` back.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

FENCE = re.compile(r"^\s{0,3}(```+|~~~+)")
HEADING = re.compile(r"^\s{0,3}#{1,6}(\s|$)")
HR = re.compile(r"^\s{0,3}([-*_])[ \t]*(\1[ \t]*){2,}$")
TABLE = re.compile(r"^\s*\|")
LIST = re.compile(r"^(\s*)([-*+]|\d+[.)])(\s+)(.*)$")
QUOTE = re.compile(r"^(\s{0,3}>[ \t]?)(.*)$")
# ⚠️ Two trailing spaces, or a trailing backslash, are Markdown's hard line break. They
# are the one case where a newline inside a paragraph was asked for on purpose.
HARD_BREAK = re.compile(r"([ \t]{2,}|\\)$")


def _is_boundary(line: str) -> bool:
    """True if `line` can never be joined onto the paragraph above it."""
    return bool(
        not line.strip()
        or HEADING.match(line)
        or HR.match(line)
        or TABLE.match(line)
        or FENCE.match(line)
        or QUOTE.match(line)
        or LIST.match(line)
        or line.lstrip().startswith("<")
    )


def unwrap(text: str) -> str:
    """Join every wrapped paragraph onto one line. Whitespace-only change."""
    lines = text.split("\n")
    out: list[str] = []
    i, n = 0, len(lines)
    prev_blank = True          # start of file counts as "after a blank line"

    while i < n:
        line = lines[i]

        fence = FENCE.match(line)
        if fence:
            marker = fence.group(1)[:3]
            out.append(line)
            i += 1
            while i < n:
                out.append(lines[i])
                closing = FENCE.match(lines[i])
                i += 1
                if closing and closing.group(1).startswith(marker):
                    break
            prev_blank = False
            continue

        # ⚠️ An indented code block looks exactly like a wrapped paragraph, and the ONLY
        # thing that distinguishes them is that a code block follows a blank line while a
        # list's continuation follows the list item. ROADMAP step 1b is one of these, and
        # joining its two lines would have silently mangled a diagram.
        if prev_blank and line[:4] == "    " and line.strip():
            while i < n and (lines[i][:4] == "    " or not lines[i].strip()):
                out.append(lines[i])
                i += 1
            prev_blank = not out[-1].strip()
            continue

        if QUOTE.match(line):
            inner: list[str] = []
            while i < n and QUOTE.match(lines[i]):
                inner.append(QUOTE.match(lines[i]).group(2))  # type: ignore[union-attr]
                i += 1
            for quoted in unwrap("\n".join(inner)).split("\n"):
                out.append(f"> {quoted}" if quoted else ">")
            prev_blank = False
            continue

        # ⛔ THE ORDER HERE IS THE BUG THAT THE LIST TESTS CAUGHT. `_is_boundary` answers
        # "can this line be joined onto the paragraph ABOVE it", and a list item cannot —
        # so `LIST` belongs in it. But using the same predicate to dispatch the CURRENT
        # line then copied every list item out verbatim and never reached the branch that
        # absorbs its continuation. One predicate, two different questions.
        item = LIST.match(line)
        if item is None and _is_boundary(line):
            out.append(line)
            prev_blank = not line.strip()
            i += 1
            continue

        if item:
            indent, bullet, gap, rest = item.groups()
            prefix = f"{indent}{bullet}{gap}"
            raw = [rest]
        else:
            prefix = ""
            raw = [line]
        i += 1
        while i < n and not HARD_BREAK.search(raw[-1]):
            if _is_boundary(lines[i]):
                break
            raw.append(lines[i])
            i += 1

        joined = " ".join(part.strip() for part in raw if part.strip())
        out.append(prefix + joined if prefix else joined)
        prev_blank = False

    return "\n".join(out)


def content(text: str) -> str:
    """Everything except whitespace and blockquote markers. Two files with the same value
    say the same thing.

    ⚠️ THE `>` MARKERS HAVE TO COME OUT, and working out why took a real failure. Joining
    two quoted lines into one legitimately removes one `> ` prefix, because the prefix
    marks a *line* and there is now one fewer line. Counting it as content made the check
    reject a correct transformation — `docs/COMMANDS.md`, on the sentence about `--yes`
    being a dry run.

    A `>` inside a sentence ("a > b") is left alone, because only leading markers are
    stripped. The cost of this loosening is that dropping an entire quote level would not
    be caught here; the tests check quoting behaviour directly instead.
    """
    bare = [re.sub(r"^\s*(>\s*)+", "", line) for line in text.split("\n")]
    return re.sub(r"\s+", "", "".join(bare))


def process(path: Path, write: bool) -> tuple[bool, str]:
    """Returns (changed, note). Refuses to write if any non-whitespace moved."""
    before = path.read_text()
    after = unwrap(before)
    if content(before) != content(after):
        return False, "⛔ REFUSED — content changed, not just whitespace. Not written."
    if before == after:
        return False, "already one line per paragraph"
    saved = before.count("\n") - after.count("\n")
    if write:
        path.write_text(after)
    return True, f"{saved} line breaks removed" + ("" if write else " (dry run)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--check", action="store_true",
                    help="report what would change and write nothing")
    args = ap.parse_args()

    refused = 0
    for path in args.paths:
        if not path.is_file():
            print(f"  ?  {path} — not a file")
            continue
        changed, note = process(path, write=not args.check)
        if note.startswith("⛔"):
            refused += 1
        print(f"  {'✓' if changed else '·'}  {path}: {note}")
    return 1 if refused else 0


if __name__ == "__main__":
    sys.exit(main())
