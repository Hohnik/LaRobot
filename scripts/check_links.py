#!/usr/bin/env python3
"""Check every relative markdown link and `#anchor` in the docs. No hardware.

    uv run scripts/check_links.py

⭐ WHY THIS EXISTS. These documents are the only thing that makes the repo
recoverable by someone with no context, and they cross-reference each other
constantly — `FINDINGS §24.1`, `HANDOFF §5.5`, `ROADMAP step 6`. A pointer to a
section that no longer exists is worse than no pointer: it reads as authoritative
and sends the reader looking for something that was renamed three sessions ago.

It has already earned its place. On its first two runs it caught **two anchors this
agent had just written** — `#35-the-gripper-2-frame-fix` for a section actually
called *"3.5 ⭐ The gripper: two 2π frame errors, not a broken mechanism"*, and a
`§9` link whose slug had one hyphen too many. Both looked right in the source and
neither would have been noticed by reading.

⚠️ Anchors are slugged GitHub-style: lowercase, drop anything that is not
alphanumeric / space / hyphen, then spaces to hyphens. Emoji and punctuation
vanish, which is why a heading full of ⭐ and ⛔ produces double hyphens where they
were — the commonest way to get one of these wrong by hand.

⚠️ Code spans and fenced blocks are stripped before scanning, so a `[1.1.2]` or a
sample URL inside backticks is not treated as a link.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
FILES = [REPO / "README.md", *sorted((REPO / "docs").glob("*.md"))]

LINK = re.compile(r"\[([^\]]*)\]\(([^)\s]+)\)")
HEADING = re.compile(r"^#{1,6}\s+(.*?)\s*$", re.M)


def slug(text: str) -> str:
    """A heading's GitHub anchor."""
    text = text.replace("`", "")
    text = "".join(c for c in text.lower() if c.isalnum() or c in " -_")
    return text.strip().replace(" ", "-")


def anchors_of(path: Path) -> set[str]:
    return {slug(h) for h in HEADING.findall(path.read_text(encoding="utf-8"))}


def strip_code(text: str) -> str:
    text = re.sub(r"```.*?```", "", text, flags=re.S)
    return re.sub(r"`[^`]*`", "", text)


def main() -> int:
    bad = checked = 0
    for path in FILES:
        body = strip_code(path.read_text(encoding="utf-8"))
        for label, target in LINK.findall(body):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            checked += 1
            file_part, _, anchor = target.partition("#")
            dest = path if not file_part else (path.parent / file_part)
            if not dest.exists():
                print(f"⛔ {path.name}: [{label[:40]}] -> {target}  (no such file)")
                bad += 1
            elif anchor and anchor not in anchors_of(dest):
                print(f"⛔ {path.name}: [{label[:40]}] -> {target}"
                      f"  (no such anchor in {dest.name})")
                bad += 1
    print(f"\n{checked - bad}/{checked} relative links resolve")
    if bad:
        print("\n  ⚠️ A heading's anchor is its text lowercased with punctuation and")
        print("     emoji removed and spaces turned to hyphens — so ⭐ and ⛔ leave a")
        print("     DOUBLE hyphen behind. Copy it from the heading rather than typing it.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
