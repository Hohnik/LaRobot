#!/usr/bin/env python3
"""Tests for the markdown unwrapper. No hardware.

    uv run tests/test_unwrap.py

⛔ WHY THESE MATTER MORE THAN MOST. This tool rewrites ~250 KB of documentation whose
exact wording is the thing the repo is for. Every test below is a newline that is
load-bearing: remove it and the file renders wrongly, or worse, still renders but says
something slightly different.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "apps"))    # ⛔ app and check scripts are files, not packages;
sys.path.insert(0, str(REPO / "checks"))  # a test OF one imports it from its directory
sys.path.insert(0, str(REPO / "scripts"))

from unwrap_markdown import content, unwrap  # noqa: E402


def test_a_wrapped_paragraph_becomes_one_line() -> None:
    assert unwrap("one two\nthree four") == "one two three four"


def test_blank_lines_still_separate_paragraphs() -> None:
    assert unwrap("a a\na a\n\nb b\nb b") == "a a a a\n\nb b b b"


def test_a_heading_is_never_joined_to_the_text_under_it() -> None:
    assert unwrap("## Title\nbody text\nmore body") == "## Title\nbody text more body"


def test_table_rows_stay_one_per_line() -> None:
    """⛔ One row per line IS the syntax. Joining them destroys the table."""
    table = "| a | b |\n|---|---|\n| 1 | 2 |\n| 3 | 4 |"
    assert unwrap(table) == table


def test_a_fenced_code_block_is_copied_untouched() -> None:
    code = "text here\n\n```bash\nuv run a.py\nuv run b.py\n```\n\nmore text"
    assert unwrap(code) == code


def test_an_INDENTED_code_block_is_copied_untouched() -> None:
    """⛔ THE ONE THAT WOULD HAVE CORRUPTED A REAL FILE. ROADMAP step 1b holds a
    two-line diagram indented by four spaces. It looks identical to a wrapped paragraph;
    the only difference is that it follows a blank line."""
    doc = "Two motors, no IK.\n\n    puck YAW   ->  motor 6\n    puck Z     ->  motor 7\n\nDone when:"
    assert unwrap(doc) == doc


def test_a_list_item_keeps_its_bullet_and_absorbs_its_continuation() -> None:
    assert unwrap("- first item\n  wrapped on\n- second") == "- first item wrapped on\n- second"


def test_a_numbered_list_survives() -> None:
    assert unwrap("1. one\n   wrapped\n2. two") == "1. one wrapped\n2. two"


def test_a_nested_list_item_is_not_swallowed_by_its_parent() -> None:
    doc = "- parent\n  - child\n  - other child"
    assert unwrap(doc) == doc


def test_a_blockquote_is_unwrapped_INSIDE_the_quote() -> None:
    """The handoff is largely blockquotes, so this is the common case, not an edge one."""
    assert unwrap("> one two\n> three four") == "> one two three four"


def test_a_blank_quote_line_separates_quoted_paragraphs() -> None:
    assert unwrap("> a a\n> a a\n>\n> b b\n> b b") == "> a a a a\n>\n> b b b b"


def test_a_heading_inside_a_blockquote_stays_its_own_line() -> None:
    assert unwrap("> ## Read this\n> body one\n> body two") == "> ## Read this\n> body one body two"


def test_two_trailing_spaces_are_a_deliberate_break_and_are_kept() -> None:
    """⚠️ Markdown's hard line break. It is the one in-paragraph newline that was asked
    for on purpose, so it must survive."""
    out = unwrap("line one  \nline two")
    assert out.count("\n") == 1, f"the deliberate break was removed: {out!r}"


def test_a_horizontal_rule_is_not_mistaken_for_text() -> None:
    assert unwrap("text\n\n---\n\nmore") == "text\n\n---\n\nmore"


def test_an_html_block_is_left_alone() -> None:
    doc = "<details>\n<summary>x</summary>\n</details>"
    assert unwrap(doc) == doc


def test_unwrapping_twice_changes_nothing_the_second_time() -> None:
    """A formatter that is not idempotent produces a diff on every run for ever."""
    doc = "# T\n\npara one\nwrapped\n\n- item\n  wrapped\n\n| a | b |\n|---|---|\n\n> quote\n> wrapped\n"
    once = unwrap(doc)
    assert unwrap(once) == once


def test_only_whitespace_ever_changes() -> None:
    """⭐ The safety property the CLI enforces before writing, tested directly."""
    doc = ("# Title\n\nSome prose that is\nwrapped over lines.\n\n- a list item that is\n"
           "  also wrapped\n\n| x | y |\n|---|---|\n| 1 | 2 |\n\n> a quote that is\n> wrapped\n\n"
           "```\ncode\nhere\n```\n\n    indented\n    code\n")
    assert content(unwrap(doc)) == content(doc)


def test_the_repo_docs_all_survive_the_content_check() -> None:
    """⭐ Runs the real safety check against the real files, so a regression in the
    unwrapper is caught by the test suite rather than by reading a mangled document."""
    targets = sorted((REPO / "docs").glob("*.md")) + [REPO / "README.md"]
    for path in targets:
        if not path.is_file():
            continue
        text = path.read_text()
        assert content(unwrap(text)) == content(text), f"{path.name} would lose content"


def test_an_indented_paragraph_KEEPS_its_indentation() -> None:
    """⛔ Found while writing ROADMAP §7.5. A paragraph indented three spaces belongs to a
    numbered list item. Strip the indent and Markdown lifts it out of the list, so the
    numbering after it restarts. ⚠️ The content check cannot catch this, because indentation
    is whitespace and that is precisely what the check ignores."""
    doc = "1. first\n\n2. second\n\n   a paragraph inside item two\n   wrapped here\n\n3. third"
    out = unwrap(doc)
    assert "   a paragraph inside item two wrapped here" in out, out


def test_a_paragraph_after_a_code_block_inside_a_list_keeps_its_indent() -> None:
    """The exact shape that failed: list item, fenced code, then more of the same item."""
    doc = ("5. the item\n\n   ```\n   kp: [80, 80]\n   ```\n\n"
           "   the rest of item five,\n   wrapped\n")
    out = unwrap(doc)
    assert "   the rest of item five, wrapped" in out, out
    assert "   kp: [80, 80]" in out, "the code block must be untouched"


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
