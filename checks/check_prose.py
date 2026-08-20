#!/usr/bin/env python3
"""Check the documents JULIEN reads against his own writing rules. No hardware, reads only.

    uv run checks/check_prose.py                 # every document in the manifest
    uv run checks/check_prose.py docs/PLAN.md    # just one
    uv run checks/check_prose.py --ceilings      # print the current counts as a manifest block

⛔⭐⭐ WHY THIS EXISTS, and it is not because the rules were missing. [HANDOFF §4](../docs/HANDOFF.md) rule 8 has carried them since 2026-08-12, refined three times. `docs/ARCHITECTURE.md` was then written on 2026-08-19 breaking essentially all of them, and his verdict was that he could not read it: *"you wrote it in a writing style again that does not make a lot of sense. It's not nicely structured. It's not understandable."*

⭐ The principle this follows is the project's own ([FINDINGS §33.3](../docs/FINDINGS.md), and Mind Understanding's `state/NOW.md` §5 states it outright): **a mistake that recurs after being documented is not a memory problem, and it needs a mechanical defence.** Rule 8 is now checkable.

## What it enforces, and what it deliberately does not

⭐ ENFORCED, because each one is wrong in a document he reads:

- **Em-dashes.** His rule: use a comma, a full stop or brackets.
- **The antithesis moves**: `it's not X, it's Y` and every disguise of it, `, not `, ` and not `, `not just X but Y`.
- **Facts bolted on with `which is` / `which means`.** They get their own sentence.
- **Sentences over 30 words.** A tired reader must parse it once, left to right, without backtracking.
- **Bold inside a sentence.** Bold a whole line or a heading, never a phrase mid-sentence.
- **A few words he named**: `precisely`, `genuinely`, `load-bearing`, `the honest answer`, `surface` as a verb.
- ⭐ **A metaphor standing in for a plain fact**, added 2026-08-20: `travel` for apply, `holds`/`carries` for contains, `bites` for the mistake you will make, `lives in` for is defined in, `rides` for is stored, `sits` for is, `earns its place` for caught a defect, `cries wolf` for reports non-faults, `goes blind` for stops detecting. Each report names the plain replacement, because a message that only says "wrong" makes every reader invent a different fix.

⛔ NOT ENFORCED, on purpose, and the reason matters. Mind Understanding's `scripts/check_style.py` is the canonical copy of these rules and it lints **chat drafts**, where `⭐`, `⛔` and ALL CAPS are shouting. In a repo file they are navigation, and his rule says so explicitly: *"Repo files keep their own conventions. This rule is about chat only."* So decoration characters and capitalised acronyms are left alone here. ⚠️ A checker that reports 219 faults on a healthy file is the cry-wolf failure, and this repo has met it twice already.

## The ceilings, and why this is a ratchet rather than a pass/fail

Some documents in the manifest are years of accumulated prose and cleaning them in one pass is not the point. So each carries a ceiling: **the count may go down and may never go up.** A document at 0 must stay at 0. Run `--ceilings` after an intentional cleanup to print a fresh manifest block.

⛔ **A ceiling is not permission.** It records where a file stands, so a new paragraph cannot quietly make it worse.

## ⚠️ What this cannot do, and it is the important half

It measures phrases. **It cannot tell whether a document is understandable**, which is the thing he actually complained about. The four faults it cannot see:

1. A heading that is a slogan rather than a description (`ArmSession: the class decides, the script narrates`).
2. A pointer standing in for an explanation (`FINDINGS §37.0 is what happened when nobody knew this layer existed`). A link may add evidence. It may never carry the meaning.
3. A term used before it is defined.
4. Three or more things crammed into a sentence instead of a list.

**A clean run is not a passing grade.** Read it aloud. If it sounds like a written style rather than like explaining something to a colleague, rewrite it.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

#: The documents Julien reads, and the fault count each may not exceed.
#:
#: ⭐ `docs/ARCHITECTURE.md` is at 0 because it was rewritten from scratch on 2026-08-19 to
#: be readable by someone with no prior context. It is the reference for what "clean" means.
#:
#: ⛔ NOT IN HERE, because they are agent files and dense on purpose: `docs/HANDOFF.md`,
#: `docs/FINDINGS.md`, `docs/ROADMAP.md`. Adding one would be a decision about who it is
#: for, never a tidying step. `docs/Setup-Plan.md` and `docs/Setup-Anleitung.md` are the
#: TEAM's own documents, quoted here rather than authored here, so they stay out too.
HIS_DOCS: dict[str, int] = {
    "docs/ARCHITECTURE.md": 0,
    "docs/PLAN.md": 38,
    "docs/LINUX.md": 44,
    "docs/COMMANDS.md": 117,
    "docs/PERFORMANCE.md": 0,
    "docs/BRIDGE.md": 0,
    "README.md": 27,
}

#: ⛔ `docs/HISTORY.md` is deliberately absent. It is an ARCHIVE: the README's first two
#: sessions, moved out on 2026-08-19 because they were two thirds of the page that says
#: "start here". Its prose is 12 days old and nobody reads it to find anything out, so
#: holding it to a readability ceiling would be busywork on frozen text. ⚠️ It carries a
#: 📖 REFERENCE banner saying exactly that, so nobody mistakes it for the live state.

#: ⛔⭐⭐ A METAPHOR STANDING IN FOR A PLAIN FACT, with the plain word beside it. Added
#: 2026-08-20 after he read `docs/PLAN.md`, found *"none of the macOS workarounds travel"*,
#: and called it "unnecessary language flowering and unhelpfully worded"
#: ([HANDOFF §4](../docs/HANDOFF.md) rule 8, fourth refinement).
#:
#: ⭐ Each entry carries its replacement, because a message that only says "this is wrong"
#: makes the reader invent the fix, and different readers invent different ones.
#:
#: ⚠️ THIS LIST IS A FLOOR, NEVER A CEILING. It holds the metaphors seen so far. A new one
#: will not be in it, and a clean run therefore says nothing about whether the prose is
#: plain. That judgement is made by reading.
#:
#: ⚠️ Two of these have literal uses and are still listed, because in these documents they
#: have never once been literal: `carries` has always meant "contains", and `sits` has always
#: meant "is". If a genuinely literal use appears, phrase it another way rather than widening
#: the list, because widening it is how a check goes quiet ([FINDINGS §59.1](../docs/FINDINGS.md)).
FLOWERY = [
    ("travel", "say \"apply\", \"be copied\" or \"is the same on both machines\""),
    ("bites", "say what the mistake is"),
    ("bite", "say what the mistake is"),
    ("lives in", "say \"is defined in\" or \"is made in\""),
    ("lives at", "say \"is at\""),
    ("live in", "say \"are defined in\""),
    ("rides", "say \"is stored in\" or \"is recorded in\""),
    ("ride", "say \"is stored in\" or \"is recorded in\""),
    ("earns", "say what it actually did"),
    ("earned its place", "say what defect it caught"),
    ("cries wolf", "say \"reports faults that are not faults\""),
    ("cry wolf", "say \"report faults that are not faults\""),
    ("went blind", "say \"stopped detecting anything, and still passed\""),
    ("goes blind", "say \"stops detecting anything while still passing\""),
    ("owes", "say what somebody still has to do"),
    ("owed", "say what somebody still had to do"),
    ("front door", "say which file or command it is"),
    ("reaches around", "say \"bypasses\""),
    ("reach around", "say \"bypass\""),
    ("paid for itself", "say what it caught"),
    ("wedged", "say \"stuck\" and say in what state"),
    ("the tell", "say \"the sign\" or name the sign"),
    ("signature failure", "say \"the failure this stack keeps producing\""),
    ("carries", "say \"contains\", \"records\" or \"has\""),
    ("carry", "say \"contain\", \"record\" or \"have\""),
    ("sits", "say \"is\" and say where"),
    ("hides", "say what is not visible, and to whom"),
    ("buys", "say what you get"),
    ("pays for it", "say what it costs"),
]

BANNED = [
    ("it's not", "\"it's not X, it's Y\". Say what it is."),
    ("it is not x", "the antithesis move. Say what it is."),
    ("not just", "\"not just X but Y\". Say the thing."),
    (", not ", "\"X, not Y\" is the same move as \"it's not X, it's Y\"."),
    (" and not ", "antithesis. Split into two sentences."),
    ("which is", "a fact bolted on. Give it its own short sentence."),
    ("which means", "a fact bolted on. Give it its own short sentence."),
    ("precisely", "banned word."),
    ("genuinely", "banned word."),
    ("load-bearing", "banned phrase."),
    ("the honest answer", "banned phrase."),
    ("smoking gun", "banned phrase."),
    ("that said", "banned phrase."),
]

EM_DASH = re.compile(r"—|(?<!-)--(?!-)")
BOLD_SPAN = re.compile(r"\*\*([^*]+)\*\*")
LIST_MARKER = re.compile(r"^\s*(?:[-*+]\s+|\d+[.)]\s+)")
THEMATIC_BREAK = re.compile(r"^\s*([-*_])\s*(\1\s*){2,}$")
SURFACE_VERB = re.compile(r"\b(to surface|surfaced|surfacing|surfaces the|surface the)\b", re.I)


def strip_inline(line: str) -> str:
    """Remove code spans, link targets and quoted material before judging prose.

    His own words quoted back are not this repo's prose, and a banned word inside
    `a_variable_name` or a `--flag` is not a style fault.
    """
    line = re.sub(r"`[^`]*`", "", line)
    line = re.sub(r"\]\([^)]*\)", "]", line)
    line = re.sub(r"\*?\"[^\"]{10,}\"\*?", "", line)
    line = re.sub(r"\*[^*]{15,}\*", "", line)          # italic quotes of him
    return line


def prose_lines(text: str) -> list[tuple[int, str]]:
    """Ordinary prose only: no fenced code, headings, table rows or horizontal rules.

    ⚠️ Block quotes (`>`) ARE prose here, and that is deliberate: this repo puts its most
    important paragraphs inside a leading `>` block, and skipping them would exempt exactly
    the text he reads first.
    """
    out, in_fence = [], False
    for no, line in enumerate(text.split("\n"), 1):
        if re.match(r"^\s*(```|~~~)", line):
            in_fence = not in_fence
            continue
        if in_fence or not line.strip():
            continue
        # ⛔ UNQUOTE FIRST, THEN DECIDE WHAT THE LINE IS. The quote marker used to be
        # stripped last, so a table row inside a block quote (`> | a | b |`) was judged as
        # prose, and this repo puts its most important tables inside block quotes. One such
        # table was reported as a 173-word sentence. Found 2026-08-20.
        bare = re.sub(r"^\s*>+\s*", "", line)
        if re.match(r"^\s*(#|\|)", bare) or THEMATIC_BREAK.match(bare):
            continue
        out.append((no, bare))
    return out


def sentences(text: str) -> list[str]:
    """Rough split, good enough to measure length. List items and headings end a sentence.

    ⚠️ Without that, a six-item bullet list reads as one 48-word sentence and the length
    check cries wolf.
    """
    parts: list[str] = []
    # ⛔ BLANK LINES MUST SURVIVE HERE. Building this from `prose_lines` dropped them, so
    # `\n\s*\n` never matched, and the paragraph after a bullet list was glued onto the last
    # bullet. That reported a 36-word "sentence" made of two unrelated pieces of text, which
    # is the cry-wolf failure this file's own docstring warns about. Found by running it.
    keep, in_fence = [], False
    for line in text.split("\n"):
        if re.match(r"^\s*(```|~~~)", line):
            in_fence = not in_fence
            keep.append("")
            continue
        # ⛔ Unquote first, same reason as `prose_lines`: a table row inside a block quote is
        # a table row, and judging it as prose reported one table as a 173-word sentence.
        bare = re.sub(r"^\s*>+\s*", "", line)
        if in_fence or re.match(r"^\s*(#|\|)", bare) or THEMATIC_BREAK.match(bare):
            keep.append("")
            continue
        keep.append(bare)
    body = "\n".join(keep)
    for block in re.split(r"\n\s*(?:[-*+]|\d+[.)])\s+|\n\s*\n", body):
        block = re.sub(r"\s+", " ", strip_inline(block)).strip()
        parts += [s.strip() for s in re.split(r"(?<=[.!?])\s+", block) if s.strip()]
    return parts


def faults(text: str) -> list[str]:
    """Every fault in one document, as lines ready to print."""
    out: list[str] = []
    for no, raw in prose_lines(text):
        line = strip_inline(raw)
        low = line.lower()
        if EM_DASH.search(line):
            out.append(f"L{no}: em-dash. Use a comma, a full stop or brackets.")
        for phrase, why in BANNED:
            if phrase in low:
                out.append(f"L{no}: \"{phrase.strip()}\" — {why}")
        for phrase, plain in FLOWERY:
            # ⭐ Word boundaries, so "override" is not a hit for "ride" and "carrying" is
            # not judged by the rule written for "carry".
            if re.search(rf"\b{re.escape(phrase)}\b", low):
                out.append(f"L{no}: \"{phrase}\" is a metaphor for a plain fact — {plain}.")
        if SURFACE_VERB.search(line):
            out.append(f"L{no}: \"surface\" as a verb. Use the ordinary word.")
        stripped = LIST_MARKER.sub("", raw).strip()
        for m in BOLD_SPAN.finditer(raw):
            if stripped != m.group(0) and not stripped.startswith(m.group(0) + ":"):
                out.append(f"L{no}: bold inside a sentence (**{m.group(1)[:40]}**). "
                           "Bold a whole line or a heading only.")
                break
    for s in sentences(text):
        n = len(s.split())
        if n > 30:
            out.append(f"SENTENCE of {n} words: \"{s[:64]}…\" — split it, or make it a list.")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="*", help="documents to check (default: the manifest)")
    ap.add_argument("--ceilings", action="store_true",
                    help="print the current counts as a HIS_DOCS block, after a cleanup")
    ap.add_argument("--verbose", "-v", action="store_true", help="list every fault")
    args = ap.parse_args()

    targets = args.files or list(HIS_DOCS)
    counts, over = {}, []
    for rel in targets:
        path = REPO / rel if not Path(rel).is_absolute() else Path(rel)
        if not path.is_file():
            print(f"⛔ {rel}: not a file")
            return 2
        found = faults(path.read_text())
        counts[rel] = len(found)
        ceiling = HIS_DOCS.get(rel)
        mark = "·"
        if ceiling is None:
            mark = "?"
        elif len(found) > ceiling:
            mark, _ = "⛔", over.append(rel)
        elif len(found) < ceiling:
            mark = "⭐"
        cap = "no ceiling" if ceiling is None else f"ceiling {ceiling}"
        print(f"  {mark} {rel:<24} {len(found):>4} fault(s), {cap}")
        if args.verbose or (ceiling is not None and len(found) > ceiling):
            # ⛔ `-v` PRINTS EVERYTHING. It used to stop at 40 and say nothing, which hid the
            # 41st fault of a 41-fault file — so a `diff` of two verbose runs came out
            # balanced while the count had gone up by one. A silent cap inside a reporting
            # tool is the exact defect this file's own findings are about (FINDINGS §76.4's
            # family). The un-verbose view still trims, and it SAYS how many it trimmed.
            shown = found if args.verbose else found[:12]
            for line in shown:
                print(f"        {line}")
            if len(shown) < len(found):
                print(f"        … {len(found) - len(shown)} more, run with -v")

    if args.ceilings:
        print("\nHIS_DOCS: dict[str, int] = {")
        for rel, n in counts.items():
            print(f'    "{rel}": {n},')
        print("}")
        return 0

    print()
    if over:
        print(f"⛔ {len(over)} document(s) got WORSE than their ceiling: {', '.join(over)}.")
        print("   Fix the faults above. If the increase is deliberate and correct, say why in")
        print("   the commit and raise the ceiling in the same commit, never separately.")
        return 1
    improved = [r for r, n in counts.items() if HIS_DOCS.get(r) is not None and n < HIS_DOCS[r]]
    if improved:
        print(f"⭐ {len(improved)} document(s) improved: {', '.join(improved)}. Lower their "
              "ceilings with --ceilings so the gain is locked in.")
    print(f"✓ {len(counts)} document(s) at or under their ceiling.")
    print("\n⚠️ This measures PHRASES. It cannot tell whether a document is understandable,")
    print("   which is the thing that actually failed. It cannot see a slogan heading, a")
    print("   pointer standing in for an explanation, or a term used before it is defined.")
    print("   Read it aloud. If it sounds written rather than spoken, rewrite it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
