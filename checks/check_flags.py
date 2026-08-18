#!/usr/bin/env python3
"""Do the commands written in docs/ actually work? Checks every documented command line
against the real argparse parser of the script it names.

    uv run checks/check_flags.py

⭐⭐ WHY THIS EXISTS. docs/ROADMAP.md §8.2 item 24. `docs/COMMANDS.md` has gone stale
**four times in two days**, and one stale line recommended a command that drives the
jaws into both mechanical stops. Every instance was found by a human reading carefully,
which does not scale and did not catch them promptly.

⛔⭐ THE FAILURE THIS PREVENTS IS NOT A TYPO. It is Julien opening a file that exists to
tell him what to run, running it, and getting either an argparse error or — much worse —
a command that parses and does something other than what the surrounding prose says.

⭐ WHAT IT CHECKS, in order of how much it matters:

1. ⛔ **A documented flag the parser does not have.** The command cannot run at all.
2. ⛔ **A documented value the parser will reject** — outside `choices`, or not parseable
   as the declared `type`. Also cannot run.
3. ⚠️ **A value given to an on/off flag**, e.g. `--yes true`. argparse then reads the
   value as a positional argument, and most of these scripts have none, so it fails.
4. ⚠️ **A flag the parser has that no document mentions.** Not a defect, but this is how
   `--max-lag` stayed invisible while Julien was asking why the arm could not keep up.

⚠️ WHAT IT DELIBERATELY DOES NOT CHECK. Whether the prose *around* a command describes
it correctly, whether a named device still exists (`--camera c920` parses fine and the
C920 was unplugged days ago), or whether the keystrokes in a procedure are right. **A
green run here means every documented command is well-formed, not that it is wise.**

⛔ It never executes anything. It reads source and markdown only.
"""

from __future__ import annotations

import ast
import re
import shlex
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DOCS = REPO / "docs"

#: A documented invocation. Stops at a markdown table pipe, a backtick, or a comment,
#: because those end the command and are not part of it.
# ⛔ The path alternation is load-bearing: when scripts/ split into apps/checks/tests
# (ROADMAP §10.5 step 4), a pattern still reading "scripts/" matched NOTHING and this
# checker went green while validating nothing — caught only by falsify_check_flags's
# catch-count dropping (FINDINGS §59.1's exact scenario, §70.8). src/yam covers the
# documented spacemouse_live diagnostic, which lives in the package.
COMMAND = re.compile(r"uv run ((?:apps|checks|tests|scripts|src)(?:/[a-z_0-9]+)*/[a-z_0-9]+\.py)([^`|\n#]*)")

#: ⛔⭐ SHELL OPERATORS END THE COMMAND, AND MISSING THIS PRODUCED NINE FALSE POSITIVES
#: ON THE FIRST RUN. The docs are full of `uv run A && uv run B --arm B --yes`, and the
#: pattern above happily read `--arm` and `--yes` as flags of **A**, then reported that
#: `check_rig.py` has no `--arm`. It does not, and nobody claimed it did.
#:
#: ⚠️ This matters more than the miscount: a checker whose output is mostly noise gets
#: skimmed and then ignored, which leaves the repo worse off than having no checker,
#: because now there is a green-looking thing nobody reads.
CHAIN = re.compile(r"&&|\|\||;|\buv run\b")


def command_args(tail: str) -> str:
    """The argument string belonging to THIS command, cut at the next shell operator."""
    cut = CHAIN.search(tail)
    return tail[:cut.start()] if cut else tail

#: argparse actions that take no value.
FLAGLIKE = {"store_true", "store_false", "store_const", "count", "help", "version"}


@dataclass
class Flag:
    """One `add_argument` as the parser will actually behave."""

    names: list[str]
    takes_value: bool = True
    choices: list[str] | None = None
    type_name: str | None = None
    nargs: object = None
    line: int = 0


@dataclass
class Parser:
    """Every flag one script declares."""

    path: Path
    flags: dict[str, Flag] = field(default_factory=dict)
    #: ⚠️ True when the file has no `add_argument` at all, which means "takes no flags"
    #: rather than "could not be read". The two must not be conflated.
    has_parser: bool = False


def literal_constants(tree: ast.Module) -> dict[str, object]:
    """Module-level constants we can evaluate, so `choices=sorted(ARM_SERIALS)` resolves.

    ⭐ Without this the arm-name check is skipped, and `--arm` is the single most
    frequently documented flag in the repo.
    """
    out: dict[str, object] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name):
                try:
                    out[target.id] = ast.literal_eval(node.value)
                except (ValueError, TypeError, SyntaxError):
                    pass
    return out


def resolve_choices(node: ast.AST, consts: dict[str, object]) -> list[str] | None:
    """`choices=` as a list of strings, or None when it cannot be known statically."""
    try:
        return [str(v) for v in ast.literal_eval(node)]
    except (ValueError, TypeError, SyntaxError):
        pass
    # `sorted(NAME)` / `list(NAME)` / bare `NAME`
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
            and node.func.id in {"sorted", "list", "tuple"} and len(node.args) == 1:
        node = node.args[0]
    if isinstance(node, ast.Name) and node.id in consts:
        value = consts[node.id]
        if isinstance(value, dict):
            return [str(k) for k in value]
        if isinstance(value, (list, tuple, set, frozenset)):
            return [str(v) for v in value]
    return None


_SRC_CONSTS: dict[str, object] | None = None


def src_constants() -> dict[str, object]:
    """Every module-level literal constant in `src/`, so IMPORTED choices resolve.

    ⛔⭐⭐ THIS GAP LET TWO DELIBERATELY BROKEN COMMANDS THROUGH ON THE FIRST
    FALSIFICATION RUN. `--arm` declares `choices=sorted(ARM_SERIALS)`, and `ARM_SERIALS`
    is a dict literal in `src/yam/can.py`, not in the script. So the choices could not be
    resolved, the check was silently skipped, and `--arm Q` was reported as fine.

    ⚠️ **A check that cannot resolve its data must not pass quietly**, and this one did.
    That is the same shape as the thermal guard treating an unreadable temperature as a
    safe one (`src/yam/robot.py::ThermalGuard`) — the most dangerous kind of default.

    ⭐ Arm names are the most frequently documented value in the whole repo, so this was
    not an edge case; it was the main case.
    """
    global _SRC_CONSTS  # noqa: PLW0603
    if _SRC_CONSTS is None:
        merged: dict[str, object] = {}
        for f in sorted((REPO / "src").rglob("*.py")):
            try:
                merged.update(literal_constants(ast.parse(f.read_text())))
            except (OSError, SyntaxError):
                continue
        _SRC_CONSTS = merged
    return _SRC_CONSTS


def read_parser(path: Path) -> Parser:
    out = Parser(path=path)
    try:
        tree = ast.parse(path.read_text())
    except (OSError, SyntaxError):
        return out
    # ⭐ The file's own constants win over src/, since a script may shadow a name.
    consts = {**src_constants(), **literal_constants(tree)}

    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_argument"):
            continue
        names = [a.value for a in node.args
                 if isinstance(a, ast.Constant) and isinstance(a.value, str)
                 and a.value.startswith("-")]
        if not names:
            continue                       # a positional argument; not our business
        out.has_parser = True
        flag = Flag(names=names, line=node.lineno)
        for kw in node.keywords:
            if kw.arg == "action" and isinstance(kw.value, ast.Constant):
                flag.takes_value = kw.value.value not in FLAGLIKE
            elif kw.arg == "choices":
                flag.choices = resolve_choices(kw.value, consts)
            elif kw.arg == "type" and isinstance(kw.value, ast.Name):
                flag.type_name = kw.value.id
            elif kw.arg == "nargs":
                flag.nargs = getattr(kw.value, "value", "?")
        for n in names:
            out.flags[n] = flag

    # ⭐ A PLURAL FLAG INHERITS ITS SINGULAR'S CHOICES. `--arms` takes a comma list of
    # arm names and declares no `choices` of its own, because the script validates it
    # with `parse_arms(..., known=ARM_SERIALS)` after parsing. So `--arms B,Q` was the
    # second miss on the falsification run. The domain is the same set as `--arm`'s, and
    # taking it from there keeps the checker honest without hard-coding arm names.
    for name, flag in list(out.flags.items()):
        if flag.choices is None and (singular := name.rstrip("s")) != name:
            source = out.flags.get(singular)
            if source is not None and source.choices is not None:
                flag.choices = list(source.choices)
    return out


#: ⭐ Documentation legitimately writes a PLACEHOLDER where a value goes, and a
#: placeholder is not a wrong value. `--arm <B|G>` means "pick one", and complaining
#: about it would train the reader to ignore this checker's output.
#:
#: ⚠️ The first real-docs run reported exactly this, as `--arm value '<B' is not one of
#: ['B', 'G']` — and the mangled `'<B'` is a second lesson: the command pattern stops at
#: `|` because that character ends a markdown table cell, so it had already cut the
#: placeholder in half before the check ever saw it.
#: ⛔⭐ AND THE FIRST VERSION OF THIS RULE WAS TOO BROAD AND BROKE A REAL CHECK. It also
#: treated any ALL-CAPS word as a placeholder, on the theory that `SLOT` or `PATH` might
#: appear. ⛔ **Arm names ARE single capital letters**, so `--arm Q` immediately became
#: "a placeholder" and the falsification run went from 7 catches to 6. Bracket notation
#: only, now: if an all-caps placeholder ever shows up the checker will complain, and
#: wrapping it in `<>` is better documentation anyway.
PLACEHOLDER = re.compile(r"[<>{}\[\]|]")


def check_value(flag: Flag, value: str) -> str | None:
    """Would the parser reject this value? Returns the complaint, or None."""
    if PLACEHOLDER.search(value):
        return None
    if flag.choices is not None and value not in flag.choices:
        # ⭐ `--arms B,G` is one string the script splits itself, so a comma list is
        # checked piecewise against the choices rather than whole.
        parts = value.split(",")
        if len(parts) > 1 and all(p in flag.choices for p in parts):
            return None
        return (f"value {value!r} is not one of {sorted(flag.choices)}")
    if flag.type_name in {"float", "int"}:
        try:
            (float if flag.type_name == "float" else int)(value)
        except ValueError:
            return f"value {value!r} is not a {flag.type_name}"
    return None


def documented_commands() -> list[tuple[Path, int, str, str]]:
    """`(doc, line, script, argument string)` for every documented invocation."""
    found = []
    for doc in sorted(DOCS.glob("*.md")):
        for i, line in enumerate(doc.read_text().splitlines(), start=1):
            for m in COMMAND.finditer(line):
                found.append((doc, i, m.group(1), command_args(m.group(2))))
    return found


def main() -> int:  # noqa: C901
    parsers: dict[str, Parser] = {}
    errors: list[str] = []
    warnings: list[str] = []
    mentioned: dict[str, set[str]] = {}
    checked = 0

    for doc, line, script, argstr in documented_commands():
        rel = f"{doc.name}:{line}"
        path = REPO / script
        if not path.exists():
            errors.append(f"{rel}: documents {script}, which does not exist")
            continue
        if script not in parsers:
            parsers[script] = read_parser(path)
        parser = parsers[script]

        try:
            tokens = shlex.split(argstr, comments=True)
        except ValueError:
            continue                       # an unbalanced quote in prose, not a command
        checked += 1

        i = 0
        while i < len(tokens):
            token = tokens[i]
            if not token.startswith("-"):
                i += 1
                continue
            name, _, inline = token.partition("=")
            if name not in parser.flags:
                if parser.has_parser:
                    errors.append(
                        f"{rel}: {script} has no flag {name}  (in: uv run {script}"
                        f"{argstr.rstrip()})")
                i += 1
                continue
            mentioned.setdefault(script, set()).add(name)
            flag = parser.flags[name]

            if not flag.takes_value:
                if inline:
                    warnings.append(f"{rel}: {name} is an on/off flag but is given "
                                    f"={inline!r}")
                i += 1
                continue

            value = inline
            if not value:
                if i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                    value = tokens[i + 1]
                    i += 1
                else:
                    warnings.append(f"{rel}: {name} needs a value and none is shown")
                    i += 1
                    continue
            complaint = check_value(flag, value)
            if complaint:
                errors.append(f"{rel}: {name} {complaint}  (in: uv run {script}"
                              f"{argstr.rstrip()})")
            i += 1

    # ---- flags that exist and nothing documents. The --max-lag problem.
    #
    # ⭐ A flag counts as documented if it appears ANYWHERE in docs/, not only inside a
    # runnable `uv run` line. The first version demanded a full command line and so
    # reported ten flags on teleop_session.py that COMMANDS.md explains at length in
    # prose and tables. That is the cry-wolf failure again: technically true, useless,
    # and it buries the flags that really are invisible.
    prose = "\n".join(d.read_text() for d in sorted(DOCS.glob("*.md")))
    for script in sorted(parsers):
        parser = parsers[script]
        seen = mentioned.get(script, set())
        undocumented = sorted(
            f.names[0] for f in {id(f): f for f in parser.flags.values()}.values()
            if not any(n in seen for n in f.names)
            and not any(re.search(rf"(?<![\w-]){re.escape(n)}(?![\w-])", prose)
                        for n in f.names))
        if undocumented:
            warnings.append(f"{script}: NO document mentions " + ", ".join(undocumented))

    print(f"CHECKED {checked} documented command line(s) against "
          f"{len(parsers)} parser(s).\n")
    for e in errors:
        print(f"⛔ {e}")
    if errors and warnings:
        print()
    for w in warnings:
        print(f"⚠️  {w}")

    print()
    if errors:
        print(f"⛔ {len(errors)} documented command(s) WOULD NOT RUN. Fix the document, "
              f"or the parser.")
    else:
        print("✓ every documented command's flags exist and every shown value is one "
              "the parser accepts.")
    print("⚠️ This cannot check whether the PROSE around a command is right, whether a "
          "named\n   device is still plugged in, or whether the keystrokes are correct.")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
