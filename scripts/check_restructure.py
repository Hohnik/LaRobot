#!/usr/bin/env python3
"""Verify one step of the `ArmSession` restructure. Reads code only; runs nothing.

    uv run scripts/check_restructure.py
    uv run scripts/check_restructure.py --moved prev_q guide_ref home_ee

⭐ WHY THIS EXISTS. [ROADMAP §6.1](../docs/ROADMAP.md) step 1 moves one arm's state out
of `main()`'s locals and onto an `ArmSession`, across **247 real edit sites**, landing as
a series of commits that each leave the script runnable. **A mechanical check after each
commit is what makes that safe**, because the failure mode is a name that still refers to
the old local: Python will not complain until that line runs, and the line that runs it is
in a control loop with the motors live.

⛔⛔ THE TWO TRAPS THIS SCRIPT EXISTS TO CATCH, both hit on the first commit:

1. **A text substitution rewrites comments and docstrings too.** `mode` appears 35 times
   in `main()`'s own comments. That is also how [FINDINGS §36.3](../docs/FINDINGS.md)'s
   published count came to be 35% too high. **So the rewrite must be driven by the
   parser**, and so must the check.
2. ⛔ **`nonlocal` names are NOT variable nodes.** They live in `ast.Nonlocal.names` as
   plain strings. The first check looked only at `ast.Name` and reported a clean move
   while three `nonlocal` statements still named the moved locals. Python raised
   `SyntaxError: no binding for nonlocal 'prev_q' found`, which caught it — but a check
   that cannot see the thing it checks is the defect this repo keeps finding
   ([FINDINGS §39.4](../docs/FINDINGS.md)). **This one looks at both.**

⚠️ What it does NOT prove: that behaviour is unchanged. Only the arm can say that, and
[ROADMAP §6.1](../docs/ROADMAP.md) is explicit that `--arms B` at N=1 is the test. This
proves the substitutions are complete and the file is coherent.
"""

from __future__ import annotations

import argparse
import ast
import builtins
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TARGET = REPO / "scripts" / "teleop_session.py"

#: State already moved onto `ArmSession`. Add to this as each commit lands, so the
#: check keeps proving that earlier groups have not regressed.
MOVED_SO_FAR = [
    "prev_q", "guide_ref", "home_ee", "gripper_value", "stall_since",
    "park_path", "park_s", "park_marks", "park_target", "park_cmd", "park_best_err",
    "park_progress_t", "park_leg_t", "park_start_t", "park_speed", "park_ramp",
    "thermal", "teleop",
]

#: Still locals of `main()`. ⛔⭐ NEITHER of these is a pure substitution any more, and
#: FINDINGS §48.3 has the detail:
#:
#:   * `thermal` is read in the closing summary, AFTER the `finally` block, on a path
#:     that runs when `build_robot()` FAILED — so `arm` does not exist there. Moving it
#:     naively replaces the "no adapter found" message with an UnboundLocalError, on the
#:     failure Julien hits most often. It needs `arm = None` before the `try`, and this
#:     checker must first learn to find the `ArmSession(` call rather than the first
#:     assignment to `arm`, or the ordering check goes blind.
#:   * `mode` is read by `build_robot()` to decide `zero_gravity`, before the robot and
#:     therefore the ArmSession exist. The script keeps a local `mode` for that decision.
STILL_TO_MOVE = ["mode"]


def main_function(tree: ast.Module) -> ast.FunctionDef:
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "main":
            return node
    raise SystemExit("⛔ no main() in the target file")


def defined_names(tree: ast.Module, fn: ast.FunctionDef) -> set[str]:
    """Everything a name inside `fn` could legitimately resolve to.

    ⚠️ Module-level `def` and `class` names are included deliberately. Leaving them
    out made the first version of this check report eleven false positives, including
    `git_commit` and `load_json`, which are module-level functions in the target.
    """
    names: set[str] = set(dir(builtins))
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(a.asname or a.name.split(".")[0] for a in node.names)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        for sub in ast.walk(node):
            if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Store):
                names.add(sub.id)
    for node in ast.walk(fn):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            names.add(node.id)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            a = node.args
            names.update(x.arg for x in [*a.posonlyargs, *a.args, *a.kwonlyargs])
            for x in (a.vararg, a.kwarg):
                if x:
                    names.add(x.arg)
        if isinstance(node, ast.ExceptHandler) and node.name:
            names.add(node.name)
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(a.asname or a.name.split(".")[0] for a in node.names)
        if isinstance(node, ast.withitem) and isinstance(node.optional_vars, ast.Name):
            names.add(node.optional_vars.id)
    return names


def run(moved: list[str]) -> int:
    src = TARGET.read_text()
    tree = ast.parse(src)
    compile(src, TARGET.name, "exec")          # catches the nonlocal-binding error
    fn = main_function(tree)
    faults = 0

    span = fn.end_lineno - fn.lineno + 1
    print(f"target : {TARGET.relative_to(REPO)}")
    print(f"main() : lines {fn.lineno}-{fn.end_lineno} ({span} lines)\n")

    # 1. No moved name may survive as a bare local, in code OR in a declaration.
    bare = sorted({(n.lineno, n.id) for n in ast.walk(fn)
                   if isinstance(n, ast.Name) and n.id in moved})
    decl = sorted({(n.lineno, nm) for n in ast.walk(fn)
                   if isinstance(n, (ast.Nonlocal, ast.Global)) for nm in n.names if nm in moved})
    if bare:
        faults += len(bare)
        print(f"⛔ {len(bare)} bare reference(s) to a moved name still in main():")
        for lineno, name in bare:
            print(f"     line {lineno}: {name}")
    if decl:
        faults += len(decl)
        print(f"⛔ {len(decl)} nonlocal/global declaration(s) still name a moved field:")
        for lineno, name in decl:
            print(f"     line {lineno}: {name}   (an attribute needs no declaration)")
    if not bare and not decl:
        print(f"✓ none of the {len(moved)} moved name(s) survive as locals: {', '.join(moved)}")

    # 2. They must actually be reached through the object.
    used = {}
    for node in ast.walk(fn):
        if (isinstance(node, ast.Attribute) and node.attr in moved
                and isinstance(node.value, ast.Name) and node.value.id == "arm"):
            used[node.attr] = used.get(node.attr, 0) + 1
    print(f"✓ arm.<field> accesses: {sum(used.values())}  {dict(sorted(used.items()))}")
    missing = [m for m in moved if m not in used]
    if missing:
        faults += len(missing)
        print(f"⛔ moved but never read through `arm`: {missing} — did the field get dropped?")

    # 3. ⛔⭐⭐ ORDERING: nothing may touch `arm` before `arm` is constructed.
    #
    # This check exists because the second commit of the series hit exactly that and
    # NOTHING ELSE CAUGHT IT. The rewriter turned `gripper_value = 0.0` — an
    # initialisation nine lines above the `try` — into `arm.gripper_value = 0.0`, which
    # would raise `UnboundLocalError` because `arm` is built after `build_robot()`.
    #
    # ⛔ The two nets both had holes: the coherence check above passes because `arm` IS
    # assigned somewhere in `main()`, and **the dry run never reaches that line at all**,
    # because `--yes` is required before the device is opened and the state initialised.
    # So the first thing to execute it would have been a real session on the arm.
    # ⭐ It would have failed safely, before anything was energised. It would still have
    # cost Julien a session.
    # ⛔⭐ FIND THE `ArmSession(` CALL, NOT THE FIRST ASSIGNMENT TO `arm`.
    #
    # The first version of this check looked for the first `arm = …` and called that the
    # construction point. That worked while `arm` was assigned exactly once. It stops
    # working the moment `arm = None` has to appear before the `try`, which FINDINGS §48.3
    # established is needed: the closing summary reads a field off `arm` on the path where
    # `build_robot()` FAILED and the object was never built.
    #
    # ⛔ Under the old rule that early `None` would read as "arm is built on line 700", and
    # every genuine ordering fault after it would pass unnoticed. **The check would have
    # gone blind exactly when the code got more complicated**, which is the worst possible
    # moment and is why this was changed BEFORE the change it protects.
    construction = [n.lineno for n in ast.walk(fn)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                    and n.func.id == "ArmSession"]
    if not construction:
        faults += 1
        print("⛔ `ArmSession(` is never called in main() — the object is not constructed")
    else:
        built_at = min(construction)
        early = sorted({n.lineno for n in ast.walk(fn)
                        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
                        and n.value.id == "arm" and n.lineno < built_at})
        if early:
            faults += len(early)
            print(f"⛔ `arm` is built on line {built_at}, but it is touched earlier, on "
                  f"line(s) {early}.")
            print("   Those run before the object exists and will raise UnboundLocalError.")
            print("   ⚠️ A dry run cannot catch this: it returns before this part of main().")
        else:
            print(f"✓ `arm` is built on line {built_at} and nothing touches it earlier")

    # 4. Nothing may be undefined. This is the scan that caught a NameError before it
    #    reached the arm on 2026-08-13 (FINDINGS, session 21): `replay_step` was called
    #    and never imported, so the first playback would have raised with motors live.
    known = defined_names(tree, fn)
    unknown = sorted({n.id for n in ast.walk(fn)
                      if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                      and n.id not in known})
    if unknown:
        faults += len(unknown)
        print(f"⛔ used in main() but never assigned or imported: {unknown}")
    else:
        print("✓ every name used in main() resolves to something")

    # 5. Progress, so the series has a visible finish line.
    remaining = {}
    for node in ast.walk(fn):
        if isinstance(node, ast.Name) and node.id in STILL_TO_MOVE:
            remaining[node.id] = remaining.get(node.id, 0) + 1
    left = sum(remaining.values())
    print(f"\nstill to move: {left} reference(s) across {len(remaining)} name(s)")
    for name in sorted(remaining, key=lambda k: -remaining[k]):
        print(f"   {name:<16} {remaining[name]:>3}")
    print("\n⚠️ `mode` moves LAST: build_robot() reads it to decide zero_gravity, and it")
    print("   runs before the robot — and therefore the ArmSession — exists.")

    print()
    if faults:
        print(f"⛔ VERDICT: {faults} fault(s). Do not commit this step.")
        return 1
    print("✓ VERDICT: this step is coherent. ⚠️ That is not the same as unchanged")
    print("  behaviour — only `--arms B` at N=1 on the real arm can say that.")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--moved", nargs="+", default=MOVED_SO_FAR,
                    help="the names that should now live on `arm` (default: MOVED_SO_FAR)")
    sys.exit(run(ap.parse_args().moved))
