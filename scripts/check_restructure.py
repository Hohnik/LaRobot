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
    "thermal", "teleop", "mode",
    # ⭐ Step 2c, 2026-08-14: this cycle's temperatures. They were session locals, so they
    # were one arm's reading available to whichever row was being painted — and the status
    # is one row per arm now. `ArmSession.read_thermal()` sets them too, so the class stays
    # coherent for a caller that does not read the chain itself.
    "hottest", "jaw_temp",
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
STILL_TO_MOVE: list[str] = []   # ⭐ step 1 is COMPLETE

#: ⛔⭐ SESSION-LEVEL NAMES THAT WERE DELETED RATHER THAN MOVED, and must not come back.
#:
#: `MOVED_SO_FAR` models one shape of change: a local named `x` becomes `arm.x`, so the
#: check is "no bare `x` survives AND `arm.x` is read somewhere". **A rename does not fit
#: that model.** `control_frame` became `ArmSession.frame`, which already existed, so
#: there is no `arm.control_frame` for check 2 to find and the old name would slip back in
#: unnoticed.
#:
#: ⚠️ Why it matters more than tidiness: a second copy of live state is what
#: [FINDINGS §52.7](../docs/FINDINGS.md) is about. `control_frame` sat beside
#: `ArmSession.frame` for two days, only the local was updated on a frame change, and the
#: object quietly disagreed with the session. Deleting one copy is the fix; this list is
#: what stops it being re-created.
RETIRED_LOCALS = ["control_frame"]


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
    # ⛔⭐⭐ AND IT COVERS THE CONSTRUCTION STATEMENT ITSELF, NOT ONLY THE LINES BEFORE IT.
    #
    # This check said *"`arm` is built on line 948 and nothing touches it earlier"* while
    # line 948 read `arm = ArmSession(robot, name=arm_names[0], frame=arm.frame, …)`. It was
    # right and useless: the read was ON the construction line, so `lineno < built_at` was
    # false. `arm` is None there, so that line raises `AttributeError` **after
    # `build_robot()` has already enabled the motors**, and the `finally` block then
    # disables them. On a raised arm that is a sag. A mechanical rewrite of the frame
    # introduced it in one pass (FINDINGS §53.1).
    #
    # ⚠️ Bounded by the call's END line, not its start: `arm = ArmSession(\n … arm.frame …)`
    # puts the bad read on a continuation line, where a `<= built_at` test would miss it
    # again. The whole statement is the window.
    construction = [n for n in ast.walk(fn)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                    and n.func.id == "ArmSession"]
    if not construction:
        faults += 1
        print("⛔ `ArmSession(` is never called in main() — the object is not constructed")
    else:
        call = min(construction, key=lambda n: n.lineno)
        built_at = call.lineno
        built_through = call.end_lineno or built_at
        early = sorted({n.lineno for n in ast.walk(fn)
                        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
                        and n.value.id == "arm" and n.lineno <= built_through})
        if early:
            faults += len(early)
            print(f"⛔ `arm` is built on line(s) {built_at}-{built_through}, but it is read "
                  f"through on line(s) {early}.")
            print("   Those run before the object exists. A read BEFORE the call raises")
            print("   UnboundLocalError; a read INSIDE the call raises AttributeError,")
            print("   because `arm` is still the `None` declared above the try.")
            print("   ⛔ Either way the motors are already enabled by build_robot() and the")
            print("      finally block then disables them, which drops a raised arm.")
            print("   ⚠️ A dry run cannot catch this: it returns before this part of main().")
        else:
            print(f"✓ `arm` is built on line(s) {built_at}-{built_through} and nothing "
                  "reads it there or earlier")

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

    # 5. ⛔⭐⭐ EVERY PER-ARM HELPER IS CALLED WITH AN ARM, AND WITH THE RIGHT COUNT.
    #
    # Step 2 turned `enter_hold()`, `enter_teleop()`, `enter_guide()`, `resync()`,
    # `begin_path()` and `park_plan_line()` from closures over one arm into functions that
    # take the arm they act on — about thirty call sites. **Python cannot see a wrong
    # argument count until the line runs**, and every one of those lines is inside the
    # control loop with the motors live.
    #
    # ⚠️ The nets that already exist do not cover this: `compile()` accepts a bad arity,
    # check 4 only asks whether a NAME resolves, and a dry run returns before the loop
    # (FINDINGS §42.0). So the first execution of a mis-called helper would be on the arm.
    #
    # ⭐ A helper is recognised by its first parameter being named `one`, which is the
    # convention this file now relies on. If that name changes, this check goes quiet
    # rather than wrong — so it also reports how many helpers it found.
    helpers: dict[str, tuple[int, int]] = {}    # name -> (required, total)
    for node in ast.walk(fn):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args.args
            if args and args[0].arg == "one":
                helpers[node.name] = (len(args) - len(node.args.defaults), len(args))
    bad = []
    for node in ast.walk(fn):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id in helpers):
            required, total = helpers[node.func.id]
            given = len(node.args) + len(node.keywords)
            if not (required <= given <= total) or not node.args:
                bad.append((node.lineno, node.func.id, given, required))
    if not helpers:
        print("⚠️ no per-arm helpers found (first parameter `one`) — check 5 is asleep")
    elif bad:
        faults += len(bad)
        print(f"⛔ {len(bad)} call(s) to a per-arm helper with the wrong arguments:")
        for lineno, name, given, required in bad:
            print(f"     line {lineno}: {name}() got {given}, needs {required} "
                  "(the first must be the arm)")
    else:
        print(f"✓ every call to the {len(helpers)} per-arm helper(s) passes an arm: "
              f"{', '.join(sorted(helpers))}")

    # 6. Retired session-level names stay retired.
    back = sorted({(n.lineno, n.id) for n in ast.walk(fn)
                   if isinstance(n, ast.Name) and n.id in RETIRED_LOCALS})
    if back:
        faults += len(back)
        print(f"⛔ {len(back)} use(s) of a RETIRED session-level name:")
        for lineno, name in back:
            print(f"     line {lineno}: {name} — it was deleted, not moved; the state "
                  "lives on the arm now")
    else:
        print(f"✓ retired session-level name(s) have not come back: "
              f"{', '.join(RETIRED_LOCALS)}")

    # 7. Progress, so the series has a visible finish line.
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
