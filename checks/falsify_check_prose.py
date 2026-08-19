#!/usr/bin/env python3
"""Feed `check_prose.py` known-bad and known-good prose, and count the catches.

    uv run checks/falsify_check_prose.py

⛔ WHY. [HANDOFF §4](../docs/HANDOFF.md) rule 4: a checker that passes might be working, or it might have quietly stopped checking. A green run cannot tell you which. This one has two jobs and both are checked here.

⭐ **It must catch the faults.** Nine deliberately broken lines, each carrying exactly one of the rules, and each must be reported.

⛔ **It must leave healthy prose alone, and that half matters more here than usual.** The rules this enforces are aimed at chat, where `⭐` and ALL CAPS are shouting; in a repo file they are navigation. So a document full of decoration, acronyms, tables, code fences and bold labels on their own lines must come back CLEAN. The first version of this checker reported 219 faults on a healthy README and three on a file that had just been rewritten to be perfect. **A checker that cries wolf gets ignored, and an ignored checker is worth less than none**, because its green runs are then quoted as evidence.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "checks"))

from check_prose import faults  # noqa: E402

#: `(what is wrong with it, the text, the substring the report must contain)`
BREAKS = [
    ("an em-dash", "The arm parks first — then the playback runs.", "em-dash"),
    ("a double hyphen used as a dash",
     "The arm parks first -- then the playback runs.", "em-dash"),
    ("the antithesis move", "It's not a speed limit, it's a following-error bound.",
     "it's not"),
    ("the mirrored antithesis", "The clamp bounds the command, not the arm.", ", not"),
    ("a fact bolted on with \"which is\"",
     "The budget is 0.011 rad per pass, which is about 0.6 degrees.", "which is"),
    ("a fact bolted on with \"which means\"",
     "The node offers YUYV, which means 10 fps at 720p.", "which means"),
    ("a banned word", "The park arrives precisely where it was told to.", "precisely"),
    ("\"surface\" as a verb", "The checker will surface the disagreement.", "surface"),
    ("bold in the middle of a sentence",
     "The command may never run **more than 0.25 rad** ahead of the arm.",
     "bold inside a sentence"),
    ("a sentence too long to parse in one pass",
     "The commanded angle may not move faster than one radian per second and the "
     "command may never run more than a quarter radian ahead of where the arm actually "
     "is right now, measured from the motors on that very same pass of the loop itself.",
     "SENTENCE of"),
]

#: Healthy prose that must produce NO faults. Each entry names what it is testing.
CLEAN = [
    ("decoration characters, which are navigation in a repo file",
     "⭐⛔⚠️ The arm parks first. Then the playback runs."),
    ("an acronym in capitals", "The CAN bus carries all seven motors. MJPG gives 30 fps."),
    ("deliberate shouting in a repo file", "⛔ NEVER bypass the gripper clamp."),
    ("a bold label on its own line", "**Limit one: how fast the command may change.**"),
    ("a bold label with a trailing colon outside the bold",
     "**How long it takes**: about twenty minutes."),
    ("a bullet list of six things, which is not one long sentence",
     "- which mode the arm is in\n- how to build a smooth path\n- the pause for a grab\n"
     "- the temperature guard\n- the jaw squeeze limit\n- the stall latch"),
    ("a markdown table", "| limit | default |\n|---|---|\n| max speed | 1.0 rad/s |"),
    ("a fenced code block holding dashes and a long line",
     "```\n  a --- diagram line with many many words in it that is not a sentence at all\n```"),
    ("a horizontal rule", "---"),
    ("a flag name containing a double hyphen", "Pass `--yes` to send anything at all."),
    ("his own words quoted back",
     "He said: \"any type of safety would have to go lower than that.\""),
    ("a paragraph after a bullet list, which must not glue onto the last bullet",
     "- the shape of the table\n- whether every number is finite\n\n"
     "The encoding is strict on purpose. The loader calculates where frame k sits."),
]


def main() -> int:
    print("⭐ Every deliberate break must be CAUGHT:\n")
    missed = []
    for what, text, want in BREAKS:
        found = faults(text)
        hit = any(want in line for line in found)
        print(f"  {'✓ caught' if hit else '⛔ MISSED'}: {what}")
        if hit:
            print(f"           → {found[0]}")
        else:
            missed.append(what)
            print(f"           → reported {found or 'nothing'}")

    print("\n⛔ Every healthy example must come back CLEAN:\n")
    cried_wolf = []
    for what, text in CLEAN:
        found = faults(text)
        print(f"  {'✓ left alone' if not found else '⛔ CRIED WOLF'}: {what}")
        for line in found:
            print(f"           → {line}")
        if found:
            cried_wolf.append(what)

    print()
    if missed or cried_wolf:
        if missed:
            print(f"⛔ {len(missed)} of {len(BREAKS)} breaks went uncaught. The checker is "
                  "not measuring what it claims.")
        if cried_wolf:
            print(f"⛔ {len(cried_wolf)} of {len(CLEAN)} healthy examples were reported. "
                  "That is the cry-wolf failure; fix it before anyone learns to skip the output.")
        # ⭐ The count prints on the FAILING path too, because that is when it matters most:
        # `run_falsifiers.py` totals it, and a total that drops is the signal.
        got = (len(BREAKS) - len(missed)) + (len(CLEAN) - len(cried_wolf))
        print(f"CATCHES: {got}/{len(BREAKS) + len(CLEAN)}")
        return 1
    print(f"✓ {len(BREAKS)}/{len(BREAKS)} breaks caught, {len(CLEAN)}/{len(CLEAN)} healthy "
          "examples left alone. Its green runs mean something.")
    print(f"CATCHES: {len(BREAKS) + len(CLEAN)}/{len(BREAKS) + len(CLEAN)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
