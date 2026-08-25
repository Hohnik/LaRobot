#!/usr/bin/env python3
"""Hold the documents Julien reads at or under their writing-fault ceilings.

    uv run tests/test_prose.py

⭐ WHY THIS IS A TEST AND NOT ONLY A CHECKER. `checks/check_prose.py` has to be run by
someone, and [HANDOFF §4](../docs/HANDOFF.md) rule 8 has now failed three times while being
present and correct. `checks/run_tests.py` collects `tests/test_*.py` and prints one total,
so putting the ceiling assertion here is what makes the standard part of the one command
everybody already runs. `tests/test_unwrap.py` does the same thing for line wrapping.

⚠️ This asserts the CEILINGS only. The rules themselves are proven by
`checks/falsify_check_prose.py`, which feeds the checker known-bad and known-good prose and
counts both the catches and the false alarms.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "checks"))    # ⛔ check scripts are files, not a package

from check_prose import HIS_DOCS, faults  # noqa: E402


def test_every_document_in_the_manifest_exists():
    for rel in HIS_DOCS:
        assert (REPO / rel).is_file(), f"{rel} is in HIS_DOCS and not on disk"


def test_no_document_is_worse_than_its_ceiling():
    worse = []
    for rel, ceiling in HIS_DOCS.items():
        n = len(faults((REPO / rel).read_text()))
        if n > ceiling:
            worse.append(f"{rel}: {n} faults, ceiling {ceiling}")
    assert not worse, ("these got worse; run `uv run checks/check_prose.py -v` for the "
                       f"detail: {'; '.join(worse)}")


def test_the_architecture_doc_is_at_zero_because_it_is_the_reference():
    # ⭐ It was rewritten from scratch on 2026-08-19 after he said he could not read it. It is
    # what "clean" means in this repo, so it is the one file whose ceiling may never rise.
    assert HIS_DOCS["docs/ARCHITECTURE.md"] == 0, (
        "the reference document's ceiling was raised — that is the standard slipping, not "
        "a document changing")
    assert faults((REPO / "docs/ARCHITECTURE.md").read_text()) == []


def test_the_agent_only_documents_are_deliberately_NOT_held_to_this():
    # ⛔ HANDOFF, FINDINGS and ROADMAP are dense on purpose and he does not read them.
    # Adding one to HIS_DOCS is a decision about who it is for, never a tidying step.
    for rel in ("docs/HANDOFF.md", "docs/FINDINGS.md", "docs/ROADMAP.md"):
        assert rel not in HIS_DOCS, f"{rel} is an agent file; see HANDOFF §4 rule 8"


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:  # noqa: PERF203
            failed += 1
            print(f"✗ {fn.__name__}: {e}")
        else:
            print(f"✓ {fn.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
