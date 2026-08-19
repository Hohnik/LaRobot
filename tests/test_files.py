#!/usr/bin/env python3
"""Tests for `yam/files.py` — listing the project's files without the operating system's litter.

    uv run tests/test_files.py

⛔ WHAT THIS IS DEFENDING. On 2026-08-19 the Linux station's `recordings/` held 813 macOS sidecar files, named `._<original>`, because the folder had been hand-copied from the Mac. They caused four separate wrong answers in one session, and the arithmetic of the third one is the reason this file exists: `glob("*.jpg")` matched `._000123.jpg` as well as `000123.jpg`, so every camera's frame count came out at exactly double and the checker reported real data as foreign ([FINDINGS §76](../docs/FINDINGS.md)).
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.files import is_mac_sidecar, is_os_litter, listing, sidecars  # noqa: E402


def a_folder(names: list[str]) -> Path:
    """A temp folder holding empty files with the given names. Sub-paths are created."""
    root = Path(tempfile.mkdtemp())
    for n in names:
        p = root / n
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("")
    return root


def test_a_sidecar_is_recognised():
    assert is_mac_sidecar("._5.json")
    assert is_mac_sidecar(Path("recordings/frames/5/c920/._000123.jpg"))
    assert not is_mac_sidecar("5.json")
    # ⚠️ A leading dot alone is not a sidecar. `.DS_Store` is litter and not an AppleDouble,
    # and the two are reported differently: one is skipped, the other is counted for the
    # operator, because 813 AppleDouble files mean their copy method needs fixing.
    assert not is_mac_sidecar(".DS_Store")


def test_every_dotfile_counts_as_litter():
    for name in ("._5.json", ".DS_Store", ".gitignore", ".#swap"):
        assert is_os_litter(name), name
    assert not is_os_litter("5.json")


def test_the_doubling_that_started_this():
    # 3 real pictures plus their 3 sidecars. The old `glob("*.jpg")` counted 6.
    root = a_folder(["000001.jpg", "000002.jpg", "000003.jpg",
                     "._000001.jpg", "._000002.jpg", "._000003.jpg", "index.json"])
    assert len(listing(root, "*.jpg")) == 3, "a sidecar is not a picture"


def test_a_sidecar_is_never_offered_as_a_recording():
    root = a_folder(["1.json", "5.json", "._5.json", ".DS_Store"])
    stems = [p.stem for p in listing(root, "*.json")]
    assert stems == ["1", "5"], f"got {stems} — '._5' was offered as a playable slot once"


def test_the_listing_is_sorted():
    root = a_folder(["9.json", "1.json", "3.json"])
    assert [p.name for p in listing(root, "*.json")] == ["1.json", "3.json", "9.json"]


def test_a_missing_folder_lists_nothing_instead_of_raising():
    assert listing(Path(tempfile.mkdtemp()) / "nope", "*.json") == []


def test_sidecars_are_found_all_the_way_down():
    # ⭐ Recursive on purpose: the counts that went wrong were three levels down, inside
    # frames/<slot>/<camera>/, not beside the recordings anyone was looking at.
    root = a_folder(["5.json", "._5.json",
                     "frames/5/c920/000001.jpg", "frames/5/c920/._000001.jpg",
                     "frames/5/d405-2603/._000002.jpg"])
    found = {p.name for p in sidecars(root)}
    assert found == {"._5.json", "._000001.jpg", "._000002.jpg"}, found


def test_sidecars_reports_nothing_for_a_clean_folder():
    root = a_folder(["1.json", "frames/1/c920/000001.jpg"])
    assert sidecars(root) == []


def test_the_repos_own_recordings_folder_is_listable():
    # ⭐ Runs against the real folder, because the point of this helper is the real folder.
    # It asserts only that nothing listed is litter, so it passes on a clean checkout too.
    got = listing(REPO / "recordings", "*.json")
    assert all(not is_os_litter(p) for p in got), [p.name for p in got]


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
