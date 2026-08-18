#!/usr/bin/env python3
"""Tests for the ±2π gripper frame reconciliation. No hardware, no CAN bus.

    uv run tests/test_gripper_frame.py

⛔ WHY THIS FILE EXISTS, AND WHY IT DID NOT UNTIL 2026-08-14.

`yam_robot.reconcile_gripper_limits()` is the fix for what FINDINGS §11 calls the
worst bug of 2026-08-10, and its failure mode is a **cooked motor**: the SDK
force-clips every gripper command into `[min(limits), max(limits)]` regardless of
where the jaws actually are, so limits written in the wrong 2π frame hold the jaws
against a mechanical stop. 43 °C to 65 °C in five seconds. **Motor 7 was destroyed
three times.**

⛔ It had **no tests**. A safety guard with no test that asserts it can fail is
exactly the pattern FINDINGS §39.4 catalogues, and this is the fourth instance.

⭐ The data below is real and it includes the event that prompted the file. On
2026-08-13 at 18:00 arm G's gripper read `-3.3343` and needed **no** shift. On
2026-08-14 at 09:45 it read `+3.0982` and needed **+2π**. Nothing was recalibrated
in between; the jaws moved about 0.15 rad and the encoder frame moved by a full
turn. Two documents had recorded "G needs no shift" as a property of arm G.
FINDINGS §40.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "apps"))    # ⛔ app and check scripts are files, not packages;
sys.path.insert(0, str(REPO / "checks"))  # a test OF one imports it from its directory
sys.path.insert(0, str(REPO / "scripts"))

from yam.robot import (  # noqa: E402
    frame_correct_gripper_limits,
    reconcile_gripper_limits,
)

TWO_PI = 2 * math.pi
MARGIN = 0.3  # reconcile_gripper_limits' default

# The real calibration, config/gripper_limits.json, order is [closed, open].
LIMITS_B = [6.4810788128480965, 1.2308308537422743]
LIMITS_G = [0.1417181658655675, -5.08640421149004]


def shift_of(saved: list[float], out: list[float]) -> float:
    return out[0] - saved[0]


# ── the real measurements, as regression data ────────────────────────────────


def test_arm_B_has_needed_the_same_minus_2pi_shift_on_both_days() -> None:
    for raw in (0.0158, 0.0154):  # 08-13 18:00, 08-14 09:45
        out = reconcile_gripper_limits(LIMITS_B, raw)
        assert out is not None, raw
        assert abs(shift_of(LIMITS_B, out) + TWO_PI) < 1e-6, f"{raw}: expected -2π, got {shift_of(LIMITS_B, out)}"


def test_arm_G_FLIPPED_from_no_shift_to_plus_2pi_overnight() -> None:
    """⭐⭐ THE TEST THIS FILE EXISTS FOR. The shift is not a property of the arm."""
    yesterday = reconcile_gripper_limits(LIMITS_G, -3.3343)
    today = reconcile_gripper_limits(LIMITS_G, 3.0982)
    assert yesterday is not None and today is not None
    assert abs(shift_of(LIMITS_G, yesterday)) < 1e-6, "08-13 needed no shift"
    assert abs(shift_of(LIMITS_G, today) - TWO_PI) < 1e-6, "08-14 needed +2π"


def test_the_jaws_barely_moved_even_though_the_frame_jumped_a_whole_turn() -> None:
    """The 6.43 rad change in the raw reading is 2π of frame plus 0.15 rad of jaw.
    A 6.43 rad *physical* move is impossible: the whole stroke is only 5.23 rad."""
    def fraction_open(saved: list[float], raw: float) -> float:
        out = reconcile_gripper_limits(saved, raw)
        assert out is not None
        return (raw - out[0]) / (out[1] - out[0])

    before = fraction_open(LIMITS_G, -3.3343)
    after = fraction_open(LIMITS_G, 3.0982)
    assert abs(after - before) < 0.05, f"jaws moved {abs(after-before)*100:.1f}% of the stroke"
    assert abs(3.0982 - (-3.3343)) > abs(LIMITS_G[0] - LIMITS_G[1]), "raw change exceeds the whole stroke"


def test_the_saved_file_on_disk_still_matches_the_values_these_tests_assume() -> None:
    """⛔ Otherwise this file tests a calibration the rig no longer has, which is the
    staleness pattern one layer up. `config/` holds measured evidence (§32.3)."""
    saved = json.loads((REPO / "config" / "gripper_limits.json").read_text())
    assert saved["B"] == LIMITS_B, saved["B"]
    assert saved["G"] == LIMITS_G, saved["G"]


# ── the invariants ───────────────────────────────────────────────────────────


def test_only_one_shift_can_ever_bracket_a_position() -> None:
    """⭐⭐ THE INVARIANT THAT MAKES THE ANSWER TRUSTWORTHY, and it is not free.

    The function tries 0, +2π then −2π and returns the FIRST that brackets. That is
    only safe if at most one ever can. Each shifted band is `stroke + 2*margin`
    wide and consecutive bands are 2π apart, so they stay disjoint while
    `stroke + 2*margin < 2π`. With a 5.25 rad stroke and a 0.3 margin that is
    5.85 < 6.283, leaving 0.43 rad of headroom.

    ⚠️ So a longer-stroke gripper, or a bigger margin, would make two frames overlap.
    This test fires if that day comes.

    ⛔ **UPDATED 2026-08-14: what happens in the overlap changed, so this docstring had
    to.** It used to end *"and the function would silently pick whichever it tried
    first"*, which was true when written. Both reconcilers now REFUSE when more than one
    shift fits, and `test_an_AMBIGUOUS_frame_is_refused_rather_than_resolved_by_list_order`
    pins that. **This test still earns its place**: it guards the precondition (the real
    strokes stay narrow), while the other guards the behaviour if they ever do not.
    """
    for saved in (LIMITS_B, LIMITS_G):
        stroke = abs(saved[0] - saved[1])
        assert stroke + 2 * MARGIN < TWO_PI, (
            f"stroke {stroke:.3f} + 2*{MARGIN} >= 2π: two frames now overlap and "
            "reconcile_gripper_limits would return the first match rather than the right one"
        )
        # And check it empirically across the whole reachable range.
        lo, hi = min(saved), max(saved)
        for i in range(400):
            raw = lo - TWO_PI - 1.0 + i * (2 * TWO_PI + 2.0) / 400
            hits = [s for s in (0.0, TWO_PI, -TWO_PI) if lo + s - MARGIN <= raw <= hi + s + MARGIN]
            assert len(hits) <= 1, f"{raw:.3f} is bracketed by {len(hits)} shifts"


def genuinely_stale(saved: list[float]) -> float:
    """A position that no 2π frame can bracket: the midpoint of the gap between bands.

    ⚠️ Computed rather than guessed, because guessing got it wrong. My first attempt
    used `saved[0] + π`, which the +2π frame brackets perfectly well. The gaps are
    narrow: each band is `stroke + 2*margin` = 5.83 rad wide and they repeat every
    6.283, so only about **0.45 rad in every 6.283, roughly 7%**, fails to reconcile.
    """
    lo, hi = min(saved), max(saved)
    return ((hi + MARGIN) + (lo + TWO_PI - MARGIN)) / 2


def test_stale_limits_return_None_rather_than_warning_and_continuing() -> None:
    """⛔ Warn-and-continue is what burned the motor. The contract is a refusal."""
    assert reconcile_gripper_limits(LIMITS_G, genuinely_stale(LIMITS_G)) is None
    assert reconcile_gripper_limits(LIMITS_B, genuinely_stale(LIMITS_B)) is None


def test_the_refusal_window_is_NARROW_and_that_is_worth_knowing() -> None:
    """⚠️ "It reconciled" is weaker evidence than it sounds. Sample the whole circle:
    the function finds a frame for the large majority of positions, so a successful
    reconciliation means the frame arithmetic worked, NOT that the calibration is
    still good for this gripper. It is a frame check and nothing more."""
    lo, hi = min(LIMITS_G), max(LIMITS_G)
    n, fails = 2000, 0
    for i in range(n):
        raw = lo + i * TWO_PI / n
        if reconcile_gripper_limits(LIMITS_G, raw) is None:
            fails += 1
    share = fails / n
    assert 0.02 < share < 0.15, f"{share*100:.1f}% of positions refuse — expected roughly 7%"


def test_the_returned_order_is_closed_then_open_and_NOT_sorted() -> None:
    """⛔ The SDK reads `gripper_limits` as `[closed, open]` and normalises with
    `(pos - limits[0]) / (limits[1] - limits[0])`. Arm B's saved pair is
    DESCENDING. "Tidying" this function to return `[lo+shift, hi+shift]` would
    invert every normalised jaw reading, so open would report as closed."""
    out = reconcile_gripper_limits(LIMITS_B, 0.0154)
    assert out is not None
    assert out[0] > out[1], "arm B's pair must stay descending"
    assert out[0] == LIMITS_B[0] - TWO_PI


def test_shifting_never_changes_the_stroke_length() -> None:
    for saved, raw in ((LIMITS_B, 0.0154), (LIMITS_G, 3.0982)):
        out = reconcile_gripper_limits(saved, raw)
        assert out is not None
        assert abs(abs(out[0] - out[1]) - abs(saved[0] - saved[1])) < 1e-9


def test_a_position_just_inside_the_margin_is_accepted_and_just_outside_is_not() -> None:
    lo = min(LIMITS_G)
    assert reconcile_gripper_limits(LIMITS_G, lo - 0.29) is not None
    assert reconcile_gripper_limits(LIMITS_G, lo - 0.31) is None


# ── the report the ping prints ───────────────────────────────────────────────


def test_the_report_names_the_shift_and_how_open_the_jaws_are() -> None:
    from ping_motors import gripper_frame_report  # noqa: PLC0415

    text = "\n".join(gripper_frame_report("G", 3.0982))
    assert "+2π" in text, text
    assert "% open" in text, text


def test_the_report_REFUSES_when_no_frame_fits() -> None:
    """⛔ It must say "do not start a session", because the next thing that happens
    otherwise is the gripper being clipped into a range it is not in."""
    from ping_motors import gripper_frame_report  # noqa: PLC0415

    text = "\n".join(gripper_frame_report("G", genuinely_stale(LIMITS_G)))
    assert "DO NOT START A SESSION" in text, text
    assert "calibrate_gripper" in text, text


def test_the_report_warns_when_there_is_almost_no_closing_travel_left() -> None:
    """Arm B's jaws sit at about 3.5% open. That is harmless and it reads as a
    fault if unexpected, which is why it is called out rather than left implicit."""
    from ping_motors import gripper_frame_report  # noqa: PLC0415

    text = "\n".join(gripper_frame_report("B", 0.0154))
    assert "closing travel left" in text, text


def test_an_arm_with_no_saved_limits_says_so_instead_of_guessing() -> None:
    from ping_motors import gripper_frame_report  # noqa: PLC0415

    text = "\n".join(gripper_frame_report("NOPE", 0.0))
    assert "no saved jaw limits" in text, text


# ------------------------------------------- an ambiguous frame must be refused ----


def test_an_AMBIGUOUS_frame_is_refused_rather_than_resolved_by_list_order() -> None:
    """⛔⭐ THE GUARD ADDED 2026-08-14, and it is dormant on the real rig by 0.43 rad.

    Each candidate shift accepts raw positions in a window of `travel + 2·margin`, and the
    candidates sit 2π apart. **A jaw travel wider than `2π − 2·margin` makes two windows
    overlap**, and then both shifts "fit". Picking the first would be picking a jaw SCALE by
    list order, and a wrong scale is what commanded the gripper 2.6 rad past its stop and
    cooked motor 7.

    ⚠️ The real arms measure 5.250 and 5.228 rad of travel, so this cannot happen today.
    That is exactly why it has to be a refusal in the code rather than a sentence in a
    comment: a wider re-calibration would re-introduce the choice silently.
    """
    wide = [3.4, -3.4]            # 6.8 rad of travel, wider than 2π
    # A position both the un-shifted and the +2π windows accept.
    assert frame_correct_gripper_limits(wide, 3.3) is None
    assert reconcile_gripper_limits(wide, 3.3) is None


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except AssertionError as exc:
            failed.append((name, str(exc)))
            print(f"  ✗ {name}\n      {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
