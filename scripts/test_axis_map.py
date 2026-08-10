#!/usr/bin/env python3
"""Tests for `src/axis_map.py`. No hardware, no simulation, no device.

    uv run scripts/test_axis_map.py

⭐ WHY THESE EXIST AT ALL, in a repo with no other tests. The axis map is the one
piece of this stack whose failure is **silent and plausible**: a wrong entry does
not raise, it moves the arm along an axis nobody asked for, and the natural
diagnosis is "the IK is broken". FINDINGS §0 is a list of nine defects that all
failed that way. Logic that can lie should be checkable without the arm.

⛔ The load-bearing test is `test_backward_compatible_with_hand_dialled_file`.
`config/spacemouse_map.json` was dialled in on real hardware; if this refactor
changed its meaning, the bench time that produced it would be silently thrown
away. So the new code is compared against the **old formula**, not against my
expectation of it.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from axis_map import (  # noqa: E402
    GESTURE_HOLD_S,
    GESTURE_MIN,
    N,
    PUCK_AXES,
    ROBOT_MOTIONS,
    UNBOUND,
    AxisMap,
    AxisMapStore,
    GestureDetector,
    ambiguity_note,
    axes_readout,
    dominant_axis,
    isolate,
    isolated_axes,
)

REAL_MAP_FILE = REPO / "config" / "spacemouse_map.json"


def old_formula(axes: list[float], sign: list[int]) -> np.ndarray:
    """Exactly what `teleop_session.py` did before this change, lines 446-452:

        axes = np.array(reader.read()) * sign
        twist = [axes[0]*ls, axes[1]*ls, axes[2]*ls, axes[3]*as, axes[4]*as, axes[5]*as]

    Scales stripped, since `AxisMap.apply` deliberately returns normalised values.
    """
    return np.asarray(axes, dtype=float) * np.asarray(sign, dtype=float)


# ---------------------------------------------------------------- the map ----


def test_identity_is_a_passthrough() -> None:
    m = AxisMap()
    axes = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    assert np.allclose(m.apply(axes), axes), m.apply(axes)
    assert m.source == [0, 1, 2, 3, 4, 5]
    assert m.sign == [1] * N


# The file's shape before `source` existed, as Julien hand-dialled it on hardware.
# ⛔ PINNED AS A FIXTURE, not read from config/. This test used to load the live
# file and guard that it was still sign-only — which meant its premise expired the
# moment Julien legitimately saved a permutation (he did, 2026-08-10). A test whose
# subject is a file the user edits has a moving target; the backward-compatibility
# property is about the FORMAT and belongs in the test.
LEGACY_FILE_CONTENT = '{"sign": [1, -1, -1, 1, 1, 1]}'


def test_backward_compatible_with_the_legacy_file_shape() -> None:
    """⭐ A sign-only file must still mean exactly what the old code did with it."""
    sign = json.loads(LEGACY_FILE_CONTENT)["sign"]
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "legacy.json"
        p.write_text(LEGACY_FILE_CONTENT)
        m = AxisMap.load(p)

    assert m.source == list(range(N)), f"no 'source' must mean identity, got {m.source}"
    assert m.sign == sign

    rng = np.random.default_rng(20260810)
    for _ in range(500):
        axes = rng.uniform(-1.0, 1.0, N)
        assert np.allclose(m.apply(axes), old_formula(list(axes), sign)), axes


def test_the_live_config_file_loads_and_round_trips() -> None:
    """Whatever Julien has dialled in must load, survive a save/load, and be sane."""
    assert REAL_MAP_FILE.exists(), f"expected {REAL_MAP_FILE} to exist"
    m = AxisMap.load(REAL_MAP_FILE)
    bound = [s for s in m.source if s != UNBOUND]
    assert len(bound) == len(set(bound)), f"live map is not injective: {m.source}"
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        m.save(p)
        assert AxisMap.load(p) == m


def test_missing_and_corrupt_files_fall_back_to_identity() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "nope.json"
        assert AxisMap.load(p) == AxisMap()

        for bad in ("not json at all", "[1,2,3]", '{"sign": [1,2]}', '{"sign": "x"}', "null"):
            p.write_text(bad)
            got = AxisMap.load(p)
            assert got == AxisMap(), f"{bad!r} should degrade to identity, got {got.one_line()}"


def test_out_of_range_entries_become_unbound() -> None:
    m = AxisMap(source=[0, 1, 2, 99, -7, 5], sign=[1, 1, 1, 1, 1, 1])
    assert m.source[3] == UNBOUND
    assert m.source[4] == UNBOUND
    assert m.source[5] == 5


def test_unbound_motion_contributes_nothing() -> None:
    m = AxisMap()
    m.unbind(2)
    out = m.apply([1.0] * N)
    assert out[2] == 0.0
    assert all(out[i] == 1.0 for i in (0, 1, 3, 4, 5))
    assert m.unbound() == [2]


def test_bind_makes_the_gesture_positive() -> None:
    """The prompt asks for a direction; the gesture must produce it."""
    for value, expect_sign in ((0.8, 1), (-0.8, -1)):
        m = AxisMap()
        m.bind(2, 1, value)              # UP  <-  puck y
        assert m.source[2] == 1
        assert m.sign[2] == expect_sign
        # Repeating the same gesture must now drive UP positively.
        axes = np.zeros(N)
        axes[1] = value
        assert m.apply(axes)[2] > 0, f"gesture {value} should drive UP positive"


def test_bind_displaces_the_previous_owner_and_stays_injective() -> None:
    m = AxisMap()                        # identity: puck y drives Y
    displaced = m.bind(2, 1, 0.9)        # give puck y to UP instead
    assert displaced == 1, displaced
    assert m.source[1] == UNBOUND
    assert m.source[2] == 1
    bound = [s for s in m.source if s != UNBOUND]
    assert len(bound) == len(set(bound)), f"map is not injective: {m.source}"


def test_rebinding_the_same_motion_displaces_nothing() -> None:
    m = AxisMap()
    assert m.bind(2, 2, 0.9) is None
    assert m.source == list(range(N))


def test_full_permutation_applies_correctly() -> None:
    # puck z drives X, puck x drives Y, puck y drives UP; rotations untouched.
    m = AxisMap(source=[2, 0, 1, 3, 4, 5], sign=[1, -1, 1, 1, 1, 1])
    axes = [0.10, 0.20, 0.30, 0.0, 0.0, 0.0]
    out = m.apply(axes)
    assert np.isclose(out[0], 0.30), out          # X   <- puck z (+)
    assert np.isclose(out[1], -0.10), out         # Y   <- puck x (−)
    assert np.isclose(out[2], 0.20), out          # UP  <- puck y (+)


def test_swap_exchanges_both_controls() -> None:
    """Julien's case: ROLL and PITCH in each other's places. One key fixes both."""
    m = AxisMap(source=[0, 1, 2, 4, 3, 5], sign=[1, 1, -1, 1, 1, -1])   # his live map
    m.swap(3, 4)                                                         # ROLL ↔ PITCH
    assert m.source[3] == 3, m.source        # ROLL  now driven by puck roll
    assert m.source[4] == 4, m.source        # PITCH now driven by puck pitch
    bound = [s for s in m.source if s != UNBOUND]
    assert len(bound) == len(set(bound)), f"not injective: {m.source}"
    assert UNBOUND not in m.source, "a swap must never orphan a motion"


def test_swap_is_an_involution() -> None:
    """Doing it twice restores the original — a mistaken swap undoes itself."""
    original = AxisMap(source=[1, 0, 2, 4, 3, 5], sign=[1, 1, -1, 1, 1, -1])
    m = original.copy()
    for a, b in ((0, 2), (3, 5), (1, 4), (0, 0)):
        m.swap(a, b)
        m.swap(a, b)
        assert m == original, f"swap({a},{b}) twice changed the map: {m.one_line()}"


def test_swap_carries_the_sign_with_the_axis() -> None:
    m = AxisMap(source=[0, 1, 2, 3, 4, 5], sign=[1, -1, 1, 1, 1, 1])
    m.swap(0, 1)
    assert m.source[0] == 1 and m.sign[0] == -1, (m.source, m.sign)
    assert m.source[1] == 0 and m.sign[1] == 1, (m.source, m.sign)


def test_swap_with_an_unbound_motion_moves_the_hole() -> None:
    m = AxisMap()
    m.unbind(5)                              # YAW drives nothing
    m.swap(0, 5)                             # give YAW what X had
    assert m.source[5] == 0, m.source
    assert m.source[0] == UNBOUND, m.source  # ...and X is now the empty one
    assert m.unbound() == [0]


def test_swap_with_itself_is_a_no_op() -> None:
    m = AxisMap(source=[1, 0, 2, 4, 3, 5], sign=[1, 1, -1, 1, 1, -1])
    before = m.copy()
    m.swap(2, 2)
    assert m == before


def test_swap_never_loses_a_control() -> None:
    """Whatever was bound before is still bound after, for every pair."""
    for a in range(N):
        for b in range(N):
            m = AxisMap(source=[1, 0, 2, 4, 3, 5], sign=[1, 1, -1, 1, 1, -1])
            before = sorted(m.source)
            m.swap(a, b)
            assert sorted(m.source) == before, (a, b, m.source)


def test_flip_inverts_one_motion_only() -> None:
    m = AxisMap()
    m.flip(4)
    out = m.apply([1.0] * N)
    assert out[4] == -1.0
    assert all(out[i] == 1.0 for i in (0, 1, 2, 3, 5))


def test_save_load_round_trip() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        m = AxisMap(source=[5, UNBOUND, 1, 0, 4, 2], sign=[1, -1, -1, 1, -1, 1])
        m.save(p)
        assert AxisMap.load(p) == m
        raw = json.loads(p.read_text())
        assert raw["source"] == m.source and raw["sign"] == m.sign


def test_saving_the_real_file_preserves_its_behaviour() -> None:
    """Adding `source` to the file must not change what it does."""
    before = AxisMap.load(REAL_MAP_FILE)
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        before.save(p)
        after = AxisMap.load(p)
    rng = np.random.default_rng(7)
    for _ in range(200):
        axes = rng.uniform(-1, 1, N)
        assert np.allclose(before.apply(axes), after.apply(axes))


# ------------------------------------------------------------- detection ----


def test_clear_push_is_detected() -> None:
    axes = [0.0, 0.0, -0.90, 0.05, 0.0, 0.0]
    hit = dominant_axis(axes)
    assert hit is not None
    assert hit[0] == 2 and hit[1] < 0, hit


def test_gentle_push_is_refused() -> None:
    axes = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
    assert dominant_axis(axes) is None
    assert "too gentle" in (ambiguity_note(axes) or "")


def test_crosstalk_is_refused_not_guessed() -> None:
    """A push that also produces pitch must not silently bind either one."""
    axes = [0.0, 0.60, 0.0, 0.0, 0.45, 0.0]
    assert dominant_axis(axes) is None
    note = ambiguity_note(axes) or ""
    assert "ambiguous" in note and "pitch" in note, note


def test_dominance_boundary() -> None:
    assert dominant_axis([0.90, 0.30, 0, 0, 0, 0]) is not None   # 3.0x  -> clear
    assert dominant_axis([0.90, 0.40, 0, 0, 0, 0]) is None       # 2.25x -> ambiguous


def test_silence_produces_no_note() -> None:
    assert ambiguity_note([0.0] * N) is None


def test_detector_requires_the_hold_time() -> None:
    d = GestureDetector()
    axes = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
    assert d.feed(axes, 0.00) is None
    assert d.feed(axes, GESTURE_HOLD_S * 0.5) is None
    got = d.feed(axes, GESTURE_HOLD_S + 0.001)
    assert got is not None and got[0] == 2, got


def test_detector_resets_when_the_axis_changes() -> None:
    d = GestureDetector()
    a = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
    b = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
    d.feed(a, 0.00)
    d.feed(b, 0.05)                      # switched axis: the clock restarts
    assert d.feed(b, 0.05 + GESTURE_HOLD_S * 0.5) is None
    got = d.feed(b, 0.05 + GESTURE_HOLD_S + 0.001)
    assert got is not None and got[0] == 0, got


def test_detector_resets_when_the_puck_is_released() -> None:
    d = GestureDetector()
    a = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
    d.feed(a, 0.00)
    assert d.feed([0.0] * N, 0.05) is None
    assert d.feed(a, 0.10) is None       # must start over, not carry the old clock
    assert d.feed(a, 0.10 + GESTURE_HOLD_S + 0.001) is not None


def test_detector_keeps_the_firmest_deflection_for_the_sign() -> None:
    d = GestureDetector()
    d.feed([0, 0, -0.40, 0, 0, 0], 0.00)
    d.feed([0, 0, -0.95, 0, 0, 0], 0.05)
    d.feed([0, 0, -0.36, 0, 0, 0], 0.10)
    got = d.feed([0, 0, -0.36, 0, 0, 0], GESTURE_HOLD_S + 0.001)
    assert got is not None
    assert np.isclose(abs(got[1]), 0.95), got


# ----------------------------------------------------------- presentation ----


def test_metadata_is_complete_and_consistent() -> None:
    assert len(ROBOT_MOTIONS) == N and len(PUCK_AXES) == N
    for m in ROBOT_MOTIONS:
        assert {"short", "long", "world", "note"} <= set(m)
        assert m["note"], m


def test_describe_names_unbound_motions() -> None:
    m = AxisMap()
    m.unbind(5)
    text = m.describe()
    assert "UNBOUND" in text and "YAW" in text
    assert "nothing" in text
    assert "YAW←" not in m.one_line()


def test_readout_shows_every_axis() -> None:
    text = axes_readout([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    for name in PUCK_AXES:
        assert name in text
    assert "+0.50" in text and "-0.50" in text


# ------------------------------------------------- CONTROLS mode primitives ----


# ---------------------------------------------------- per-arm map store ----


def test_store_defaults_to_one_shared_map() -> None:
    s = AxisMapStore()
    assert s.for_arm("arm1") is s.shared
    assert s.for_arm("arm2") is s.shared
    assert s.is_shared("arm1") and s.is_shared("arm2")
    assert "BOTH" in s.scope_note("arm1")


def test_store_reads_a_legacy_flat_file_as_the_shared_map() -> None:
    """⛔ His hand-dialled file must not be lost by introducing per-arm scope."""
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        p.write_text(LEGACY_FILE_CONTENT)
        s = AxisMapStore.load(p)
        assert s.per_arm == {}
        assert s.shared.sign == [1, -1, -1, 1, 1, 1]
        assert s.shared.source == list(range(N))
        assert s.for_arm("arm2").sign == [1, -1, -1, 1, 1, 1]


def test_store_reads_the_current_nested_shape_too() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        p.write_text(json.dumps({
            "shared": {"source": [0, 1, 2, 3, 4, 5], "sign": [1] * N},
            "arm2": {"source": [1, 0, 2, 3, 4, 5], "sign": [1, -1, 1, 1, 1, 1]},
        }))
        s = AxisMapStore.load(p)
        assert s.is_shared("arm1")
        assert not s.is_shared("arm2")
        assert s.for_arm("arm2").source == [1, 0, 2, 3, 4, 5]
        assert "arm2 ONLY" in s.scope_note("arm2")


def test_store_editing_a_shared_map_affects_both_arms() -> None:
    """The blast radius that scope_note() exists to announce."""
    s = AxisMapStore()
    m = s.for_arm("arm1")
    m.flip(2)
    s.set("arm1", m)
    assert s.for_arm("arm2").sign[2] == -1, "shared means shared — this is the point of the warning"


def test_store_fork_isolates_one_arm() -> None:
    s = AxisMapStore()
    s.fork("arm2")
    assert not s.is_shared("arm2") and s.is_shared("arm1")
    m = s.for_arm("arm2")
    m.flip(2)
    s.set("arm2", m)
    assert s.for_arm("arm2").sign[2] == -1
    assert s.for_arm("arm1").sign[2] == 1, "forking must stop arm2 edits reaching arm1"


def test_store_fork_seeds_from_what_the_arm_already_used() -> None:
    s = AxisMapStore(shared=AxisMap(source=[1, 0, 2, 3, 4, 5], sign=[1, -1, 1, 1, 1, 1]))
    s.fork("arm2")
    assert s.for_arm("arm2").source == [1, 0, 2, 3, 4, 5]
    assert s.for_arm("arm2").sign == [1, -1, 1, 1, 1, 1]


def test_store_unfork_returns_to_shared() -> None:
    s = AxisMapStore()
    s.fork("arm2")
    s.unfork("arm2")
    assert s.is_shared("arm2")
    assert s.for_arm("arm2") is s.shared


def test_store_fork_is_idempotent_and_unfork_is_safe() -> None:
    s = AxisMapStore()
    s.fork("arm2")
    before = s.copy()
    s.fork("arm2")
    assert s == before
    s.unfork("arm1")          # never forked; must not raise
    assert s == before


def test_store_round_trips_with_overrides() -> None:
    s = AxisMapStore(
        shared=AxisMap(source=[1, 0, 2, 4, 3, 5], sign=[1, 1, -1, 1, 1, -1]),
        per_arm={"arm2": AxisMap(source=[0, 1, 2, 3, 4, 5], sign=[-1, -1, 1, 1, 1, 1])},
    )
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        s.save(p)
        assert AxisMapStore.load(p) == s


def test_store_survives_a_corrupt_file() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        for bad in ("nonsense", "[1,2]", "null", '{"shared": 5}', '{"arm2": "x"}'):
            p.write_text(bad)
            s = AxisMapStore.load(p)
            assert s.for_arm("arm1") == AxisMap(), bad


def test_store_saving_the_live_file_preserves_every_arm() -> None:
    s = AxisMapStore.load(REAL_MAP_FILE)
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "m.json"
        s.save(p)
        after = AxisMapStore.load(p)
    for arm in ("arm1", "arm2"):
        rng = np.random.default_rng(11)
        for _ in range(100):
            axes = rng.uniform(-1, 1, N)
            assert np.allclose(s.for_arm(arm).apply(axes), after.for_arm(arm).apply(axes)), arm


def test_isolate_keeps_only_the_strongest_axis() -> None:
    axes = [0.10, 0.62, 0.05, 0.0, 0.20, 0.0]
    keep, value = isolate(axes)
    assert keep == 1 and np.isclose(value, 0.62), (keep, value)
    out = isolated_axes(axes, keep)
    assert np.isclose(out[1], 0.62)
    assert np.count_nonzero(out) == 1, out


def test_isolate_preserves_the_sign() -> None:
    keep, value = isolate([0.0, 0.0, -0.80, 0.0, 0.0, 0.0])
    assert keep == 2 and value < 0, (keep, value)


def test_isolate_returns_nothing_when_the_puck_is_centred() -> None:
    keep, value = isolate([0.0] * N)
    assert keep is None and value == 0.0
    assert np.count_nonzero(isolated_axes([0.0] * N, None)) == 0


def test_isolate_hysteresis_prevents_flicker() -> None:
    """Two near-equal axes must not make the arm jitter between two motions."""
    # y is marginally stronger, but x is the incumbent -> keep x.
    keep, _ = isolate([0.50, 0.55, 0, 0, 0, 0], current=0)
    assert keep == 0, keep
    # A decisive win takes over.
    keep, _ = isolate([0.50, 0.90, 0, 0, 0, 0], current=0)
    assert keep == 1, keep


def test_isolate_with_no_incumbent_is_plain_argmax() -> None:
    keep, _ = isolate([0.50, 0.55, 0, 0, 0, 0], current=None)
    assert keep == 1, keep


def test_isolate_never_edits_the_map() -> None:
    """⛔ The regression that destroyed the hand-dialled file: deflection must be
    observation only. isolate() is a pure function over axes — it takes no map."""
    import inspect

    sig = inspect.signature(isolate)
    assert "axes" in sig.parameters
    assert not any("map" in p.lower() for p in sig.parameters), sig


def test_motion_driven_by_is_the_reverse_lookup() -> None:
    m = AxisMap(source=[2, 0, 1, 3, 4, 5], sign=[1] * N)
    assert m.motion_driven_by(2) == 0      # puck z drives X
    assert m.motion_driven_by(0) == 1      # puck x drives Y
    assert m.motion_driven_by(1) == 2      # puck y drives UP
    m.unbind(2)
    assert m.motion_driven_by(1) is None   # puck y now drives nothing


def test_flip_via_reverse_lookup_reverses_that_control() -> None:
    """The f key: push an axis, press f, the same push must now go the other way."""
    m = AxisMap.load(REAL_MAP_FILE)
    axes = np.zeros(N)
    axes[1] = 0.7                                   # push puck y
    before = m.apply(axes)
    motion = m.motion_driven_by(1)
    assert motion is not None
    m.flip(motion)
    after = m.apply(axes)
    assert np.isclose(after[motion], -before[motion]), (before, after)
    assert not np.isclose(before[motion], 0.0), "the test axis must actually drive something"


def test_reassign_uses_the_last_push_direction_as_positive() -> None:
    """The 1-6 keys: 'the way I was just pushing' becomes that motion's positive."""
    m = AxisMap()
    m.bind(2, 1, -0.8)                              # was pushing puck y negative
    axes = np.zeros(N)
    axes[1] = -0.8
    assert m.apply(axes)[2] > 0, "repeating that push must drive UP positive"


def test_gesture_min_clears_the_deadzone() -> None:
    """The threshold must be a deliberate push, not a resting wobble."""
    from spacemouse import DEFAULT_DEADZONE  # noqa: PLC0415

    assert GESTURE_MIN > DEFAULT_DEADZONE * 3, (GESTURE_MIN, DEFAULT_DEADZONE)


def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ✓ {name}")
        except AssertionError as exc:
            failed.append((name, f"assertion: {exc}"))
            print(f"  ✗ {name}\n      {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((name, f"{type(exc).__name__}: {exc}"))
            print(f"  ✗ {name}\n      {type(exc).__name__}: {exc}")

    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    if failed:
        print("\nFAILED:")
        for name, why in failed:
            print(f"  {name}: {why}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
