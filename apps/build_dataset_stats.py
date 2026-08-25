#!/usr/bin/env python3
"""Write the normalisation statistics the trainer expects (guide C5), from the TRAIN split only.

    uv run apps/build_dataset_stats.py --root recordings/datasets

⭐ WHAT THIS IS. [Setup-Anleitung.md](../docs/Setup-Anleitung.md) C5: the trainer looks for `norm_stats.json` in its cache root, holding z-score statistics (mean and standard deviation) for the states and actions, computed **from your own data**. This reads every exported episode's `states_actions.bin` and writes exactly that, column by column, with the columns named so a mismatch is findable instead of mysterious.

⛔⭐ IT USES THE TRAIN SPLIT ONLY, and that is a correctness point rather than a preference. Statistics computed over the validation episodes too would leak information from the held-out set into the normalisation every training batch sees, which quietly flatters every validation number afterwards. The guide already insists the val split be separate EPISODES from the start; this is the same rule one step downstream.

⛔ IT REFUSES rather than guessing when the train split is empty, and it WARNS loudly when the val split is, because the guide's own words are that a validation split made later out of training frames is worthless.

⚠️ ONE HONESTY POINT WORTH READING: a column that never moved has standard deviation zero, and dividing by it produces infinities downstream. Those columns are floored to `EPS` and **named in the output and in the file**, because a silently floored column turns a motionless joint into pure amplified noise for the model — a plausible-looking dataset that trains on nothing.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

from yam.dataset import (  # noqa: E402
    META_FILE,
    ROW_WIDTH,
    STATES_FILE,
    STD_FLOOR,
    norm_stats,
)
from yam.episode import ROW_COLUMNS  # noqa: E402

def shown(path: Path) -> str:
    """A path printed relative to the repo when it is inside it, absolute when it is not.

    ⚠️ Small but load-bearing: `Path("recordings/datasets").relative_to(REPO)` RAISES, because
    a relative path is not under an absolute one. The first real run of this tool with a
    relative `--root` crashed on exactly that, in a print statement — so the fix belongs in
    one helper rather than at four call sites.
    """
    try:
        return str(path.resolve().relative_to(REPO))
    except ValueError:
        return str(path)


def load_split(split_dir: Path):  # noqa: ANN201
    """Every episode's table in one split, as `(episodes, stacked_rows)`. Refuses on a bad shape."""
    import numpy as np

    episodes, blocks = [], []
    for path in sorted(p for p in split_dir.glob("episode_*") if p.is_dir()):
        states = path / STATES_FILE
        meta_path = path / META_FILE
        if not (states.is_file() and meta_path.is_file()):
            print(f"  ⚠️ {path.name}: missing {STATES_FILE} or {META_FILE} — skipped")
            continue
        steps = int(json.loads(meta_path.read_text())["num_steps"])
        raw = states.read_bytes()
        if len(raw) != steps * ROW_WIDTH * 8:
            raise SystemExit(
                f"⛔ {path.name}: {STATES_FILE} is {len(raw)} bytes but its metadata says "
                f"{steps} steps × {ROW_WIDTH} × 8 = {steps * ROW_WIDTH * 8}. Refusing to "
                "compute statistics over a file whose shape is in doubt — run "
                "`uv run checks/check_dataset.py` first.")
        blocks.append(np.frombuffer(raw, dtype=np.float64).reshape(steps, ROW_WIDTH))
        episodes.append(path.name)
    if not blocks:
        return episodes, None
    return episodes, np.concatenate(blocks, axis=0)


def main() -> int:
    import numpy as np

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default="", help="dataset root holding train/ and val/ "
                                              "(default: recordings/datasets)")
    ap.add_argument("--task-name", default="",
                    help="the language prompt this data is for (guide C5: it must be "
                         "IDENTICAL at training and at deployment). Recorded, not invented")
    args = ap.parse_args()

    # ⭐ Resolved immediately, so every later use (globbing, writing, printing) works the
    # same whether the caller typed a relative or an absolute path.
    root = (Path(args.root).resolve() if args.root
            else REPO / "recordings" / "datasets")
    train_dir, val_dir = root / "train", root / "val"
    if not train_dir.is_dir():
        raise SystemExit(f"⛔ {shown(train_dir)} does not exist. Export episodes into it first:\n"
                         "   uv run apps/export_dataset.py --all --split train --left G "
                         "--right B --top c920 \\\n     --left-wrist d405-260323072846 "
                         "--right-wrist d405-255323071773")

    print(f"reading the TRAIN split: {shown(train_dir)}")
    train_names, train_rows = load_split(train_dir)
    if train_rows is None:
        raise SystemExit(f"⛔ no usable episode in {shown(train_dir)}. Statistics over nothing would "
                         "be a file full of zeros that every training run would trust.")
    val_names, _ = load_split(val_dir) if val_dir.is_dir() else ([], None)

    stats = norm_stats(train_rows)
    mean = np.asarray(stats["mean"])
    std = np.asarray(stats["std"])
    floored = stats["floored_columns"]
    stats.update({
        "episodes": train_names,
        "computed_from": "train split only — statistics that included val would leak the "
                         "held-out set into every training batch",
        "task_name": args.task_name,
    })
    out = root / "norm_stats.json"
    out.write_text(json.dumps(stats, indent=2) + "\n")

    # ⭐ VERIFY THE CONSEQUENCE, not the intention: normalise the train data with what was
    # just written and check it really comes out zero-mean and unit-variance. A statistics
    # file that is subtly wrong produces a model that trains badly and nothing that raises.
    normalised = (train_rows - mean) / std
    live = [i for i in range(ROW_WIDTH) if ROW_COLUMNS[i] not in floored]
    worst_mean = float(np.max(np.abs(normalised[:, live].mean(axis=0)))) if live else 0.0
    worst_std = float(np.max(np.abs(normalised[:, live].std(axis=0) - 1.0))) if live else 0.0

    print(f"\n⭐ WROTE {shown(out)}")
    print(f"   {len(train_names)} train episode(s), {stats['count']} rows, "
          f"{ROW_WIDTH} columns")
    print(f"   verified: worst |mean| after normalising is {worst_mean:.2e}, "
          f"worst |std-1| is {worst_std:.2e}")
    if worst_mean > 1e-9 or worst_std > 1e-9:
        print("   ⛔ those should be ~0. The statistics do not normalise their own data, "
              "which means something is wrong in this tool, not in the data.")
        return 1
    if floored:
        print(f"   ⚠️ {len(floored)} column(s) never moved and were floored to {STD_FLOOR}: "
              f"{', '.join(floored)}")
        print("      A floored column carries no signal, and normalising it amplifies pure "
              "noise.\n      Expected for a jaw that stayed shut or an arm that held still; "
              "worth a look otherwise.")
    if not val_names:
        print(f"\n   ⚠️ the val split ({shown(val_dir)}) is EMPTY. The guide is "
              "blunt about this:\n      a validation split made later out of training frames "
              "is worthless. Record separate\n      episodes and export them with "
              "`--split val` before training for real.")
    else:
        print(f"   ✓ val split holds {len(val_names)} episode(s), and no statistic here "
              "came from them")
    if not args.task_name:
        print("\n   ⚠️ no --task-name given, so the field is empty. Guide C5: it is the "
              "language prompt\n      the policy is conditioned on, and it must be "
              "IDENTICAL at training and deployment.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
