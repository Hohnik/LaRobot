#!/usr/bin/env python3
"""Inspect interrupted slot saves and retained frames without changing files."""
import argparse
import json
from pathlib import Path
import sys

from yam.recording_recovery import inspect_recovery


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", type=Path, default=Path("recordings"),
                        help="one recording directory; inspect recordings/sim separately")
    parser.add_argument("--json", action="store_true", help="print structured evidence")
    args = parser.parse_args()
    try:
        report = inspect_recovery(args.dir)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"{args.dir}: {len(report['published'])} published JSON file(s), "
              f"{len(report['interrupted_saves'])} interrupted save directory(s), "
              f"{len(report['retained_frames'])} unclaimed/pending frame entry(s)")
        for name, summary in report['published'].items():
            if summary['state'] != 'readable':
                print(f"  {name}: {summary}")
        for entry in report['interrupted_saves'] + report['retained_frames']:
            print(json.dumps(entry, indent=2))
        print(report['limits'])
        print("Inspection needed; preserve these files." if report['needs_inspection']
              else "No interrupted-save or retained-frame evidence found in this directory.")
    return int(report['needs_inspection'])


if __name__ == '__main__':
    sys.exit(main())
