"""Write down everything known at the moment a session stopped badly.

⛔⭐⭐ WHY THIS EXISTS, and it is one specific event.

On 2026-08-14 the arm fell mid-session: motor 5 stopped answering the CAN bus, I2RT's
control thread exited, and the arm sagged. **Everything about the moment of failure was
then gone.** The console had the pose and the loop rate, and nothing else survived —
no torques, no temperatures at that instant, and no record of the USB bus, which is
where the leading explanation lies (FINDINGS §44.3: both CAN adapters ended up in their
bootloader behind a shared hub, and one camera stopped reporting its serial).

Reconstructing it took a simulation of his joint angles to recover the gravity torques
that the arm had already measured and thrown away. **That is a bad trade, and this file
is the fix.** FINDINGS §35.5 records that the underlying fault will recur, so the next
occurrence should be cheap to diagnose rather than expensive.

⭐⭐ ONE HARD RULE, AND THE DESIGN FOLLOWS FROM IT

**Nothing here may ever delay or prevent the motors being disabled.** A crash report
that interferes with the teardown is worse than no crash report, on a rig with no
emergency stop. So:

- every field is gathered inside its own `try`, and a failure records a note instead of
  raising;
- `write_incident()` never raises, and its return value says whether it wrote;
- the caller invokes it **after** `shutdown_robot()`, so the motors are already off.

⚠️ It writes into `recordings/incidents/`, which is gitignored like the rest of
`recordings/`. That is deliberate: an incident file can contain a full pose history and
it is evidence rather than source. **Paste the file, do not commit it.**
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent
INCIDENT_DIR = REPO / "recordings" / "incidents"


def _safe(label: str, fn) -> Any:  # noqa: ANN001
    """Call `fn`, and record why it failed rather than letting it raise.

    ⛔ This is the whole safety property. A dying chain makes half these reads throw:
    `get_joint_pos()` on a stopped chain raises, reading a USB string descriptor on a
    claimed device raises, and the thermal read may already be blind. **Any of those
    must produce a note in the file, never an exception on the shutdown path.**
    """
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001
        return f"<unavailable: {type(exc).__name__}: {exc}>"


def usb_snapshot() -> Any:
    """Every USB device, with bus, address, ids and serial.

    ⭐ The single most valuable field, and the one that was missing. FINDINGS §32 has
    asked since 2026-08-13 for the bus state at the moment a DFU fault appears, and
    nobody had captured it *during* a failure. FINDINGS §44.3 had to reconstruct the
    topology afterwards to notice that both adapters share a hub chain with a dock.
    """
    try:
        import usb.core
    except Exception as exc:  # noqa: BLE001
        return f"<pyusb unavailable: {type(exc).__name__}: {exc}>"

    out = []
    for dev in usb.core.find(find_all=True) or []:
        entry = {}
        for name, getter in (
            ("bus", lambda d=dev: d.bus),
            ("addr", lambda d=dev: d.address),
            ("vid", lambda d=dev: f"0x{d.idVendor:04x}"),
            ("pid", lambda d=dev: f"0x{d.idProduct:04x}"),
            ("product", lambda d=dev: str(d.product or "")),
            ("serial", lambda d=dev: str(d.serial_number or "")),
        ):
            entry[name] = _safe(name, getter)
        out.append(entry)
    return out


def write_incident(reason: str, facts: dict, *, directory: Path | None = None) -> Path | None:
    """Write one incident file. Returns its path, or `None` if it could not write.

    ⛔ Never raises. The caller is on the shutdown path.
    """
    try:
        from provenance import dt_now, git_commit
    except Exception:  # noqa: BLE001
        def dt_now() -> str:  # type: ignore[misc]
            return "unknown"

        def git_commit() -> str:  # type: ignore[misc]
            return "unknown"

    try:
        folder = directory or INCIDENT_DIR
        folder.mkdir(parents=True, exist_ok=True)
        stamp = _safe("time", dt_now)
        payload = {
            "reason": reason,
            "at": stamp,
            "commit": _safe("commit", git_commit),
            "note": "Written by src/incident.py when a session stopped badly. Everything "
                    "here was read AFTER the motors were disabled, so a field may be "
                    "missing if the chain had already gone. See docs/FINDINGS.md §45.",
            **facts,
            "usb": usb_snapshot(),
        }
        safe_stamp = "".join(c if c.isalnum() else "-" for c in str(stamp))
        path = folder / f"{safe_stamp}.json"
        path.write_text(json.dumps(payload, indent=2, default=str) + "\n")
        return path
    except Exception:  # noqa: BLE001
        return None


def describe(path: Path | None) -> str:
    """One line for the operator. ⭐ It has to say what to DO with the file."""
    if path is None:
        return "⚠️  could not write an incident file (the failure is only in the console above)"
    try:
        rel = path.relative_to(REPO)
    except ValueError:
        rel = path
    return (f"⭐ incident recorded → {rel}\n"
            f"   It holds the pose, the limits, the temperatures, the loop rate and the full\n"
            f"   USB bus at the moment this stopped. PASTE IT rather than committing it;\n"
            f"   recordings/ is gitignored on purpose.")
