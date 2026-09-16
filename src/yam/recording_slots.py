"""Recording selection policy shared by export applications.

Exports prefer real recordings; operator simulation deliberately has the opposite
precedence and does not use this resolver. Selection never modifies a recording.
"""
from pathlib import Path


def find_export_slot(recordings_dir: Path, slot: str, *, emit=print) -> Path:
    """Find a real slot, falling back to simulation with an explicit warning.

    Raise FileNotFoundError when neither exists. Callers translate it to their CLI
    error. Pass a silent emitter when inventorying a batch before actual export.
    """
    root = Path(recordings_dir)
    real, sim = root / f"{slot}.json", root / "sim" / f"{slot}.json"
    if real.is_file():
        if sim.is_file():
            emit(f"  ⚠️ slot {slot} exists both real and simulated — exporting the REAL one.")
        return real
    if sim.is_file():
        emit(f"  ⚠️ slot {slot} is a SIMULATED recording — fine for pipeline tests, "
             "never for training.")
        return sim
    raise FileNotFoundError(
        f"⛔ nothing saved in slot {slot} (checked recordings/ and recordings/sim/).")
