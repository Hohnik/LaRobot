import hashlib
import json
import subprocess
import tarfile
import tempfile
from pathlib import Path

import abc_sim

BASE = "https://abc-data.timehorizons.org"
POLICIES = Path(__file__).resolve().parent / "policies"
SIM_DIR = Path(abc_sim.__file__).resolve().parent


def sha256(path):
    """Compute a file's SHA-256 digest.

    Parameters
    ----------
    path : pathlib.Path
        File to hash.

    Returns
    -------
    Hexadecimal digest.
    ```
    str
    ```
    """
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download(url, path, expected_sha=None):
    """Download a file unless an acceptable local copy exists.

    Parameters
    ----------
    url : str
        Source URL.
    path : pathlib.Path
        Destination file.
    expected_sha : str, optional
        Expected SHA-256 digest; defaults to no checksum check.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and (expected_sha is None or sha256(path) == expected_sha):
        return

    partial = path.with_name(path.name + ".part")
    subprocess.run(
        [
            "curl",
            "--fail",
            "--location",
            "--continue-at",
            "-",
            "--user-agent",
            "abc-prepare/1.0",
            "--output",
            str(partial),
            url,
        ],
        check=True,
    )
    if expected_sha and sha256(partial) != expected_sha:
        partial.unlink()
        raise RuntimeError(f"Checksum mismatch: {path.name}")
    partial.replace(path)


def setup():
    """Prepare the policy checkpoint and required simulation assets.

    Returns
    -------
    Local checkpoint path.
    ```
    pathlib.Path
    ```
    """
    # Policy and matching prompt metadata.
    stem = "abc_dit_xl_200k_model"
    metadata_path = POLICIES / f"{stem}.json"
    download(f"{BASE}/checkpoints/{stem}.json", metadata_path)
    metadata = json.loads(metadata_path.read_text())

    checkpoint = POLICIES / f"{stem}.pt"
    download(metadata["uri"], checkpoint, metadata["sha256"])

    # The manifest ships inside the installed abc_sim package.
    manifest = json.loads((SIM_DIR / "models" / "assets_manifest.json").read_text())
    packages = {p["name"]: p for p in manifest["packages"]}

    # Required specifically for put_plastic_bottles_in_bin.
    for name in ("i2rt_yam", "task_water_bottles"):
        package = packages[name]
        relative = Path(package["extract_dir"]).relative_to("abc_sim")
        destination = SIM_DIR / relative
        marker = destination / f".{name}.sha256"

        if (
            marker.exists()
            and marker.read_text() == package["sha256"]
            and all((destination / p).exists() for p in package["contains"])
        ):
            continue

        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp) / package["archive"]
            download(package["uri"], archive, package["sha256"])
            destination.mkdir(parents=True, exist_ok=True)
            with tarfile.open(archive) as tar:
                tar.extractall(destination, filter="data")
        marker.write_text(package["sha256"])

    return checkpoint


if __name__ == "__main__":
    checkpoint = setup()

    from abc_minimal.config import VizPolicyConfig, VizSimEvalConfig
    from abc_minimal.viz_policy import main

    main(
        VizPolicyConfig(
            sim=VizSimEvalConfig(
                checkpoint=str(checkpoint),
                task="put_plastic_bottles_in_bin",
            ),
            port=8080,
        )
    )
