import subprocess
import tempfile
from pathlib import Path
from zipfile import ZipFile

import requests


def get_project_root() -> Path:
    """Get the root directory of the user's project"""
    # Start from current working directory and look for common project markers
    cwd = Path.cwd()
    markers = ["pyproject.toml", "setup.py", ".git"]

    current = cwd
    while current != current.parent:
        if any((current / marker).exists() for marker in markers):
            return current
        current = current.parent

    # Fallback to cwd if no project root found
    return cwd


def get_current_version(models_dir: Path) -> str:
    version_file = models_dir / "version.txt"
    if version_file.exists():
        return version_file.read_text().strip()
    return ""


def get_latest_release_version() -> str:
    api_url = (
        "https://api.github.com/repos/wandelbotsgmbh/wandelbots-js-react-components/releases/latest"
    )
    response = requests.get(api_url)
    if response.status_code != 200:
        raise Exception(f"Failed to get latest release: {response.status_code}")
    return response.json()["tag_name"].lstrip("v")


def download_and_extract(version: str, tmp_dir: Path) -> Path:
    zip_url = f"https://github.com/wandelbotsgmbh/wandelbots-js-react-components/archive/refs/tags/v{version}.zip"
    print(f"Downloading {zip_url}...")

    response = requests.get(zip_url)
    if response.status_code != 200:
        raise Exception(f"Failed to download: {response.status_code}")

    zip_path = tmp_dir / f"{version}.zip"
    zip_path.write_bytes(response.content)

    with ZipFile(zip_path, "r") as zip_ref:
        zip_ref.extractall(tmp_dir)

    return tmp_dir / f"wandelbots-js-react-components-{version}/public/models"


def check_gltf_transform():
    try:
        subprocess.run(["gltf-transform", "--version"], check=True, capture_output=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "gltf-transform not found. Install with: npm install -g @gltf-transform/cli"
        )


def decompress_glb(input_path: Path, output_path: Path):
    """Process GLB using gltf-transform"""
    temp_path = output_path.with_suffix(".temp.glb")

    # First decompress draco
    subprocess.run(
        ["gltf-transform", "dedup", str(input_path), str(temp_path)],
        check=True,
        capture_output=True,
    )

    # Then convert to unlit
    subprocess.run(
        ["gltf-transform", "unlit", str(temp_path), str(output_path)],
        check=True,
        capture_output=True,
    )

    temp_path.unlink()


def update_robot_models():
    check_gltf_transform()
    version = get_latest_release_version()
    models_dir = Path.cwd() / "models"
    current_version = get_current_version(models_dir)

    if version == current_version:
        print("Models are up to date")
        return

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        new_models_dir = download_and_extract(version, tmp_path)

        # Create models directory if it doesn't exist
        models_dir.mkdir(parents=True, exist_ok=True)

        # Process all GLB files
        for glb_file in new_models_dir.rglob("*.glb"):
            output_path = models_dir / glb_file.name
            print(f"Processing {glb_file.name}...")
            decompress_glb(glb_file, output_path)

        # Write version file
        (models_dir / "version.txt").write_text(version)

        print(f"Successfully updated robot models {current_version} ➜ {version}")


if __name__ == "__main__":
    update_robot_models()
