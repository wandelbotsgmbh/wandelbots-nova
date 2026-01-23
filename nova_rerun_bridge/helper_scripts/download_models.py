import tempfile
from pathlib import Path
from zipfile import ZipFile

import requests

"""
This script downloads the pre-processed robot models from the wandelbots-js-react-components repository.
These models are already optimized and don't require additional processing.
"""


def get_project_root() -> Path:
    """Get the root directory of the user's project"""
    cwd = Path.cwd()
    markers = ["pyproject.toml", "setup.py", ".git"]

    current = cwd
    while current != current.parent:
        if any((current / marker).exists() for marker in markers):
            return current
        current = current.parent

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


def download_and_extract(version: str, models_dir: Path) -> None:
    """Download and extract pre-processed models directly to models directory"""
    zip_url = f"https://github.com/wandelbotsgmbh/wandelbots-js-react-components/releases/download/v{version}/no-draco-models.zip"
    print(f"Downloading {zip_url}...")

    response = requests.get(zip_url)
    if response.status_code != 200:
        raise Exception(f"Failed to download: {response.status_code}")

    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        zip_path = tmp_path / "models.zip"
        zip_path.write_bytes(response.content)

        with ZipFile(zip_path, "r") as zip_ref:
            zip_ref.extractall(models_dir)


def update_robot_models():
    version = get_latest_release_version()
    models_dir = Path.cwd() / "models"
    current_version = get_current_version(models_dir)

    if version == current_version:
        print("Models are up to date")
        return

    # Create models directory if it doesn't exist
    models_dir.mkdir(parents=True, exist_ok=True)

    # Download and extract pre-processed models
    download_and_extract(version, models_dir)

    # Write version file
    (models_dir / "version.txt").write_text(version)

    print(f"Successfully updated robot models {current_version} ➜ {version}")


if __name__ == "__main__":
    update_robot_models()
