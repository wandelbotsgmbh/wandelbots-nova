from pathlib import Path
from typing import Any

import yaml


def get_module_path() -> Path:
    return Path(__file__).resolve().parent


def get_dataset_path() -> Path:
    return get_module_path() / "dataset"


def demo_raw() -> dict[str, Any]:
    path = get_dataset_path() / "demo_set.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def motion_benchmaker_raw() -> dict[str, Any]:
    path = get_dataset_path() / "mb_set.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def mpinets_raw() -> dict[str, Any]:
    path = get_dataset_path() / "mpinets_set.yaml"
    with open(path) as f:
        return yaml.safe_load(f)
