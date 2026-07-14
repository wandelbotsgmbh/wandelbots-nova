"""Read execution settings stored in a LeRobot policy checkpoint."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from huggingface_hub import hf_hub_download

_CONFIG_NAME = "config.json"


@dataclass(frozen=True, slots=True)
class LeRobotExecutionSettings:
    """Chunking settings defined by a LeRobot policy checkpoint."""

    policy_type: str
    chunk_size: int
    n_action_steps: int


def load_execution_settings(
    pretrained_name_or_path: str | Path,
) -> LeRobotExecutionSettings:
    """Load chunking settings from a local checkpoint, config file, or Hub model.

    A local directory is expected to contain ``config.json``. A local JSON file
    can be supplied directly. Any other non-absolute string is resolved as a
    Hugging Face model id.

    Server-local absolute paths cannot be inspected by the NOVA client. Supply
    a client-local copy of ``config.json`` or configure both chunking values
    explicitly in that case.
    """
    config_path = _resolve_config_path(pretrained_name_or_path)
    with config_path.open(encoding="utf-8") as config_file:
        config: Any = json.load(config_file)

    if not isinstance(config, dict):
        raise ValueError(f"LeRobot checkpoint config must contain a JSON object: {config_path}")

    policy_type = config.get("type")
    if not isinstance(policy_type, str) or not policy_type:
        raise ValueError(f"LeRobot checkpoint config has no valid 'type': {config_path}")

    chunk_size = _positive_int(config, "chunk_size", config_path)
    n_action_steps = _positive_int(config, "n_action_steps", config_path)
    if n_action_steps > chunk_size:
        raise ValueError(
            "LeRobot checkpoint config has n_action_steps greater than chunk_size: "
            f"{n_action_steps} > {chunk_size} ({config_path})"
        )

    return LeRobotExecutionSettings(
        policy_type=policy_type,
        chunk_size=chunk_size,
        n_action_steps=n_action_steps,
    )


def _resolve_config_path(pretrained_name_or_path: str | Path) -> Path:
    source = Path(pretrained_name_or_path).expanduser()
    if source.is_dir():
        config_path = source / _CONFIG_NAME
        if config_path.is_file():
            return config_path
        raise FileNotFoundError(f"{_CONFIG_NAME} not found in LeRobot checkpoint: {source}")
    if source.is_file():
        return source

    source_text = str(pretrained_name_or_path)
    if source.is_absolute() or source_text.startswith((".", "~")):
        raise FileNotFoundError(
            f"LeRobot checkpoint is not accessible on the client: {source_text}"
        )

    return Path(hf_hub_download(repo_id=source_text, filename=_CONFIG_NAME))


def _positive_int(config: dict[str, Any], key: str, config_path: Path) -> int:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(
            f"LeRobot checkpoint config has no valid positive integer '{key}': {config_path}"
        )
    return value
