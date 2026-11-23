"""Read execution settings stored in a LeRobot policy checkpoint.

Read through LeRobot's ``PreTrainedConfig``; the chunk length is ``len(action_delta_indices)``
for every policy type (ADR 0014).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any

from lerobot.configs.types import FeatureType

if TYPE_CHECKING:
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.configs.types import PolicyFeature

logger = logging.getLogger(__name__)

_CONFIG_NAME = "config.json"
_PREPROCESSOR_NAME = "policy_preprocessor.json"
_NORMALIZER_STEP = "normalizer_processor"
# LeRobot's registered name for the step that tokenizes the language instruction; a
# checkpoint whose pipeline contains it reads ``observation["task"]``.
_TOKENIZER_STEP = "tokenizer_processor"

# Width of the plausible band derived from mean/std statistics.
_STD_SIGMA = 4.0

# Feature types the NOVA observation builder can produce; others are carried, not demanded.
_COMPARABLE_FEATURE_TYPES = (FeatureType.STATE, FeatureType.VISUAL)


@dataclass(frozen=True, slots=True)
class FeatureStats:
    """Per-dimension training statistics for one feature: min/max or mean/std, by mode."""

    minimum: tuple[float, ...] | None = None
    maximum: tuple[float, ...] | None = None
    mean: tuple[float, ...] | None = None
    std: tuple[float, ...] | None = None

    def bounds(self, index: int) -> tuple[float, float] | None:
        """Plausible ``(low, high)`` for one dimension, or ``None`` if unknowable."""
        if self.minimum is not None and self.maximum is not None:
            if index < len(self.minimum) and index < len(self.maximum):
                return self.minimum[index], self.maximum[index]
            return None
        if self.mean is not None and self.std is not None:
            if index >= len(self.mean) or index >= len(self.std):
                return None
            spread = self.std[index] * _STD_SIGMA
            if spread <= 0:
                return None
            return self.mean[index] - spread, self.mean[index] + spread
        return None


@dataclass(frozen=True, slots=True)
class LeRobotExecutionSettings:
    """Execution settings and the feature contract of a LeRobot checkpoint.

    Attributes:
        policy_type: Checkpoint ``type``, such as ``"act"``.
        chunk_size: Actions the policy predicts per inference, or ``None`` for a
            policy that declares no action chunk at all.
        n_action_steps: Actions the checkpoint intends to execute per chunk, or
            ``None`` when the policy does not declare one.
        input_features: Observation features the policy expects. Empty when the
            checkpoint infers them from the dataset at train time.
        output_features: Action features the policy produces.
        requires_task: Whether the checkpoint conditions on a language instruction
            (``False`` without a ``policy_preprocessor.json``).
    """

    policy_type: str
    chunk_size: int | None = None
    n_action_steps: int | None = None
    input_features: dict[str, PolicyFeature] = field(default_factory=dict)
    output_features: dict[str, PolicyFeature] = field(default_factory=dict)
    stats: dict[str, FeatureStats] = field(default_factory=dict)
    """Training statistics per feature, when the checkpoint's preprocessor is readable."""
    requires_task: bool = False

    @property
    def comparable_input_features(self) -> dict[str, PolicyFeature]:
        """Input features a ``PolicySchema`` can be held against."""
        return {
            key: feature
            for key, feature in self.input_features.items()
            if feature.type in _COMPARABLE_FEATURE_TYPES
        }

    @property
    def image_shapes(self) -> dict[str, tuple[int, ...]]:
        """Channel-first shapes of every declared visual input, keyed by feature."""
        return {
            key: feature.shape
            for key, feature in self.input_features.items()
            if feature.type is FeatureType.VISUAL
        }


def load_execution_settings(
    pretrained_name_or_path: str | Path,
) -> LeRobotExecutionSettings:
    """Load execution settings from a checkpoint directory, its ``config.json``, or a Hub id.

    A JSON file under another name is not accepted; the path must exist on this machine.

    Raises:
        FileNotFoundError: The checkpoint is not reachable from this machine.
        ImportError: The policy's config needs a package this environment lacks.
        ValueError: LeRobot rejected the checkpoint config.
    """
    source = _resolve_source(pretrained_name_or_path)
    settings = _from_policy_config(_load_policy_config(source))
    steps = _preprocessor_steps(source)
    return replace(
        settings,
        stats=_load_stats(source, steps),
        requires_task=any(step.get("registry_name") == _TOKENIZER_STEP for step in steps),
    )


def try_load_execution_settings(
    pretrained_name_or_path: str | Path | None,
) -> LeRobotExecutionSettings | None:
    """Load execution settings, or ``None`` when the checkpoint cannot be read here.

    A checkpoint LeRobot rejects still raises.
    """
    if pretrained_name_or_path is None:
        return None
    try:
        return load_execution_settings(pretrained_name_or_path)
    except (FileNotFoundError, OSError) as exc:
        logger.debug("LeRobot checkpoint is not readable by the client: %s", exc)
        return None
    except ImportError as exc:
        logger.warning(
            "LeRobot cannot load this checkpoint's config in this environment: %s. "
            "Install the policy's extra dependencies to derive settings from it.",
            exc,
        )
        return None


def _resolve_source(pretrained_name_or_path: str | Path) -> str | Path:
    """Resolve to a directory or Hub id: a ``config.json`` path maps to its directory."""
    source = Path(pretrained_name_or_path).expanduser()
    if source.is_dir():
        return source
    if source.is_file():
        # Only config.json maps to its directory; any other file would read the one beside it.
        if source.name != _CONFIG_NAME:
            msg = (
                f"LeRobot checkpoint must be a directory or its {_CONFIG_NAME}, "
                f"not {source.name!r}: {source}"
            )
            raise FileNotFoundError(msg)
        return source.parent

    source_text = str(pretrained_name_or_path)
    if source.is_absolute() or source_text.startswith((".", "~")):
        raise FileNotFoundError(
            f"LeRobot checkpoint is not accessible on the client: {source_text}"
        )
    return source_text


def _load_policy_config(source: str | Path) -> PreTrainedConfig:
    """Load the checkpoint's typed LeRobot config (imported lazily: it loads torch)."""
    # Importing the policies package registers the config choices draccus resolves ``type`` by.
    from draccus.utils import ParsingError  # ruff: ignore[import-outside-top-level]
    from lerobot.configs.policies import PreTrainedConfig  # ruff: ignore[import-outside-top-level]
    import lerobot.policies  # ruff: ignore[import-outside-top-level,unused-import]

    if isinstance(source, Path) and not (source / _CONFIG_NAME).is_file():
        raise FileNotFoundError(f"{_CONFIG_NAME} not found in LeRobot checkpoint: {source}")

    try:
        return PreTrainedConfig.from_pretrained(source)
    except ParsingError as exc:
        # Keep draccus' exception type out of this module's contract.
        msg = f"LeRobot rejected the checkpoint config at {source}: {exc}"
        raise ValueError(msg) from exc


def _from_policy_config(config: PreTrainedConfig) -> LeRobotExecutionSettings:
    action_indices = config.action_delta_indices
    n_action_steps = getattr(config, "n_action_steps", None)

    return LeRobotExecutionSettings(
        policy_type=config.type,
        chunk_size=len(action_indices) if action_indices else None,
        n_action_steps=n_action_steps if isinstance(n_action_steps, int) else None,
        input_features=dict(config.input_features or {}),
        output_features=dict(config.output_features or {}),
    )


def _checkpoint_file(source: str | Path, filename: str) -> Path | None:
    """Locate one file in the checkpoint, downloading it for a Hub id; ``None`` if absent."""
    if isinstance(source, Path):
        candidate = source / filename
        return candidate if candidate.is_file() else None

    from huggingface_hub import hf_hub_download  # ruff: ignore[import-outside-top-level]
    from huggingface_hub.errors import (  # ruff: ignore[import-outside-top-level]
        EntryNotFoundError,
    )

    try:
        return Path(hf_hub_download(repo_id=source, filename=filename))
    except (EntryNotFoundError, OSError, ValueError) as exc:
        logger.debug("Checkpoint %s has no %s: %s", source, filename, exc)
        return None


def _preprocessor_steps(source: str | Path) -> list[dict[str, Any]]:
    """The steps of the checkpoint's preprocessor pipeline; none if absent or unreadable."""
    preprocessor = _checkpoint_file(source, _PREPROCESSOR_NAME)
    if preprocessor is None:
        return []
    try:
        with preprocessor.open(encoding="utf-8") as handle:
            steps = json.load(handle).get("steps", [])
    except (OSError, ValueError, AttributeError) as exc:
        logger.debug("Cannot read %s: %s", preprocessor, exc)
        return []
    return [step for step in steps if isinstance(step, dict)] if isinstance(steps, list) else []


def _load_stats(source: str | Path, steps: list[dict[str, Any]]) -> dict[str, FeatureStats]:
    """Read the normalizer step's safetensors statistics beside ``config.json``; never raises."""
    state_files = [
        step["state_file"]
        for step in steps
        if step.get("registry_name") == _NORMALIZER_STEP and step.get("state_file")
    ]

    raw: dict[str, list[float]] = {}
    for name in state_files:
        path = _checkpoint_file(source, name)
        if path is None:
            continue
        try:
            from safetensors.numpy import load_file  # ruff: ignore[import-outside-top-level]

            raw.update({
                key: [float(value) for value in array.reshape(-1)]
                for key, array in load_file(path).items()
            })
        except (OSError, ValueError, ImportError) as exc:
            logger.debug("Cannot read normalization statistics from %s: %s", path, exc)

    return _group_stats(raw)


def _group_stats(raw: dict[str, list[float]]) -> dict[str, FeatureStats]:
    """Turn flat ``"<feature>.<stat>"`` keys into one entry per feature."""
    fields = {"min": "minimum", "max": "maximum", "mean": "mean", "std": "std"}
    grouped: dict[str, dict[str, tuple[float, ...]]] = {}
    for key, values in raw.items():
        feature, _, stat = key.rpartition(".")
        if not feature or stat not in fields:
            continue
        grouped.setdefault(feature, {})[fields[stat]] = tuple(values)
    return {feature: FeatureStats(**parts) for feature, parts in grouped.items()}
