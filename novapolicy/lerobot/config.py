"""Read execution settings stored in a LeRobot policy checkpoint.

Everything here is read through LeRobot's own ``PreTrainedConfig`` rather than
by parsing ``config.json`` field by field. Policies do not agree on what the
chunk length is called — ACT says ``chunk_size``, diffusion says ``horizon``,
fastwam says ``action_horizon``, and vqbet names it nothing at all — but every
policy config must implement the abstract ``action_delta_indices`` property, so
its length is the chunk length for all of them. Deriving it that way means a new
or renamed LeRobot policy needs no change here.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING

from lerobot.configs.types import FeatureType

if TYPE_CHECKING:
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.configs.types import PolicyFeature

logger = logging.getLogger(__name__)

_CONFIG_NAME = "config.json"
_PREPROCESSOR_NAME = "policy_preprocessor.json"
_NORMALIZER_STEP = "normalizer_processor"

# Width of the plausible band derived from mean/std statistics.
_STD_SIGMA = 4.0

# Feature types the NOVA observation builder can produce. A checkpoint may
# declare others (LANGUAGE for task-conditioned models, ENV for simulators);
# those are carried but never demanded of the schema.
_COMPARABLE_FEATURE_TYPES = (FeatureType.STATE, FeatureType.VISUAL)


@dataclass(frozen=True, slots=True)
class FeatureStats:
    """Per-dimension training statistics for one feature.

    Which fields are populated depends on the checkpoint's ``NormalizationMode``:
    ``MIN_MAX`` fills min/max, ``MEAN_STD`` fills mean/std. Either is enough to
    say roughly where the training data lay.
    """

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
    """

    policy_type: str
    chunk_size: int | None = None
    n_action_steps: int | None = None
    input_features: dict[str, PolicyFeature] = field(default_factory=dict)
    output_features: dict[str, PolicyFeature] = field(default_factory=dict)
    stats: dict[str, FeatureStats] = field(default_factory=dict)
    """Training statistics per feature, when the checkpoint's preprocessor is readable."""

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
    """Load execution settings from a local checkpoint or a Hugging Face model id.

    The source can be a checkpoint directory, the ``config.json`` inside one, or
    a Hugging Face model id. LeRobot's own loader is used, and it reads a file
    named ``config.json`` from a checkpoint directory — a JSON file under any
    other name is not accepted. Server-local absolute paths cannot be inspected
    by the NOVA client; supply a client-local copy or set the values explicitly.

    Raises:
        FileNotFoundError: The checkpoint is not reachable from this machine.
        ImportError: The policy's config needs a package this environment lacks.
        ValueError: LeRobot rejected the checkpoint config.
    """
    source = _resolve_source(pretrained_name_or_path)
    settings = _from_policy_config(_load_policy_config(source))
    stats = _load_stats(source)
    return replace(settings, stats=stats) if stats else settings


def try_load_execution_settings(
    pretrained_name_or_path: str | Path | None,
) -> LeRobotExecutionSettings | None:
    """Load execution settings, or return ``None`` when the checkpoint is unusable.

    Used on the derivation path, where an unreadable checkpoint is an expected
    outcome: the client falls back to explicit arguments. A checkpoint LeRobot
    itself rejects still raises — that is a real configuration error, not a
    missing file.
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
    """Resolve to something ``PreTrainedConfig.from_pretrained`` accepts.

    It takes a directory or a Hub model id, not a path to ``config.json``, so a
    file is mapped to its parent directory.
    """
    source = Path(pretrained_name_or_path).expanduser()
    if source.is_dir():
        return source
    if source.is_file():
        # Only the checkpoint's own config.json maps to its directory. Accepting
        # any file would silently read the config.json beside it instead, which
        # is worse than refusing.
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
    """Load the checkpoint's typed LeRobot config.

    Imported lazily: pulling in the policy registry costs seconds and loads
    torch, which callers who never touch a checkpoint should not pay for.
    """
    # Importing the policies package registers every policy config choice; without
    # it draccus cannot resolve the checkpoint's ``type``.
    from draccus.utils import ParsingError  # ruff: ignore[import-outside-top-level]
    from lerobot.configs.policies import PreTrainedConfig  # ruff: ignore[import-outside-top-level]
    import lerobot.policies  # ruff: ignore[import-outside-top-level,unused-import]

    if isinstance(source, Path) and not (source / _CONFIG_NAME).is_file():
        raise FileNotFoundError(f"{_CONFIG_NAME} not found in LeRobot checkpoint: {source}")

    try:
        return PreTrainedConfig.from_pretrained(source)
    except ParsingError as exc:
        # draccus raises its own exception type, which is not a ValueError and
        # would leak an internal dependency through this module's contract.
        msg = f"LeRobot rejected the checkpoint config at {source}: {exc}"
        raise ValueError(msg) from exc


def _from_policy_config(config: PreTrainedConfig) -> LeRobotExecutionSettings:
    action_indices = config.action_delta_indices
    n_action_steps = getattr(config, "n_action_steps", None)

    return LeRobotExecutionSettings(
        policy_type=config.type,
        # Length of the action window the policy predicts. Uniform across
        # policies; the field it is computed from is not.
        chunk_size=len(action_indices) if action_indices else None,
        n_action_steps=n_action_steps if isinstance(n_action_steps, int) else None,
        input_features=dict(config.input_features or {}),
        output_features=dict(config.output_features or {}),
    )


def _checkpoint_file(source: str | Path, filename: str) -> Path | None:
    """Locate one file in the checkpoint, downloading it when the source is a Hub id.

    Statistics live beside ``config.json``, so a Hub checkpoint has to fetch them
    the same way the config itself was fetched — a local-directory check alone
    would silently skip the range check for every Hub model. Returns ``None``
    when the file is simply not there, which is a normal way to ship a
    checkpoint.
    """
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


def _load_stats(source: str | Path) -> dict[str, FeatureStats]:
    """Read the checkpoint's normalization statistics, if they are present.

    They live beside ``config.json`` as the normalizer step's safetensors state,
    keyed ``"<feature>.<stat>"``. Absent statistics are normal — a checkpoint may
    be distributed as ``config.json`` alone — so this never raises; the range
    check simply does not run.
    """
    preprocessor = _checkpoint_file(source, _PREPROCESSOR_NAME)
    if preprocessor is None:
        return {}

    try:
        with preprocessor.open(encoding="utf-8") as handle:
            config = json.load(handle)
        state_files = [
            step["state_file"]
            for step in config.get("steps", [])
            if step.get("registry_name") == _NORMALIZER_STEP and step.get("state_file")
        ]
    except (OSError, ValueError, TypeError, KeyError) as exc:
        logger.debug("Cannot read %s: %s", preprocessor, exc)
        return {}

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
