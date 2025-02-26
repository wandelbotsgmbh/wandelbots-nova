import json
from pathlib import Path
from typing import Any

from nova.api import models


def _convert_sets_to_lists(obj: Any) -> Any:
    """Convert sets to lists in nested structures."""
    if isinstance(obj, set):
        return list(obj)
    if isinstance(obj, dict):
        return {k: _convert_sets_to_lists(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_convert_sets_to_lists(item) for item in obj]
    return obj


def record_motion_data(
    motion_id: str,
    model_from_controller: str,
    motion_group: str,
    optimizer_config: models.OptimizerSetup,
    trajectory: list[models.TrajectorySample],
    collision_scenes: dict[str, models.CollisionScene],
    output_file: Path,
) -> None:
    """Record motion data to a JSON file for testing."""
    data = {
        "motion_id": motion_id,
        "model_from_controller": model_from_controller,
        "motion_group": motion_group,
        "optimizer_config": _convert_sets_to_lists(optimizer_config.model_dump()),
        "trajectory": [_convert_sets_to_lists(t.model_dump()) for t in trajectory],
        "collision_scenes": {
            k: _convert_sets_to_lists(v.model_dump()) for k, v in collision_scenes.items()
        },
    }

    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(data, indent=2))
