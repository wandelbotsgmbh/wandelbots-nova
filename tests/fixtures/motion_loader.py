import json
from pathlib import Path
from typing import Any

from nova.api import models


def load_motion_data(fixture_path: Path) -> dict[str, Any]:
    """Load recorded motion data from JSON file."""
    data = json.loads(fixture_path.read_text())

    return {
        "motion_id": data["motion_id"],
        "model_from_controller": data["model_from_controller"],
        "motion_group": data["motion_group"],
        "optimizer_config": models.OptimizerSetup.model_validate(data["optimizer_config"]),
        "trajectory": [models.TrajectorySample.model_validate(t) for t in data["trajectory"]],
        "collision_scenes": {
            k: models.CollisionScene.model_validate(v) for k, v in data["collision_scenes"].items()
        },
    }
