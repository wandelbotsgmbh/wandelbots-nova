from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import torch


class ObservationSource(Protocol):
    def next_observation(self, *, task: str | None = None) -> dict[str, object]:
        """Return one policy observation payload in LeRobot feature-key format."""


@dataclass(slots=True)
class MockObservationSource:
    input_features: dict[str, object]

    def next_observation(self, *, task: str | None = None) -> dict[str, object]:
        observation: dict[str, object] = {}

        for feature_name, feature in self.input_features.items():
            feature_type = str(getattr(feature, "type", "")).lower()
            shape_value = getattr(feature, "shape", None)

            if feature_name == "task" or "text" in feature_type or "string" in feature_type:
                observation[feature_name] = [task or ""]
                continue

            if isinstance(shape_value, tuple):
                feature_shape = shape_value
            elif isinstance(shape_value, list):
                feature_shape = tuple(int(dim) for dim in shape_value)
            else:
                feature_shape = ()

            tensor_shape = (1, *feature_shape) if len(feature_shape) > 0 else (1,)
            observation[feature_name] = torch.zeros(tensor_shape, dtype=torch.float32)

        return observation
