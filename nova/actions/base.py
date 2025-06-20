from __future__ import annotations

from abc import ABC, abstractmethod
from typing import ClassVar

import pydantic

from nova.core.logging import logger


class Action(pydantic.BaseModel, ABC):
    _registry: ClassVar[dict[str, type[Action]]] = {}

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)

        action_type = getattr(cls, "type", None)
        # when no type is found -> skip
        if not isinstance(action_type, str):
            logger.warning(f"Action class '{cls.__name__}' does not have a valid type")
            return

        if action_type in Action._registry:
            logger.warning(f"Duplicate action type '{action_type}'")
            return
        Action._registry[action_type] = cls
        logger.debug(f"Registered action type: {action_type}")

    @classmethod
    def from_dict(cls, data: dict) -> Action:
        """
        Pick the correct concrete Action from the `_registry`
        and let Pydantic validate against that class.
        """
        if not isinstance(data, dict):
            raise TypeError("`data` must be a dict")

        action_type = data.get("type")
        if action_type is None:
            raise ValueError("Missing required key `type`")

        try:
            concrete_cls = Action._registry[action_type]
        except KeyError:
            raise ValueError(f"Unknown action type '{action_type}'")

        return concrete_cls.model_validate(data)

    @abstractmethod
    def is_motion(self) -> bool:
        """Return whether the action is a motion"""

    @abstractmethod
    def to_api_model(self):
        """Convert the action to an API model"""
