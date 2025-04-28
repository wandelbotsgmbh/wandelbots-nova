import inspect
from abc import ABC, abstractmethod

import pydantic


class Action(pydantic.BaseModel, ABC):
    @abstractmethod
    @pydantic.model_serializer
    def serialize_model(self):
        """Serialize the model to a dictionary"""

    @abstractmethod
    def is_motion(self) -> bool:
        """Return whether the action is a motion"""

    @classmethod
    def create_from_dict(cls, data: dict) -> "Action":
        """Create an action instance from a dictionary based on type"""
        action_type = data.get("type")
        if not action_type:
            raise ValueError("Missing 'type' field in action data")

        # Find the appropriate action class for this type
        action_class = cls._find_action_class_by_type(action_type)
        if not action_class:
            raise ValueError(f"Unknown action type: {action_type}")

        return action_class.model_construct(**data)

    @classmethod
    def _find_action_class_by_type(cls, action_type: str):
        """Dynamically find an action class that has the given type literal value"""
        # First, import all relevant modules to ensure classes are defined
        from nova.actions import io, motions

        # Look for the action class in all submodules
        for module in [motions, io]:
            for name, obj in inspect.getmembers(module):
                # Check if this is a class that inherits from Action
                if (
                    inspect.isclass(obj)
                    and issubclass(obj, Action)
                    and obj is not Action
                    and hasattr(obj, "model_fields")
                    and "type" in obj.model_fields
                ):
                    # Check if this class has the right type value
                    if hasattr(obj, "type") and getattr(obj, "type") == action_type:
                        return obj

                    # For Literal types, check default value
                    type_field = obj.model_fields.get("type")
                    if (
                        type_field is not None
                        and hasattr(type_field, "default")
                        and type_field.default == action_type
                    ):
                        return obj

        return None
