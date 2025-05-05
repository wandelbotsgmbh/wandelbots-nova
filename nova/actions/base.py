import inspect
from abc import ABC, abstractmethod

import pydantic

from nova.types.pose import Pose


class Action(pydantic.BaseModel, ABC):
    @abstractmethod
    @pydantic.model_serializer
    def serialize_model(self):
        """Serialize the model to a dictionary"""

    @abstractmethod
    def is_motion(self) -> bool:
        """Return whether the action is a motion"""

    @classmethod
    def from_dict(cls, data: dict) -> "Action":
        """Create an action instance from a dictionary based on type"""
        processed_data = {}

        if isinstance(data, dict):
            processed_data = data.copy()

        # Convert target to Pose if present
        if isinstance(processed_data, dict):
            if "target_pose" in processed_data and isinstance(processed_data["target_pose"], dict):
                position = processed_data["target_pose"]["position"]
                orientation = processed_data["target_pose"]["orientation"]
                processed_data["target"] = Pose(
                    (
                        position[0],
                        position[1],
                        position[2],
                        orientation[0],
                        orientation[1],
                        orientation[2],
                    )
                )

        # Convert target_joint_position to target tuple if present
        if "target_joint_position" in processed_data and isinstance(
            processed_data["target_joint_position"], list
        ):
            processed_data["target"] = tuple(processed_data["target_joint_position"])

        # Handle circular motion's via_pose
        if "via_pose" in processed_data and isinstance(processed_data["via_pose"], dict):
            position = processed_data["via_pose"]["position"]
            orientation = processed_data["via_pose"]["orientation"]
            processed_data["intermediate"] = Pose(
                (
                    position[0],
                    position[1],
                    position[2],
                    orientation[0],
                    orientation[1],
                    orientation[2],
                )
            )

        action_type = processed_data.get("type")
        if not action_type:
            raise ValueError("Missing 'type' field in action data")

        # Find the appropriate action class for this type
        action_class = cls._find_action_class_by_type(action_type)
        if not action_class:
            raise ValueError(f"Unknown action type: {action_type}")

        return action_class.model_validate(processed_data)

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
