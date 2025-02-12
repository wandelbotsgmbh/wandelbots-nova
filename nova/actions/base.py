from abc import ABC, abstractmethod

import pydantic


class Action(pydantic.BaseModel, ABC):
    @abstractmethod
    @pydantic.model_serializer
    def serialize_model(self):
        """Serialize the model to a dictionary"""
