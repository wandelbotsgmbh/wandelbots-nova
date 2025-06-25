import pydantic
from typing import Any, Dict


class BaseProgramModel(pydantic.BaseModel):
    """Base model that all program models should inherit from"""
    name: str
    description: str = ""


class ProgramTemplate:
    """Represents a program template with its model and function"""
    def __init__(self, name: str, model_class: type, function: callable, schema: Dict[str, Any]):
        self.name = name
        self.model_class = model_class
        self.function = function
        self.schema = schema


class ProgramInstance:
    """Represents an actual program instance with data"""
    def __init__(self, name: str, template: ProgramTemplate, data: Dict[str, Any]):
        self.name = name
        self.template = template
        self.data = data
        self.model_instance = template.model_class(**data)