import functools
from typing import Dict
from .store.models import ProgramTemplate, BaseProgramModel

# Store program templates by name
REGISTERED_PROGRAM_TEMPLATES: Dict[str, ProgramTemplate] = {}

def program(name: str, model):
    def decorator(func):
        # Validate that the model inherits from BaseProgramModel
        if not issubclass(model, BaseProgramModel):
            raise ValueError(f"Program model for {func.__name__} must inherit from BaseProgramModel")
        
        # Register the program template
        if hasattr(model, 'model_json_schema'):
            schema = model.model_json_schema()
            
            template = ProgramTemplate(
                name=name,
                model_class=model,
                function=func,
                schema=schema
            )
            REGISTERED_PROGRAM_TEMPLATES[name] = template
        else:
            print(f"Warning: Model for {func.__name__} does not have model_json_schema method.")

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            print(f"Program model: {model}")
            return func(*args, **kwargs)
        return wrapper
    return decorator
