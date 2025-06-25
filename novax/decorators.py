"""
Decorators and registration system for NovaX program templates.
"""

from typing import Callable

from .store.models import BaseProgramModel, ProgramTemplate


def program(
    name: str | None = None,
    model: type[BaseProgramModel] | None = None,
    description: str | None = None,
) -> Callable:
    """
    Decorator to register a function as a NovaX program template.

    Note: This decorator is mainly for compatibility. The actual discovery
    of @nova.program decorated functions is handled by the auto_discovery module.

    Args:
        name: Optional name for the program template. If not provided, uses function name.
        model: Optional Pydantic model class for program parameters.
        description: Optional description of the program.

    Returns:
        The decorated function without any registration side effects.
        Registration happens during auto-discovery phase.
    """

    def decorator(func: Callable) -> Callable:
        # This decorator now just marks the function for later discovery
        # The actual registration happens in the auto_discovery module
        # when it finds @nova.program decorated functions

        # Store metadata on the function for later use during discovery
        func._novax_template_name = name or func.__name__
        func._novax_model_class = model
        func._novax_description = description

        return func

    return decorator


# Legacy functions kept for compatibility, but they now work with the store
def clear_registered_programs():
    """Clear all program templates from the store."""
    # This would need to be implemented by the store interface
    # For now, this is a no-op as clearing is handled during discovery
    pass


def get_registered_programs() -> dict[str, ProgramTemplate]:
    """Get all program templates from the store."""
    from .database import template_store

    try:
        # Get all templates from the store
        all_templates = template_store.list()
        return {template.name: template for template in all_templates}
    except Exception:
        # If store is not available or has issues, return empty dict
        return {}


# Legacy global registry for backward compatibility
# This is implemented as a property that delegates to the template store
class _LegacyRegistryProxy:
    """Proxy class to provide backward compatibility for REGISTERED_PROGRAM_TEMPLATES"""

    def clear(self):
        """Clear all program templates from the store."""
        # This is a no-op as clearing is handled during discovery
        pass

    def keys(self):
        """Get all template names."""
        return get_registered_programs().keys()

    def values(self):
        """Get all templates."""
        return get_registered_programs().values()

    def items(self):
        """Get all template name-template pairs."""
        return get_registered_programs().items()

    def get(self, key, default=None):
        """Get a template by name."""
        return get_registered_programs().get(key, default)

    def __getitem__(self, key):
        """Get a template by name."""
        programs = get_registered_programs()
        if key not in programs:
            raise KeyError(f"Template '{key}' not found")
        return programs[key]

    def __contains__(self, key):
        """Check if a template exists."""
        return key in get_registered_programs()

    def __len__(self):
        """Get the number of registered templates."""
        return len(get_registered_programs())


# Global instance for backward compatibility
REGISTERED_PROGRAM_TEMPLATES = _LegacyRegistryProxy()
