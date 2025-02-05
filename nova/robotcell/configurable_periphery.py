from typing import Any, ClassVar, Literal, get_origin, get_type_hints

import pydantic


class ConfigurablePeriphery:
    """A device which is configurable"""

    all_classes: ClassVar[dict] = {}

    def __init_subclass__(cls, is_abstract=False):
        super().__init_subclass__()
        if not is_abstract:
            assert (
                hasattr(cls, "Configuration") and get_origin(get_type_hints(cls.Configuration)["type"]) is Literal
            ), f"{cls.__name__} has no type literal"
            assert ConfigurablePeriphery.Configuration is not cls.Configuration
            cls.all_classes[cls.Configuration] = cls

    class Configuration(pydantic.BaseModel):
        """Minimum configuration of a configurable periphery

        Args:
            identifier: A unique identifier to reference the periphery
        """

        model_config = pydantic.ConfigDict(frozen=True)

        type: str
        identifier: str

    _configuration: Configuration

    def __init__(self, configuration: Configuration, **kwargs):
        super().__init__(**kwargs)
        self._configuration = configuration

    @property
    def configuration(self):
        return self._configuration

    @property
    def identifier(self):
        return self.configuration.identifier

    @classmethod
    def from_dict(cls, data):
        """Constructs a new configurable periphery from a dict

        Returns:
            cls: the newly created ConfigurablePeriphery object

        """
        return cls(cls.Configuration(**data))

    def to_dict(self) -> dict[str, Any]:
        """Creates a json dict from the configurable periphery parameters which can be transformed to a json string

        Returns:
            Dict[str, Any]: a json string
        """
        return self._configuration.model_dump()
