import argparse
import asyncio
import inspect
import json
from collections.abc import Callable, Mapping
from typing import (
    Annotated,
    Any,
    Coroutine,
    Generic,
    ParamSpec,
    TypeVar,
    Union,
    get_args,
    get_origin,
    get_type_hints,
)

from docstring_parser import Docstring
from docstring_parser import parse as parse_docstring
from pydantic import BaseModel, Field, PrivateAttr, RootModel, create_model, validate_call
from pydantic.fields import FieldInfo
from pydantic.json_schema import JsonSchemaValue, models_json_schema

from nova import Nova, api
from nova.core.exceptions import ControllerCreationFailed
from nova.core.logging import logger

Parameters = ParamSpec("Parameters")
Return = TypeVar("Return")


class ProgramPreconditions(BaseModel):
    controllers: list[api.models.RobotController] | None = None
    cleanup_controllers: bool = False


class Program(BaseModel, Generic[Parameters, Return]):
    _wrapped: Callable[Parameters, Any] = PrivateAttr(
        default_factory=lambda: lambda *args, **kwargs: None
    )
    program_id: str
    name: str | None
    description: str | None
    input: type[BaseModel]
    output: type[BaseModel]
    preconditions: ProgramPreconditions | None = None

    @classmethod
    def validate(cls, value: Callable[Parameters, Return]) -> "Program[Parameters, Return]":
        if isinstance(value, Program):
            return value
        if not callable(value):
            raise TypeError("value must be callable")

        program_id = value.__name__
        docstring = parse_docstring(value.__doc__ or "")
        description = docstring.description

        input, output = input_and_output_types(value, docstring)

        function = cls(
            program_id=program_id, name=None, description=description, input=input, output=output
        )
        function._wrapped = validate_call(validate_return=True)(value)
        return function

    async def __call__(self, *args: Parameters.args, **kwargs: Parameters.kwargs) -> Return:  # pylint: disable=no-member
        return await self._wrapped(*args, **kwargs)

    def _log(self, level: str, message: str) -> None:
        """Log a message with program prefix."""
        prefix = f"Nova Program '{self.name}'"
        formatted_message = f"{prefix}: {message}"

        # TODO: use logger.log(...)
        if level == "info":
            logger.info(formatted_message)
        elif level == "error":
            logger.error(formatted_message)
        elif level == "warning":
            logger.warning(formatted_message)
        elif level == "debug":
            logger.debug(formatted_message)
        else:
            logger.info(formatted_message)

    async def _create_controllers(self) -> list[str]:
        """Create controllers based on controller_configs and return their IDs."""
        created_controllers: list[str] = []
        if not self.preconditions or not self.preconditions.controllers:
            return created_controllers

        async with Nova() as nova:
            cell = nova.cell()
            controller_config = None
            try:
                for controller_config in self.preconditions.controllers:
                    controller_name = controller_config.name or "unnamed_controller"
                    controller = await cell.ensure_controller(robot_controller=controller_config)
                    created_controllers.append(controller.controller_id)
                    self._log(
                        "info",
                        f"Created controller '{controller_name}' with ID {controller.controller_id}",
                    )

                # Setup viewers after controllers are created and available
                try:
                    from nova.viewers import _setup_active_viewers_after_preconditions

                    await _setup_active_viewers_after_preconditions()
                except ImportError:
                    pass

            except Exception as e:
                controller_name = (
                    controller_config.name if controller_config else "unnamed_controller"
                )
                raise ControllerCreationFailed(controller_name, str(e))

        return created_controllers

    async def _cleanup_controllers(self, controller_ids: list[str]) -> None:
        """Clean up controllers by their IDs."""
        if (
            not self.preconditions
            or not self.preconditions.cleanup_controllers
            or not controller_ids
        ):
            return

        try:
            async with Nova() as nova:
                cell = nova.cell()
                for controller_id in controller_ids:
                    try:
                        await cell.delete_robot_controller(controller_id)
                        self._log("info", f"Cleaned up controller with ID '{controller_id}'")
                    except Exception as e:
                        # WORKAROUND: {"code":9, "message":"Failed to 'Connect to Host' due the
                        #   following reason:\nConnection refused (2)!\nexception::CommunicationException: Configured robot connection is not reachable.", "details":[]}
                        # Log and suppress errors for individual controller cleanup
                        self._log("error", f"Error cleaning up controller '{controller_id}': {e}")
        except Exception as e:
            # Log and suppress errors for the overall cleanup process
            self._log("error", f"Error during controller cleanup: {e}")

    @property
    def input_schema(self) -> dict[str, Any]:
        return self.input.model_json_schema()

    @property
    def output_schema(self) -> dict[str, Any]:
        return self.output.model_json_schema()

    @property
    def json_schema(self, title: str | None = None) -> JsonSchemaValue:
        _, top_level_schema = models_json_schema(
            [(self.input, "validation"), (self.output, "validation")], title=title or self.name
        )
        return top_level_schema

    def __repr__(self) -> str:
        input_fields = ", ".join(
            f"{k}: {v.annotation.__name__}"  # type: ignore
            for k, v in self.input.model_fields.items()
        )

        # Get the actual output type from RootModel
        if hasattr(self.output, "model_fields") and "root" in self.output.model_fields:
            root_annotation = self.output.model_fields["root"].annotation
            # If it's a TypeVar, get its bound type
            if hasattr(root_annotation, "__bound__") and root_annotation.__bound__:  # type: ignore
                output_type = root_annotation.__bound__.__name__  # type: ignore
            else:
                output_type = root_annotation.__name__  # type: ignore
        else:
            output_type = self.output.__name__

        desc_part = f", description='{self.description}'" if self.description else ""
        return (
            f"Program(name='{self.name}'{desc_part}, input=({input_fields}), output={output_type})"
        )

    def create_parser(self) -> argparse.ArgumentParser:
        """Create an argument parser based on the function's input model.

        Returns:
            argparse.ArgumentParser: A parser configured with arguments matching the input model fields.
        """
        parser = argparse.ArgumentParser(description=self.description or self.name)

        for name, field in self.input.model_fields.items():
            # Convert field type to appropriate Python type
            field_type = field.annotation
            if hasattr(field_type, "__origin__") and field_type.__origin__ is Annotated:  # type: ignore
                field_type = field_type.__origin__  # type: ignore

            # Handle optional fields
            is_optional = False
            if hasattr(field_type, "__origin__") and field_type.__origin__ is Union:  # type: ignore
                field_type = field_type.__args__[0]  # type: ignore
                is_optional = True

            # For complex types (like Pydantic models), use JSON parsing
            if isinstance(field_type, type) and issubclass(field_type, BaseModel):

                def json_type(value: str) -> Any:
                    try:
                        return json.loads(value)
                    except json.JSONDecodeError as e:
                        raise argparse.ArgumentTypeError(f"Invalid JSON for {name}: {e}")

                parser.add_argument(
                    f"--{name}",
                    dest=name,
                    type=json_type,
                    default=field.default if field.default is not None else None,
                    required=not is_optional and field.default is None,
                    help=field.description or f"{name} parameter (JSON format)",
                )
            else:
                # Add argument to parser
                parser.add_argument(
                    f"--{name}",
                    dest=name,
                    type=field_type,  # type: ignore
                    default=field.default if field.default is not None else None,
                    required=not is_optional and field.default is None,
                    help=field.description or f"{name} parameter",
                )

        return parser


def input_and_output_types(
    func: Callable, docstring: Docstring
) -> tuple[type[BaseModel], type[BaseModel]]:
    signature = inspect.signature(func)
    input_types = get_type_hints(func)
    output_type = input_types.pop("return", None)

    input_field_definitions: Mapping[str, Any] = {}
    for order, (name, parameter) in enumerate(signature.parameters.items()):
        default: FieldInfo = (
            Field(...)
            if parameter.default is parameter.empty
            else (
                parameter.default
                if isinstance(parameter.default, FieldInfo)
                else Field(parameter.default)
            )
        )

        # Add field order
        default.json_schema_extra = {"x-order": order}

        # Add description from docstring if available
        if not default.description:
            if param_doc := next((p for p in docstring.params if p.arg_name == name), None):
                default.description = param_doc.description

        input_field_definitions[name] = (parameter.annotation, default)  # type: ignore
    input = create_model("Input", **input_field_definitions, __module__=func.__module__)

    if output_type and isinstance(output_type, type) and issubclass(output_type, BaseModel):
        output = output_type
    else:
        T = TypeVar("T")
        description = None

        # Check if return type is Annotated
        if get_origin(signature.return_annotation) is Annotated:
            base_type, *annotations = get_args(signature.return_annotation)
            for annotation in annotations:
                if isinstance(annotation, FieldInfo):
                    description = annotation.description
                    break
            T = TypeVar("T", bound=base_type)  # type: ignore
        else:
            T = TypeVar("T", bound=output_type)  # type: ignore
            if docstring.returns:
                description = docstring.returns.description

        class Output(RootModel[T]):  # pylint: disable=redefined-outer-name
            root: T = Field(..., description=description)

        output = Output

    return input, output


def program(
    id: str | None = None,
    name: str | None = None,
    description: str | None = None,
    preconditions: ProgramPreconditions | None = None,
    viewer: Any | None = None,
):
    """
    Decorator factory for creating Nova programs with declarative controller setup.

    Args:
        name: Name of the program
        preconditions: ProgramPreconditions containing controller configurations and cleanup settings
        viewer: Optional viewer instance for program visualization (e.g., nova.viewers.Rerun())
    """

    def decorator(
        function: Callable[Parameters, Return],
    ) -> Program[Parameters, Coroutine[Any, Any, Return]]:
        # Validate that the function is async
        if not asyncio.iscoroutinefunction(function):
            raise TypeError(f"Program function '{function.__name__}' must be async")

        func_obj = Program.validate(function)
        if id:
            func_obj.program_id = id
        if name:
            func_obj.name = name
        if description:
            func_obj.description = description
        func_obj.preconditions = preconditions

        # Create a wrapper that handles controller lifecycle
        original_wrapped = func_obj._wrapped

        async def async_wrapper(*args: Parameters.args, **kwargs: Parameters.kwargs) -> Return:
            """Async wrapper that handles controller creation and cleanup."""
            created_controllers = []
            try:
                # Create controllers before execution
                created_controllers = await func_obj._create_controllers()

                # Configure viewers if any are active
                if viewer is not None:
                    # Configure the viewer when Nova instance becomes available in the function
                    # This will be done via a hook in the Nova context manager
                    pass

                # Execute the wrapped function
                result = await original_wrapped(*args, **kwargs)
                return result
            finally:
                # Clean up controllers after execution
                await func_obj._cleanup_controllers(created_controllers)

                # Clean up viewers
                if viewer is not None:
                    from nova.viewers import _cleanup_active_viewers

                    _cleanup_active_viewers()

        # Update the wrapped function to our async wrapper
        func_obj._wrapped = async_wrapper
        return func_obj

    return decorator
