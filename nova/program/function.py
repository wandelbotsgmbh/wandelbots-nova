import argparse
import asyncio
import inspect
import json
import logging
from collections.abc import Callable
from typing import (
    Annotated,
    Any,
    Coroutine,
    Generic,
    Literal,
    ParamSpec,
    TypeVar,
    Union,
    cast,
    get_args,
    get_origin,
    get_type_hints,
)

from docstring_parser import Docstring
from docstring_parser import parse as parse_docstring
from pydantic import BaseModel, ConfigDict, Field, PrivateAttr, RootModel, create_model
from pydantic.fields import FieldInfo
from pydantic.json_schema import JsonSchemaValue, models_json_schema

from nova import Nova, api
from nova.exceptions import ControllerCreationFailed

logger = logging.getLogger(__name__)

Parameters = ParamSpec("Parameters")
Return = TypeVar("Return")


class ProgramPreconditions(BaseModel):
    controllers: list[api.models.RobotController] | None = None
    cleanup_controllers: bool = False


class ProgramContext:
    """Context passed into every program execution."""

    def __init__(self, nova: Nova, program_id: str | None = None):
        self._nova = nova
        self._program_id = program_id
        # Not all Nova stand-ins (e.g., test fakes) implement `cell()`. Cache the cell
        # when available; otherwise leave as None.
        cell_fn = getattr(nova, "cell", None)
        self._cell = cell_fn() if callable(cell_fn) else None

    @property
    def nova(self) -> Nova:
        """Returns the Nova instance for the program."""
        return self._nova

    @property
    def cell(self):
        """Returns the default cell for the program, if available."""
        return self._cell

    @property
    def program_id(self) -> str | None:
        """Returns the program ID for the program."""
        return self._program_id

    def cycle(self, extra: dict[str, Any] | None = None):
        """Create a Cycle with program pre-populated in the extra data."""
        from nova.events import Cycle

        if self._cell is None:
            raise AttributeError(
                "ProgramContext.cell is not available; the provided Nova instance does not expose cell()."
            )

        merged_extra = {"program": self.program_id} if self.program_id else {}
        if extra:
            merged_extra.update(extra)
        return Cycle(self.cell, extra=merged_extra)


"""
## Define programs:

# Program without inputs or context
async def program1(ctx: ProgramContext):
    ...

async def program2(ctx: ProgramContext, count: int = 1, ...):
    ...

## Call programs:

await program1()
await program2(count=3)

"""


class Program(BaseModel, Generic[Parameters, Return]):
    _impl: Callable[..., Coroutine[Any, Any, Return]] = PrivateAttr(
        default_factory=lambda *args, **kwargs: None  # type: ignore[assignment]
    )
    _viewer: Any | None = PrivateAttr(default=None)
    program_id: str
    name: str | None
    description: str | None
    input_model: type[BaseModel] | None
    output_model: type[BaseModel]
    preconditions: ProgramPreconditions | None = None

    @classmethod
    def validate(cls, value: Callable[Parameters, Return]) -> "Program[Parameters, Return]":
        if isinstance(value, Program):
            return value

        # Enforce that the first parameter is named `ctx`
        signature = inspect.signature(value)
        params = list(signature.parameters.values())
        if not params or params[0].name != "ctx":
            raise TypeError(
                f"Program function '{value.__name__}' must have 'ctx' as its first parameter. "
                "Define it as 'async def "
                f"{value.__name__}(ctx, ...):'."
            )

        # If ctx is untyped, annotate it with NovaProgramContext for better introspection
        if params[0].annotation is inspect._empty:
            annotations = dict(getattr(value, "__annotations__", {}))
            annotations.setdefault("ctx", ProgramContext)
            value.__annotations__ = annotations

        program_id = value.__name__
        docstring = parse_docstring(value.__doc__ or "")
        description = docstring.description

        input_model_, output_type = input_and_output_types(value, docstring)

        program: "Program[Parameters, Return]" = cls(
            program_id=program_id,
            name=None,
            description=description,
            input_model=input_model_,
            output_model=output_type,
        )

        # mypy does not recognise that `value` is an async function returning a coroutine,
        # so we help it with a cast here.
        program._impl = cast(Callable[..., Coroutine[Any, Any, Return]], value)

        return program

    async def __call__(self, *args: Parameters.args, **kwargs: Parameters.kwargs) -> Return:  # pylint: disable=no-member
        if args:
            raise TypeError(
                "Nova programs must be called with keyword arguments only "
                "(e.g. 'await program(count=3)' or 'await program(ctx=..., count=3)')."
            )

        ctx = kwargs.pop("ctx", None)
        nova_override_obj = kwargs.pop("nova", None)
        nova_override: Nova | None = cast(Nova | None, nova_override_obj)

        if ctx is not None and not isinstance(ctx, ProgramContext):
            raise TypeError("ctx must be a nova.ProgramContext instance")

        if ctx is None:
            if nova_override is not None:
                ctx = ProgramContext(nova=nova_override, program_id=self.program_id)
            else:
                ctx = ProgramContext(nova=Nova(), program_id=self.program_id)

        # Remaining keyword arguments are treated as input parameters for the program.
        input_values: dict[str, Any] = kwargs

        if self.input_model is None:
            if input_values:
                unexpected = ", ".join(sorted(input_values.keys()))
                raise TypeError(
                    f"Program '{self.program_id}' does not accept any input parameters "
                    f"(unexpected: {unexpected})."
                )
            validated_kwargs: dict[str, Any] = {}
        else:
            input_instance = self.input_model.model_validate(input_values)
            # Use attribute access instead of model_dump() so that nested
            # BaseModel instances (e.g. Person) are preserved instead of
            # being converted to plain dictionaries.
            validated_kwargs = {
                field_name: getattr(input_instance, field_name)
                for field_name in input_instance.model_fields.keys()
            }

        created_controllers: list[str] = []
        try:
            created_controllers = await self._create_controllers()

            # Execute the wrapped function with ctx and validated inputs.
            result = await self._impl(ctx, **validated_kwargs)
            return result
        finally:
            await self._cleanup_controllers(created_controllers)

            # Clean up viewers if configured.
            if self._viewer is not None:
                from nova.viewers import _cleanup_active_viewers

                _cleanup_active_viewers()

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
        if not self.preconditions or not self.preconditions.controllers:
            return []

        created_controllers: list[str] = []
        async with Nova() as nova:
            cell = nova.cell()
            controller_config = None

            async def ensure_controller(controller_config: api.models.RobotController):
                """Ensure a controller is created and return its ID."""
                controller_name = controller_config.name or "unnamed_controller"
                self._log("info", f"Creating controller '{controller_name}'")
                try:
                    controller = await cell.ensure_controller(controller_config=controller_config)
                    created_controllers.append(controller.id)
                    self._log(
                        "info", f"Created controller '{controller_name}' with ID {controller.id}"
                    )
                    return controller.id
                except Exception as e:
                    raise ControllerCreationFailed(controller_name, str(e))

            try:
                async with asyncio.TaskGroup() as tg:
                    for controller_config in self.preconditions.controllers:
                        tg.create_task(ensure_controller(controller_config))

            except Exception as e:
                controller_name = (
                    controller_config.name if controller_config else "unnamed_controller"
                )
                raise ControllerCreationFailed(controller_name, str(e))

            # Setup viewers after controllers are created and available
            try:
                from nova.viewers import _setup_active_viewers_after_preconditions

                await _setup_active_viewers_after_preconditions()
            except ImportError as e:
                logger.error(f"Could not import viewers: {e}")

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
    def input_json_schema(self) -> dict[str, Any]:
        return self.input_model.model_json_schema() if self.input_model else {}

    @property
    def output_schema(self) -> dict[str, Any]:
        return self.output_model.model_json_schema()

    @property
    def json_schema(self, title: str | None = None) -> JsonSchemaValue:
        schemas: list[tuple[type[BaseModel], Literal["validation"]]] = []
        if self.input_model:
            schemas.append((self.input_model, "validation"))
        schemas.append((self.output_model, "validation"))
        _, top_level_schema = models_json_schema(schemas, title=title or self.name)
        return top_level_schema

    def __repr__(self) -> str:
        if self.input_model:
            input_fields = ", ".join(
                f"{k}: {v.annotation.__name__}"  # type: ignore
                for k, v in self.input_model.model_fields.items()
            )
        else:
            input_fields = "none"

        # Get the actual output type from RootModel
        if hasattr(self.output_model, "model_fields") and "root" in self.output_model.model_fields:
            root_annotation = self.output_model.model_fields["root"].annotation
            # If it's a TypeVar, get its bound type
            if hasattr(root_annotation, "__bound__") and root_annotation.__bound__:  # type: ignore
                output_type = root_annotation.__bound__.__name__  # type: ignore
            else:
                output_type = root_annotation.__name__  # type: ignore
        else:
            output_type = self.output_model.__name__

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

        if not self.input_model:
            return parser

        # Helper for JSON-typed arguments (used for complex config inputs).
        def json_type_for(field_name: str) -> Callable[[str], Any]:
            def _parse(value: str) -> Any:
                try:
                    return json.loads(value)
                except json.JSONDecodeError as e:
                    raise argparse.ArgumentTypeError(f"Invalid JSON for {field_name}: {e}")

            return _parse

        for name, field in self.input_model.model_fields.items():
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
                parser.add_argument(
                    f"--{name}",
                    dest=name,
                    type=json_type_for(name),
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

        # Additionally, provide a generic JSON `--config` argument for complex
        # configurations. This is especially useful for CLI usage where a single
        # JSON object can represent all inputs.
        parser.add_argument(
            "--config",
            dest="config",
            type=json_type_for("config"),
            required=False,
            help="Configuration object (JSON format)",
        )

        return parser


def input_and_output_types(
    func: Callable, docstring: Docstring
) -> tuple[type[BaseModel] | None, type[BaseModel]]:
    signature = inspect.signature(func)
    input_types = get_type_hints(func)
    output_type = input_types.pop("return", None)

    # Build an InputModel from all parameters *after* the first `ctx` parameter.
    input_field_definitions: dict[str, Any] = {}

    raw_doc = inspect.getdoc(func) or ""

    parameters_items = list(signature.parameters.items())
    for order, (name, parameter) in enumerate(parameters_items[1:], start=0):
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
            param_doc = next((p for p in docstring.params if p.arg_name == name), None)
            if param_doc and param_doc.description:
                default.description = param_doc.description
            else:
                # Fallback: simple parsing of "Args:" section for this parameter.
                for line in raw_doc.splitlines():
                    stripped = line.strip()
                    if stripped.startswith(f"{name}:"):
                        _, _, desc = stripped.partition(":")
                        desc = desc.strip()
                        if desc:
                            default.description = desc
                        break

        input_field_definitions[name] = (parameter.annotation, default)

    input_type = (
        create_model(
            "Input",
            **input_field_definitions,
            __module__=func.__module__,
            __config__=ConfigDict(extra="forbid"),
        )
        if input_field_definitions
        else None
    )

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

    return input_type, output


def program(
    # allows bare @nova.program
    _func: Callable[Parameters, Return] | None = None,
    *,
    id: str | None = None,
    name: str | None = None,
    description: str | None = None,
    preconditions: ProgramPreconditions | None = None,
    viewer: Any | None = None,
):
    """
    Decorator factory for creating Nova programs with declarative controller setup.

    Args:
        id: ID of the program (needs to be unique across all programs)
        name: Readable name of the program
        description: Description of the program
        preconditions: ProgramPreconditions containing controller configurations and cleanup settings
            Based on the program preconditions, a robot cell is created when running the program in a runner
            Only devices that are part of the preconditions are opened and listened for e.g. estop handling
        viewer: Optional viewer instance for program visualization (e.g., nova.viewers.Rerun())

    Decorator / decorator-factory for creating Nova programs.
        - Bare usage:        @nova.program
        - With options:      @nova.program(id="...", name="...")

    Examples:
        >>> import nova
        >>> @nova.program
        ... async def simple_program(ctx: nova.ProgramContext):
        ...     print("Hello World!")
        >>> simple_program.program_id
        'simple_program'

        >>> @nova.program(id="my_program", name="My Program")
        ... async def program_with_options(ctx: nova.ProgramContext):
        ...     print("Hello from My Program!")
        >>> program_with_options.program_id
        'my_program'
    """

    def decorator(
        function: Callable[Parameters, Return],
    ) -> Program[Parameters, Coroutine[Any, Any, Return]]:
        # Validate that the function is async
        if not asyncio.iscoroutinefunction(function):
            raise TypeError(f"Program function '{function.__name__}' must be async")

        program_obj = Program.validate(function)
        if id:
            program_obj.program_id = id
        if name:
            program_obj.name = name
        if description:
            program_obj.description = description
        program_obj.preconditions = preconditions
        program_obj._viewer = viewer
        return program_obj

    # If used as @nova.program(), return the decorator.
    # If used as @nova.program, decorate immediately.
    return decorator if _func is None else decorator(_func)
