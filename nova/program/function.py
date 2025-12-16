import argparse
import asyncio
import inspect
import json
import logging
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
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    PrivateAttr,
    RootModel,
    create_model,
    validate_call,
)
from pydantic.fields import FieldInfo
from pydantic.json_schema import JsonSchemaValue, models_json_schema

from nova import Nova, api
from nova.exceptions import ControllerCreationFailed

logger = logging.getLogger(__name__)

Parameters = ParamSpec("Parameters")
Return = TypeVar("Return")


def _param_is_ctx(param: inspect.Parameter) -> bool:
    return param.annotation is NovaProgramContext or param.name == "ctx"


def _param_is_inputs(param: inspect.Parameter) -> bool:
    if _param_is_ctx(param):
        return False
    if param.name in {"inputs", "input"}:
        return True
    if isinstance(param.annotation, type) and issubclass(param.annotation, BaseModel):
        return True
    return True


class ProgramPreconditions(BaseModel):
    controllers: list[api.models.RobotController] | None = None
    cleanup_controllers: bool = False


class NovaProgramContext:
    """Context passed into every program execution."""

    def __init__(self, nova: Nova):
        self.nova = nova


"""
## Define programs:

# Program without inputs or context
async def program():
    ...

async def program(inputs: InputModel):
    ...

# Program with inputs and context
async def program(inputs: InputModel, ctx: NovaProgramContext):
    ...

# Program with context keyword-only (do we need this?)
async def program(*, ctx: NovaProgramContext):
    ...

### Con
- "*" for using only ctx
- avoid too much magic


## Call programs:

await program(inputs=InputModel())
await program()

"""


class Program(BaseModel, Generic[Parameters, Return]):
    _wrapped: Callable[..., Any] = PrivateAttr(default_factory=lambda: lambda *args, **kwargs: None)
    _impl: Callable[..., Coroutine[Any, Any, Return]] = PrivateAttr(
        default_factory=lambda: lambda *args, **kwargs: None
    )
    program_id: str
    name: str | None
    description: str | None
    input_model: type[BaseModel] | None
    output_model: type[BaseModel]
    preconditions: ProgramPreconditions | None = None

    @classmethod
    def validate(
        cls, value: Callable[Parameters, Return], *, input_model: type[BaseModel] | None = None
    ) -> "Program[Parameters, Return]":
        if isinstance(value, Program):
            return value

        program_id = value.__name__
        docstring = parse_docstring(value.__doc__ or "")
        description = docstring.description

        input_model_, output_type = input_and_output_types(value, docstring, input_model=input_model)

        program = cls(
            program_id=program_id,
            name=None,
            description=description,
            input_model=input_model_,
            output_model=output_type,
        )

        impl = validate_call(validate_return=True, config={"arbitrary_types_allowed": True})(value)
        program._impl = impl
        program._wrapped = impl

        params = list(inspect.signature(value).parameters.values())
        if len(params) > 2:
            raise TypeError("Program functions may have at most two parameters (inputs, ctx).")

        count_ctx = sum(1 for p in params if _param_is_ctx(p))
        count_inputs = sum(1 for p in params if _param_is_inputs(p))
        if count_ctx > 1 or count_inputs > 1:
            raise TypeError("Program functions may only declare one ctx and one inputs parameter.")

        if program.input_model is None and count_inputs:
            raise TypeError("This program declares no inputs; remove the inputs parameter.")
        if program.input_model is not None and count_inputs == 0:
            raise TypeError("Program declares an input_model but no inputs parameter.")

        return program

    def _build_input_model(self, inputs: BaseModel | Mapping[str, Any] | None) -> BaseModel | None:
        if self.input_model is None:
            if inputs is not None:
                raise TypeError("This program does not accept inputs.")
            return None

        if inputs is None:
            raise TypeError("Missing required inputs.")

        if isinstance(inputs, BaseModel):
            values = inputs.model_dump()
        elif isinstance(inputs, Mapping):
            values = dict(inputs)
        else:
            raise TypeError("inputs must be a mapping or BaseModel instance")
        return self.input_model.model_validate(values)

    async def _invoke_declared_signature(
        self, *, inputs: BaseModel | None, ctx: NovaProgramContext, extra_kwargs: dict[str, Any]
    ) -> Return:
        sig = inspect.signature(self._impl)
        params = list(sig.parameters.values())
        call_kwargs: dict[str, Any] = {}

        for p in params:
            if _param_is_ctx(p):
                call_kwargs[p.name] = ctx
            elif _param_is_inputs(p):
                call_kwargs[p.name] = inputs
            elif p.kind == inspect.Parameter.VAR_KEYWORD:
                call_kwargs.update(extra_kwargs)
            elif p.name in extra_kwargs:
                call_kwargs[p.name] = extra_kwargs[p.name]

        return await self._impl(**call_kwargs)

    async def invoke(
        self,
        inputs: BaseModel | Mapping[str, Any] | None = None,
        ctx: NovaProgramContext | None = None,
        **kwargs: Any,
    ) -> Return:
        validated_inputs = self._build_input_model(inputs)
        ctx_instance = ctx or NovaProgramContext(nova=Nova())
        return await self._wrapped(ctx_instance, validated_inputs, **kwargs)

    async def __call__(self, *args: Parameters.args, **kwargs: Parameters.kwargs) -> Return:  # pylint: disable=no-member
        inputs = kwargs.pop("inputs", None)
        ctx = kwargs.pop("ctx", None)
        nova_override = kwargs.pop("nova", None)
        if args:
            if len(args) == 1 and isinstance(args[0], BaseModel):
                inputs = args[0]
            else:
                raise TypeError("Use keyword arguments inputs=... and ctx=... to call a program.")
        if inputs is None and kwargs:
            inputs = kwargs
            kwargs = {}
        if ctx is None and nova_override is not None:
            ctx = NovaProgramContext(nova=nova_override)
        return await self.invoke(inputs=inputs, ctx=ctx, **kwargs)

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
    def input_schema(self) -> dict[str, Any]:
        return self.input_model.model_json_schema() if self.input_model else {}

    @property
    def output_schema(self) -> dict[str, Any]:
        return self.output_model.model_json_schema()

    @property
    def json_schema(self, title: str | None = None) -> JsonSchemaValue:
        schemas: list[tuple[type[BaseModel], str]] = []
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
    func: Callable, docstring: Docstring, *, input_model: type[BaseModel] | None = None
) -> tuple[type[BaseModel] | None, type[BaseModel]]:
    signature = inspect.signature(func)
    input_types = get_type_hints(func)
    output_type = input_types.pop("return", None)

    if input_model is not None:
        input_type: type[BaseModel] | None = create_model(
            input_model.__name__,
            __base__=input_model,
            __module__=input_model.__module__,
            __config__=ConfigDict(extra="forbid"),
        )
    else:
        input_field_definitions: Mapping[str, Any] = {}
        for order, (name, parameter) in enumerate(signature.parameters.items()):
            if parameter.annotation is NovaProgramContext:
                continue

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
    input_model: type[BaseModel] | None = None,
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
        input_model: Optional pydantic BaseModel describing the program inputs.
            The model instance is available on NovaProgramContext.inputs. If not provided,
            the input model is inferred from the function signature.

    Decorator / decorator-factory for creating Nova programs.
        - Bare usage:        @nova.program
        - With options:      @nova.program(id="...", name="...")

    Examples:
        >>> import nova
        >>> @program
        ... async def simple_program():
        ...     print("Hello World!")
        >>> simple_program.program_id
        'simple_program'

        >>> @program(id="my_program", name="My Program")
        ... async def program_with_options():
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

        program_obj = Program.validate(function, input_model=input_model)
        if id:
            program_obj.program_id = id
        if name:
            program_obj.name = name
        if description:
            program_obj.description = description
        program_obj.preconditions = preconditions

        # Create a wrapper that handles controller lifecycle
        original_wrapped = program_obj._wrapped
        user_parameters = list(inspect.signature(original_wrapped).parameters.values())
        # Check if the function expects a NovaProgramContext or a ctx parameter
        expects_nova_context = any(_param_is_ctx(param) for param in user_parameters)

        async def async_wrapper(
            ctx: NovaProgramContext,
            inputs: BaseModel | None,
            *args: Parameters.args,
            **kwargs: Parameters.kwargs,
        ) -> Return:
            """Async wrapper that handles controller creation and cleanup."""
            created_controllers = []
            try:
                # Create controllers before execution
                created_controllers = await program_obj._create_controllers()

                # Configure viewers if any are active
                if viewer is not None:
                    # Configure the viewer when Nova instance becomes available in the function
                    # This will be done via a hook in the Nova context manager
                    pass

                # Execute the wrapped function using declared signature
                result = await program_obj._invoke_declared_signature(
                    inputs=inputs, ctx=ctx, extra_kwargs=kwargs
                )
                return result
            finally:
                # Clean up controllers after execution
                await program_obj._cleanup_controllers(created_controllers)

                # Clean up viewers
                if viewer is not None:
                    from nova.viewers import _cleanup_active_viewers

                    _cleanup_active_viewers()

        # Update the wrapped function to our async wrapper
        program_obj._wrapped = async_wrapper
        return program_obj

    # If used as @nova.program(), return the decorator.
    # If used as @nova.program, decorate immediately.
    return decorator if _func is None else decorator(_func)
