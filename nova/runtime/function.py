import argparse
import inspect
import json
from collections.abc import Callable, Mapping
from typing import (
    Annotated,
    Any,
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

Parameters = ParamSpec("Parameters")
Return = TypeVar("Return")


class Function(BaseModel, Generic[Parameters, Return]):
    _wrapped: Callable[Parameters, Return] = PrivateAttr(  # type: ignore
        default_factory=lambda: lambda *args, **kwargs: None
    )
    name: str
    description: str | None
    input: type[BaseModel]
    output: type[BaseModel]

    @classmethod
    def validate(cls, value: Callable[Parameters, Return]) -> "Function[Parameters, Return]":
        if isinstance(value, Function):
            return value
        if not callable(value):
            raise TypeError("value must be callable")

        name = value.__name__

        docstring = parse_docstring(value.__doc__ or "")
        description = docstring.description

        input, output = input_and_output_types(value, docstring)

        function = cls(name=name, description=description, input=input, output=output)
        function._wrapped = validate_call(validate_return=True)(value)
        return function

    def __call__(self, *args: Parameters.args, **kwargs: Parameters.kwargs) -> Return:  # pylint: disable=no-member
        return self._wrapped(*args, **kwargs)

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
            f"Function(name='{self.name}'{desc_part}, input=({input_fields}), output={output_type})"
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


def wrap(function: Callable[Parameters, Return]) -> Function[Parameters, Return]:
    return Function.validate(function)
