from unittest.mock import Mock

import pytest
from pydantic import BaseModel, Field

import nova
from nova.core.nova import Nova
from nova.program.function import Program


def _connected_nova() -> Nova:
    nova_mock = Mock(spec=Nova)
    nova_mock.is_connected.return_value = True
    nova_mock.cell.return_value = None
    return nova_mock


class TestOutput(BaseModel):
    message: str = Field(..., description="Greeting message")
    __test__ = False


@pytest.mark.asyncio
async def test_function_wrapping():
    @nova.program(
        name="greet",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def greet(
        ctx,
        name: str = Field(..., description="Name of the person"),
        age: int = Field(..., description="Age of the person"),
    ) -> TestOutput:
        """Greet a person with their name and age.

        Args:
            name: Name of the person
            age: Age of the person

        Returns:
            A greeting message
        """
        return TestOutput(message=f"Hello {name}, you are {age} years old!")

    assert isinstance(greet, Program)
    assert greet.name == "greet"
    assert "Greet a person" in greet.description
    assert isinstance(greet.input_model, type(BaseModel))

    # Verify input model fields
    input_fields = greet.input_model.model_fields
    assert "name" in input_fields
    assert "age" in input_fields

    # Check name field
    name_field = input_fields["name"]
    assert name_field.annotation is str
    assert name_field.description == "Name of the person"
    assert name_field.is_required()

    # Check age field
    age_field = input_fields["age"]
    assert age_field.annotation is int
    assert age_field.description == "Age of the person"
    assert age_field.is_required()

    assert isinstance(greet.output_model, type(BaseModel))


@pytest.mark.asyncio
async def test_function_validation():
    with pytest.raises(TypeError):
        Program.validate("not a function")


@pytest.mark.asyncio
async def test_function_calling():
    class AddInput(BaseModel):
        a: int
        b: int

    @nova.program(
        name="add",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def add(ctx, a: int, b: int) -> int:
        return a + b

    result = await add(nova=_connected_nova(), a=5, b=3)
    assert result == 8


@pytest.mark.asyncio
async def test_function_with_complex_types():
    class Address(BaseModel):
        street: str = Field(..., description="Street address")
        city: str = Field(..., description="City name")

    class Person(BaseModel):
        name: str = Field(..., description="Name of the person")
        address: Address = Field(..., description="Address of the person")

    class ProcessPersonInput(BaseModel):
        person: Person = Field(..., description="Person information")

    @nova.program(
        name="process_person",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def process_person(ctx, person: Person) -> str:
        """Process a person's information.

        Args:
            person: Person information

        Returns:
            Formatted string with person's details
        """
        return f"{person.name} lives in {person.address.city}"

    person = Person(name="John", address=Address(street="123 Main St", city="New York"))
    result = await process_person(nova=_connected_nova(), person=person)
    assert "John lives in New York" in result


@pytest.mark.asyncio
async def test_function_schema_generation():
    class CalculateAreaInput(BaseModel):
        length: float
        width: float

    @nova.program(
        name="calculate_area",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def calculate_area(ctx, length: float, width: float) -> float:
        """Calculate the area of a rectangle.

        Args:
            length: Length of the rectangle
            width: Width of the rectangle

        Returns:
            Area of the rectangle
        """
        return length * width

    input_schema = calculate_area.input_json_schema
    assert "length" in input_schema["properties"]
    assert "width" in input_schema["properties"]
    assert input_schema["properties"]["length"]["type"] == "number"
    assert input_schema["properties"]["width"]["type"] == "number"

    output_schema = calculate_area.output_schema
    assert output_schema["type"] == "number"


@pytest.mark.asyncio
async def test_function_argument_parser():
    @nova.program(
        id="process_data",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def process_data(ctx, name: str, count: int = 0) -> str:
        """Process some data.

        Args:
            name: Name of the data
            count: Count of items (default: 0)

        Returns:
            Processed data string
        """
        return f"Processed {count} items of {name}"

    parser = process_data.create_parser()
    args = parser.parse_args(["--name", "test", "--count", "5"])
    assert args.name == "test"
    assert args.count == 5


@pytest.mark.asyncio
async def test_function_with_optional_parameters():
    class GreetOptionalInput(BaseModel):
        name: str
        title: str | None = None

    @nova.program(
        name="greet_optional",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def greet_optional(ctx, name: str, title: str | None = None) -> str:
        """Greet someone with an optional title.

        Args:
            name: Name of the person
            title: Optional title of the person

        Returns:
            Greeting message
        """
        if title:
            return f"Hello {title} {name}!"
        return f"Hello {name}!"

    result1 = await greet_optional(nova=_connected_nova(), name="John")
    assert result1 == "Hello John!"

    result2 = await greet_optional(nova=_connected_nova(), name="John", title="Dr.")
    assert result2 == "Hello Dr. John!"


@pytest.mark.asyncio
async def test_function_with_json_complex_types():
    @nova.program()
    async def process_config(ctx, setting1: str, setting2: int) -> str:
        """Process a configuration.

        Args:
            config: Configuration object

        Returns:
            Processed configuration string
        """
        return f"Processed {setting1} with value {setting2}"

    parser = process_config.create_parser()
    args = parser.parse_args(["--config", '{"setting1": "test", "setting2": 42}'])
    assert isinstance(args.config, dict)
    assert args.config["setting1"] == "test"
    assert args.config["setting2"] == 42


@pytest.mark.asyncio
async def test_function_repr():
    @nova.program(
        name="example_func",
        preconditions=nova.ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def example_func(ctx, x: int, y: str) -> float:
        """Example function.

        Args:
            x: First parameter
            y: Second parameter

        Returns:
            A float value
        """
        return float(x)

    func_repr = repr(example_func)
    assert "Program(name='example_func'" in func_repr
    assert "x: int" in func_repr
    assert "y: str" in func_repr
    assert "output=float" in func_repr


def test_input_schema_should_include_additional_fields_false():
    """
    Input schema for functions decorated with @nova.program
    should include "additionalProperties": false
    to prevent extra fields in input.
    """

    @nova.program()
    async def sample_function(ctx, param1: int, param2: str):
        return param1 + len(param2)

    input_schema = sample_function.input_json_schema
    assert input_schema.get("additionalProperties") is False
