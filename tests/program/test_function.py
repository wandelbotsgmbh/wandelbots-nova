import pytest
from pydantic import BaseModel, Field

import nova
from nova.program.function import NovaProgramContext, Program, ProgramPreconditions


class TestInput(BaseModel):
    name: str = Field(..., description="Name of the person")
    age: int = Field(..., description="Age of the person")
    __test__ = False


class TestOutput(BaseModel):
    message: str = Field(..., description="Greeting message")
    __test__ = False


@pytest.mark.asyncio
async def test_function_wrapping():
    @nova.program(
        name="greet",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=TestInput,
    )
    async def greet(inputs: TestInput, ctx: NovaProgramContext) -> TestOutput:
        """Greet a person with their name and age.

        Args:
            name: Name of the person
            age: Age of the person

        Returns:
            A greeting message
        """
        return TestOutput(message=f"Hello {inputs.name}, you are {inputs.age} years old!")

    assert isinstance(greet, Program)
    assert greet.name == "greet"
    assert "Greet a person" in greet.description
    assert isinstance(greet.input, type(BaseModel))

    # Verify input model fields
    input_fields = greet.input.model_fields
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

    assert isinstance(greet.output, type(BaseModel))


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
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=AddInput,
    )
    async def add(inputs: AddInput, ctx: NovaProgramContext) -> int:
        return inputs.a + inputs.b

    result = await add(a=5, b=3)
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
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=ProcessPersonInput,
    )
    async def process_person(inputs: ProcessPersonInput, ctx: NovaProgramContext) -> str:
        """Process a person's information.

        Args:
            person: Person information

        Returns:
            Formatted string with person's details
        """
        return f"{inputs.person.name} lives in {inputs.person.address.city}"

    person = Person(name="John", address=Address(street="123 Main St", city="New York"))
    result = await process_person(person=person)
    assert "John lives in New York" in result


@pytest.mark.asyncio
async def test_function_schema_generation():
    class CalculateAreaInput(BaseModel):
        length: float
        width: float

    @nova.program(
        name="calculate_area",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=CalculateAreaInput,
    )
    async def calculate_area(inputs: CalculateAreaInput, ctx: NovaProgramContext) -> float:
        """Calculate the area of a rectangle.

        Args:
            length: Length of the rectangle
            width: Width of the rectangle

        Returns:
            Area of the rectangle
        """
        return inputs.length * inputs.width

    input_schema = calculate_area.input_schema
    assert "length" in input_schema["properties"]
    assert "width" in input_schema["properties"]
    assert input_schema["properties"]["length"]["type"] == "number"
    assert input_schema["properties"]["width"]["type"] == "number"

    output_schema = calculate_area.output_schema
    assert output_schema["type"] == "number"


@pytest.mark.asyncio
async def test_function_argument_parser():
    class ProcessDataInput(BaseModel):
        name: str
        count: int = 0

    @nova.program(
        id="process_data",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=ProcessDataInput,
    )
    async def process_data(inputs: ProcessDataInput, ctx: NovaProgramContext) -> str:
        """Process some data.

        Args:
            name: Name of the data
            count: Count of items (default: 0)

        Returns:
            Processed data string
        """
        return f"Processed {inputs.count} items of {inputs.name}"

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
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=GreetOptionalInput,
    )
    async def greet_optional(inputs: GreetOptionalInput, ctx: NovaProgramContext) -> str:
        """Greet someone with an optional title.

        Args:
            name: Name of the person
            title: Optional title of the person

        Returns:
            Greeting message
        """
        if inputs.title:
            return f"Hello {inputs.title} {inputs.name}!"
        return f"Hello {inputs.name}!"

    result1 = await greet_optional(name="John")
    assert result1 == "Hello John!"

    result2 = await greet_optional(name="John", title="Dr.")
    assert result2 == "Hello Dr. John!"


@pytest.mark.asyncio
async def test_function_with_json_complex_types():
    class Config(BaseModel):
        setting1: str
        setting2: int

    class ConfigInput(BaseModel):
        config: Config

    @nova.program(input_model=ConfigInput)
    async def process_config(inputs: ConfigInput, ctx: NovaProgramContext) -> str:
        """Process a configuration.

        Args:
            config: Configuration object

        Returns:
            Processed configuration string
        """
        return f"Processed {inputs.config.setting1} with value {inputs.config.setting2}"

    parser = process_config.create_parser()
    args = parser.parse_args(["--config", '{"setting1": "test", "setting2": 42}'])
    assert isinstance(args.config, dict)
    assert args.config["setting1"] == "test"
    assert args.config["setting2"] == 42


@pytest.mark.asyncio
async def test_function_repr():
    class ExampleInput(BaseModel):
        x: int
        y: str

    @nova.program(
        name="example_func",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=ExampleInput,
    )
    async def example_func(inputs: ExampleInput, ctx: NovaProgramContext) -> float:
        """Example function.

        Args:
            x: First parameter
            y: Second parameter

        Returns:
            A float value
        """
        return float(inputs.x)

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

    class SampleInput(BaseModel):
        param1: int
        param2: str

    @nova.program(input_model=SampleInput)
    async def sample_function(inputs: SampleInput, ctx: NovaProgramContext):
        return inputs.param1 + len(inputs.param2)

    input_schema = sample_function.input_schema
    assert input_schema.get("additionalProperties") is False
