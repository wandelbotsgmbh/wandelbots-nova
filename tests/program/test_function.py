import pytest
from pydantic import BaseModel, Field

from nova import Nova
from nova.program.function import Program, ProgramPreconditions, program


class TestInput(BaseModel):
    name: str = Field(..., description="Name of the person")
    age: int = Field(..., description="Age of the person")


class TestOutput(BaseModel):
    message: str = Field(..., description="Greeting message")


@pytest.mark.asyncio
async def test_function_wrapping():
    @program(
        name="greet",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def greet(
        nova: Nova,
        name: str = Field(..., description="Name of the person"),
        age: int = Field(..., description="Age of the person"),
    ) -> TestOutput:
        """Greet a person with their name and age.

        Args:
            nova: Nova instance
            name: Name of the person
            age: Age of the person

        Returns:
            A greeting message
        """
        return TestOutput(message=f"Hello {name}, you are {age} years old!")

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
    @program(
        name="add",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def add(nova: Nova, a: int, b: int) -> int:
        return a + b

    # Call without passing nova - it should be automatically injected
    result = await add(5, 3)
    assert result == 8


@pytest.mark.asyncio
async def test_function_with_complex_types():
    class Address(BaseModel):
        street: str = Field(..., description="Street address")
        city: str = Field(..., description="City name")

    class Person(BaseModel):
        name: str = Field(..., description="Name of the person")
        address: Address = Field(..., description="Address of the person")

    @program(
        name="process_person",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def process_person(nova: Nova, person: Person) -> str:
        """Process a person's information.

        Args:
            nova: Nova instance
            person: Person information

        Returns:
            Formatted string with person's details
        """
        return f"{person.name} lives in {person.address.city}"

    person = Person(name="John", address=Address(street="123 Main St", city="New York"))
    # Call without passing nova - it should be automatically injected
    result = await process_person(person)
    assert "John lives in New York" in result


@pytest.mark.asyncio
async def test_function_schema_generation():
    @program(
        name="calculate_area",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def calculate_area(nova: Nova, length: float, width: float) -> float:
        """Calculate the area of a rectangle.

        Args:
            nova: Nova instance
            length: Length of the rectangle
            width: Width of the rectangle

        Returns:
            Area of the rectangle
        """
        return length * width

    input_schema = calculate_area.input_schema
    assert "length" in input_schema["properties"]
    assert "width" in input_schema["properties"]
    assert input_schema["properties"]["length"]["type"] == "number"
    assert input_schema["properties"]["width"]["type"] == "number"

    output_schema = calculate_area.output_schema
    assert output_schema["type"] == "number"


@pytest.mark.asyncio
async def test_function_argument_parser():
    @program(
        name="process_data",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def process_data(nova: Nova, name: str, count: int = 0) -> str:
        """Process some data.

        Args:
            nova: Nova instance
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
    @program(
        name="greet_optional",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def greet_optional(nova: Nova, name: str, title: str | None = None) -> str:
        """Greet someone with an optional title.

        Args:
            nova: Nova instance
            name: Name of the person
            title: Optional title of the person

        Returns:
            Greeting message
        """
        if title:
            return f"Hello {title} {name}!"
        return f"Hello {name}!"

    # Call without passing nova - it should be automatically injected
    result1 = await greet_optional("John")
    assert result1 == "Hello John!"

    result2 = await greet_optional("John", "Dr.")
    assert result2 == "Hello Dr. John!"


@pytest.mark.asyncio
async def test_function_with_json_complex_types():
    class Config(BaseModel):
        setting1: str
        setting2: int

    @program(test_mode=True)
    async def process_config(nova: Nova, config: Config) -> str:
        """Process a configuration.

        Args:
            nova: Nova instance
            config: Configuration object

        Returns:
            Processed configuration string
        """
        return f"Processed {config.setting1} with value {config.setting2}"

    parser = process_config.create_parser()
    args = parser.parse_args(["--config", '{"setting1": "test", "setting2": 42}'])
    assert isinstance(args.config, dict)
    assert args.config["setting1"] == "test"
    assert args.config["setting2"] == 42


@pytest.mark.asyncio
async def test_function_repr():
    @program(
        name="example_func",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        test_mode=True,
    )
    async def example_func(nova: Nova, x: int, y: str) -> float:
        """Example function.

        Args:
            nova: Nova instance
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


@pytest.mark.asyncio
async def test_function_without_nova_parameter_raises_error():
    """Test that functions without nova parameter raise an error."""
    with pytest.raises(ValueError, match="must have 'nova: Nova' as the first parameter"):

        @program(test_mode=True)
        async def invalid_func(name: str) -> str:
            return f"Hello {name}"


@pytest.mark.asyncio
async def test_function_with_wrong_first_parameter_raises_error():
    """Test that functions with wrong first parameter type raise an error."""
    with pytest.raises(ValueError, match="must have 'nova: Nova' as the first parameter"):

        @program(test_mode=True)
        async def invalid_func(name: str, nova: Nova) -> str:
            return f"Hello {name}"


@pytest.mark.asyncio
async def test_function_without_parameters_raises_error():
    """Test that functions without any parameters raise an error."""
    with pytest.raises(ValueError, match="must have at least one parameter"):

        @program(test_mode=True)
        async def invalid_func() -> str:
            return "Hello"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_function_with_nova_access():
    """Test that the nova parameter is properly injected and accessible."""

    @program()
    async def test_nova_access(nova: Nova, message: str) -> str:
        """Test function that uses the nova instance.

        Args:
            nova: Nova instance
            message: Message to return

        Returns:
            Message with nova instance info
        """
        # Verify nova is a Nova instance
        assert isinstance(nova, Nova)
        return f"{message} - Nova instance available"

    result = await test_nova_access("Hello")
    assert "Hello - Nova instance available" in result
