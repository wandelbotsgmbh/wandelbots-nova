import pytest
from pydantic import BaseModel, Field

from nova.runtime.function import Function, wrap


class TestInput(BaseModel):
    name: str = Field(..., description="Name of the person")
    age: int = Field(..., description="Age of the person")


class TestOutput(BaseModel):
    message: str = Field(..., description="Greeting message")


def test_function_wrapping():
    @wrap
    def greet(
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

    assert isinstance(greet, Function)
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


def test_function_validation():
    with pytest.raises(TypeError):
        Function.validate("not a function")


def test_function_calling():
    @wrap
    def add(a: int, b: int) -> int:
        return a + b

    result = add(5, 3)
    assert result == 8


def test_function_with_complex_types():
    class Address(BaseModel):
        street: str = Field(..., description="Street address")
        city: str = Field(..., description="City name")

    class Person(BaseModel):
        name: str = Field(..., description="Name of the person")
        address: Address = Field(..., description="Address of the person")

    @wrap
    def process_person(person: Person) -> str:
        """Process a person's information.

        Args:
            person: Person information

        Returns:
            Formatted string with person's details
        """
        return f"{person.name} lives in {person.address.city}"

    person = Person(name="John", address=Address(street="123 Main St", city="New York"))
    result = process_person(person)
    assert "John lives in New York" in result


def test_function_schema_generation():
    @wrap
    def calculate_area(length: float, width: float) -> float:
        """Calculate the area of a rectangle.

        Args:
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


def test_function_argument_parser():
    @wrap
    def process_data(name: str, count: int = 0) -> str:
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


def test_function_with_optional_parameters():
    @wrap
    def greet_optional(name: str, title: str | None = None) -> str:
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

    result1 = greet_optional("John")
    assert result1 == "Hello John!"

    result2 = greet_optional("John", "Dr.")
    assert result2 == "Hello Dr. John!"


def test_function_with_json_complex_types():
    class Config(BaseModel):
        setting1: str
        setting2: int

    @wrap
    def process_config(config: Config) -> str:
        """Process a configuration.

        Args:
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


def test_function_repr():
    @wrap
    def example_func(x: int, y: str) -> float:
        """Example function.

        Args:
            x: First parameter
            y: Second parameter

        Returns:
            A float value
        """
        return float(x)

    func_repr = repr(example_func)
    assert "Function(name='example_func'" in func_repr
    assert "x: int" in func_repr
    assert "y: str" in func_repr
    assert "output=float" in func_repr
