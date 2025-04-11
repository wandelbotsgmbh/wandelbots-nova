import ast
import importlib.util
import json
import sys
from pathlib import Path


class PydanticClassFinder(ast.NodeVisitor):
    def __init__(self):
        self.program_parameter_class = None

    def visit_ClassDef(self, node):
        # Check if the class inherits from nova.ProgramParameter or ProgramParameter
        for base in node.bases:
            if (isinstance(base, ast.Attribute) and base.attr == "ProgramParameter") or (
                isinstance(base, ast.Name) and base.id == "ProgramParameter"
            ):
                self.program_parameter_class = node.name


def find_program_parameter_class(file_path: Path) -> str | None:
    """Find the ProgramParameter class name in the given file."""
    with open(file_path, "r") as f:
        tree = ast.parse(f.read())

    finder = PydanticClassFinder()
    finder.visit(tree)
    return finder.program_parameter_class


def generate_schema_from_file(file_path: Path) -> None:
    """Generate JSON schema from a Python file containing a ProgramParameter class.

    Args:
        file_path: Path to the Python file
    """
    # Import the module dynamically
    spec = importlib.util.spec_from_file_location("dynamic_module", file_path)
    if not spec or not spec.loader:
        raise ImportError(f"Could not load module from {file_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules["dynamic_module"] = module
    spec.loader.exec_module(module)

    # Find the ProgramParameter class name
    class_name = find_program_parameter_class(file_path)
    if not class_name:
        raise ValueError(f"No ProgramParameter class found in {file_path}")

    # Get the class from the module
    parameter_class = getattr(module, class_name)

    # Generate the schema
    schema = parameter_class.model_json_schema()

    # Create output filename
    output_path = file_path.with_suffix(".json")

    # Write the schema to file
    with open(output_path, "w") as f:
        json.dump(schema, f, indent=2)

    print(f"Schema generated: {output_path}")


def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <path_to_python_file>")
        sys.exit(1)

    file_path = Path(sys.argv[1])

    if not file_path.exists():
        print(f"File not found: {file_path}")
        sys.exit(1)

    try:
        generate_schema_from_file(file_path)
    except Exception as e:
        print(f"Error generating schema: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
