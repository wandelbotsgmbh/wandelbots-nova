# e.g. PYTHONPATH=. poetry run python scripts/create_json_schema.py examples/10_standalone_program.py
import ast
import importlib.util
import json
import sys
from pathlib import Path


def find_main_function(file_path: Path) -> str | None:
    """Find the main function in the given file."""
    with open(file_path, "r") as f:
        tree = ast.parse(f.read())

    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "main":
            return "main"
    return None


def generate_schema_from_file(file_path: Path) -> None:
    """Generate JSON schema from a Python file containing a main function.

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

    # Find the main function
    func_name = find_main_function(file_path)
    if not func_name:
        raise ValueError(f"No main function found in {file_path}")

    # Get the function from the module
    main_func = getattr(module, func_name)

    # Generate the schema
    schema = main_func.json_schema

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

    generate_schema_from_file(file_path)


if __name__ == "__main__":
    main()
