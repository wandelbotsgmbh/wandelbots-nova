# Wandelscript

## Introduction

Wandelscript is a robust framework designed for defining, parsing, type checking, and executing
advanced scripting capabilities for robot automation. It integrates grammar-based parsing, runtime
execution, and plugin extensibility, making it suitable for complex robot programming scenarios.

## Features

- **Grammar Parsing**: The Wandelscript grammar is defined and easily extensible. It is powered by ANTLR4 to generate efficient parsers.
- **Runtime Execution**: Includes a runtime environment capable of executing Wandelscript programs with support for custom plugins and built-in functions.
- **Extensible Plugins**: Supports adding custom functionality via Python-based plugins for robot control, kinematics, vision, and more.
- **Built-in Libraries**: Provides libraries for mathematical operations, string manipulations, pose interpolations, and interaction with industrial systems like OPC UA.
- **Examples**: Comprehensive example scripts are available to demonstrate use cases such as collision handling, multi-robot coordination, and TCP manipulation.
- **Testing**: Includes a set of unit tests to validate the functionality of the built-in components and plugins.

## Grammar

The Wandelscript grammar is defined under `wandelscript/grammar/`. To generate the parser, use the
following command:
```bash
cd wandelscript/grammar && uv run antlr4 -Dlanguage=Python3 -visitor *.g4
```

## Structure

The repository is organized as follows:

- **Core Modules**:
  - `runtime.py`: Implements the execution context for Wandelscript.
  - `exception.py`: Defines custom exceptions for Wandelscript.
  - `models.py`: Contains data models for serialization and execution.
  - `metamodel.py`: Facilitates runtime extensions and plugin registration.
- **Built-ins**: Found under `wandelscript/builtins`, these are pre-defined plugins for:
  - Math operations (`math.py`)
  - String manipulation (`string.py`)
  - Robotics control (`controller.py`)
- **Examples**: Demonstrates Wandelscript's capabilities with real-world use cases in `wandelscript/examples/`.

## Dependencies

The project uses the `uv` package manager. Install the dependencies using:
```bash
uv sync
```

Key dependencies:
- Python 3.10+
- [ANTLR4](https://www.antlr.org/) for grammar processing
- [wandelbots-nova](https://pypi.org/project/wandelbots-nova/) for data types and interaction with robots
- [Pydantic](https://docs.pydantic.dev/) for data validation

## Usage

1. **CLI Tool**: Use the `wandelscript` CLI tool
2. **Extend Functionality**: Add new plugins or built-ins to customize the scripting environment.
3. **Examples**: Refer to the `/examples` folder for sample Wandelscript files.

### CLI Tool

There is a CLI tool `wandelscript`, as well as its shortcut pendant `ws` that gets installed into
the Python environment when calling `uv sync`.

Example uses of the `wandelscript` CLI tool:
```bash
uv run wandelscript --help
uv run wandelscript my_script.cli
uv run ws my_script.cli  # shortcut alternative to `uv run wandelscript`
```

See `uv run wandelscript --help` for information about available CLI arguments.

## Contributing

Contributions are welcome! Here’s how you can help:
- Report bugs or suggest features via the issue tracker.
- Submit pull requests to improve code or documentation.
- Share example use cases or applications of Wandelscript.

## Contact

Authors and contributors:
- Christoph Biering: [Email](mailto:christoph.biering@wandelbots.com)
- Daniel Vorberg
- Dirk Sonnemann: [Email](mailto:dirk.sonnemann@wandelbots.com)
- Andreas Langenhagen: [Email](mailto:andreas.langenhagen@wandelbots.com)
- Mahsum Demir: [Email](mailto:mahsum.demir@wandelbots.com)

Feel free to reach out with questions, suggestions, or feedback!
