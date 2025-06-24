"""
Command line interface utilities for program execution.

This module provides utilities to parse command line arguments and create
Pydantic model instances from them.
"""

import argparse
import json
import sys
from typing import TypeVar, Type

from pydantic import BaseModel

T = TypeVar('T', bound=BaseModel)


def create_parser_for_model(model_class: Type[T]) -> argparse.ArgumentParser:
    """
    Create an ArgumentParser for a Pydantic model.
    
    Args:
        model_class: The Pydantic model class to create parser for
        
    Returns:
        Configured ArgumentParser instance
    """
    
    parser = argparse.ArgumentParser(
        description="Argument parser for Pydantic model",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  # Using JSON string:
  python script.py --json '{_get_example_json(model_class)}'
  
  # Using individual arguments:
  python script.py {_get_example_args(model_class)}
        """
    )
    
    # Add JSON argument
    parser.add_argument(
        "--json",
        type=str,
        help="JSON string containing the program model data"
    )
    
    # Add individual arguments based on model fields
    _add_model_fields_to_parser(parser, model_class)
    
    return parser


def _add_model_fields_to_parser(parser: argparse.ArgumentParser, model_class: Type[BaseModel]) -> None:
    """Add individual field arguments to parser based on model fields."""
    model_fields = model_class.model_fields
    
    for field_name, field_info in model_fields.items():
        arg_name = f"--{field_name.replace('_', '-')}"
        field_type = field_info.annotation
        
        # Handle basic types
        if field_type is int:
            parser.add_argument(arg_name, type=int, help=f"{field_name} (integer)")
        elif field_type is float:
            parser.add_argument(arg_name, type=float, help=f"{field_name} (float)")
        elif field_type is str:
            parser.add_argument(arg_name, type=str, help=f"{field_name} (string)")
        elif field_type is bool:
            parser.add_argument(arg_name, action='store_true', help=f"{field_name} (boolean)")
        else:
            # For complex types, treat as string and let Pydantic handle validation
            parser.add_argument(arg_name, type=str, help=f"{field_name} ({field_type})")


def _get_example_json(model_class: Type[BaseModel]) -> str:
    """Generate example JSON for help text."""
    example_data = {}
    model_fields = model_class.model_fields
    
    for field_name, field_info in model_fields.items():
        field_type = field_info.annotation
        if field_type is int:
            example_data[field_name] = 1
        elif field_type is float:
            example_data[field_name] = 1.0
        elif field_type is str:
            example_data[field_name] = "example"
        elif field_type is bool:
            example_data[field_name] = True
        else:
            example_data[field_name] = "value"
    
    return json.dumps(example_data)


def _get_example_args(model_class: Type[BaseModel]) -> str:
    """Generate example individual arguments for help text."""
    model_fields = model_class.model_fields
    args = []
    
    for field_name, field_info in model_fields.items():
        arg_name = f"--{field_name.replace('_', '-')}"
        field_type = field_info.annotation
        
        if field_type is int:
            args.append(f"{arg_name} 1")
        elif field_type is float:
            args.append(f"{arg_name} 1.0")
        elif field_type is str:
            args.append(f"{arg_name} example")
        elif field_type is bool:
            args.append(arg_name)
        else:
            args.append(f"{arg_name} value")
    
    return " ".join(args)


# TODO: the idea is that we somehow pass data into the program run, there can be several ways of doing this, passing from command line looks like the fastest way.
def parse_model_from_args(model_class: Type[T]) -> T:
    """
    Parse command line arguments and create a model instance.
    
    Args:
        model_class: The Pydantic model class to create
        program_name: Optional program name for help text
        
    Returns:
        Instance of the model class
        
    Raises:
        SystemExit: If parsing fails or validation errors occur
    """
    parser = create_parser_for_model(model_class)
    args = parser.parse_args()
    
    # Create model from parsed arguments
    if args.json:
        # JSON mode - ignore individual arguments
        try:
            model_data = json.loads(args.json)
            return model_class(**model_data)
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON: {e}", file=sys.stderr)
            sys.exit(1)
        except Exception as e:
            print(f"Error creating model from JSON: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        # Individual arguments mode
        model_data = {}
        model_fields = model_class.model_fields
        
        for field_name in model_fields.keys():
            arg_name = field_name.replace('_', '-')
            # Get the actual argument value using the dashed name
            arg_value = getattr(args, arg_name.replace('-', '_'), None)
            
            if arg_value is not None:
                model_data[field_name] = arg_value
        
        # Check if we have all required fields
        required_fields = [
            name for name, field_info in model_fields.items() 
            if field_info.is_required()
        ]
        
        missing_fields = [field for field in required_fields if field not in model_data]
        if missing_fields:
            print(f"Error: Missing required fields: {', '.join(missing_fields)}", file=sys.stderr)
            print("Use --help for usage information", file=sys.stderr)
            print("You can use --json for JSON input or provide all individual arguments", file=sys.stderr)
            sys.exit(1)
        
        try:
            return model_class(**model_data)
        except Exception as e:
            print(f"Error creating model: {e}", file=sys.stderr)
            sys.exit(1)
    
