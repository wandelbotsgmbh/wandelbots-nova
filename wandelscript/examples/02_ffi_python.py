from dataclasses import dataclass

from pydantic import BaseModel

from wandelscript import ffi
from wandelscript.datatypes import as_builtin_type
from wandelscript.runtime import ExecutionContext


# This is how you mark a function as a foreign function.
# Marked functions will be available in Wandelscript.
# The function name will be the same as the Python function name.
@ffi.foreign_function()
def basic():
    return "Hello, world!"


# If you want your python function to have a different name in wandelscript,
# use the name parameter of the decorator.
@ffi.foreign_function(name="wandelscript_name")
def python_name():
    return "Hello, Wandelscript!"


# If your function needs access to the wandelscript execution context, you can set
# pass_context=True and the context will be passed as the first argument to your function.
@ffi.foreign_function(pass_context=True)
def function_using_context(ctx: ExecutionContext):
    # Be careful with the power the access to the context provides.
    # For example, you can access the robot cell and the action queue and use it to
    # push motions or attache actions to it.
    # This here just returns the location in the wandelscript code where this function was called.
    return ctx.location_in_code


# The scalar types in Wandelscript are the same as in Python.
# If you want to use a your own custom types, you can use dataclasses or pydantic models.
# The decorator will automatically convert function arguments from Wandelscipt records
# to the type the parameter expects. And it will convert return values to Wandelscript records.
class CustomTypePydantic(BaseModel):
    str_attr: str
    float_attr: float


@dataclass
class CustomTypeDataclass:
    str_attr: str
    int_attr: int


@ffi.foreign_function()
def dataclass_to_pydantic(input: CustomTypeDataclass):
    return CustomTypePydantic(str_attr=input.str_attr, float_attr=float(input.int_attr))


# You can disable the automatic conversion by passing autoconvert_types=False to the decorator.
# This is useful if you just want to pass the data from one function to another without
# accessing it in Wandelscript.
@ffi.foreign_function(autoconvert_types=False)
def get_custom_dataclass():
    return CustomTypeDataclass("Answer", 42)


# If you want the user to be able to create instances of your custom type in Wandelscript,
# you currently have to provide a constructor function, like so:
@ffi.foreign_function(autoconvert_types=False)
def dataclass_from_params(str_attr: str, int_attr: int):
    return CustomTypeDataclass(str_attr=str_attr, int_attr=int_attr)


# Or like this:
@ffi.foreign_function(autoconvert_types=False)
def dataclass_from_record(record: dict):
    return CustomTypeDataclass(**record)


# Returning a list of custom types is also possible.
@ffi.foreign_function()
def get_list_of_customs(length: int):
    return [CustomTypeDataclass(f"Value {i}", i) for i in range(length)]


# But passing collections of custom types requires the use of constructor functions.
@ffi.foreign_function(autoconvert_types=False)
def receive_record_of_customs(input: dict[str, CustomTypeDataclass]):
    print(input)


# If you don't want to use the autoconvert_types feature but still want to return
# a builtin type, you can use the as_builtin_type function directly.
@ffi.foreign_function(autoconvert_types=False)
def return_builtin_type():
    return as_builtin_type(CustomTypePydantic(str_attr="Have some Pi", float_attr=3.14))
