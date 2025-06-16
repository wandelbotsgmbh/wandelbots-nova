# run this script with `wandelscript -i 02_ffi_python.py 02_ffi_python.ws`

print(basic())
# > Hello, world!

print(wandelscript_name())
# > Hello, Wandelscript!

print(function_using_context().start.line)
# > 9

print(dataclass_to_pydantic({str_attr: "Answer", int_attr: 42}))
# > {'str_attr': 'Answer', 'float_attr': 42.0}

print(dataclass_to_pydantic(get_custom_dataclass()))
# > {'str_attr': 'Answer', 'float_attr': 42.0}

# If you have a foreign function returning a custom type and with autoconversion disabled, 
# you can use `as_builtin_type` to convert it to a builtin type and use it as a record.
custom = get_custom_dataclass()
print(custom)
# > CustomTypeDataclass(str_attr='Answer', int_attr=42)
record = as_builtin_type(custom)
print(record)
# > {'str_attr': 'Answer', 'int_attr': 42}
print(record.str_attr)
# > Answer

print(dataclass_from_params("New Answer", 21))
# > CustomTypeDataclass(str_attr='New Answer', int_attr=21)

print(dataclass_from_record(record))
# > CustomTypeDataclass(str_attr='Answer', int_attr=42)

print(get_list_of_customs(3))
# > ({'str_attr': 'Value 0', 'int_attr': 0}, {'str_attr': 'Value 1', 'int_attr': 1}, {'str_attr': 'Value 2', 'int_attr': 2})

receive_record_of_customs(
    {
        custom1: dataclass_from_record({str_attr: 'Value 0', int_attr: 0}),
        custom2: dataclass_from_record({str_attr: 'Value 1', int_attr: 1})
    }
)
# > {'custom1': CustomTypeDataclass(str_attr='Value 0', int_attr=0), 'custom2': CustomTypeDataclass(str_attr='Value 1', int_attr=1)}

print(return_builtin_type())
# > {'str_attr': 'Have some Pi', 'float_attr': 3.14}