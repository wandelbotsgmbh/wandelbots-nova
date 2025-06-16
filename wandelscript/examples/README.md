# Wandelscript Examples

This directory contains examples of Wandelscript code. Each example is a separate file that demonstrates a 
specific feature or use case of Wandelscript.

## Examples

To run the examples you can add the 

### 01_basic

Run the example with

```bash
NOVA_API=<nova_api> uv run python /examples/01_basic.py
```

### 02_ffi_python

Run the example with

```bash
uv run python -m wandelscript.cli -n <nova_api> -i examples/02_ffi_python.py examples/02_ffi_python.ws
```
