from wandelscript.metamodel import register_builtin_func


@register_builtin_func()
async def wait_for_bool_io(device, io_id: str, value: bool):
    """Wait for a boolean value on an IO"""
    await device.wait_for_bool_io(io_id, value)
