from wandelscript.metamodel import register_builtin_func
from wandelscript.runtime import ExecutionContext


@register_builtin_func(pass_context=True)
def experimental_set_robot_as_default(context, robot):
    context.default_robot_identifier = robot


@register_builtin_func(pass_context=True)
async def get_controller(context: ExecutionContext, name: str):
    """Retrieve a controller reference by its name"""
    return context.robot_cell.get_controller(name)
