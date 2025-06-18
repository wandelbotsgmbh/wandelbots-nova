from nova.actions import CombinedActions
from nova.cell.robot_cell import AbstractRobot
from wandelscript.metamodel import register_builtin_func
from wandelscript.runtime import ExecutionContext


@register_builtin_func(pass_context=True)
async def motion_trajectory_to_json_string(context: ExecutionContext, robot: AbstractRobot):
    robot_name = robot.configuration.id
    # pylint: disable=protected-access
    return context.action_queue._record[robot_name].model_dump_json()


@register_builtin_func(pass_context=True)
def motion_trajectory_from_json_string(
    context: ExecutionContext, robot: AbstractRobot, json_string: str, tcp_name: str | None = None
):
    trajectory = CombinedActions.model_validate_json(json_string)
    context.action_queue._record[robot.configuration.id] = trajectory  # pylint: disable=protected-access
    if tcp_name is not None:
        context.action_queue._tcp[robot.configuration.id] = tcp_name  # pylint: disable=protected-access
