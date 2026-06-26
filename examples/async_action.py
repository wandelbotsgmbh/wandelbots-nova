"""
Example: Execute async actions during trajectory execution.

This example shows how to:
- register async action handlers
- trigger non-blocking handlers while the robot keeps moving
- use await_action to pause motion until an async action completes
- use wait_until to pause motion until a predicate on shared state is True
- use ExecutionState to communicate between async actions and predicates

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio

import nova
from nova import api, run_program
from nova.actions import (
    ActionExecutionContext,
    async_action,
    await_action,
    get_default_registry,
    joint_ptp,
    linear,
    register_async_action,
    wait_until,
)
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


async def log_state(ctx: ActionExecutionContext) -> None:
    """Print the current robot state when the action triggers."""
    label = ctx.action_kwargs.get("label", "state")
    print(
        f"[{label}] location={ctx.trigger_location:.3f} "
        f"pose={tuple(round(value, 3) for value in ctx.current_state.pose)}"
    )


async def notify_external_system(ctx: ActionExecutionContext) -> None:
    """Simulate a short non-blocking external notification."""
    message = ctx.action_kwargs.get("message", "step completed")
    print(f"[notify] {message}")
    await asyncio.sleep(0.2)
    print(f"[notify] delivered: {message}")


async def inspect_pose(ctx: ActionExecutionContext) -> None:
    """Simulate a blocking inspection step that sets shared state."""
    inspection_name = ctx.action_args[0] if ctx.action_args else "inspection"
    print(f"[inspect] starting {inspection_name} at {ctx.trigger_location:.3f}")
    await asyncio.sleep(0.5)
    # Signal completion via shared execution state
    await ctx.state.set("inspection_passed", True)
    print(f"[inspect] finished {inspection_name}")


def register_example_async_actions() -> None:
    """Register example actions once per process."""
    registry = get_default_registry()
    handlers = {
        "examples.async_action.log_state": log_state,
        "examples.async_action.notify_external_system": notify_external_system,
        "examples.async_action.inspect_pose": inspect_pose,
    }

    for name, handler in handlers.items():
        if not registry.is_registered(name):
            register_async_action(name, handler)


@nova.program(
    id="async_action",
    name="Async Actions",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext) -> None:
    """Plan and execute a trajectory with async actions."""
    register_example_async_actions()

    controller = await ctx.cell.controller("ur10e")
    async with controller[0] as motion_group:
        home_joints = await motion_group.joints()
        tcp = (await motion_group.tcp_names())[0]
        current_pose = await motion_group.tcp_pose(tcp)

        slow = MotionSettings(tcp_velocity_limit=75)
        normal = MotionSettings(tcp_velocity_limit=150)

        point_a = current_pose @ Pose((120, 0, 0, 0, 0, 0))
        point_b = current_pose @ Pose((120, 120, 0, 0, 0, 0))
        point_c = current_pose @ Pose((0, 120, 0, 0, 0, 0))

        actions = [
            joint_ptp(home_joints, settings=slow),
            # Non-blocking: log state in background
            async_action("examples.async_action.log_state", action_id="log1", label="home"),
            linear(point_a, settings=normal),
            # Non-blocking: fire-and-forget notification
            async_action(
                "examples.async_action.notify_external_system",
                action_id="notify1",
                message="Reached point A",
            ),
            linear(point_b, settings=normal),
            # Start inspection in background, then immediately await it (blocking equivalent)
            async_action(
                "examples.async_action.inspect_pose", "camera_check", action_id="inspect1"
            ),
            await_action("inspect1", timeout=2.0),
            linear(point_c, settings=normal),
            # Wait until the inspection set shared state before continuing
            wait_until(lambda s: s.get("inspection_passed"), timeout=5.0),
            async_action(
                "examples.async_action.log_state", action_id="log2", label="after inspection"
            ),
            joint_ptp(home_joints, settings=slow),
        ]

        print("Planning trajectory with async actions...")
        trajectory = await motion_group.plan(actions, tcp)

        print("Executing trajectory...")
        async for _ in motion_group.stream_execute(trajectory, tcp, actions=actions):
            pass

        print("Trajectory execution completed.")


if __name__ == "__main__":
    run_program(main)
