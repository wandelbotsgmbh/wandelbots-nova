"""
Inspect and remove planned trajectories stored in a controller's cache.

Prerequisites:
- Create a NOVA instance
- Set NOVA_API and NOVA_ACCESS_TOKEN (for example in a .env file)
"""

import nova
from nova.actions import jnt
from nova.cell import virtual_controller


@nova.program(
    name="Trajectory Cache",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=False,
    ),
)
async def trajectory_cache(ctx: nova.ProgramContext):
    controller = await ctx.cell.controller("ur10e")
    motion_group = controller[0]
    tcp = await motion_group.active_tcp_name()

    # Start from a known cache state, then execute a motion which is cached by NOVA.
    await controller.clear_trajectory_cache()
    joints = await motion_group.joints()
    target = (joints[0] + 0.1,) + joints[1:]
    actions = [jnt(target)]
    trajectory = await motion_group.plan(actions, tcp)
    await motion_group.execute(trajectory, tcp, actions=actions)

    trajectory_ids = await controller.list_cached_trajectories()
    print(f"Cached trajectories: {trajectory_ids}")

    trajectory_id = trajectory_ids[0]
    cached_trajectory = await controller.get_cached_trajectory(trajectory_id)
    print(
        f"Trajectory {trajectory_id} belongs to motion group "
        f"{cached_trajectory.motion_group} and uses TCP {cached_trajectory.tcp}."
    )

    await controller.delete_cached_trajectory(trajectory_id)
    print(f"Cached trajectories after deletion: {await controller.list_cached_trajectories()}")

    # Clear any other trajectories that may have been loaded while the program ran.
    await controller.clear_trajectory_cache()


if __name__ == "__main__":
    nova.run_program(trajectory_cache)
