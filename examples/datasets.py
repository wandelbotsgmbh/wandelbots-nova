"""
This example shows how to load a NOVA dataset (poses, frames and
command routines) and use its poses inside a motion program.

This demonstrates:
- Loading a dataset from a local JSON file
- Loading a dataset from the remote NOVA API (a fixed revision)
- Reading poses from the loaded dataset and using them directly as motion targets

Note: This example only uses dataset poses that are defined in the world
frame (frame=None), e.g. "home", "pick" and "place".
Poses that are defined relative to a dataset frame (e.g.
"table-corner" or "fixture-slot-a") first need to be resolved to world
coordinates via `nova.datasets.resolve_to_world` - that is outside the scope
of this minimal example.
"""

import asyncio

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import MotionSettings


async def _move_through_dataset_poses(ctx: nova.ProgramContext, count: int):
    """Load 'pick' and 'place' poses from the dataset attached to the program and move to them."""
    assert ctx.dataset is not None and ctx.dataset.poses, (
        "This program requires a dataset with poses to be loaded."
    )

    cell = ctx.cell
    controller = await cell.controller("kuka-kr16-r2010")
    cycle = ctx.cycle(extra={"app": "visual-studio-code"})

    normal = MotionSettings(tcp_velocity_limit=100)

    motion_group = controller[0]
    home_joints = await motion_group.joints()
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    pick_pose = ctx.dataset.poses["pick"]

    # Translating a pose between world and local frame
    pick_pose_fixture = (
        await ds.localize_pose_from_world(
            ctx.nova, [pick_pose.pose], frame="fixture", dataset=ctx.dataset.dataset
        )
    )[0]
    pick_pose_world = (
        await ds.resolve_to_world(
            ctx.nova, [pick_pose_fixture], frame="fixture", dataset=ctx.dataset.dataset
        )
    )[0]

    place_pose = ctx.dataset.poses["place"]

    # Actions define the sequence of movements and other actions to be executed by the robot
    actions = [
        joint_ptp(home_joints, settings=normal),  # Move to home position
        cartesian_ptp(pick_pose_world, settings=normal),  # Move to the dataset's "pick" pose
        cartesian_ptp(place_pose, settings=normal),  # Move to the dataset's "place" pose
        joint_ptp(home_joints, settings=normal),  # Return to home
    ]

    # Start the cycle
    await cycle.start()

    # Plan the movements (shows in 3D viewer or creates an rrd file)
    joint_trajectory = await motion_group.plan(actions, tcp)

    # OPTIONAL: Execute the planned movements
    # You can comment out the lines below to only see the plan in Rerun
    print("Executing planned movements...")
    for i in range(count):
        print(f"Executing movement {i + 1} of {count}")
        await motion_group.execute(joint_trajectory, tcp, actions=actions)
        print(f"Movement {i + 1} completed")
        await asyncio.sleep(1)

    # Finish the cycle
    await cycle.finish()
    print("Movement execution completed!")


# Configure a robot program that loads its dataset from a local JSON file.
@nova.program(
    id="load_local_dataset",
    name="Load local dataset",
    # viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            )
        ],
        dataset=ds.local_dataset("examples/example_dataset.json"),
        cleanup_controllers=False,
    ),
)
async def load_local_dataset(ctx: nova.ProgramContext, count: int = 1):
    """Load a dataset from a local file and move to some of its poses."""
    await _move_through_dataset_poses(ctx, count)


# Configure a robot program that loads its dataset from the remote NOVA API.
# "default" / revision 1 is the dataset pre-seeded on a fresh NOVA cell - its
# content is identical to examples/example_dataset.json.
@nova.program(
    id="load_remote_dataset",
    name="Load remote dataset",
    # viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            )
        ],
        dataset=ds.remote_dataset("default", revision=1),
        cleanup_controllers=False,
    ),
)
async def load_remote_dataset(ctx: nova.ProgramContext, count: int = 1):
    """Load the "default" dataset (revision 1) from the NOVA API and move to some of its poses."""
    await _move_through_dataset_poses(ctx, count)


if __name__ == "__main__":
    # Switch to `load_remote_dataset` to load the same dataset from the NOVA API instead.
    run_program(load_local_dataset)
