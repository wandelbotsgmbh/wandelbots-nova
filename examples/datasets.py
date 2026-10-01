"""
This example shows how to load a NOVA dataset (poses, frames and
command routines) and use its poses inside a motion program.

This demonstrates:
- Loading a dataset from a local JSON file
- Loading a dataset from the remote NOVA API (a fixed revision)
- The difference between `.pose` and `.as_world()` on a dataset pose

A dataset pose carries the pose exactly as it was taught, relative to its `frame`
(`frame=None` means it is already a world pose). A dataset frame is in turn expressed
relative to its `reference_frame`, so frames form a chain up to `world`. `as_world()`
walks that chain locally - no API calls - and raises if a frame along the way is missing.

For what you can do with the resolved poses - offsets, inverses, re-expressing a pose in
another frame - see examples/pose_transformations.py.
"""

import asyncio

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose

APPROACH = Pose((0, 0, -100, 0, 0, 0))


def _show_pose_vs_world_pose(dataset: ds.Dataset):
    """`.pose` is the pose as taught, `.as_world()` resolves its frame chain."""
    # "pick" is taught in world (frame=None), so both are the same pose.
    pick = dataset.poses["pick"]
    assert pick.frame is None
    assert pick.pose == pick.as_world()

    # "fixture-slot-a" is taught in the "fixture" frame, which itself sits on "table".
    # `.pose` is the raw taught value and is NOT a world pose - moving there would send
    # the robot to the wrong place. `.as_world()` composes table <- fixture <- slot.
    slot = dataset.poses["fixture-slot-a"]
    assert slot.frame == "fixture"
    assert slot.pose != slot.as_world()

    # A frame resolves to the transform that takes a pose from that frame into world.
    fixture = dataset.frames["fixture"]

    print(f"pick.pose            = {pick.pose}")
    print(f"pick.as_world()      = {pick.as_world()}")
    print(f"slot.pose            = {slot.pose}  (in frame '{slot.frame}')")
    print(f"slot.as_world()      = {slot.as_world()}")
    print(f"fixture.pose         = {fixture.pose}  (in frame '{fixture.reference_frame}')")
    print(f"fixture.as_world()   = {fixture.as_world()}")


async def _move_through_dataset_poses(ctx: nova.ProgramContext, count: int):
    """Load 'pick' and 'place' poses from the dataset attached to the program and move to them."""
    assert ctx.dataset is not None and ctx.dataset.poses, (
        "This program requires a dataset with poses to be loaded."
    )

    controller = await ctx.cell.controller("kuka-kr16-r2010")
    cycle = ctx.cycle(extra={"app": "visual-studio-code"})

    normal = MotionSettings(tcp_velocity_limit=100)

    motion_group = controller[0]
    home_joints = await motion_group.joints()
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    _show_pose_vs_world_pose(ctx.dataset)

    # Always move to `as_world()` - a motion target is a world pose.
    pick_pose = ctx.dataset.poses["pick"].as_world()
    place_pose = ctx.dataset.poses["place"].as_world()

    # Actions define the sequence of movements and other actions to be executed by the robot
    actions = [
        joint_ptp(home_joints, settings=normal),  # Move to home position
        cartesian_ptp(pick_pose @ APPROACH, settings=normal),  # Approach above "pick"
        cartesian_ptp(pick_pose, settings=normal),  # Move to the dataset's "pick" pose
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
        dataset=ds.local_dataset("example_dataset.json"),
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
