"""
Palletizing: stack parts from a fixed pick station onto a pallet, layer by layer.

The slot grid is defined once in pallet coordinates. The pallet itself is a dataset frame
sitting on the "table" frame, so `frames["pallet"].as_world()` gives the transform that
turns every slot into a world pose with a single `@`.

Because the pallet is only dropped roughly into place, its frame is replaced at runtime with
what a camera measured. `set_frame()` rebinds it in the in-memory dataset and every slot -
and anything else taught on the pallet - follows. The grid never changes.
"""

import numpy as np

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.actions import Action, cartesian_ptp, joint_ptp, linear
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose

COLS, ROWS, LAYERS = 4, 3, 2
PITCH_X, PITCH_Y, LAYER_H = 150.0, 120.0, 200.0
APPROACH = 100.0

# Parts are gripped from above, so the tool points down the pallet's -Z axis.
GRIP = Pose((0, 0, 0, np.pi, 0, 0))
RETREAT = Pose((0, 0, -APPROACH, 0, 0, 0))


def pallet_grid() -> list[Pose]:
    """Every slot in pallet coordinates, in the order the pallet gets filled."""
    return [
        Pose((col * PITCH_X, row * PITCH_Y, layer * LAYER_H, 0, 0, 0)) @ GRIP
        for layer in range(LAYERS)
        for row in range(ROWS)
        for col in range(COLS)
    ]


def measure_pallet(taught: Pose) -> Pose:
    """Stand-in for a camera: the pallet landed 12 mm off and rotated by 1.5 degrees."""
    return taught @ Pose((12, -5, 0, 0, 0, np.radians(1.5)))


def rebind_pallet_frame(dataset: ds.Dataset) -> Pose:
    """Replace the taught pallet frame with the measured one and return its world transform."""
    taught = dataset.frames["pallet"]
    before = taught.as_world()

    dataset.set_frame("pallet", measure_pallet(taught.pose), reference_frame=taught.reference_frame)
    after = dataset.frames["pallet"].as_world()

    print(f"pallet as taught:   {before}")
    print(f"pallet as measured: {after}")
    return after


@nova.program(
    id="palletizing",
    name="Palletizing",
    # viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            )
        ],
        dataset=ds.local_dataset("palletizing_dataset.json"),
        cleanup_controllers=False,
    ),
)
async def palletize(ctx: nova.ProgramContext):
    """Pick from the "pick" pose and fill the pallet grid, slot by slot."""
    assert ctx.dataset is not None, "This program requires the palletizing dataset."

    motion_group = (await ctx.cell.controller("kuka-kr16-r2010"))[0]
    tcp = (await motion_group.tcp_names())[0]
    home = await motion_group.joints()
    fast = MotionSettings(tcp_velocity_limit=500)
    slow = MotionSettings(tcp_velocity_limit=80)

    pick = ctx.dataset.poses["pick"].as_world()
    pallet = rebind_pallet_frame(ctx.dataset)
    slots = [pallet @ slot for slot in pallet_grid()]

    actions: list[Action] = [joint_ptp(home, settings=fast)]
    for place in slots:
        above = place @ RETREAT
        actions += [
            cartesian_ptp(pick @ RETREAT, settings=fast),
            linear(pick, settings=slow),
            linear(pick @ RETREAT, settings=slow),
            cartesian_ptp(above, settings=fast),
            linear(place, settings=slow),
            linear(above, settings=slow),
        ]
    actions.append(joint_ptp(home, settings=fast))

    print(f"Palletizing {len(slots)} slots ({COLS}x{ROWS} per layer, {LAYERS} layers)")
    trajectory = await motion_group.plan(actions, tcp)
    await motion_group.execute(trajectory, tcp, actions=actions)
    print("Pallet complete")


if __name__ == "__main__":
    run_program(palletize)
