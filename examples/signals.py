"""
Example: This example shows the usage of Bus IOs in Nova.
We will demonstrate setting, getting, and waiting for Bus IO values,
as well as using Bus IOs in a motion plan.

Prerequisites:
- A nova instance
- Enable virtual profinet from the setup application
- Add bus io variables used in the example (test_bool, test_int)
  - test_bool: Boolean type
  - test_int: Integer type
- Run this example script
    e.g. uv run python examples/signals.py
- You will see the script will hang at the end until the bus io 'test_bool' changes from True to False.
- Navigate to the signals application and update the value of 'test_bool' to False to see the script complete.
"""

import nova
from nova import api, run_program
from nova.actions import ptp
from nova.actions.io import io_write
from nova.cell import virtual_controller
from nova.types.pose import Pose
from nova.utils.io import IOChange, get_bus_io_value, set_bus_io_value, wait_for_bus_io


@nova.program(
    id="bus_io_example",
    name="Bus IO Example",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR5E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext) -> None:
    nova_instance = ctx.nova
    cell = nova_instance.cell()

    # set value of the bus io names used in this example
    await set_bus_io_value({"test_bool": True, "test_int": 42})

    # get bus io values
    values = await get_bus_io_value(["test_bool", "test_int"])
    print(f"value of test_bool: {values['test_bool']}, value of test_int: {values['test_int']}")

    # Set io on the path
    controller = await cell.controller("ur5")
    motion_group = controller[0]
    current_pose = await motion_group.tcp_pose("Flange")
    target_pose = current_pose @ Pose((100, 0, 0, 0, 0, 0))

    actions = [
        io_write("test_bool", False, origin=api.models.IOOrigin.BUS_IO),
        ptp(target_pose),
        io_write("test_bool", True, origin=api.models.IOOrigin.BUS_IO),
    ]

    await motion_group.plan_and_execute(actions, "Flange")

    # wait for bus io value
    def on_change(changes: dict[str, IOChange]) -> bool:
        if "test_bool" not in changes:
            return False

        if changes["test_bool"].new_value == False and changes["test_bool"].old_value == True:  # noqa: E712
            # returning true stops the wait
            return True

        # continue waiting
        return False

    print("waiting for test_bool to become False...")
    await wait_for_bus_io(["test_bool"], on_change=on_change)
    print("test_bool is now False, waiting stopped.")


if __name__ == "__main__":
    run_program(main)
