"""
Minimal repro for action-list handling on the current branch.

This example:
- creates the required boolean bus IOs through the API before execution
- executes short out-and-back `ptp` motions with `io_write` actions placed at
  different positions in the list
- covers no-motion `io_write` and `wait` action lists supported on this branch
- prints the case name and final bus IO values for each case

Run:
    uv run python examples/io_write_wait_action_list_repro.py
"""

import nova
import wandelbots_api_client.v2_pydantic as wb

from nova import api, run_program
from nova.actions import ptp, wait
from nova.actions.io import io_write
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from nova.types.pose import Pose
from nova.utils.io import get_bus_io_value, set_bus_io_value

SIGNALS = [
    "io_write_case_start",
    "io_write_case_middle",
    "io_write_case_end",
    "io_write_case_many",
    "io_write_case_write_only_a",
    "io_write_case_write_only_b",
    "io_write_case_wait_only_a",
    "io_write_case_wait_only_b",
]


async def setup_signals(ctx: nova.ProgramContext) -> None:
    for signal in SIGNALS:
        try:
            await ctx.nova.api.bus_ios_api.add_profinet_io(
                cell=ctx.cell.id,
                io=signal,
                profinet_io_data=wb.ProfinetIOData(
                    description=signal,
                    type=wb.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL,
                    direction=wb.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                    byte_address=800,
                    bit_address=None,
                ),
            )
        except Exception as exc:
            print(f"Signal '{signal}' may already exist: {exc}")


async def reset_signals() -> None:
    await set_bus_io_value({signal: False for signal in SIGNALS})


def log_case(name: str, values: dict[str, bool | int | float]) -> None:
    print(f"{name}: {values}")


@nova.program(
    name="IO Write Wait Action List Repro",
    preconditions=ProgramPreconditions(
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
    await setup_signals(ctx)
    await reset_signals()

    controller = await ctx.cell.controller("ur10e")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]

    start_pose = await motion_group.tcp_pose(tcp)
    forward_pose = start_pose @ Pose((40, 0, 0, 0, 0, 0))
    fast_back_and_forth = MotionSettings(tcp_velocity_limit=80, tcp_acceleration_limit=160)

    cases: list[tuple[str, list]] = [
        (
            "write_before_motion",
            [
                io_write("io_write_case_start", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(forward_pose, fast_back_and_forth),
                ptp(start_pose, fast_back_and_forth),
            ],
        ),
        (
            "write_between_motions",
            [
                ptp(forward_pose, fast_back_and_forth),
                io_write("io_write_case_middle", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(start_pose, fast_back_and_forth),
            ],
        ),
        (
            "write_after_motion",
            [
                ptp(forward_pose, fast_back_and_forth),
                ptp(start_pose, fast_back_and_forth),
                io_write("io_write_case_end", True, origin=api.models.IOOrigin.BUS_IO),
            ],
        ),
        (
            "multiple_writes",
            [
                io_write("io_write_case_many", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(forward_pose, fast_back_and_forth),
                io_write("io_write_case_many", False, origin=api.models.IOOrigin.BUS_IO),
                ptp(start_pose, fast_back_and_forth),
                io_write("io_write_case_many", True, origin=api.models.IOOrigin.BUS_IO),
            ],
        ),
        (
            "write_only",
            [
                io_write("io_write_case_write_only_a", True, origin=api.models.IOOrigin.BUS_IO),
                io_write("io_write_case_write_only_b", True, origin=api.models.IOOrigin.BUS_IO),
                io_write("io_write_case_write_only_a", False, origin=api.models.IOOrigin.BUS_IO),
            ],
        ),
        ("wait_only", [wait(0.2)]),
        (
            "write_wait_write_only",
            [
                io_write("io_write_case_wait_only_a", True, origin=api.models.IOOrigin.BUS_IO),
                wait(0.2),
                io_write("io_write_case_wait_only_b", True, origin=api.models.IOOrigin.BUS_IO),
            ],
        ),
        (
            "wait_write_only",
            [
                wait(0.2),
                io_write("io_write_case_wait_only_a", True, origin=api.models.IOOrigin.BUS_IO),
            ],
        ),
    ]

    for name, actions in cases:
        await reset_signals()
        print(f"Running case: {name}")
        try:
            await motion_group.plan_and_execute(actions, tcp=tcp)
        except Exception as exc:
            raise RuntimeError(f"Case '{name}' failed") from exc

        log_case(name, await get_bus_io_value(SIGNALS))


if __name__ == "__main__":
    run_program(main)
