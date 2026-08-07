"""
Minimal repro for `wait_until(...)` after `async_action(...)` with no motions.

This example demonstrates that `wait_until` only observes `ExecutionState`,
not bus IO values directly.

Case 1 intentionally times out:
- all bus IO signals are set to True
- the async action runs and completes
- but it does not write anything to `ctx.state`
- `wait_until(...)` therefore waits until its timeout

Case 2 succeeds:
- the async action mirrors the expected keys into `ctx.state`
- `wait_until(...)` sees those keys and returns immediately

If you remove the timeout from case 1, it should wait indefinitely.

Run:
    uv run python examples/async_wait_until_state_repro.py
"""

import time

import wandelbots_api_client.v2_pydantic as wb

import nova
from nova import api, run_program
from nova.actions import (
    ActionExecutionContext,
    async_action,
    await_action,
    get_default_registry,
    register_async_action,
    wait_until,
)
from nova.actions.io import io_write
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.utils.io import get_bus_io_value, set_bus_io_value

SIGNALS = [
    "A23_safety_gate_locked",
    "E41_robot_release_1",
    "E43_robot_release_3",
    "E45_robot_release_5",
    "E80_welding_release",
    "M30_motion_conditions_summary",
    "M95_ez1_axis7_safety",
]


def all_releases_ready(state: dict[str, object]) -> bool:
    return bool(
        state.get("E41_robot_release_1")
        and state.get("E43_robot_release_3")
        and state.get("E45_robot_release_5")
        and state.get("E80_welding_release")
        and state.get("M30_motion_conditions_summary")
        and state.get("M95_ez1_axis7_safety")
    )


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


async def read_bus_but_do_not_update_state(ctx: ActionExecutionContext) -> None:
    print("[async] completed bus read simulation without touching ctx.state")


async def mirror_expected_flags_into_state(ctx: ActionExecutionContext) -> None:
    for signal in SIGNALS[1:]:
        await ctx.state.set(signal, True)
    print("[async] mirrored expected release flags into ctx.state")


def register_example_async_actions() -> None:
    registry = get_default_registry()
    handlers = {
        "examples.async_wait_until_state_repro.read_bus_only": read_bus_but_do_not_update_state,
        "examples.async_wait_until_state_repro.mirror_state": mirror_expected_flags_into_state,
    }

    for name, handler in handlers.items():
        if not registry.is_registered(name):
            register_async_action(name, handler)


@nova.program(
    name="Async WaitUntil State Repro",
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
    register_example_async_actions()
    await setup_signals(ctx)

    controller = await ctx.cell.controller("ur10e")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]

    await set_bus_io_value({signal: True for signal in SIGNALS})
    values = await get_bus_io_value(SIGNALS)
    print(f"Initial bus IO values: {values}")

    cases = [
        (
            "wait_until_times_out_without_state_updates",
            [
                io_write("A23_safety_gate_locked", True, origin=api.models.IOOrigin.BUS_IO),
                async_action(
                    "examples.async_wait_until_state_repro.read_bus_only", action_id="read_bus_only"
                ),
                await_action("read_bus_only"),
                wait_until(all_releases_ready, timeout=1.0),
            ],
        ),
        (
            "wait_until_passes_when_async_action_sets_state",
            [
                io_write("A23_safety_gate_locked", True, origin=api.models.IOOrigin.BUS_IO),
                async_action(
                    "examples.async_wait_until_state_repro.mirror_state", action_id="mirror_state"
                ),
                await_action("mirror_state"),
                wait_until(all_releases_ready, timeout=1.0),
            ],
        ),
    ]

    for case_name, actions in cases:
        start = time.perf_counter()
        print(f"Running case: {case_name}")
        await motion_group.plan_and_execute(actions, tcp=tcp)
        duration = time.perf_counter() - start
        print(f"{case_name}: completed in {duration:.3f}s")


if __name__ == "__main__":
    run_program(main)
