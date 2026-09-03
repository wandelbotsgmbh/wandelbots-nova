"""Synchronized start of two motion groups on *different* controllers.

A controller output can only gate the groups of the controller that owns it. A
bus IO is cell-scoped, so it can gate groups across controllers: every group is
armed with the same ``StartOnIO`` on the bus variable, and the barrier sets that
variable once all of them report they are waiting for it.

The trigger is a boolean bus variable, declared here on whichever fieldbus the
cell runs; a cell without a bus IO service at all gets NOVA's virtual MODBUS one.
"""

import asyncio
import json
from datetime import datetime

import nova
from nova import api, run_program
from nova.cell import GroupArgs, TrajectoryExecutor, virtual_controller

CONTROLLERS = ("kuka-a", "kuka-b")
SYNC_IO = "sync-bus"

# Both groups ramp their last joint by this much, over one shared parameterization.
RAMP_RADIANS = 0.1
RAMP_SECONDS = 2.0
RAMP_SAMPLES = 21


async def bus_io_service(ctx: nova.ProgramContext, gateway, cell: str) -> api.models.BusIOType:
    """The cell's bus IO service, adding the virtual MODBUS one if it has none."""
    try:
        return await gateway.bus_ios_api.get_bus_io_service(cell=cell)
    except Exception:
        pass

    connected = asyncio.Event()

    async def on_status(message) -> None:
        state = json.loads(message.data)["state"]
        if state == api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED.value:
            connected.set()

    await ctx.nova.nats.subscribe(f"nova.v2.cells.{cell}.bus-ios.status", cb=on_status)
    service = api.models.BusIOModbusVirtual()
    await gateway.bus_ios_api.add_bus_io_service(cell=cell, bus_io_type=service)
    # A service that just reported itself connected still rejects the first
    # requests, so the settle is on top of waiting for the state.
    await asyncio.wait_for(connected.wait(), timeout=60)
    await asyncio.sleep(5)
    return service


async def declare_bus_trigger(ctx: nova.ProgramContext, gateway, cell: str) -> None:
    """Declare the trigger as a boolean on whichever bus the cell runs."""
    service = await bus_io_service(ctx, gateway, cell)
    match service:
        case api.models.BusIOProfinet() | api.models.BusIOProfinetVirtual():
            await gateway.bus_ios_api.add_profinet_io(
                cell=cell,
                io=SYNC_IO,
                profinet_io_data=api.models.ProfinetIOData(
                    description="synchronized start trigger",
                    type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL,
                    direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                    byte_address=900,
                    bit_address=0,
                ),
            )
        case _:
            await gateway.bus_ios_api.add_modbus_io(
                cell=cell,
                io=SYNC_IO,
                modbus_io_data=api.models.ModbusIOData(
                    description="synchronized start trigger",
                    address=0,
                    type=api.models.ModbusIOTypeEnum.MODBUS_IO_TYPE_BOOL,
                    byte_order=api.models.ModbusIOByteOrder.MODBUS_IO_BYTE_ORDER_ABCD,
                    area=api.models.ModbusIOArea.MODBUS_IO_AREA_COILS,
                ),
            )


async def ramp_from_here(
    groups: dict[str, nova.cell.MotionGroup],
) -> api.models.MultiJointTrajectory:
    """A joint ramp for every group over one shared times/locations parameterization."""
    parameter = [RAMP_SECONDS * i / (RAMP_SAMPLES - 1) for i in range(RAMP_SAMPLES)]
    joints = {name: tuple(await group.joints()) for name, group in groups.items()}
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={
            name: [[*start[:-1], start[-1] + RAMP_RADIANS * p / RAMP_SECONDS] for p in parameter]
            for name, start in joints.items()
        },
        times=parameter,
        locations=parameter,
    )


async def report_start(group: nova.cell.MotionGroup, name: str, seen: dict[str, datetime]) -> None:
    """Record when the controller reported this group armed and running."""
    async for state in group.stream_state():
        details = state.execute.details if state.execute else None
        match details.state if details else None:
            case api.models.TrajectoryWaitForIO():
                seen.setdefault(f"{name} armed", state.timestamp)
            case api.models.TrajectoryRunning():
                seen.setdefault(f"{name} running", state.timestamp)
                return
            case _:
                pass


@nova.program(
    name="bus_io_sync_two_controllers",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=name, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr210_r2700_2"
            )
            for name in CONTROLLERS
        ]
    ),
)
async def bus_io_sync_two_controllers(ctx: nova.ProgramContext):
    """Start one motion group on each of two controllers on the same bus IO."""
    cell = ctx.nova.cell()
    groups = {}
    for name in CONTROLLERS:
        controller = await cell.controller(name)
        groups[f"0@{name}"] = controller.motion_group(f"0@{name}")

    gateway = next(iter(groups.values()))._api_client
    await declare_bus_trigger(ctx, gateway, cell.cell_id)

    trajectory = await ramp_from_here(groups)
    executor = (
        TrajectoryExecutor.builder(groups)
        .sync_on_io(SYNC_IO, origin=api.models.IOOrigin.BUS_IO)
        .build()
    )

    seen: dict[str, datetime] = {}
    watchers = [
        asyncio.create_task(report_start(group, name, seen)) for name, group in groups.items()
    ]
    try:
        tcps = {name: (await group.tcp_names())[0] for name, group in groups.items()}
        await executor.execute(
            trajectory, groups={name: GroupArgs(tcp=tcp) for name, tcp in tcps.items()}
        )
    finally:
        for watcher in watchers:
            watcher.cancel()

    for label in sorted(seen):
        print(f"{label}: {seen[label].isoformat()}")
    running = [seen[f"{name} running"] for name in groups if f"{name} running" in seen]
    if len(running) == len(groups):
        skew = (max(running) - min(running)).total_seconds()
        print(f"start skew across controllers: {skew * 1000:.1f} ms")


if __name__ == "__main__":
    run_program(bus_io_sync_two_controllers)
