"""Per-motion-group pause on a Profinet bus IO across two controllers (ADR 002).

Requires a running NOVA instance; creates two virtual controllers and the virtual
Profinet bus-IO service with one bool output per motion group. Pausing one group's
signal must pause only that group while the other keeps running; clearing it lets
``execute()`` finish. Starts are staggered because the controller answers a second
``StartMovementRequest`` with a ``BUS_IO`` pause condition issued within ~200 ms of the
first with "I/O not found on bus-io" (server-side race, measured 2026-09-03).
"""

import asyncio
from math import pi

import pytest

from nova import Nova, api
from nova.actions import jnt
from nova.cell import virtual_controller
from nova.types.motion_settings import MotionSettings

KUKA_HOME = [0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0]
UR_HOME = [pi / 2, -pi / 2, pi / 2, 0.0, pi / 2, 0.0, 0.0]
PAUSE_IOS = {"pause-kuka": (820, 0), "pause-ur": (821, 0)}


async def _ensure_bus_ios(nova: Nova, cell_name: str) -> None:
    bus = nova.api.bus_ios_api
    try:
        await bus.get_bus_io_service(cell_name)
    except Exception:
        await bus.add_bus_io_service(cell=cell_name, bus_io_type=api.models.BusIOProfinetVirtual())
    for _ in range(120):
        try:
            state = await bus.get_bus_io_state(cell_name)
        except Exception:
            state = None
        if state is not None and state.state == api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED:
            break
        await asyncio.sleep(1)
    else:
        raise RuntimeError("bus IO service did not connect")
    existing = {io.io for io in await bus.list_profinet_ios(cell_name)}
    for name, (byte, bit) in PAUSE_IOS.items():
        if name not in existing:
            await bus.add_profinet_io(
                cell=cell_name,
                io=name,
                profinet_io_data=api.models.ProfinetIOData(
                    type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL,
                    description=f"pause signal {name}",
                    direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                    byte_address=byte,
                    bit_address=bit,
                ),
            )
    await asyncio.sleep(2)


async def _set(nova: Nova, cell_name: str, **values: bool) -> None:
    await nova.api.bus_ios_api.set_bus_io_values(
        cell=cell_name,
        io_value=[api.models.IOBooleanValue(io=k, value=v) for k, v in values.items()],
    )


def _pause_on(io: str) -> api.models.PauseOnIO:
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io=io, value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.BUS_IO,
    )


async def _wait_for_state(mg, kind: type, *, standstill: bool | None = None):
    async for state in mg.stream_state(None):
        details = state.execute.details if state.execute else None
        if isinstance(details, api.models.TrajectoryDetails) and isinstance(details.state, kind):
            if standstill is None or state.standstill == standstill:
                return state


async def _observe(mg, kind: type, *, standstill: bool, timeout: float, task: asyncio.Task):
    """Wait for a trajectory state, surfacing a failed execute() instead of timing out on
    a state that will never come."""
    waiter = asyncio.ensure_future(_wait_for_state(mg, kind, standstill=standstill))
    done, _ = await asyncio.wait(
        {waiter, task}, timeout=timeout, return_when=asyncio.FIRST_COMPLETED
    )
    if waiter not in done:
        waiter.cancel()
        if task in done:
            task.result()
        raise TimeoutError(f"no {kind.__name__} (standstill={standstill}) in {timeout}s")
    return waiter.result()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_bus_io_pause_signal_pauses_only_its_motion_group():
    async with Nova() as nova:
        cell = nova.cell()
        cell_name = "cell"
        kuka = await cell.ensure_controller(
            virtual_controller(
                name="kuka-bus-pause-test",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr6_r700_sixx",
                position=KUKA_HOME,
            )
        )
        ur = await cell.ensure_controller(
            virtual_controller(
                name="ur-bus-pause-test",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=UR_HOME,
            )
        )
        await _ensure_bus_ios(nova, cell_name)
        await _set(nova, cell_name, **{name: False for name in PAUSE_IOS})

        async with kuka[0] as kuka_mg, ur[0] as ur_mg:
            # Both runs traverse the full 1.5 rad; start from home so repeated runs on
            # the shared virtual controllers stay inside the joint limits.
            await kuka_mg.plan_and_execute([jnt(KUKA_HOME[:6])], tcp="Flange")
            await ur_mg.plan_and_execute([jnt(UR_HOME[:6])], tcp="Flange")
            kuka_start = await kuka_mg.joints()
            ur_start = await ur_mg.joints()
            kuka_target = [kuka_start[0] + 1.5, *kuka_start[1:]]
            ur_target = [ur_start[0] + 1.5, *ur_start[1:]]
            settings = MotionSettings(tcp_velocity_limit=30)

            kuka_task = asyncio.create_task(
                kuka_mg.plan_and_execute(
                    [jnt(kuka_target, settings=settings)],
                    tcp="Flange",
                    pause_on_io=_pause_on("pause-kuka"),
                )
            )
            await _observe(
                kuka_mg, api.models.TrajectoryRunning, standstill=False, timeout=20, task=kuka_task
            )
            ur_task = asyncio.create_task(
                ur_mg.plan_and_execute(
                    [jnt(ur_target, settings=settings)],
                    tcp="Flange",
                    pause_on_io=_pause_on("pause-ur"),
                )
            )
            try:
                await _observe(
                    ur_mg, api.models.TrajectoryRunning, standstill=False, timeout=20, task=ur_task
                )

                await _set(nova, cell_name, **{"pause-kuka": True})
                await _observe(
                    kuka_mg,
                    api.models.TrajectoryPausedOnIO,
                    standstill=True,
                    timeout=10,
                    task=kuka_task,
                )
                # The other group is unaffected and still running.
                await _observe(
                    ur_mg, api.models.TrajectoryRunning, standstill=False, timeout=5, task=ur_task
                )
                await asyncio.sleep(1.0)
                assert not kuka_task.done(), "execute() returned on a resumable IO pause"
                paused_joints = await kuka_mg.joints()
                assert abs(paused_joints[0] - kuka_target[0]) > 0.5, "KUKA was not paused early"

                await _set(nova, cell_name, **{"pause-kuka": False})
                await asyncio.wait_for(asyncio.gather(kuka_task, ur_task), timeout=90)

                assert abs((await kuka_mg.joints())[0] - kuka_target[0]) < 0.01
                assert abs((await ur_mg.joints())[0] - ur_target[0]) < 0.01
            finally:
                await _set(nova, cell_name, **{name: False for name in PAUSE_IOS})
                for task in (kuka_task, ur_task):
                    if not task.done():
                        task.cancel()
