import asyncio
import json
import logging
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator

import pytest

from nova import Nova, api
from nova.actions import ptp
from nova.actions.base import Action
from nova.actions.io import io_write
from nova.cell import virtual_controller
from nova.cell.controller import Controller
from nova.cell.io import IOAccess
from nova.types.pose import Pose
from nova.utils.io import IOChange, get_bus_io_value, set_bus_io_value, wait_for_bus_io

logger = logging.getLogger(__name__)


@pytest.fixture
@pytest.mark.integration
async def setup_controllers() -> AsyncGenerator[tuple[Controller, Controller], None]:
    async with Nova() as nova:
        cell = nova.cell()

        ur = await cell.ensure_controller(
            virtual_controller(
                name="ur-test",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        )

        # kuka has special IO naming conventions, so we test that as well
        kuka = await cell.ensure_controller(
            virtual_controller(
                name="kuka-test",
                manufacturer=api.models.Manufacturer.KUKA,
                type=api.models.VirtualControllerTypes.KUKA_KR16_R2010_2,
            )
        )

        yield ur, kuka


@pytest.fixture
@pytest.mark.integration
async def setup_virtual_profinet() -> AsyncGenerator[tuple[str, ...], None]:
    async with Nova() as nova:
        bus_io_service_ready = asyncio.Event()

        async def on_bus_io_state(message):
            data = json.loads(message.data)
            if data["state"] == api.models.BusIOsStateEnum.BUS_IOS_STATE_CONNECTED.value:
                bus_io_service_ready.set()

        _ = await nova.nats.subscribe("nova.v2.cells.cell.bus-ios.status", cb=on_bus_io_state)
        try:
            await nova.api.bus_ios_api.get_bus_io_service("cell")
            bus_io_service_ready.set()
        except Exception:
            await nova.api.bus_ios_api.add_bus_io_service(
                cell="cell", bus_io_type=api.models.BusIOType(api.models.BusIOProfinetVirtual())
            )
            # sometimes we get error if we communicate with bus io right after we get the "connected" message
            await asyncio.sleep(30)

        await bus_io_service_ready.wait()

        await nova.api.bus_ios_api.add_profinet_io(
            cell="cell",
            io="test_bool",
            profinet_io_data=api.models.ProfinetIOData(
                type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL,
                description="Test bool data",
                direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                byte_address=800,
                bit_address=1,
            ),
        )

        await nova.api.bus_ios_api.add_profinet_io(
            cell="cell",
            io="test_bool_2",
            profinet_io_data=api.models.ProfinetIOData(
                type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL,
                description="Test bool data",
                direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                byte_address=801,
                bit_address=1,
            ),
        )

        await nova.api.bus_ios_api.add_profinet_io(
            cell="cell",
            io="test_int",
            profinet_io_data=api.models.ProfinetIOData(
                type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_INT,
                description="Test int data",
                direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                byte_address=802,
            ),
        )

        await nova.api.bus_ios_api.add_profinet_io(
            cell="cell",
            io="test_float",
            profinet_io_data=api.models.ProfinetIOData(
                type=api.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_REAL,
                description="Test float data",
                direction=api.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
                byte_address=804,
            ),
        )

        yield "test_bool", "test_bool_2", "test_int", "test_float"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_io_write(setup_controllers: tuple[Controller, Controller]):
    ur, kuka = setup_controllers

    # Test UR IO write
    await ur.write("digital_in[0]", False)
    value = await ur.read("digital_in[0]")
    assert value is False

    await ur.write("digital_in[0]", True)
    value = await ur.read("digital_in[0]")
    assert value is True

    # Test Kuka IO write
    await kuka.write("OUT#555", False)
    value = await kuka.read("OUT#555")
    assert value is False

    await kuka.write("OUT#555", True)
    value = await kuka.read("OUT#555")
    assert value is True


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_io_descriptions(setup_controllers: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-test")
        io_descriptions = await io.get_io_descriptions()
        assert len(io_descriptions) > 0
        filtered_io_descriptions = IOAccess.filter_io_descriptions(
            io_descriptions,
            api.models.IOValueType.IO_VALUE_BOOLEAN,
            api.models.IODirection.IO_TYPE_INPUT,
        )
        assert len(filtered_io_descriptions) < len(io_descriptions)


@pytest.mark.asyncio
@pytest.mark.integration
async def test_read(setup_controllers: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-test")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        value2 = await io.read("digital_out[0]")
        assert value2 is False


@pytest.mark.asyncio
@pytest.mark.integration
async def test_write(setup_controllers: tuple[Controller, Controller]):
    async with Nova() as nova:
        cell = nova.cell()
        io = IOAccess(api_client=nova.apis, cell=cell.cell_id, controller_id="ur-test")
        value1 = await io.read("tool_out[0]")
        assert value1 is False
        await io.write("tool_out[0]", True)
        value2 = await io.read("tool_out[0]")
        assert value2 is True
        await io.write("tool_out[0]", False)
        value3 = await io.read("tool_out[0]")
        assert value3 is False


@dataclass
class SetIOOnPathTestCase:
    description: str

    # setup values
    controller_io_prestate: dict[str, Any] = field(default_factory=dict)
    bus_io_prestate: dict[str, Any] = field(default_factory=dict)

    # exercise values
    actions: list[Action] = field(default_factory=list)

    # verification values
    expected_controller_io: dict[str, Any] = field(default_factory=dict)
    expected_bus_io: dict[str, Any] = field(default_factory=dict)


SET_IO_ON_PATH_TEST_CASES = [
    pytest.param(
        SetIOOnPathTestCase(
            description="set controller io at the beginning of the action list",
            controller_io_prestate={"digital_in[1]": False},
            actions=[io_write("digital_in[1]", True), ptp(Pose(700, 0, 500, 0, 0, 0))],
            expected_controller_io={"digital_in[1]": True},
        ),
        id="controller_io_at_start",
    ),
    pytest.param(
        SetIOOnPathTestCase(
            description="set multiple controller io at the beginning of the action list",
            controller_io_prestate={"digital_in[0]": False, "digital_in[1]": False},
            actions=[
                io_write("digital_in[0]", True),
                io_write("digital_in[1]", True),
                ptp(Pose(600, 0, 500, 0, 0, 0)),
                ptp(Pose(700, 0, 500, 0, 0, 0)),
            ],
            expected_controller_io={"digital_in[0]": True, "digital_in[1]": True},
        ),
        id="multiple_controller_io_at_start",
    ),
    pytest.param(
        SetIOOnPathTestCase(
            description="set multiple controller io at the end of the action list",
            controller_io_prestate={"digital_in[0]": False, "digital_in[1]": False},
            actions=[
                ptp(Pose(600, 0, 500, 0, 0, 0)),
                ptp(Pose(700, 0, 500, 0, 0, 0)),
                io_write("digital_in[0]", True),
                io_write("digital_in[1]", True),
            ],
            expected_controller_io={"digital_in[0]": True, "digital_in[1]": True},
        ),
        id="multiple_controller_io_at_end",
    ),
    pytest.param(
        SetIOOnPathTestCase(
            description="set multiple bus io at the start of the action list",
            bus_io_prestate={"test_bool": False, "test_bool_2": False},
            actions=[
                io_write("test_bool", True, origin=api.models.IOOrigin.BUS_IO),
                io_write("test_bool_2", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(Pose(600, 0, 500, 0, 0, 0)),
                ptp(Pose(700, 0, 500, 0, 0, 0)),
            ],
            expected_bus_io={"test_bool": True, "test_bool_2": True},
        ),
        id="multiple_bus_io_at_the_start",
    ),
    pytest.param(
        SetIOOnPathTestCase(
            description="set multiple io with a wait command in between",
            controller_io_prestate={"digital_in[0]": False, "digital_in[1]": False},
            bus_io_prestate={"test_bool": False, "test_bool_2": False},
            actions=[
                io_write("digital_in[0]", True),
                io_write("test_bool", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(Pose(600, 0, 500, 0, 0, 0)),
                # TODO: adding wait here causes FeedbackOutOfWorkspace error, how?
                io_write("digital_in[1]", True),
                io_write("test_bool_2", True, origin=api.models.IOOrigin.BUS_IO),
                ptp(Pose(700, 0, 500, 0, 0, 0)),
            ],
            expected_controller_io={"digital_in[0]": True, "digital_in[1]": True},
            expected_bus_io={"test_bool": True, "test_bool_2": True},
        ),
        id="mixed_controller_and_bus_io_with_wait",
    ),
    pytest.param(
        SetIOOnPathTestCase(
            description="set multiple bus io at the end of the action list",
            bus_io_prestate={"test_bool": False, "test_bool_2": False},
            actions=[
                ptp(Pose(600, 0, 500, 0, 0, 0)),
                ptp(Pose(700, 0, 500, 0, 0, 0)),
                io_write("test_bool", True, origin=api.models.IOOrigin.BUS_IO),
                io_write("test_bool_2", True, origin=api.models.IOOrigin.BUS_IO),
            ],
            expected_bus_io={"test_bool": True, "test_bool_2": True},
        ),
        id="multiple_bus_io_at_the_end",
    ),
]


@pytest.mark.integration
@pytest.mark.asyncio
@pytest.mark.flaky(reruns=3, reruns_delay=1)
@pytest.mark.parametrize("test_case", SET_IO_ON_PATH_TEST_CASES)
async def test_set_io_on_path(
    setup_controllers: tuple[Controller, Controller],
    setup_virtual_profinet: tuple[str, ...],
    test_case: SetIOOnPathTestCase,
):
    async with Nova() as nova:
        # SET UP
        ur, _ = setup_controllers
        for io in test_case.controller_io_prestate:
            await ur.write(io, test_case.controller_io_prestate[io])

        for io in test_case.bus_io_prestate:
            await set_bus_io_value({io: test_case.bus_io_prestate[io]}, nova=nova)

        # EXECUTE
        await ur[0].plan_and_execute(test_case.actions, "Flange")

        # VERIFY
        for io in test_case.expected_controller_io:
            value = await ur.read(io)
            assert test_case.expected_controller_io[io] == value, (
                f"Controller IO: {io} doesn't match the expected value."
            )

        for io in test_case.expected_bus_io:
            values = await get_bus_io_value([io], nova=nova)
            assert values[io] == test_case.expected_bus_io[io], (
                f"Bus IO: {io} doesn't match the expected value"
            )


@pytest.mark.asyncio
@pytest.mark.integration
async def test_bus_io_get_set_bool_int(setup_virtual_profinet: tuple[str, ...]):
    test_bool, _, test_int, _ = setup_virtual_profinet

    async with Nova() as nova:
        await set_bus_io_value({test_bool: True, test_int: 42}, nova=nova)

        values = await get_bus_io_value([test_bool, test_int], nova=nova)
        assert values[test_bool] is True
        assert values[test_int] == 42

        single = await get_bus_io_value([test_bool], nova=nova)
        assert single[test_bool] is True


@pytest.mark.asyncio
@pytest.mark.integration
async def test_bus_io_get_set_float(setup_virtual_profinet: tuple[str, ...]):
    _ = setup_virtual_profinet

    async with Nova() as nova:
        await set_bus_io_value({"test_float": 12.5}, nova=nova)

        values = await get_bus_io_value(["test_float"], nova=nova)
        assert values["test_float"] == pytest.approx(12.5)


@pytest.mark.asyncio
@pytest.mark.integration
async def test_wait_io(setup_virtual_profinet: tuple[str, ...]):
    async with Nova() as nova:
        test_bool, _, _, _ = setup_virtual_profinet

        def on_change(changes: dict[str, IOChange]) -> bool:
            if test_bool not in changes:
                return False

            if changes[test_bool].new_value is True and changes[test_bool].old_value is False:
                return True

            return False

        wait_task = asyncio.create_task(
            wait_for_bus_io([test_bool], on_change=on_change, nova=nova)
        )

        await set_bus_io_value({test_bool: False}, nova=nova)
        await set_bus_io_value({test_bool: True}, nova=nova)

        async with asyncio.timeout(5):
            await wait_task


@pytest.mark.asyncio
@pytest.mark.integration
async def test_wait_io_with_noisy_integer(setup_virtual_profinet: tuple[str, ...]):
    test_bool, _, test_int, _ = setup_virtual_profinet

    async with Nova() as nova:
        # SETUP
        await set_bus_io_value({test_bool: False}, nova=nova)
        await set_bus_io_value({test_int: 0}, nova=nova)

        def on_change(changes: dict[str, IOChange]) -> bool:
            if test_bool not in changes:
                return False

            change = changes[test_bool]
            return change.old_value is False and change.new_value is True

        update_count = 0

        async def update_integer_io() -> None:
            nonlocal update_count
            for value in range(50):
                await set_bus_io_value({test_int: value}, nova=nova)
                update_count += 1
                await asyncio.sleep(0.01)

        wait_task = asyncio.create_task(
            wait_for_bus_io([test_bool], on_change=on_change, nova=nova)
        )
        _ = asyncio.create_task(update_integer_io())

        while update_count < 5:
            await asyncio.sleep(0.1)

        assert not wait_task.done()

        # EXECUTE
        await set_bus_io_value({test_bool: True}, nova=nova)

        # VERIFY
        async with asyncio.timeout(5):
            await wait_task
