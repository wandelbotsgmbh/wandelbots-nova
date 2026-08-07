import asyncio

import wandelbots_api_client.v2 as wb_v2
from decouple import config
from icecream import ic

from nova.core.nova import Nova

ic.configureOutput(includeContext=True)

NATS_BROKER = config("NATS_BROKER")

BUS_IO_VALUES = [
    *[
        wb_v2.models.IOBooleanValue(io=f"b{index}", value=value)
        for index, value in enumerate([True] * 4)
    ],
    *[
        wb_v2.models.IOIntegerValue(io=f"i{index}", value=f"{value}")
        for index, value in enumerate(range(1, 5))
    ],
]


async def setup_bus_ios(api_client: wb_v2.ApiClient, cell_id: str):
    bus_io_api = wb_v2.BUSInputsOutputsApi(api_client)
    bus_io_values = [wb_v2.models.IOValue(actual_instance=value) for value in BUS_IO_VALUES]
    ic(bus_io_values)

    for io_value in bus_io_values:
        profinet_io_data = wb_v2.models.ProfinetIOData(
            description=f"{io_value.actual_instance.io}",
            type=(
                wb_v2.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL
                if isinstance(io_value.actual_instance, wb_v2.models.IOBooleanValue)
                else wb_v2.models.ProfinetIOTypeEnum.PROFINET_IO_TYPE_INT
            ),
            direction=wb_v2.models.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
            byte_address=800,
            bit_address=None,
        )
        await bus_io_api.add_profinet_io(
            cell_id, io=profinet_io_data.description, profinet_io_data=profinet_io_data
        )
    await bus_io_api.set_bus_io_values(cell_id, bus_io_values)
    ic()


async def main():
    """Main robot control function."""
    async with Nova() as nova:
        cell = nova.cell()

        v2_config = nova._api_client._api_client.configuration
        v2_config.host = v2_config.host[:-1] + "2"
        v2_api_client: wb_v2.ApiClient = wb_v2.ApiClient(v2_config)
        bus_io_api = wb_v2.BUSInputsOutputsApi(v2_api_client)
        await setup_bus_ios(v2_api_client, cell.cell_id)
        ic(await bus_io_api.get_bus_io_values(cell.cell_id, ios=["b0", "b1", "i0", "i1"]))


if __name__ == "__main__":
    asyncio.run(main())
