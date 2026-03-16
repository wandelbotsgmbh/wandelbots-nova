import asyncio

import wandelbots_api_client.v2_pydantic as api
from decouple import config
from icecream import ic

from nova.core.nova import Nova

ic.configureOutput(includeContext=True)

NATS_BROKER = config("NATS_BROKER")


async def main():
    """Main robot control function."""
    async with Nova() as nova:
        cell = nova.cell()

        v2_config = nova._api_client._api_client.configuration
        v2_config.host = v2_config.host[:-1] + "2"
        v2_api_client: api.ApiClient = api.ApiClient(v2_config)
        controller_api = api.ControllerApi(v2_api_client)
        ic(await controller_api.list_robot_controllers(cell.cell_id))


if __name__ == "__main__":
    asyncio.run(main())
