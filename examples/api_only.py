import asyncio

import wandelbots_api_client.v2_pydantic as api
from decouple import config
from icecream import ic

ic.configureOutput(includeContext=True)

NOVA_API: str = config("NOVA_API")
NOVA_USERNAME = config("NOVA_USERNAME")
NOVA_PASSWORD = config("NOVA_PASSWORD")


async def main():
    client_config = api.Configuration(host=f"{NOVA_API}/api/v2")
    api_client: api.ApiClient = api.ApiClient(client_config)
    controller_api = api.ControllerApi(api_client)
    ic(await controller_api.list_robot_controllers("cell"))
    await api_client.close()


if __name__ == "__main__":
    asyncio.run(main())
