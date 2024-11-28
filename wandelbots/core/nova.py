import wandelbots_api_client as wb
from decouple import config

NOVA_HOST = config("NOVA_HOST")
NOVA_USERNAME = config("NOVA_USERNAME", default=None)
NOVA_PASSWORD = config("NOVA_PASSWORD", default=None)
NOVA_ACCESS_TOKEN = config("NOVA_ACCESS")


def use_nova(
    host: str = NOVA_HOST,
    username: str | None = NOVA_USERNAME,
    password: str | None = NOVA_PASSWORD,
    access_token: str | None = NOVA_ACCESS_TOKEN,
    version: str = "v1",
) -> wb.ApiClient:
    config = wb.Configuration(
        host=f"http://{host}/api/{version}",
        username=username,
        password=password,
        access_token=access_token,
        ssl_ca_cert=False,
    )
    return wb.ApiClient(config)


def use_nova_basic_auth(
    host: str = NOVA_HOST,
    username: str | None = NOVA_USERNAME,
    password: str | None = NOVA_PASSWORD,
    version: str = "v1",
) -> wb.ApiClient:
    return use_nova(host=host, username=username, password=password, access_token=None, version=version)


def use_nova_access_token(
    host: str = NOVA_HOST, access_token: str | None = NOVA_ACCESS_TOKEN, version: str = "v1"
) -> wb.ApiClient:
    return use_nova(host=host, username=None, password=None, access_token=access_token, version=version)
