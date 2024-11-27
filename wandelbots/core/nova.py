import wandelbots_api_client as wb


def use_nova(
    host: str,
    username: str | None = None,
    password: str | None = None,
    access_token: str | None = None,
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
    host: str, username: str | None = None, password: str | None = None, version: str = "v1"
) -> wb.ApiClient:
    return use_nova(host=host, username=username, password=password, access_token=None)


def use_nova_access_token(host: str, access_token: str | None = None, version: str = "v1") -> wb.ApiClient:
    return use_nova(host=host, username=None, password=None, access_token=access_token)
