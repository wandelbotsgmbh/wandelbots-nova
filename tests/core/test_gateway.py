import os

import pytest

from nova.core.gateway import ApiGateway


@pytest.fixture
def nova_api_env():
    """Cleanup env vars after test"""
    original_value = os.environ.get("NOVA_API")

    def _set_nova_api(value: str):
        os.environ["NOVA_API"] = value

    yield _set_nova_api

    if original_value is None:
        os.environ.pop("NOVA_API", None)
    else:
        os.environ["NOVA_API"] = original_value


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "given,expected",
    [
        ("http://some-host.net", "http://some-host.net"),
        ("https://some-host.net", "https://some-host.net"),
        ("some-host.net", "http://some-host.net"),
        ("someinstance.wandelbots.io", "https://someinstance.wandelbots.io"),
        ("http://someinstance.wandelbots.io", "https://someinstance.wandelbots.io"),
        ("https://someinstance.wandelbots.io", "https://someinstance.wandelbots.io"),
        ("https://172.30.1.2", "https://172.30.1.2"),
        ("http://172.30.1.2", "http://172.30.1.2"),
        ("172.30.1.2", "http://172.30.1.2"),
    ],
)
async def test_api_gateway_host(nova_api_env, given, expected):
    """
    NOVA_API env might not provide any prefix,
    thus ApiGateway should make sure to use the prefix depending on the environment.

    https://wandelbots.atlassian.net/browse/RPS-1208
    """
    gateway = ApiGateway(host=given)
    assert gateway._host == expected

    nova_api_env(given)
    gateway = ApiGateway()
    assert gateway._host == expected
