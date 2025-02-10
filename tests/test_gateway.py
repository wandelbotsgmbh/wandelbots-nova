

import pytest

from nova.gateway import ApiGateway


@pytest.mark.asyncio
async def test_api_gateway_host():
    """
    NOVA_API env does not provide any prefix, 
    thus ApiGateway should make sure to use the prefix depending on the environment.

    https://wandelbots.atlassian.net/browse/RPS-1208
    """
    expected_host = "http://some-host.net"
    gateway = ApiGateway(host=expected_host)
    assert gateway._host == expected_host

    expected_host = "https://some-host.net"
    gateway = ApiGateway(host=expected_host)
    assert gateway._host == expected_host

    expected_host = "http://some-host.net"
    gateway = ApiGateway(host="some-host.net")
    assert gateway._host == expected_host

    expected_host = "https://someinstance.wandelbots.io"
    gateway = ApiGateway(host="someinstance.wandelbots.io")
    assert gateway._host == expected_host

