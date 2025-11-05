import pytest

import nova.core.gateway as gateway_module
from nova.core.gateway import ApiGateway


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
async def test_api_gateway_host(monkeypatch, given, expected):
    """
    NOVA_API env might not provide any prefix,
    thus ApiGateway should make sure to use the prefix depending on the environment.

    https://wandelbots.atlassian.net/browse/RPS-1208
    """
    monkeypatch.setattr(gateway_module, "NOVA_API", given)
    gateway = ApiGateway(host=given)
    assert gateway._host == expected
    gateway = ApiGateway()
    assert gateway._host == expected


def test_custom_quote_for_ios_preserves_hash_characters():
    """
    Test that _custom_quote_for_ios preserves hash characters in KUKA IO names.
    KUKA uses naming convention like 'OUT#1', 'OUT#2' which should not be URL encoded.
    """
    from nova.core.gateway import _custom_quote_for_ios

    assert _custom_quote_for_ios("OUT#1") == "OUT#1"
    assert _custom_quote_for_ios("OUT#2") == "OUT#2"
    assert _custom_quote_for_ios("IN#5") == "IN#5"

    assert _custom_quote_for_ios("tool_out[0]") == "tool_out[0]"
    assert _custom_quote_for_ios("digital_out[1]") == "digital_out[1]"

    assert _custom_quote_for_ios("OUT#1[0]") == "OUT#1[0]"
