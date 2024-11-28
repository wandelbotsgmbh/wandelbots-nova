import pytest
from wandelbots import use_nova


@pytest.fixture()
def nova_api_client():
    return use_nova()
