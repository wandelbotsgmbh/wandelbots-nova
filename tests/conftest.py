import pytest
from wandelbots import Nova


@pytest.fixture()
def nova():
    return Nova()
