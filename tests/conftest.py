import pytest
from nova import Nova


@pytest.fixture()
def nova():
    return Nova()
