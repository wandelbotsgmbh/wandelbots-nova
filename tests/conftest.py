import pytest

from nova.cell.movement_controller.policy import ExecutionPolicy, set_default_execution_policy


@pytest.fixture(autouse=True)
def _strict_execution_policy():
    """Pin the process-wide execution policy to ``strict`` for the whole suite.

    ``strict`` reads the raw ``standstill`` flag and fails on contradicting frames —
    the rules the existing tests were written against, so the suite passing under it
    proves the policy switch changes nothing there. Tests of the robust behaviour
    pass their policy explicitly (or set the default themselves).
    """
    set_default_execution_policy(ExecutionPolicy.strict_policy())
    yield
    set_default_execution_policy(None)
