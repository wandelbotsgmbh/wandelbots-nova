"""Tests for the trajectory cache wrappers on Controller.

The trajectory caching endpoints are controller scoped
(`/cells/{cell}/controllers/{controller}/trajectories`), so the wrappers live on
`Controller` and forward the cell id and controller id from the configuration.
"""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from nova.cell.controller import Controller
from nova.config import NovaConfig
from nova.core.gateway import ApiGateway


@pytest.fixture
def controller_with_gateway() -> tuple[Controller, MagicMock]:
    """Create a Controller whose ApiGateway is replaced by a mock."""
    gateway = MagicMock(spec=ApiGateway)
    gateway.trajectory_caching_api = MagicMock()

    # NovaDevice._nova_api resolves ApiGateway from nova.core.gateway
    with patch("nova.core.gateway.ApiGateway", return_value=gateway):
        controller = Controller(
            Controller.Configuration(
                config=NovaConfig(host="http://localhost"), cell_id="cell", controller_id="ur10"
            )
        )

    return controller, gateway


async def test_list_cached_trajectories(controller_with_gateway):
    controller, gateway = controller_with_gateway
    response = MagicMock()
    response.trajectories = ["t1", "t2"]
    gateway.trajectory_caching_api.list_trajectories = AsyncMock(return_value=response)

    assert await controller.list_cached_trajectories() == ["t1", "t2"]
    gateway.trajectory_caching_api.list_trajectories.assert_awaited_once_with(
        cell="cell", controller="ur10"
    )


async def test_list_cached_trajectories_returns_empty_list_when_none(controller_with_gateway):
    controller, gateway = controller_with_gateway
    response = MagicMock()
    response.trajectories = None
    gateway.trajectory_caching_api.list_trajectories = AsyncMock(return_value=response)

    assert await controller.list_cached_trajectories() == []


async def test_get_cached_trajectory(controller_with_gateway):
    controller, gateway = controller_with_gateway
    response = MagicMock()
    gateway.trajectory_caching_api.get_trajectory = AsyncMock(return_value=response)

    assert await controller.get_cached_trajectory("t1") is response
    gateway.trajectory_caching_api.get_trajectory.assert_awaited_once_with(
        cell="cell", controller="ur10", trajectory="t1"
    )


async def test_delete_cached_trajectory(controller_with_gateway):
    controller, gateway = controller_with_gateway
    gateway.trajectory_caching_api.delete_trajectory = AsyncMock(return_value=None)

    assert await controller.delete_cached_trajectory("t1") is None
    gateway.trajectory_caching_api.delete_trajectory.assert_awaited_once_with(
        cell="cell", controller="ur10", trajectory="t1"
    )


async def test_clear_trajectory_cache(controller_with_gateway):
    controller, gateway = controller_with_gateway
    gateway.trajectory_caching_api.clear_trajectories = AsyncMock(return_value=None)

    assert await controller.clear_trajectory_cache() is None
    gateway.trajectory_caching_api.clear_trajectories.assert_awaited_once_with(
        cell="cell", controller="ur10"
    )
