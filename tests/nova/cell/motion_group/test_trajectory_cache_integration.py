"""Integration tests for the trajectory cache wrappers on Controller.

Covers the full round trip: cache a planned trajectory, list it, read it back,
delete it, and clear the cache.
"""

from math import pi

import pytest

from nova.actions import jnt
from nova.api import models
from nova.cell.controllers import virtual_controller
from nova.core.nova import Nova


@pytest.fixture
async def ur_controller_and_mg():
    """Set up a virtual UR controller and yield it together with its motion group."""
    controller_name = "ur-trajectory-cache-test"

    async with Nova() as nova:
        cell = nova.cell()
        await cell.ensure_controller(
            virtual_controller(
                name=controller_name,
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=[0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0],
            )
        )

        ur = await cell.controller(controller_name)
        async with ur[0] as mg:
            yield ur, mg


@pytest.mark.asyncio
@pytest.mark.integration
async def test_trajectory_cache_round_trip(ur_controller_and_mg):
    controller, mg = ur_controller_and_mg
    tcp = await mg.active_tcp_name()

    await controller.clear_trajectory_cache()
    assert await controller.list_cached_trajectories() == []

    joints = await mg.joints()
    target = (joints[0] + 0.1,) + joints[1:]
    joint_trajectory = await mg.plan([jnt(target)], tcp)
    trajectory_id = await mg._load_planned_motion(joint_trajectory, tcp)

    assert trajectory_id in await controller.list_cached_trajectories()

    cached = await controller.get_cached_trajectory(trajectory_id)
    assert cached.motion_group == mg.id
    assert cached.tcp == tcp

    await controller.delete_cached_trajectory(trajectory_id)
    assert trajectory_id not in await controller.list_cached_trajectories()


@pytest.mark.asyncio
@pytest.mark.integration
async def test_clear_trajectory_cache(ur_controller_and_mg):
    controller, mg = ur_controller_and_mg
    tcp = await mg.active_tcp_name()

    joints = await mg.joints()
    target = (joints[0] + 0.1,) + joints[1:]
    joint_trajectory = await mg.plan([jnt(target)], tcp)
    await mg._load_planned_motion(joint_trajectory, tcp)

    assert await controller.list_cached_trajectories() != []

    await controller.clear_trajectory_cache()
    assert await controller.list_cached_trajectories() == []
