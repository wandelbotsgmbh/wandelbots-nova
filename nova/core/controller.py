from typing import final

from nova.core.motion_group import MotionGroup
from loguru import logger

from nova.gateway import ApiGateway


class Controller:
    def __init__(self, *, api_gateway: ApiGateway, cell: str, controller_host: str):
        self._api_gateway = api_gateway
        self._controller_api = api_gateway.controller_api
        self._motion_group_api = api_gateway.motion_group_api
        self._cell = cell
        self._controller_host = controller_host
        self._motion_groups: dict[str, MotionGroup] = {}

    @final
    async def __aenter__(self):
        activate_all_motion_groups_response = (
            await self._motion_group_api.activate_all_motion_groups(
                cell=self._cell, controller=self._controller_host
            )
        )
        motion_groups = activate_all_motion_groups_response.instances
        for mg in motion_groups:
            logger.info(f"Found motion group {mg.motion_group}")
            motion_group = MotionGroup(
                api_gateway=self._api_gateway, cell=self._cell, motion_group_id=mg.motion_group
            )
            self._motion_groups[motion_group.motion_group_id] = motion_group
        return self

    @final
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        for motion_group_id in self._motion_groups.keys():
            logger.info(f"Deactivating motion group {motion_group_id}")
            await self._motion_group_api.deactivate_motion_group(self._cell, motion_group_id)

        await self._api_gateway.close()

    def get_motion_groups(self) -> dict[str, MotionGroup]:
        return self._motion_groups

    def get_motion_group(self, motion_group_id: str = "0") -> MotionGroup | None:
        return self._motion_groups.get(f"{motion_group_id}@{self._controller_host}", None)
