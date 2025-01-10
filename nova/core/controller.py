from typing import Sized, final

from loguru import logger

from nova.core.motion_group import MotionGroup
from nova.gateway import ApiGateway


class Controller(Sized):
    def __init__(self, *, api_gateway: ApiGateway, cell: str, controller_host: str):
        self._api_gateway = api_gateway
        self._controller_api = api_gateway.controller_api
        self._motion_group_api = api_gateway.motion_group_api
        self._cell = cell
        self._controller_host = controller_host
        self._activated_motion_group_ids: list[str] = []

    @final
    async def __aenter__(self):
        motion_group_ids = await self.activated_motion_group_ids()
        self._activated_motion_group_ids = motion_group_ids
        logger.info(f"Found motion group {motion_group_ids}")
        return self

    @final
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        for motion_group_id in self._activated_motion_group_ids:
            logger.info(f"Deactivating motion group {motion_group_id}")
            await self._motion_group_api.deactivate_motion_group(self._cell, motion_group_id)

    def __len__(self) -> int:
        return len(self._activated_motion_group_ids)

    def motion_group(self, motion_group_id: int = 0) -> MotionGroup:
        return MotionGroup(
            api_gateway=self._api_gateway,
            cell=self._cell,
            motion_group_id=f"{motion_group_id}@{self._controller_host}",
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(motion_group_id)

    async def activated_motion_group_ids(self) -> list[str]:
        activate_all_motion_groups_response = (
            await self._motion_group_api.activate_all_motion_groups(
                cell=self._cell, controller=self._controller_host
            )
        )
        motion_groups = activate_all_motion_groups_response.instances
        return [mg.motion_group for mg in motion_groups]

    async def activated_motion_groups(self) -> list[MotionGroup]:
        motion_group_ids = await self.activated_motion_group_ids()
        return [self.motion_group(int(mg.split("@")[0])) for mg in motion_group_ids]
