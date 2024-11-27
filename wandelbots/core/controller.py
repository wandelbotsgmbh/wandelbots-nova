from typing import final

import wandelbots_api_client as wb
from wandelbots.core.motion_group import MotionGroup
from loguru import logger


class Controller:
    def __init__(self, nova: wb.ApiClient, cell: str, controller_host: str):
        self._nova_client = nova
        self._controller_api = wb.ControllerApi(api_client=self._nova_client)
        self._motion_group_api = wb.MotionGroupApi(api_client=self._nova_client)
        self._cell = cell
        self._controller_host = controller_host
        self._motion_groups: dict[str, MotionGroup] = {}

    async def _get_controller(self, host: str) -> wb.models.ControllerInstance | None:
        controller_list_response = await self._controller_api.list_controllers(cell=self._cell)
        controller_list = list(controller_list_response.instances)
        return next((c for c in controller_list if c.host == host), None)

    @final
    async def __aenter__(self):
        logger.info(f"Get controller {self._controller_host}...")
        controller = await self._get_controller(self._controller_host)
        if not controller:
            raise RuntimeError(f"Controller host {self._controller_host} could not be registered.")

        logger.info("Activate all motion groups...")
        activate_all_motion_groups_response = await self._motion_group_api.activate_all_motion_groups(
            cell=self._cell, controller=self._controller_host
        )
        motion_groups = activate_all_motion_groups_response.instances
        for mg in motion_groups:
            logger.info(f"Found motion group {mg.motion_group}")
            motion_group = MotionGroup(nova=self._nova_client, cell=self._cell, motion_group_id=mg.motion_group)
            self._motion_groups[motion_group.motion_group_id] = motion_group
        return self

    @final
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self._nova_client.close()
        pass

    def get_motion_groups(self) -> dict[str, MotionGroup]:
        return self._motion_groups

    def get_motion_group(self, motion_group_id: str) -> MotionGroup:
        return self._motion_groups[motion_group_id]

    def __getitem__(self, item):
        return self._motion_groups[f"{item}@{self._controller_host}"]
