from typing import final

import wandelbots_api_client as wb
from wandelbots.core.motion_group import MotionGroup
from loguru import logger

from wandelbots.gateway.api_gateway import ApiGateway


class Controller:
    def __init__(self, *, api_gateway: ApiGateway, cell: str, controller_host: str):
        self._api_gateway = api_gateway
        self._controller_api = api_gateway.controller_api
        self._motion_group_api = api_gateway.motion_group_api
        self._cell = cell
        self._controller_host = controller_host
        self._motion_groups: dict[str, MotionGroup] = {}

    async def _get_controller(self, host: str) -> wb.models.ControllerInstance | None:
        controller_list_response = await self._controller_api.list_controllers(cell=self._cell)
        controller_list = list(controller_list_response.instances)
        return next((c for c in controller_list if c.host == host), None)

    @final
    async def __aenter__(self):
        activate_all_motion_groups_response = (
            await self._motion_group_api.activate_all_motion_groups(
                cell=self._cell, controller=self._controller_host
            )
        )
        # TODO: should we store these states? I dont like storing state, then you have to manage them
        #       stateless looks simpler
        # TODO: should we deactivate these motions groups? what does wandelscript does?
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
        # TODO: should we deactivate these motions groups? what does wandelscript does?
        pass

    def get_motion_groups(self) -> dict[str, MotionGroup]:
        return self._motion_groups

    def get_motion_group(self, motion_group_id: str = "0") -> MotionGroup:
        # TODO: I know this doesnt looks good :)
        #       here are some considerations for a better implementation:
        #       If possible I would prefer stateless approach,
        #       so we dont return it from the internal state, but we fetch it from the API with the id
        #       in that case having str id is more fleixble than having an int id
        return self._motion_groups[f"{motion_group_id}@{self._controller_host}"]

    def __getitem__(self, item):
        return self._motion_groups[f"{item}@{self._controller_host}"]
