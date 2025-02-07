from typing import Literal, Sized

from loguru import logger

from nova.api import models
from nova.core.io import IOAccess
from nova.core.motion_group import MotionGroup
from nova.core.robot_cell import (
    AbstractController,
    AbstractRobot,
    ConfigurablePeriphery,
    Device,
    IODevice,
    ValueType,
)
from nova.gateway import ApiGateway


# TODO: Device is not associated to IODevice so it is pretty confusing and we should change it
class Controller(Sized, AbstractController, ConfigurablePeriphery, Device, IODevice):
    class Configuration(ConfigurablePeriphery.Configuration):
        type: Literal["controller"] = "controller"
        identifier: str = "controller"
        controller_id: str
        # TODO: needs to be removed
        plan: bool = False

    def __init__(
        self, *, api_gateway: ApiGateway, cell: str, controller_instance: models.ControllerInstance
    ):
        self._api_gateway = api_gateway
        self._controller_api = api_gateway.controller_api
        self._motion_group_api = api_gateway.motion_group_api
        self._cell = cell
        self._controller_id = controller_instance.controller
        self._activated_motion_group_ids: list[str] = []
        self._io_access = IOAccess(
            api_gateway=api_gateway, cell=cell, controller_id=controller_instance.controller
        )

        configuration = self.Configuration(
            identifier=controller_instance.controller,
            controller_id=controller_instance.controller,
            plan=False,
        )
        super().__init__(configuration=configuration)

    @property
    def controller_id(self) -> str:
        return self._controller_id

    async def open(self):
        motion_group_ids = await self.activated_motion_group_ids()
        self._activated_motion_group_ids = motion_group_ids
        logger.info(f"Found motion group {motion_group_ids}")
        return self

    async def close(self):
        # RPS-1174: when a motion group is deactivated, RAE closes all open connections
        #           this behaviour is not desired in some cases,
        #           so for now we will not deactivate for the user
        pass

    def __len__(self) -> int:
        return len(self._activated_motion_group_ids)

    # TODO: should accept the exact motion group id as str
    def motion_group(self, motion_group_id: int = 0) -> MotionGroup:
        return MotionGroup(
            api_gateway=self._api_gateway,
            cell=self._cell,
            motion_group_id=f"{motion_group_id}@{self._controller_id}",
        )

    def __getitem__(self, motion_group_id: int) -> MotionGroup:
        return self.motion_group(motion_group_id)

    async def activated_motion_group_ids(self) -> list[str]:
        activate_all_motion_groups_response = (
            await self._motion_group_api.activate_all_motion_groups(
                cell=self._cell, controller=self._controller_id
            )
        )
        motion_groups = activate_all_motion_groups_response.instances
        return [mg.motion_group for mg in motion_groups]

    async def activated_motion_groups(self) -> list[MotionGroup]:
        motion_group_ids = await self.activated_motion_group_ids()
        return [self.motion_group(int(mg.split("@")[0])) for mg in motion_group_ids]

    def get_robots(self) -> dict[str, AbstractRobot]:
        return {
            motion_group_id: self.motion_group(int(motion_group_id.split("@")[0]))
            for motion_group_id in self._activated_motion_group_ids
        }

    async def read(self, key: str) -> ValueType:
        return await self._io_access.read(key)

    async def write(self, key: str, value: ValueType) -> None:
        return await self._io_access.write(key, value)
