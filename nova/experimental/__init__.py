from logging import getLogger

import wandelbots_api_client as wb

from nova.core.nova import Nova


class AsyncRobotController:
    def __init__(self, *, nova: Nova, cell="cell", name: str):
        self._nova = nova
        self._name = name
        self._logger = getLogger(__name__)
        self._cell = cell

    async def stream_state(self, reponse_rate=200):
        """List the state of the robot controller."""
        async for (
            state
        ) in await self._nova._api_client.controller_api.stream_robot_controller_state(
            cell=self._cell, controller=self._name, response_rate=200
        ):
            state: wb.models.RobotControllerState = state
            if len(state.motion_groups) == 0:
                continue

            motion_group = state.motion_groups[0]
            joints = motion_group.joint_position.joints
            velocities = motion_group.joint_velocity
            self._logger.debug(
                f"[({joints[0]},{velocities[0]}), ({joints[1]},{velocities[1]}), ({joints[2]},{velocities[2]}), ({joints[3]},{velocities[3]}), ({joints[4]},{velocities[4]}, ({joints[5]},{velocities[5]}))]"
            )
            pass


__all__ = ["AsyncRobotController"]
