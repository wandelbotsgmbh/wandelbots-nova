from __future__ import annotations

from typing import Annotated, Callable

import pydantic

from nova import api
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.actions.motions import CollisionFreeMotion, Motion
from nova.types import MotionSettings, MovementControllerFunction, Pose


class ActionLocation(pydantic.BaseModel):
    """A container for an action at a specific path parameter"""

    path_parameter: float = 1.0
    action: WriteAction


# TODO: all actions should be allowed (Action)
ActionContainerItem = Motion | WriteAction | CollisionFreeMotion | WaitAction


class CombinedActions(pydantic.BaseModel):
    """A trajectory of motions and actions"""

    # See: https://docs.pydantic.dev/latest/concepts/serialization/#serialize_as_any-runtime-setting
    items: tuple[
        Annotated[
            pydantic.SerializeAsAny[ActionContainerItem], pydantic.Field(discriminator="type")
        ],
        ...,
    ] = ()

    def __len__(self):
        return len(self.items)

    def __getitem__(self, item):
        return self.items[item]

    def __setattr__(self, key, value):
        if key == "items":
            raise TypeError("Cannot set items directly")
        super().__setattr__(key, value)

    def __iter__(self):
        return iter(self.items)

    def append(self, item: ActionContainerItem):
        super().__setattr__("items", self.items + (item,))

    def _generate_trajectory(
        self,
    ) -> tuple[list[Motion | CollisionFreeMotion], list[ActionLocation]]:
        """Generate two lists: one of Motion objects and another of ActionContainer objects,
        where each ActionContainer wraps a non-Motion action with its path parameter.

        The path parameter is the index of the last Motion object in the list of Motion objects.
        S - M - M - A - A - M - M - A - M - M
        0 - 1 - 2 - 3 - 3 - 3 - 4 - 5 - 5 - 6

        Returns:
            tuple: A tuple containing:
                - list of Motion objects from self.items.
                - list of ActionContainer objects with indexed path parameters.
        """
        motions = []
        actions = []
        last_motion_index = 0

        for item in self.items:
            if isinstance(item, WaitAction):
                continue  # Skip WaitAction items
            if isinstance(item, Motion) or isinstance(item, CollisionFreeMotion):
                motions.append(item)
                last_motion_index += 1  # Increment the motion index for each new Motion
            else:
                # Assign the current value of last_motion_index as path_parameter for actions
                actions.append(ActionLocation(path_parameter=last_motion_index, action=item))

        return motions, actions

    @property
    def motions(self) -> list[Motion | CollisionFreeMotion]:
        motions, _ = self._generate_trajectory()
        return motions

    @property
    def actions(self) -> list[ActionLocation]:
        _, actions = self._generate_trajectory()
        return actions

    @property
    def start(self) -> ActionContainerItem | None:
        return self.motions[0] if self.motions else None

    @property
    def end(self) -> ActionContainerItem | None:
        return self.motions[-1] if self.motions else None

    def poses(self) -> list[Pose]:
        """Returns the positions of all motions. If a motion is not a cartesian motion, the position is ignored

        Returns: the positions

        """
        motions, _ = self._generate_trajectory()
        return [
            Pose(position=motion.target.position, orientation=motion.target.orientation)
            for motion in motions
            if isinstance(motion.target, Pose)
        ]

    def positions(self):
        """Returns the positions of all motions. If a motion is not a cartesian motion, the position is ignored

        Returns: the positions

        """
        return [pose.position for pose in self.poses()]

    def orientations(self):
        """Returns the orientations of all motions. If a motion is not a cartesian motion, the orientation is ignored

        Returns: the orientations

        """
        return [pose.orientation for pose in self.poses()]

    def __add__(self, other: CombinedActions) -> CombinedActions:
        return CombinedActions(items=self.items + other.items)

    def to_motion_command(self) -> list[api.models.MotionCommand]:
        motion_commands = []
        for motion in self.motions:
            path = api.models.MotionCommandPath.from_dict(motion.to_api_model().model_dump())
            settings = motion.settings or MotionSettings()
            blending = settings.as_blending_setting() if settings.has_blending_settings() else None
            limits_override = (
                settings.as_limits_settings() if settings.has_limits_override() else None
            )
            motion_commands.append(
                api.models.MotionCommand(
                    path=path, blending=blending, limits_override=limits_override
                )
            )
        return motion_commands

    def to_set_io(self) -> list[api.models.SetIO]:
        return [
            api.models.SetIO(
                io=api.models.IOValue(**action.action.to_api_model()),
                location=action.path_parameter,
            )
            for action in self.actions
        ]


# TODO: should not be located here
class MovementControllerContext(pydantic.BaseModel):
    combined_actions: CombinedActions
    motion_id: str


MovementController = Callable[[MovementControllerContext], MovementControllerFunction]
