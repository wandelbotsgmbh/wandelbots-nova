from __future__ import annotations

from collections.abc import Iterable
from typing import Annotated, AsyncIterator, Callable

import pydantic

from nova import api
from nova.actions.base import Action
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.actions.motions import CollisionFreeMotion, Motion
from nova.types import MotionSettings, MovementControllerFunction, Pose


def located_writes(actions: Iterable[Action]) -> list[tuple[int, WriteAction]]:
    """Pair each :class:`WriteAction` with its motion index along the path.

    The index is the number of motions preceding the write; waits are skipped
    and do not advance it. This is the location convention IO rides on — an
    integer location marks a motion-command boundary — and it is shared by both
    the single-motion-group (:meth:`CombinedActions.to_set_io`) and the
    synchronized multi-motion-group IO overlays.
    """
    out: list[tuple[int, WriteAction]] = []
    motion_index = 0
    for action in actions:
        if action.is_motion():
            motion_index += 1
        elif isinstance(action, WriteAction):
            out.append((motion_index, action))
    return out


def write_to_set_io(write: WriteAction, location: float) -> api.models.SetIO:
    """Build the :class:`api.models.SetIO` overlay entry for a write at ``location``."""
    return api.models.SetIO(io=write.to_api_model(), location=location, io_origin=write.origin)


class ActionLocation(pydantic.BaseModel):
    """A container for an action at a specific path parameter"""

    path_parameter: float = 1.0
    action: WriteAction


# TODO: all actions should be allowed (Action)
ActionContainerItem = Motion | WriteAction | WaitAction


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

    def _generate_trajectory(self) -> tuple[list[Motion], list[ActionLocation]]:
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
            if isinstance(item, Motion):
                motions.append(item)
                last_motion_index += 1  # Increment the motion index for each new Motion
            else:
                # Assign the current value of last_motion_index as path_parameter for actions
                actions.append(ActionLocation(path_parameter=last_motion_index, action=item))

        return motions, actions

    @property
    def motions(self) -> list[Motion]:
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
            if isinstance(motion, CollisionFreeMotion):
                continue

            settings = motion.settings or MotionSettings()
            blending = settings.as_blending_setting() if settings.has_blending_settings() else None
            limits_override = (
                settings.as_limits_settings() if settings.has_limits_override() else None
            )
            motion_command = api.models.MotionCommand(
                path=motion.to_api_model(), blending=blending, limits_override=limits_override
            )
            motion_commands.append(motion_command)
        return motion_commands

    def to_set_io(self) -> list[api.models.SetIO]:
        return [write_to_set_io(write, location) for location, write in located_writes(self.items)]


# TODO: should not be located here
class MovementControllerContext(pydantic.BaseModel):
    combined_actions: CombinedActions
    motion_id: str
    start_on_io: api.models.StartOnIO | None = None
    pause_on_io: api.models.PauseOnIO | None = None
    motion_group_state_stream_gen: Callable[[], AsyncIterator[api.models.MotionGroupState]]
    # The planned trajectory being executed. Optional: only location-bounded
    # cursor operations need it, one-shot execution does not.
    joint_trajectory: api.models.JointTrajectory | None = None


MovementController = Callable[[MovementControllerContext], MovementControllerFunction]
