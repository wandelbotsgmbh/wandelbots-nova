from __future__ import annotations

from typing import Annotated, AsyncIterator, Awaitable, Callable

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
        return [
            api.models.SetIO(
                io=action.action.to_api_model(),
                location=action.path_parameter,
                io_origin=action.action.origin,
            )
            for action in self.actions
            if isinstance(action.action, WriteAction)
        ]


# TODO: should not be located here
class MovementControllerContext(pydantic.BaseModel):
    combined_actions: CombinedActions
    motion_id: str
    start_on_io: api.models.StartOnIO | None = None
    pause_on_io: api.models.PauseOnIO | None = None
    # Awaits until the ``pause_on_io`` condition no longer holds. Set by the
    # motion group (which has the API client); the one-shot movement controller
    # uses it to resume a controller-side IO pause, since the controller never
    # resumes by itself. Without it an IO pause ends the execution early.
    wait_for_pause_on_io_release: Callable[[], Awaitable[None]] | None = None
    # Awaits until the source of the ``pause_on_io`` signal is gone (e.g. the bus-IO
    # service is not connected). The controller stops evaluating the condition in
    # that case and would keep moving; the one-shot movement controller pauses the
    # robot itself instead and resumes through ``wait_for_pause_on_io_release``.
    wait_for_pause_signal_loss: Callable[[], Awaitable[None]] | None = None
    motion_group_state_stream_gen: Callable[[], AsyncIterator[api.models.MotionGroupState]]
    # The planned trajectory being executed. Optional: only location-bounded
    # cursor operations need it, one-shot execution does not.
    joint_trajectory: api.models.JointTrajectory | None = None
    # The resolved server-side IO overlay (``StartMovementRequest.set_outputs``).
    # Set when write actions carry path triggers, which are resolved against the
    # planned trajectory (see nova.actions.path_trigger_resolver). When ``None``
    # controllers fall back to ``combined_actions.to_set_io()``.
    set_outputs: list[api.models.SetIO] | None = None


MovementController = Callable[[MovementControllerContext], MovementControllerFunction]
