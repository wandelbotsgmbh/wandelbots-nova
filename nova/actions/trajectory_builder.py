from contextlib import contextmanager
from typing import Optional

from nova.actions import Action
from nova.actions.motions import Motion
from nova.types import MotionSettings


class TrajectoryBuilder:
    def __init__(self, settings: Optional[MotionSettings] = None):
        """A builder for trajectories.

        Args:
            settings (Optional[MotionSettings], optional): The global settings to use for the trajectory. Defaults to None.
        """
        self._actions: list[Action] = []
        self._settings_stack: list[MotionSettings] = [settings or MotionSettings()]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    @property
    def actions(self) -> list[Action]:
        """The built actions in the trajectory."""
        return self._actions

    def _current_settings(self) -> MotionSettings:
        """The current settings to use for the trajectory."""
        return self._settings_stack[-1]

    def move(self, motion: Motion):
        """Add a motion to the trajectory."""
        motion.settings = motion.settings or self._current_settings()
        self._actions.append(motion)

    def trigger(self, action: Action):
        """Add a action to the trajectory."""
        self._actions.append(action)

    def sequence(self, *args: Action | list[Action] | tuple[Action, ...]):
        """Add a sequence of actions to the trajectory.

        Args:
            *args: Actions can be passed as individual Action objects, or as lists/tuples of Actions.
                  For example: sequence(action1, action2) or sequence([action1, action2])
        """
        for arg in args:
            if isinstance(arg, (list, tuple)):
                # Handle list/tuple of actions
                for action in arg:
                    if isinstance(action, Motion):
                        self.move(action)
                    else:
                        self.trigger(action)
            else:
                # Handle individual action
                if isinstance(arg, Motion):
                    self.move(arg)
                else:
                    self.trigger(arg)

    @contextmanager
    def set(self, settings: MotionSettings):
        """Set the settings for the trajectory context."""
        self._settings_stack.append(settings)
        try:
            yield self
        finally:
            self._settings_stack.pop()

    def __iter__(self):
        return iter(self._actions)

    def __len__(self):
        return len(self._actions)

    def __getitem__(self, idx):
        return self._actions[idx]
