from contextlib import contextmanager
from typing import Optional, Sequence

from nova.actions import Action
from nova.actions.motions import Motion
from nova.types import MotionSettings


class TrajectoryBuilder:
    def __init__(self, settings: Optional[MotionSettings] = None):
        self._actions: list[Action] = []
        self._settings_stack: list[MotionSettings] = [] if settings is None else [settings]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    @property
    def actions(self) -> list[Action]:
        return self._actions

    def _current_settings(self) -> MotionSettings:
        print([s.tcp_velocity_limit for s in self._settings_stack])
        return self._settings_stack[-1]

    def move(self, motion: Motion):
        motion.settings = motion.settings or self._current_settings()
        self._actions.append(motion)

    def trigger(self, action: Action):
        self._actions.append(action)

    def sequence(self, actions: Sequence[Action]):
        for action in actions:
            if isinstance(action, Motion):
                self.move(action)
            else:
                self.trigger(action)

    @contextmanager
    def set(self, settings: MotionSettings):
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
