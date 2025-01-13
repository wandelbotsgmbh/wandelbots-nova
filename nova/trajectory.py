from contextlib import contextmanager
from nova.actions import Action, WriteAction
from nova.types import Pose
from nova.actions import ptp, jnt, lin, cir, spl, MotionSettings

JntTarget = tuple[float, ...]

class Trajectory:
    def __init__(self, actions: list[Action] = []):
        self._actions: list[Action] = actions
        self._in_context_settings = None

    def move(self, *, via: str = "ptp", to: Pose | JntTarget | None = None, **kwargs):
        if to is None:
            raise ValueError("Please provide a destination for the movement")

        # Combine default settings with user overrides.
        # User overrides in kwargs take precedence if there's a conflict.
        combined_settings = {**self._in_context_settings, **kwargs} if self._in_context_settings else kwargs
        settings = MotionSettings(**combined_settings)

        match via:
            case "ptp":
                action = ptp(to, settings=settings)
            case "lin":
                action = lin(to, settings=settings)
            case "jnt":
                action = jnt(to, settings=settings)
            case "cir":
                action = cir(to, settings=settings)
            case "spl":
                action = spl(to, settings=settings)
            case _:
                raise ValueError(f"Unknown via option: {via}")

        self._actions.append(action)


    @contextmanager
    def using_settings(self, **kwargs):
        """
        Context manager to apply settings to all actions within the context.
        Args:
            **kwargs: TODO: provide the settings
        """
        # validate that we can build a settings out of the kwargs
        MotionSettings(**kwargs)

        self._in_context_settings = kwargs
        yield
        self._in_context_settings = None


    def _context_aware_settings(self, **kwargs) -> MotionSettings:
        combined_settings = {**self._in_context_settings, **kwargs} if self._in_context_settings else kwargs
        return MotionSettings(**combined_settings)



    def write(self, io_name: str, value: bool | int | float):
        self._actions.append(WriteAction(key=io_name, value=value))

    def build(self) -> list[Action]:
        return self._actions
