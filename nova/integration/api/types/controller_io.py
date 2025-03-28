from dataclasses import dataclass


@dataclass
class ControllerIO:
    key: str
    value: bool | int | float
