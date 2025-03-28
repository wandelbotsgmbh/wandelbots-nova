from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass
class ControllerIO:
    """Represents controller IO data"""

    name: str
    value: bool | int | float


class ApiInterface(ABC):
    """Base interface for API clients regardless of version"""

    @abstractmethod
    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]:
        """Get current values of specified IOs"""
        pass

    @abstractmethod
    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None:
        """Set the value of a specific IO"""
        pass

    @abstractmethod
    async def close(self) -> None:
        """Close any open connections"""
        pass
