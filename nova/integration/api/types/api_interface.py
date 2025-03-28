from typing import Any, Protocol

from nova.api.types.controller_io import ControllerIO
from nova.api.types.system import SystemInfo, SystemVersion


class ApiInterface(Protocol):
    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]: ...

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None: ...

    async def get_system_info(self) -> SystemInfo: ...

    async def get_system_version(self) -> SystemVersion: ...

    async def close(self) -> None: ...
