from typing import Any, Protocol

from nova.api.types.controller_io import ControllerIO


class ApiInterface(Protocol):
    async def list_io_values(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[ControllerIO]: ...

    async def set_io_value(self, cell: str, controller: str, io_name: str, value: Any) -> None: ...
