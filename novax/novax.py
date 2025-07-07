from typing import Any, Optional

from decouple import config
from fastapi import FastAPI

from nova.program.function import Program
from novax.program_manager import ProgramDetails, ProgramManager

CELL_ID = config("CELL_ID", default="cell", cast=str)
BASE_PATH = config("BASE_PATH", default="", cast=str)
app = FastAPI(title="schaeffler", root_path=BASE_PATH)


class Novax:
    def __init__(self, program_manager_override: ProgramManager | None = None):
        self._program_manager: ProgramManager = program_manager_override or ProgramManager()
        self._app: FastAPI | None = None

    @property
    def program_manager(self) -> ProgramManager:
        """Get the program manager instance"""
        return self._program_manager

    def register_program(self, program: Program) -> str:
        """
        Register a function or wandelscript file as a program.

        Args:
            program: A Program object (decorated with @nova.program)

        Returns:
            str: The program ID
        """
        return self._program_manager.register_program(program)

    async def get_programs(self) -> dict[str, ProgramDetails]:
        """Get all registered programs"""
        return await self._program_manager.get_programs()

    async def get_program(self, program_id: str) -> Optional[ProgramDetails]:
        """Get a specific program by ID"""
        return await self._program_manager.get_program(program_id)

    async def execute_program(
        self, program_id: str, parameters: Optional[dict[str, Any]] = None
    ) -> Any:
        """Execute a registered program with given parameters"""
        return await self._program_manager.run_program(program_id, parameters)

    def create_app(self, title: str = "Novax API", version: str = "1.0.0") -> FastAPI:
        """
        Create a FastAPI application with the programs router included.

        Args:
            title: The title of the API
            version: The version of the API

        Returns:
            FastAPI: The configured FastAPI application
        """
        if self._app is not None:
            return self._app

        self._app = FastAPI(
            title=title,
            version=version,
            description="Novax API for managing and executing programs",
        )
        return self._app

    def include_programs_router(self, app: FastAPI) -> FastAPI:
        """
        Include the programs router in the FastAPI application.

        Args:
            app: The FastAPI application to include the programs router in

        Returns:
            FastAPI: The configured FastAPI application
        """
        from .api.programs import get_program_manager
        from .api.programs import router as programs_router

        # Override the dependency function to return our program manager
        def get_program_manager_override():
            return self._program_manager

        # Replace the dependency function on the FastAPI app
        app.dependency_overrides[get_program_manager] = get_program_manager_override

        # Include the programs router
        app.include_router(programs_router)

        return app
