from typing import Any, Optional

from fastapi import FastAPI

from nova.program.function import Program
from novax.program_manager import ProgramDetails, ProgramManager, ProgramSource


class Novax:
    def __init__(self):
        self._program_manager: ProgramManager = ProgramManager()
        self._app: FastAPI | None = None

    @property
    def program_manager(self) -> ProgramManager:
        """Get the program manager instance"""
        return self._program_manager

    def register_program_source(self, program_source: ProgramSource) -> None:
        """Register a program source"""
        self._program_manager.register_program_source(program_source)

    def deregister_program_source(self, program_source: ProgramSource) -> None:
        """Deregister a program source"""
        self._program_manager.deregister_program_source(program_source)

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

    def create_app(self, title: str = "Novax API", version: str = "1.0.0", root_path="") -> FastAPI:
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
            root_path=root_path,
            docs_url="/",
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
