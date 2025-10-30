from contextlib import asynccontextmanager
from typing import AsyncIterator, Optional

from fastapi import APIRouter, FastAPI

from nova.core.nova import Nova
from nova.logging import logger
from nova.program.function import Program
from nova.program.store import Program as StoreProgram
from nova.program.store import ProgramStore
from novax.config import APP_NAME, CELL_NAME
from novax.program_manager import ProgramDetails, ProgramManager


class Novax:
    def __init__(self, *, app_name: str | None = None):
        """Initialize the Novax class.

        Args:
            app_name (str | None, optional): This one is read from the environment variable APP_NAME. Only change it for development purposes. Defaults to None.
        """
        app_name = app_name or APP_NAME

        nova = Nova()
        self._nova = nova
        self._cell = self._nova.cell(cell_id=CELL_NAME)

        self._program_manager: ProgramManager = ProgramManager(
            cell_id=CELL_NAME, app_name=app_name, nova_config=nova.config
        )
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

    def deregister_program(self, program_id: str):
        """
        Deregister a program

        Args:
            program_id: The ID of the program to deregister
        """
        self._program_manager.deregister_program(program_id)

    @asynccontextmanager
    async def program_store_lifespan(self, router: APIRouter) -> AsyncIterator[None]:
        """
        Lifespan context manager for FastAPI application lifecycle.
        Handles startup and shutdown events.
        """
        await self._nova.connect()
        logger.info("Novax: Connected to Nova API")
        store = ProgramStore(cell_id=self._cell.cell_id, nats_client=self._nova.nats)
        await self._register_programs(store)
        logger.info("Novax: Programs registered to store on startup")

        yield
        await self._deregister_programs(store)
        await self._stop_program()
        await self._nova.close()

    async def _stop_program(self):
        """
        Stop the currently running program, if any.
        """
        try:
            program_id = self._program_manager.running_program
            if program_id:
                await self._program_manager.stop_program(program_id)
        except Exception as e:
            logger.error(f"Failed to stop program: {e}")

    async def _register_programs(self, program_store: ProgramStore):
        """
        Handle FastAPI startup - discover and register programs from sources to store
        """
        try:
            logger.info("Novax: Starting program discovery and registration to store")
            programs = await self._program_manager.get_programs()

            store_programs = {}
            for program_id, program_details in programs.items():
                try:
                    # TODO: schema is not present in ProgramDetails
                    store_program = StoreProgram(
                        program=program_details.program,
                        name=program_details.name,
                        description=program_details.description,
                        app=APP_NAME,
                        preconditions=program_details.preconditions,
                        input_schema=program_details.input_schema,
                    )

                    store_programs[program_id] = store_program
                except Exception as e:
                    logger.error(f"Failed to convert program {program_id} to store format: {e}")

            for program_id, store_program in store_programs.items():
                try:
                    await program_store.put(f"{APP_NAME}.{program_id}", store_program)
                    logger.debug(f"Program {program_id} synced to store")
                except Exception as e:
                    logger.error(f"Failed to sync program {program_id} to store: {e}")

            logger.info(
                f"Novax: {len(store_programs)} programs discovered and synced to store on startup"
            )
        except Exception as e:
            logger.error(f"Novax startup error: Failed to register programs to store: {e}")
            # Don't raise the exception to prevent app startup failure

    async def _deregister_programs(self, program_store: ProgramStore):
        """
        Handle FastAPI shutdown - cleanup programs from store
        """
        try:
            logger.info("Novax: Starting program cleanup from store on shutdown")
            programs = self._program_manager._programs
            program_ids = list(programs.keys())
            program_count = len(program_ids)

            for program_id in program_ids:
                try:
                    await program_store.delete(f"{APP_NAME}.{program_id}")
                    logger.debug(f"Program {program_id} removed from store")
                except Exception as e:
                    logger.error(f"Failed to remove program {program_id} from store: {e}")

            logger.info(f"Novax: Shutdown complete, removed {program_count} programs from store")

        except Exception as e:
            logger.error(f"Novax shutdown error: {e}")

    async def get_programs(self) -> dict[str, ProgramDetails]:
        """Get all registered programs"""
        return await self._program_manager.get_programs()

    async def get_program(self, program_id: str) -> Optional[ProgramDetails]:
        """Get a specific program by ID"""
        return await self._program_manager.get_program(program_id)

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

        if not CELL_NAME:
            logger.error(
                "Novax: CELL_NAME environment variable is not set, your programs will not be registered"
            )
        else:
            programs_router.lifespan_context = self.program_store_lifespan

        # Include the programs router
        app.include_router(programs_router)

        return app
