from contextlib import asynccontextmanager
from typing import AsyncIterator, Optional

from decouple import config
from fastapi import APIRouter, FastAPI

from nova.cell.robot_cell import RobotCell
from nova.core.logging import logger
from nova.program.function import Program
from nova.program.store import Program as StoreProgram
from nova.program.store import ProgramStore
from novax.program_manager import ProgramDetails, ProgramManager

# Read BASE_PATH environment variable and extract app name
_BASE_PATH = config("BASE_PATH", default="/default/novax")
_APP_NAME = _BASE_PATH.split("/")[-1] if "/" in _BASE_PATH else "novax"
logger.info(f"Extracted app name '{_APP_NAME}' from BASE_PATH '{_BASE_PATH}'")

# Create nats programs bucket name
_CELL_NAME = config("CELL_NAME", default="")
_NATS_PROGRAM_BUCKET = f"nova_{_CELL_NAME}_programs"
_NATS_CLIENT_CONFIG = {"connect_timeout": 2.0, "allow_reconnect": True, "max_reconnect_attempts": 2}


class Novax:
    def __init__(self, robot_cell_override: RobotCell | None = None):
        self._program_manager: ProgramManager = ProgramManager(
            robot_cell_override=robot_cell_override
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
        await self._register_programs()

        try:
            yield
        finally:
            await self._deregister_programs()

    async def _register_programs(self):
        """
        Handle FastAPI startup - discover and register programs from sources to store
        """
        try:
            logger.info("Novax: Starting program discovery and registration to store")
            programs = await self._program_manager.get_programs()

            store_programs = {}
            for program_id, program_details in programs.items():
                try:
                    preconditions_dict = None
                    if program_details.preconditions:
                        preconditions_dict = program_details.preconditions.model_dump()

                    # TODO: schema is not present in ProgramDetails
                    store_program = StoreProgram(
                        program=program_details.program,
                        name=program_details.name,
                        description=program_details.description,
                        app=_APP_NAME,
                        preconditions=preconditions_dict,
                    )

                    store_programs[program_id] = store_program
                except Exception as e:
                    logger.error(f"Failed to convert program {program_id} to store format: {e}")

            async with ProgramStore(
                nats_bucket_name=_NATS_PROGRAM_BUCKET, nats_client_config=_NATS_CLIENT_CONFIG
            ) as program_store:
                for program_id, store_program in store_programs.items():
                    try:
                        await program_store.put(f"{_APP_NAME}.{program_id}", store_program)
                        logger.debug(f"Program {program_id} synced to store")
                    except Exception as e:
                        logger.error(f"Failed to sync program {program_id} to store: {e}")

            logger.info(
                f"Novax: {len(store_programs)} programs discovered and synced to store on startup"
            )
        except Exception as e:
            logger.error(f"Novax startup error: Failed to register programs to store: {e}")
            # Don't raise the exception to prevent app startup failure

    async def _deregister_programs(self):
        """
        Handle FastAPI shutdown - cleanup programs from store
        """
        try:
            logger.info("Novax: Starting program cleanup from store on shutdown")
            # Get current programs first
            programs = self._program_manager._programs
            program_ids = list(programs.keys())
            program_count = len(program_ids)

            async with ProgramStore(
                nats_bucket_name=_NATS_PROGRAM_BUCKET, nats_client_config=_NATS_CLIENT_CONFIG
            ) as program_store:
                for program_id in program_ids:
                    try:
                        await program_store.delete(f"{_APP_NAME}.{program_id}")
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

        if not _CELL_NAME:
            logger.error(
                "Novax: CELL_NAME environment variable is not set, your programs will not be registered"
            )
        else:
            programs_router.lifespan_context = self.program_store_lifespan

        # Include the programs router
        app.include_router(programs_router)

        return app
