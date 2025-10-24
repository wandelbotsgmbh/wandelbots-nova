import asyncio

import pytest
from fastapi.testclient import TestClient
from pydantic import BaseModel
from wandelbots_api_client.v2.models.program import Program

import nova
from nova import api
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program.function import Program as DecoratedProgram
from nova.program.function import ProgramPreconditions
from novax import Novax


async def get_from_nats(program_id: str, cell_id: str = "cell") -> Program:
    """Get a single program from NATS using ProgramStore."""
    from nova.program.store import ProgramStore

    async with Nova() as nova:
        store = ProgramStore(cell_id=cell_id, nats_client=nova.nats)
        program = await store.get(f"novax.{program_id}")
        if program is None:
            raise ValueError(f"Program {program_id} not found in NATS")
        return program


async def get_all_from_nats(cell_id: str = "cell") -> list[Program]:
    """Get all programs from NATS using ProgramStore."""
    from nova.program.store import ProgramStore

    async with Nova() as nova:
        store = ProgramStore(cell_id=cell_id, nats_client=nova.nats)
        programs = await store.get_all()
        return programs


async def get_from_novax(client: TestClient, program_id: str) -> Program | None:
    """Get a single program from Novax REST API using FastAPI TestClient."""
    loop = asyncio.get_running_loop()
    response = await loop.run_in_executor(None, lambda: client.get(f"/programs/{program_id}"))
    if response.status_code != 200:
        return None
    return Program(**response.json())


async def get_all_from_novax(client: TestClient) -> list[Program]:
    """Get all programs from Novax REST API using FastAPI TestClient."""
    loop = asyncio.get_running_loop()
    response = await loop.run_in_executor(None, lambda: client.get("/programs"))
    if response.status_code == 200:
        return [Program(**program) for program in response.json()]
    return []


async def get_from_discovery_service(program_id: str, cell: str = "cell") -> Program:
    """Get a single program from Discovery Service using Wandelbots API client."""
    import wandelbots_api_client.v2 as wb
    from wandelbots_api_client.v2.api import ProgramApi

    async with Nova() as nova:
        v1_api_client = nova._api_client._api_client
        config = wb.Configuration(
            host=v1_api_client.configuration.host.replace("/v1", "/v2"),
            username=v1_api_client.configuration.username,
            password=v1_api_client.configuration.password,
            access_token=v1_api_client.configuration.access_token,
        )

        program_discovery_api = ProgramApi(wb.ApiClient(configuration=config))
        return await program_discovery_api.get_program(cell=cell, program=f"novax.{program_id}")


async def get_all_from_discovery_service(cell: str = "cell") -> list[Program]:
    """Get all programs from Discovery Service using Wandelbots API client."""
    import wandelbots_api_client.v2 as wb
    from wandelbots_api_client.v2.api import ProgramApi

    async with Nova() as nova:
        v1_api_client = nova._api_client._api_client
        config = wb.Configuration(
            host=v1_api_client.configuration.host.replace("/v1", "/v2"),
            username=v1_api_client.configuration.username,
            password=v1_api_client.configuration.password,
            access_token=v1_api_client.configuration.access_token,
        )

        program_discovery_api = ProgramApi(wb.ApiClient(configuration=config))
        return await program_discovery_api.list_programs(cell=cell)


def filter_programs_by_name(programs: list[Program], id: str, app: str) -> Program | None:
    """Filter a list of programs by name and app."""
    for program in programs:
        if program.program == id and program.app == app:
            return program
    return None


def assert_program_definition_matches(
    expected_program: DecoratedProgram | None, found_program: Program | None
) -> None:
    """Helper function to assert that a program definition matches the expected program."""
    if expected_program is None:
        assert found_program is None
        return

    assert found_program is not None, (
        f"Expected program '{expected_program.program_id}' but none was found"
    )

    assert expected_program.program_id == found_program.program
    assert expected_program.name == found_program.name
    assert expected_program.description == found_program.description
    assert expected_program.input_schema == found_program.input_schema
    assert (
        expected_program.preconditions.model_dump(mode="json")
        if expected_program.preconditions
        else None
    ) == found_program.preconditions


async def verify_program_definition_all_sources(
    client: TestClient, decorated_program: DecoratedProgram
):
    # NOVAX
    all_programs = await get_all_from_novax(client)
    found_program = filter_programs_by_name(all_programs, decorated_program.program_id, "novax")
    assert_program_definition_matches(decorated_program, found_program)

    found_program = await get_from_novax(client, decorated_program.program_id)
    assert_program_definition_matches(decorated_program, found_program)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_program_definition_for_simple_program():
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax()
    app = novax.create_app()

    # Define a program to test
    @nova.program(
        id="program_with_cycle_failure",
        name="Test cycle failed",
        description="A program that report cycle failure",
    )
    async def example_program():
        pass

    novax.register_program(example_program)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        await verify_program_definition_all_sources(client, example_program)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_program_definition_for_program_with_preconditions():
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax()
    app = novax.create_app()

    @nova.program(
        id="program_with_preconditions",
        name="Test cycle failed",
        description="A program that report cycle failure",
        preconditions=ProgramPreconditions(
            controllers=[
                virtual_controller(
                    name="controller1",
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
                ),
                virtual_controller(
                    name="controller2",
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
                ),
            ],
            cleanup_controllers=False,
        ),
    )
    async def program_with_preconditions():
        pass

    novax.register_program(program_with_preconditions)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        await verify_program_definition_all_sources(client, program_with_preconditions)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_program_definition_for_program_with_input_schema():
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax()
    app = novax.create_app()

    @nova.program(
        id="program_with_preconditions",
        name="Test cycle failed",
        description="A program that report cycle failure",
    )
    async def program_with_preconditions(should_reset: bool, enable_opc_ua: bool):
        pass

    novax.register_program(program_with_preconditions)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        await verify_program_definition_all_sources(client, program_with_preconditions)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_program_definition_for_program_with_pydantic_input_schema():
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax()
    app = novax.create_app()

    class ProgramInputSchema(BaseModel):
        should_reset: bool
        enable_opc_ua: bool

    @nova.program(
        id="program_with_preconditions",
        name="Test cycle failed",
        description="A program that report cycle failure",
    )
    async def program_with_preconditions(program_input: ProgramInputSchema):
        pass

    novax.register_program(program_with_preconditions)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        await verify_program_definition_all_sources(client, program_with_preconditions)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_program_definition_for_program_with_input_schema_and_preconditions():
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax()
    app = novax.create_app()

    @nova.program(
        id="program_with_preconditions",
        name="Test cycle failed",
        description="A program that report cycle failure",
        preconditions=ProgramPreconditions(
            controllers=[
                virtual_controller(
                    name="controller1",
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
                ),
                virtual_controller(
                    name="controller2",
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
                ),
            ],
            cleanup_controllers=False,
        ),
    )
    async def program_with_preconditions(should_reset: bool, enable_opc_ua: bool):
        pass

    novax.register_program(program_with_preconditions)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        await verify_program_definition_all_sources(client, program_with_preconditions)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_empty_program_registry():
    """Test program registry when no programs are registered."""
    # Create app without any programs
    novax = Novax()
    app = novax.create_app()
    novax.include_programs_router(app)  # Just add the router, no programs

    with TestClient(app) as client:
        # Test the registry is empty using helper function
        programs = await get_all_from_novax(client)
        assert isinstance(programs, list)
        assert len(programs) == 0
