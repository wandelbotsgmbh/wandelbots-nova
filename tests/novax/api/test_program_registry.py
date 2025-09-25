import asyncio
import threading

import httpx
import pytest
import uvicorn
from wandelbots_api_client.v2.models.program import Program

import nova
from nova.cell.simulation import SimulatedRobotCell
from nova.core.nova import Nova
from nova.program.function import Program as DecoratedProgram
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


async def get_from_novax(base_url: str, program_id: str) -> Program | None:
    """Get a single program from Novax REST API."""
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{base_url}/programs/{program_id}")
        if response.status_code != 200:
            return None
        return Program(**response.json())


async def get_all_from_novax(base_url: str) -> list[Program]:
    """Get all programs from Novax REST API."""
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{base_url}/programs")
        if response.status_code == 200:
            return [Program(**program) for program in response.json()]
        return []


async def get_from_discovery_service(program_id: str, cell: str = "cell") -> Program:
    """Get a single program from Discovery Service using Wandelbots API client."""
    import wandelbots_api_client.v2 as wb
    from wandelbots_api_client.v2.api import ProgramApi

    async with Nova(version="v2") as nova:
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
    expected_program: DecoratedProgram, found_program: Program
) -> None:
    """Helper function to assert that a program definition matches the expected program."""
    assert expected_program.program_id == found_program.program
    assert expected_program.name == found_program.name
    assert expected_program.description == found_program.description
    assert expected_program.input_schema == found_program.input_schema
    assert expected_program.preconditions == found_program.preconditions


# TODO: this approach is closer to what happens in reality, web server runs in a thread an we interact with it from an external system
#       however this is not standard approach, usually we test with some test clients etc...
#       evaluate if we should switch and if this makes integration test quality less or more
@pytest.fixture
async def server_runner():
    """Fixture that provides a server runner for any FastAPI app. Only one server at a time."""
    current_server = {"thread": None, "port": None}

    async def run_app(app, port=8001, timeout=10):
        """Run the given app on the specified port. Shuts down previous server first."""
        if current_server["thread"] is not None:
            current_server["thread"].join(timeout=2)
            await asyncio.sleep(1)

        def run_server():
            uvicorn.run(app, host="0.0.0.0", port=port, log_level="error")

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        current_server["thread"] = server_thread
        current_server["port"] = port

        # Wait for server to be ready
        counter = 0
        base_url = f"http://localhost:{port}"
        while counter < timeout:
            try:
                async with httpx.AsyncClient() as client:
                    response = await client.get(f"{base_url}/programs")
                    if response.status_code == 200:
                        break
            except Exception:
                pass
            finally:
                counter += 1
                await asyncio.sleep(1)

        if counter == timeout:
            raise TimeoutError(f"Failed to start server on port {port}")

        return base_url

    yield run_app

    # Cleanup: shutdown the current server
    if current_server["thread"] is not None:
        current_server["thread"].join(timeout=5)


@nova.program(
    id="program_with_cycle_failure",
    name="Test cycle failed",
    description="A program that report cycle failure",
)
async def example_program():
    pass


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_program_definition_across_all_sources(server_runner):
    """Test program definition retrieval from all sources: Novax API, NATS, and Discovery Service."""
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.register_program(example_program)
    novax.include_programs_router(app)

    # Run the app (server will shutdown any previous server first)
    base_url = await server_runner(app)

    # NOVAX
    all_programs = await get_all_from_novax(base_url)
    found_program = filter_programs_by_name(all_programs, example_program.program_id, "novax")
    assert_program_definition_matches(example_program, found_program)

    found_program = await get_from_novax(base_url, example_program.program_id)
    assert_program_definition_matches(example_program, found_program)

    # NATS
    all_programs = await get_all_from_nats()
    found_program = filter_programs_by_name(all_programs, example_program.program_id, "novax")
    assert_program_definition_matches(example_program, found_program)

    found_program = await get_from_nats(example_program.program_id)
    assert_program_definition_matches(example_program, found_program)

    # Discovery Service
    all_programs = await get_all_from_discovery_service()
    found_program = filter_programs_by_name(all_programs, example_program.program_id, "novax")
    assert_program_definition_matches(example_program, found_program)

    found_program = await get_from_discovery_service(example_program.program_id)
    assert_program_definition_matches(example_program, found_program)


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_empty_program_registry(server_runner):
    """Test program registry when no programs are registered."""
    # Create app without any programs
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.include_programs_router(app)  # Just add the router, no programs

    # Run the app (server will shutdown previous server first)
    base_url = await server_runner(app)

    # Test the registry is empty using helper function
    programs = await get_all_from_novax(base_url)
    assert isinstance(programs, list)
    assert len(programs) == 0
