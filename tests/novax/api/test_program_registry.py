import asyncio
import threading

import httpx
import pytest
import uvicorn

import nova
from nova.cell.simulation import SimulatedRobotCell
from nova.core.nova import Nova
from novax import Novax


# TODO: this approach is closer to what happens in reality, web server runs in a thread an we interact with it from an external system
#       however this is not standard approach, usually we test with some test clients etc...
#       evaluate if we should switch and if this makes integration test quality less or more
@pytest.fixture
async def server_runner():
    """Fixture that provides a server runner for any FastAPI app. Only one server at a time."""
    current_server = {"thread": None, "port": None}

    async def run_app(app, port=8000, timeout=10):
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
async def test_novax_api_get_all_programs(server_runner):
    """Test program registry with the example program."""
    # Create app with example program
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.register_program(example_program)
    novax.include_programs_router(app)

    # Run the app (server will shutdown any previous server first)
    base_url = await server_runner(app)

    # Test the registry
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{base_url}/programs")
        assert response.status_code == 200

        programs = response.json()
        assert isinstance(programs, list)
        assert len(programs) >= 1

        assert example_program.program_id in programs[0]["program"]
        assert example_program.name == programs[0]["name"]
        assert example_program.description == programs[0]["description"]
        assert example_program.input_schema == programs[0]["input_schema"]
        assert example_program.preconditions == programs[0]["preconditions"]


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_api_get_all_programs_when_no_programs_registered(server_runner):
    """Test program registry with no programs registered."""
    # Create app without any programs
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.include_programs_router(app)  # Just add the router, no programs

    # Run the app (server will shutdown previous server first)
    base_url = await server_runner(app)

    # Test the registry is empty
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{base_url}/programs")
        assert response.status_code == 200

        programs = response.json()
        assert isinstance(programs, list)
        assert len(programs) == 0


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_nats_program_store_get_program(server_runner):
    """Test program registry with the example program."""
    # Create app with example program
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.register_program(example_program)
    novax.include_programs_router(app)

    # Run the app (server will shutdown any previous server first)
    base_url = await server_runner(app)

    from nova import Nova
    from nova.program.store import ProgramStore

    async with Nova() as nova:
        store = ProgramStore(cell_id=novax._cell.cell_id, nats_client=nova.nats)
        program_definition = await store.get(f"novax.{example_program.program_id}")

        assert example_program.program_id in program_definition.program
        assert example_program.name == program_definition.name
        assert example_program.description == program_definition.description
        assert example_program.input_schema == program_definition.input_schema
        assert example_program.preconditions == program_definition.preconditions


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_nats_jetstream_get_all(server_runner):
    """Test program registry with the example program."""
    # Create app with example program
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.register_program(example_program)
    novax.include_programs_router(app)

    # Run the app (server will shutdown any previous server first)
    base_url = await server_runner(app)

    from nova import Nova
    from nova.program.store import ProgramStore

    async with Nova() as nova:
        store = ProgramStore(cell_id=novax._cell.cell_id, nats_client=nova.nats)
        all_programs = await store.get_all()
        program_definition = [
            p for p in all_programs if p.program == example_program.program_id and p.app == "novax"
        ][0]

        assert example_program.program_id in program_definition.program
        assert example_program.name == program_definition.name
        assert example_program.description == program_definition.description
        assert example_program.input_schema == program_definition.input_schema
        assert example_program.preconditions == program_definition.preconditions


@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_program_definition_from_discovery_service(server_runner):
    """Test program registry with the example program."""
    # Create app with example program
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.register_program(example_program)
    novax.include_programs_router(app)

    # Run the app (server will shutdown any previous server first)
    base_url = await server_runner(app)

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
        program_definition = await program_discovery_api.get_program(
            cell="cell", program=f"novax.{example_program.program_id}"
        )
        assert example_program.program_id in program_definition.program
        assert example_program.name == program_definition.name
        assert example_program.description == program_definition.description
        assert example_program.input_schema == program_definition.input_schema
        assert example_program.preconditions == program_definition.preconditions

        programs = await program_discovery_api.list_programs(cell="cell")
        program_definition = [
            p for p in programs if p.program == example_program.program_id and p.app == "novax"
        ][0]

        assert example_program.program_id in program_definition.program
        assert example_program.name == program_definition.name
        assert example_program.description == program_definition.description
        assert example_program.input_schema == program_definition.input_schema
        assert example_program.preconditions == program_definition.preconditions
