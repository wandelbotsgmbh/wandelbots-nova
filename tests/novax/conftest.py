import asyncio
import threading
import time

import httpx
import pytest
import uvicorn

import nova
from nova.cell.simulation import SimulatedRobotCell
from novax import Novax
from tests.novax.api.test_program_cycle import (
    program_with_cycle_data,
    program_with_cycle_extra,
    program_with_cycle_failure,
)
from tests.novax.api.test_program_run import (
    failing_program,
    long_running_program,
    sucessful_program,
)


@nova.program(name="simple_program")
async def simple_program(number_of_steps: int = 30):
    print("Hello World!")

    for i in range(number_of_steps):
        print(f"Step: {i}")
        await asyncio.sleep(1)

    print("Finished Hello World!")


@pytest.fixture
def novax_app():
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.include_programs_router(app)

    novax.register_program(simple_program)

    # programs for integration tests
    novax.register_program(sucessful_program)
    novax.register_program(failing_program)
    novax.register_program(program_with_cycle_data)
    novax.register_program(program_with_cycle_failure)
    novax.register_program(program_with_cycle_extra)
    novax.register_program(long_running_program)

    yield app


@pytest.fixture
def novax_server(novax_app):
    """Fixture that starts a Novax server in a separate thread and returns the test client."""

    # Start server in a separate thread
    def run_server():
        uvicorn.run(novax_app, host="0.0.0.0", port=8000, log_level="error")

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    counter = 0
    while counter < 10:
        try:
            response = httpx.get("http://localhost:8000/programs")
            if response.status_code == 200:
                break
        except Exception:
            pass
        finally:
            counter += 1
            time.sleep(1)

    if counter == 10:
        raise TimeoutError("Failed to start Novax server")

    yield "http://localhost:8000"

    server_thread.join(timeout=5)
    # Note: uvicorn.run() doesn't provide a clean shutdown mechanism
    # The daemon thread will be automatically cleaned up when the process exits
