import threading
import time

import pytest
import uvicorn
from fastapi.testclient import TestClient

from nova.cell.simulation import SimulatedRobotCell
from novax import Novax

from .programs.test_programs import (
    test_cycle,
    test_cycle_failed,
    test_program_run_failed,
    test_program_run_succeded,
)


@pytest.fixture
def novax_app():
    novax = Novax(robot_cell_override=SimulatedRobotCell())
    app = novax.create_app()
    novax.include_programs_router(app)

    # Register test programs
    novax.register_program(test_program_run_succeded)
    novax.register_program(test_program_run_failed)
    novax.register_program(test_cycle)
    novax.register_program(test_cycle_failed)

    yield app


@pytest.fixture
def novax_server(novax_app):
    """Fixture that starts a Novax server in a separate thread and returns the test client."""

    # Start server in a separate thread
    def run_server():
        uvicorn.run(novax_app, host="0.0.0.0", port=8000, log_level="error")

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # Wait for server to start
    test_client = TestClient(novax_app)
    max_retries = 30
    for _ in range(max_retries):
        try:
            response = test_client.get("/programs", timeout=1)
            if response.status_code == 200:
                break
        except Exception:
            time.sleep(1)
    else:
        raise RuntimeError("Novax server did not start correctly")

    yield test_client

    server_thread.join(timeout=30)
    # Note: uvicorn.run() doesn't provide a clean shutdown mechanism
    # The daemon thread will be automatically cleaned up when the process exits
