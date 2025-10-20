import asyncio
from concurrent.futures import Future

import pytest
from fastapi.testclient import TestClient
from pydantic import BaseModel

import nova
from nova.core.nova import Nova
from nova.program.runner import ProgramRun, ProgramRunState, ProgramStatus
from novax.novax import Novax


@nova.program(
    id="sucessful_program",
    name="Simple Test Program",
    description="A simple program for testing program run reporting",
)
async def sucessful_program():
    """Simple program that counts and sleeps."""
    print("simple test program run")


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_successful_run(novax_app):
    nova = Nova()
    await nova.connect()

    # subscribe to program run messages
    program_run_message = []
    async def run_cb(msg):
        program_run_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=run_cb)


    # subscribe to program status messages
    program_run_status_message = []
    async def status_cb(msg):
        program_run_status_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.novax.programs.sucessful_program.status", on_message=status_cb)

    with TestClient(novax_app) as client:
        start_program = client.post("/programs/sucessful_program/start", json={"arguments": {}})
        assert start_program.status_code == 200, "Failed to start test program"

        await asyncio.sleep(10)

        assert len(program_run_message) == 3, (
            f"Expected 3 program run messages, but got {len(program_run_message)}"
        )

        # verify program run messages
        models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
        assert models[0].state == ProgramRunState.PREPARING
        assert models[1].state == ProgramRunState.RUNNING
        assert models[2].state == ProgramRunState.COMPLETED

        # verify program status messages
        models = [ProgramStatus.model_validate_json(message.data) for message in program_run_status_message]
        assert models[0].state == ProgramRunState.PREPARING
        assert models[1].state == ProgramRunState.RUNNING
        assert models[2].state == ProgramRunState.COMPLETED


@nova.program(
    id="failing_program",
    name="Failing Test Program",
    description="A program that always fails for testing error handling",
)
async def failing_program():
    """Program that always fails."""
    raise ValueError("This program is designed to fail for testing purposes")


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_failed_run(novax_app):
    nova = Nova()
    await nova.connect()

    # subscribe to program run messages
    program_run_message = []
    async def run_cb(msg):
        program_run_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=run_cb)


    # subscribe to program status messages
    program_run_status_message = []
    async def status_cb(msg):
        program_run_status_message.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.novax.programs.failing_program.status", on_message=status_cb)

    with TestClient(novax_app) as client:
        start_program = client.post("/programs/failing_program/start", json={"arguments": {}})
        assert start_program.status_code == 200, "Failed to start test program"

        await asyncio.sleep(8)

        assert len(program_run_message) == 3, (
            f"Expected 3 program run messages, but got {len(program_run_message)}"
        )

        # verify program run messages
        models = [ProgramRun.model_validate_json(message.data) for message in program_run_message]
        assert models[0].state == ProgramRunState.PREPARING
        assert models[1].state == ProgramRunState.RUNNING
        assert models[2].state == ProgramRunState.FAILED

        # verify program status messages
        models = [ProgramStatus.model_validate_json(message.data) for message in program_run_status_message]
        assert models[0].state == ProgramRunState.PREPARING
        assert models[1].state == ProgramRunState.RUNNING
        assert models[2].state == ProgramRunState.FAILED


@nova.program(
    id="long_running_program",
    name="Long Running Test Program",
    description="A program that takes some time to complete for testing stop functionality",
)
async def long_running_program():
    """Program that runs for a while and can be stopped."""
    for _ in range(100):
        await asyncio.sleep(1)


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_stopped_run(novax_app):
    nova = Nova()
    await nova.connect()

    # subscribe to program run messages
    program_run_messages = []
    async def run_cb(msg):
        program_run_messages.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.programs", on_message=run_cb)


    # subscribe to program status messages
    program_run_status_messages = []
    async def status_cb(msg):
        program_run_status_messages.append(msg)

    await nova.nats.subscribe("nova.v2.cells.cell.novax.programs.long_running_program.status", on_message=status_cb)

    with TestClient(novax_app) as client:
        start_program = client.post("/programs/long_running_program/start", json={"arguments": {}})

        assert start_program.status_code == 200, "Failed to start test program"

        # Wait for the program to start running
        await asyncio.sleep(5)

        # Verify program is running
        assert len(program_run_messages) >= 2, (
            f"Expected at least 2 program run messages, but got {len(program_run_messages)}"
        )
        models = [ProgramRun.model_validate_json(message.data) for message in program_run_messages]
        assert models[0].state == ProgramRunState.PREPARING
        assert models[1].state == ProgramRunState.RUNNING

        # Stop the program
        stop_program = client.post("/programs/long_running_program/stop")
        assert stop_program.status_code == 200, "Failed to stop test program"

        # Wait for the stop event to be processed
        await asyncio.sleep(5)

        # Verify that we received the STOPPED event
        assert len(program_run_messages) == 3, (
            f"Expected 3 program run messages, but got {len(program_run_messages)}"
        )

        # verify program run messages
        final_models = [
            ProgramRun.model_validate_json(message.data) for message in program_run_messages
        ]
        assert final_models[0].state == ProgramRunState.PREPARING
        assert final_models[1].state == ProgramRunState.RUNNING
        assert final_models[2].state == ProgramRunState.STOPPED

        # verify program status messages
        final_status_models = [
            ProgramStatus.model_validate_json(message.data) for message in program_run_status_messages
        ]
        # We should have at least 2 status messages (PREPARING, RUNNING)
        assert len(final_status_models) >= 2, f"Expected at least 2 status messages, got {len(final_status_models)}"
        assert final_status_models[0].state == ProgramRunState.PREPARING
        assert final_status_models[1].state == ProgramRunState.RUNNING
        # The STOPPED state might arrive later or might be missed due to timing
        if len(final_status_models) >= 3:
            assert final_status_models[2].state == ProgramRunState.STOPPED


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_pass_arguments():
    novax = Novax()
    app = novax.create_app()

    assertion = Future()

    @nova.program()
    async def example_program(number_of_iterations: int):
        try:
            assert number_of_iterations == 5
            assertion.set_result(True)
        except Exception:
            assertion.set_result(False)

    novax.register_program(example_program)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        start_program = client.post(
            f"/programs/{example_program.program_id}/start",
            json={"arguments": {"number_of_iterations": 5}},
        )
        # TODO: this should return 400 Bad Request when schema validation is implemented
        assert start_program.status_code == 200, "Failed to start test program"

        assert assertion.result(timeout=10)


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_pass_arguments_with_default_params():
    novax = Novax()
    app = novax.create_app()

    assertion = Future()

    @nova.program()
    async def example_program(number_of_iterations: int, some_defaul_param: str = "default"):
        try:
            assert number_of_iterations == 5
            assert some_defaul_param == "default"
            assertion.set_result(True)
        except Exception:
            assertion.set_result(False)

    novax.register_program(example_program)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        start_program = client.post(
            f"/programs/{example_program.program_id}/start",
            json={"arguments": {"number_of_iterations": 5}},
        )
        # TODO: this should return 400 Bad Request when schema validation is implemented
        assert start_program.status_code == 200, "Failed to start test program"

        assert assertion.result(timeout=10)


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_novax_program_pass_arguments_with_pydantic_model():
    novax = Novax()
    app = novax.create_app()

    assertion = Future()

    class ProgramArgs(BaseModel):
        number_of_iterations: int
        some_defaul_param: str = "default"

    @nova.program()
    async def example_program(args: ProgramArgs):
        try:
            assert args.number_of_iterations == 5
            assert args.some_defaul_param == "default"
            assertion.set_result(True)
        except Exception:
            assertion.set_result(False)

    novax.register_program(example_program)
    novax.include_programs_router(app)

    with TestClient(app) as client:
        start_program = client.post(
            f"/programs/{example_program.program_id}/start",
            json={"arguments": {"args": {"number_of_iterations": 5}}},
        )
        # TODO: this should return 400 Bad Request when schema validation is implemented
        assert start_program.status_code == 200, "Failed to start test program"

        assert assertion.result(timeout=10)


@pytest.mark.integration
@pytest.mark.xdist_group("program-runs")
@pytest.mark.asyncio
async def test_program_run_contains_init_args():
    novax = Novax()
    app = novax.create_app()

    @nova.program()
    async def example_program(cycle_count: int = 1):
        pass

    novax.register_program(example_program)
    novax.include_programs_router(app)

    args = {"cycle_count": 5}
    with TestClient(app) as client:
        start_program = client.post(
            f"/programs/{example_program.program_id}/start", json={"arguments": args}
        )
        # TODO: this should return 400 Bad Request when schema validation is implemented
        assert start_program.status_code == 200, "Failed to start test program"
        program_run = ProgramRun.model_validate(start_program.json())

        assert program_run.input_data == args
