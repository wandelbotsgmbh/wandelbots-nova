import asyncio

import pytest
from fastapi.testclient import TestClient


def test_get_programs(novax_app):
    """Test GET /programs - list all programs"""
    client = TestClient(novax_app)
    response = client.get("/programs")

    assert response.status_code == 200
    programs = response.json()
    assert isinstance(programs, list)
    assert len(programs) >= 1

    # Check that simple_program is in the list
    program_ids = [p["program_id"] for p in programs]
    assert "simple_program" in program_ids

    # Check program structure
    simple_program = next(p for p in programs if p["program_id"] == "simple_program")
    assert "program_id" in simple_program
    assert "created_at" in simple_program
    assert "updated_at" in simple_program


def test_get_program_success(novax_app):
    """Test GET /programs/{program} - get program details"""
    client = TestClient(novax_app)
    response = client.get("/programs/simple_program")

    assert response.status_code == 200
    program = response.json()

    assert program["program_id"] == "simple_program"
    assert "created_at" in program
    assert "updated_at" in program
    assert "input_schema" in program
    assert "_links" in program
    assert "self" in program["_links"]
    assert "runs" in program["_links"]


def test_get_program_not_found(novax_app):
    """Test GET /programs/{program} - program not found"""
    client = TestClient(novax_app)
    response = client.get("/programs/nonexistent_program")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_list_program_runs_empty(novax_app):
    """Test GET /programs/{program}/runs - list runs (empty)"""
    client = TestClient(novax_app)
    response = client.get("/programs/simple_program/runs")

    assert response.status_code == 200
    runs = response.json()
    assert isinstance(runs, list)
    assert len(runs) == 0


def test_list_program_runs_program_not_found(novax_app):
    """Test GET /programs/{program}/runs - program not found"""
    client = TestClient(novax_app)
    response = client.get("/programs/nonexistent_program/runs")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_run_program_success(novax_app):
    """Test POST /programs/{program}/runs - run program"""
    client = TestClient(novax_app)
    response = client.post(
        "/programs/simple_program/runs", json={"parameters": {"number_of_steps": 5}}
    )

    assert response.status_code == 200
    run = response.json()

    assert "run_id" in run
    assert "program_id" in run
    assert "state" in run


def test_run_program_without_parameters(novax_app):
    """Test POST /programs/{program}/runs - run program without parameters"""
    client = TestClient(novax_app)
    response = client.post("/programs/simple_program/runs", json={})

    assert response.status_code == 200
    run = response.json()

    assert "run_id" in run
    assert "program_id" in run
    assert "state" in run


def test_run_program_not_found(novax_app):
    """Test POST /programs/{program}/runs - program not found"""
    client = TestClient(novax_app)
    response = client.post("/programs/nonexistent_program/runs", json={"parameters": {}})

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_get_program_run_success(novax_app):
    """Test GET /programs/{program}/runs/{run} - get run details"""
    client = TestClient(novax_app)

    # First create a run
    run_response = client.post(
        "/programs/simple_program/runs", json={"parameters": {"number_of_steps": 1}}
    )
    assert run_response.status_code == 200
    run_id = run_response.json()["run_id"]

    # Then get the run details
    response = client.get(f"/programs/simple_program/runs/{run_id}")

    assert response.status_code == 200
    run = response.json()

    assert run["run_id"] == run_id
    assert run["program_id"] == "simple_program"
    assert "state" in run
    assert "_links" in run
    assert "self" in run["_links"]
    assert "stop" in run["_links"]


def test_get_program_run_program_not_found(novax_app):
    """Test GET /programs/{program}/runs/{run} - program not found"""
    client = TestClient(novax_app)
    response = client.get("/programs/nonexistent_program/runs/some_run_id")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_stop_program_run_success(novax_app):
    """Test POST /programs/{program}/runs/{run}/stop - stop run"""
    client = TestClient(novax_app)

    # First create a run
    run_response = client.post(
        "/programs/simple_program/runs", json={"parameters": {"number_of_steps": 10}}
    )
    assert run_response.status_code == 200
    print(run_response.json())
    run_id = run_response.json()["run_id"]
    print(run_id)

    await asyncio.sleep(4)

    r = client.get(f"/programs/simple_program/runs/{run_id}")
    print(r.json())

    # Then stop the run
    response = client.post(f"/programs/simple_program/runs/{run_id}/stop")

    assert response.status_code == 204


def test_stop_program_run_program_not_found(novax_app):
    """Test POST /programs/{program}/runs/{run}/stop - program not found"""
    client = TestClient(novax_app)
    response = client.post("/programs/nonexistent_program/runs/some_run_id/stop")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"
