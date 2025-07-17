from fastapi.testclient import TestClient


def test_get_programs(novax_app):
    client = TestClient(novax_app)
    response = client.get("/programs")

    assert response.status_code == 200
    programs = response.json()
    assert isinstance(programs, list)
    assert len(programs) >= 1

    # Check that simple_program is in the list
    program_ids = [p["program"] for p in programs]
    assert "simple_program" in program_ids

    # Check program structure
    simple_program = next(p for p in programs if p["program"] == "simple_program")
    assert "program" in simple_program
    assert "name" in simple_program
    assert "description" in simple_program
    assert "created_at" in simple_program


def test_get_program_success(novax_app):
    client = TestClient(novax_app)
    response = client.get("/programs/simple_program")

    assert response.status_code == 200
    program = response.json()

    assert program["program"] == "simple_program"
    assert "created_at" in program
    assert "name" in program
    assert "description" in program
    assert "input_schema" in program


def test_get_program_not_found(novax_app):
    client = TestClient(novax_app)
    response = client.get("/programs/nonexistent_program")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_start_program_success(novax_app):
    client = TestClient(novax_app)
    response = client.post(
        "/programs/simple_program/start", json={"parameters": {"number_of_steps": 5}}
    )

    assert response.status_code == 200
    run = response.json()

    assert "run" in run
    assert "program" in run
    assert "state" in run


def test_start_program_without_parameters(novax_app):
    client = TestClient(novax_app)
    response = client.post("/programs/simple_program/start", json={})

    assert response.status_code == 200
    run = response.json()

    assert "run" in run
    assert "program" in run
    assert "state" in run


def test_start_program_not_found(novax_app):
    client = TestClient(novax_app)
    response = client.post("/programs/nonexistent_program/start", json={"parameters": {}})

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_start_program(novax_app):
    client = TestClient(novax_app)

    # Start first program
    response1 = client.post(
        "/programs/simple_program/start", json={"parameters": {"number_of_steps": 10}}
    )
    assert response1.status_code == 200


def test_stop_program_success(novax_app):
    client = TestClient(novax_app)

    # First start a program
    start_response = client.post(
        "/programs/simple_program/start", json={"parameters": {"number_of_steps": 30}}
    )
    assert start_response.status_code == 200

    # Then stop the program
    response = client.post("/programs/simple_program/stop")
    assert response.status_code == 200, response.json()


def test_stop_program_not_found(novax_app):
    client = TestClient(novax_app)
    response = client.post("/programs/nonexistent_program/stop")

    assert response.status_code == 404
    assert response.json()["detail"] == "Program not found"


def test_stop_program_not_running(novax_app):
    client = TestClient(novax_app)
    response = client.post("/programs/simple_program/stop")

    assert response.status_code == 400
    assert response.json()["detail"] == "No program is running"


def test_stop_program_wrong_program(novax_app):
    client = TestClient(novax_app)

    # Start a program
    start_response = client.post(
        "/programs/simple_program/start", json={"parameters": {"number_of_steps": 10}}
    )
    assert start_response.status_code == 200

    # Try to stop a non existing program
    response = client.post("/programs/different_program/stop")
    assert response.status_code == 404
