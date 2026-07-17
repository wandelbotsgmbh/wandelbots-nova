import os
import sys

import pytest

import novax.novax as novax_module
from novax.cli import main


@pytest.fixture
def record_serve(monkeypatch):
    """Replace ``Novax.serve`` with a recorder so the CLI never starts uvicorn."""
    calls: dict = {}

    def fake_serve(self, host="0.0.0.0", port=3000, **kwargs):
        calls["host"] = host
        calls["port"] = port

    monkeypatch.setattr(novax_module.Novax, "serve", fake_serve)
    return calls


def test_cli_run_without_target_skips_import(monkeypatch, record_serve):
    imported: list = []
    monkeypatch.setattr(novax_module, "_import_module", lambda *a, **k: imported.append(a))
    monkeypatch.setattr(sys, "argv", ["novax", "run"])

    main()

    # No target -> nothing imported explicitly; serving still happens (scan is in serve()).
    assert imported == []
    assert record_serve["host"] == "0.0.0.0"
    assert record_serve["port"] == 3000


def test_cli_run_with_target_imports_it(monkeypatch, record_serve):
    imported: list = []
    monkeypatch.setattr(
        novax_module, "_import_module", lambda target, *a, **k: imported.append(target)
    )
    monkeypatch.setattr(sys, "argv", ["novax", "run", "pkg/one.py", "--port", "1234"])

    main()

    assert imported == ["pkg/one.py"]
    assert record_serve["port"] == 1234


def test_cli_run_sets_cell_env(monkeypatch, record_serve):
    # Baseline via setenv so monkeypatch restores CELL_NAME after the test.
    monkeypatch.setenv("CELL_NAME", "original")
    monkeypatch.setattr(novax_module, "_import_module", lambda *a, **k: None)
    monkeypatch.setattr(sys, "argv", ["novax", "run", "--cell", "mycell"])

    main()

    assert os.environ["CELL_NAME"] == "mycell"
