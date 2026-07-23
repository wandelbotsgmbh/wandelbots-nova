"""Tests for LeRobot checkpoint execution settings."""

from __future__ import annotations

import json

import pytest

config_module = pytest.importorskip("novapolicy.lerobot.config")
load_execution_settings = config_module.load_execution_settings


def test_load_execution_settings_from_checkpoint_directory(tmp_path) -> None:
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 11, "n_action_steps": 8}),
        encoding="utf-8",
    )

    settings = load_execution_settings(tmp_path)

    assert settings.policy_type == "act"
    assert settings.chunk_size == 11
    assert settings.n_action_steps == 8


def test_load_execution_settings_from_config_file(tmp_path) -> None:
    config_path = tmp_path / "policy.json"
    config_path.write_text(
        json.dumps({"type": "act", "chunk_size": 16, "n_action_steps": 4}),
        encoding="utf-8",
    )

    settings = load_execution_settings(config_path)

    assert settings.chunk_size == 16
    assert settings.n_action_steps == 4


def test_load_execution_settings_rejects_invalid_execution_horizon(tmp_path) -> None:
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 8, "n_action_steps": 11}),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="n_action_steps greater than chunk_size"):
        load_execution_settings(tmp_path)


def test_server_only_absolute_checkpoint_requires_local_config() -> None:
    with pytest.raises(FileNotFoundError, match="not accessible on the client"):
        load_execution_settings("/server-only/checkpoint")
