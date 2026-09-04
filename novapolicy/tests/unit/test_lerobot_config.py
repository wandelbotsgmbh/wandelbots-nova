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
    """A path to the checkpoint's config.json resolves to its directory."""
    config_path = tmp_path / "config.json"
    config_path.write_text(
        json.dumps({"type": "act", "chunk_size": 16, "n_action_steps": 4}),
        encoding="utf-8",
    )

    settings = load_execution_settings(config_path)

    assert settings.chunk_size == 16
    assert settings.n_action_steps == 4


def test_a_json_file_under_another_name_is_not_a_checkpoint(tmp_path) -> None:
    """A differently named file must not silently read the config.json beside it."""
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 11, "n_action_steps": 4}), encoding="utf-8"
    )
    other = tmp_path / "policy.json"
    other.write_text(json.dumps({"type": "act", "chunk_size": 99}), encoding="utf-8")

    with pytest.raises(FileNotFoundError, match=r"must be a directory or its config\.json"):
        load_execution_settings(other)


def test_load_execution_settings_rejects_invalid_execution_horizon(tmp_path) -> None:
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 8, "n_action_steps": 11}),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="LeRobot rejected the checkpoint config"):
        load_execution_settings(tmp_path)


def test_server_only_absolute_checkpoint_requires_local_config() -> None:
    with pytest.raises(FileNotFoundError, match="not accessible on the client"):
        load_execution_settings("/server-only/checkpoint")


def _checkpoint(tmp_path, **overrides) -> object:
    config = {
        "type": "act",
        "chunk_size": 16,
        "n_action_steps": 8,
        "input_features": {
            "observation.state": {"type": "STATE", "shape": [7]},
            "observation.images.scene": {"type": "VISUAL", "shape": [3, 240, 320]},
        },
        "output_features": {"action": {"type": "ACTION", "shape": [7]}},
    }
    config.update(overrides)
    (tmp_path / "config.json").write_text(json.dumps(config), encoding="utf-8")
    return tmp_path


def test_parses_input_and_output_features(tmp_path) -> None:
    settings = load_execution_settings(_checkpoint(tmp_path))

    assert set(settings.input_features) == {
        "observation.state",
        "observation.images.scene",
    }
    assert settings.input_features["observation.state"].shape == (7,)
    assert settings.output_features["action"].shape == (7,)


def test_image_shapes_exposes_visual_features_only(tmp_path) -> None:
    settings = load_execution_settings(_checkpoint(tmp_path))

    assert settings.image_shapes == {"observation.images.scene": (3, 240, 320)}


def test_comparable_features_drop_language_and_env(tmp_path) -> None:
    settings = load_execution_settings(
        _checkpoint(
            tmp_path,
            input_features={
                "observation.state": {"type": "STATE", "shape": [7]},
                "task": {"type": "LANGUAGE", "shape": [1]},
                "observation.environment_state": {"type": "ENV", "shape": [4]},
            },
        )
    )

    assert set(settings.input_features) == {
        "observation.state",
        "task",
        "observation.environment_state",
    }
    assert set(settings.comparable_input_features) == {"observation.state"}


def test_absent_features_load_as_empty(tmp_path) -> None:
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 8, "n_action_steps": 8}),
        encoding="utf-8",
    )

    settings = load_execution_settings(tmp_path)

    assert settings.input_features == {}
    assert settings.comparable_input_features == {}
    assert settings.output_features == {}


def test_null_features_load_as_empty(tmp_path) -> None:
    settings = load_execution_settings(
        _checkpoint(tmp_path, input_features=None, output_features=None)
    )

    assert settings.input_features == {}
    assert settings.output_features == {}


def test_an_unknown_feature_type_is_rejected(tmp_path) -> None:
    """Feature parsing is LeRobot's, so an unknown type fails the load outright."""
    checkpoint = _checkpoint(
        tmp_path,
        input_features={
            "observation.state": {"type": "STATE", "shape": [7]},
            "broken": {"type": "NOT_A_TYPE", "shape": [3]},
        },
    )

    with pytest.raises(ValueError, match="LeRobot rejected the checkpoint config"):
        load_execution_settings(checkpoint)


def test_try_load_returns_none_for_server_only_checkpoint() -> None:
    assert config_module.try_load_execution_settings("/server-only/checkpoint") is None


def test_try_load_returns_none_for_missing_argument() -> None:
    assert config_module.try_load_execution_settings(None) is None


def test_try_load_still_raises_on_a_readable_but_malformed_checkpoint(tmp_path) -> None:
    (tmp_path / "config.json").write_text(
        json.dumps({"type": "act", "chunk_size": 8, "n_action_steps": 11}),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="LeRobot rejected the checkpoint config"):
        config_module.try_load_execution_settings(tmp_path)


@pytest.mark.parametrize(
    ("policy_type", "extra", "expected_chunk", "expected_steps"),
    [
        # ACT names it chunk_size...
        ("act", {"chunk_size": 16, "n_action_steps": 8}, 16, 8),
        # ...diffusion names it horizon...
        ("diffusion", {"horizon": 64, "n_action_steps": 32}, 64, 32),
        # ...tdmpc names it horizon too, with a one-step execution horizon...
        ("tdmpc", {"horizon": 5, "n_action_steps": 1}, 5, 1),
        # ...and pi05 and molmoact2 are back to chunk_size.
        ("pi05", {"chunk_size": 50, "n_action_steps": 50}, 50, 50),
        ("molmoact2", {"chunk_size": 30, "n_action_steps": 30}, 30, 30),
    ],
)
def test_chunk_length_is_derived_for_any_policy(
    tmp_path, policy_type, extra, expected_chunk, expected_steps
) -> None:
    """The chunk field differs per policy; action_delta_indices does not.

    Nothing in novapolicy names these fields — the length comes from LeRobot's
    own abstract property, so a new or renamed policy needs no change here.
    """
    (tmp_path / "config.json").write_text(
        json.dumps({"type": policy_type, **extra}), encoding="utf-8"
    )

    settings = load_execution_settings(tmp_path)

    assert settings.policy_type == policy_type
    assert settings.chunk_size == expected_chunk
    assert settings.n_action_steps == expected_steps


def test_a_policy_without_an_action_chunk_derives_none(tmp_path) -> None:
    """vqbet declares no chunk field at all, but still reports an action window."""
    (tmp_path / "config.json").write_text(json.dumps({"type": "vqbet"}), encoding="utf-8")

    settings = load_execution_settings(tmp_path)

    assert settings.policy_type == "vqbet"
    assert settings.chunk_size == 11
    assert settings.n_action_steps is None


def _write_stats(tmp_path, **stats) -> None:
    """Write a checkpoint's normalizer state the way LeRobot's pipeline saves it."""
    import numpy as np
    from safetensors.numpy import save_file

    state = "normalizer_processor.safetensors"
    (tmp_path / "policy_preprocessor.json").write_text(
        json.dumps({
            "name": "policy_preprocessor",
            "steps": [
                {"registry_name": "device_processor", "config": {}},
                {"registry_name": "normalizer_processor", "config": {}, "state_file": state},
            ],
        }),
        encoding="utf-8",
    )
    save_file(
        {key: np.asarray(values, dtype=np.float32) for key, values in stats.items()},
        tmp_path / state,
    )


def test_min_max_statistics_are_read_from_the_checkpoint(tmp_path) -> None:
    _write_stats(
        _checkpoint(tmp_path),
        **{"observation.state.min": [-3.14] * 7, "observation.state.max": [3.14] * 7},
    )

    settings = load_execution_settings(tmp_path)

    stats = settings.stats["observation.state"]
    assert stats.minimum is not None
    assert len(stats.minimum) == 7
    assert stats.bounds(0) == pytest.approx((-3.14, 3.14))


def test_mean_std_statistics_become_a_four_sigma_band(tmp_path) -> None:
    _write_stats(
        _checkpoint(tmp_path),
        **{"observation.state.mean": [1.0] * 7, "observation.state.std": [0.5] * 7},
    )

    settings = load_execution_settings(tmp_path)

    assert settings.stats["observation.state"].bounds(0) == pytest.approx((-1.0, 3.0))


def test_a_zero_standard_deviation_has_no_usable_bounds(tmp_path) -> None:
    """A constant dimension says nothing about plausibility."""
    _write_stats(
        _checkpoint(tmp_path),
        **{"observation.state.mean": [1.0], "observation.state.std": [0.0]},
    )

    settings = load_execution_settings(tmp_path)

    assert settings.stats["observation.state"].bounds(0) is None


def test_a_checkpoint_without_statistics_loads_with_none(tmp_path) -> None:
    """config.json alone is a normal way to ship a checkpoint."""
    settings = load_execution_settings(_checkpoint(tmp_path))

    assert settings.stats == {}


def test_statistics_are_fetched_for_a_hub_checkpoint(tmp_path, monkeypatch) -> None:
    """A Hub id must reach the stats too, not only a local directory.

    Regression: gating the stats load on ``isinstance(source, Path)`` silently
    skipped the observation range check for every Hub checkpoint — which is how
    checkpoints are normally distributed.
    """
    import numpy as np
    from safetensors.numpy import save_file

    state = "policy_preprocessor_step_3_normalizer_processor.safetensors"
    (tmp_path / "policy_preprocessor.json").write_text(
        json.dumps({"steps": [{"registry_name": "normalizer_processor", "state_file": state}]}),
        encoding="utf-8",
    )
    save_file(
        {
            "observation.state.min": np.zeros(7, dtype=np.float32),
            "observation.state.max": np.ones(7, dtype=np.float32),
        },
        tmp_path / state,
    )
    requested: list[str] = []

    def fake_download(repo_id: str, filename: str, **_kwargs):
        requested.append(filename)
        if filename == "config.json":
            path = tmp_path / "config.json"
            path.write_text(
                json.dumps({"type": "act", "chunk_size": 16, "n_action_steps": 8}),
                encoding="utf-8",
            )
            return str(path)
        candidate = tmp_path / filename
        if not candidate.is_file():
            raise FileNotFoundError(filename)
        return str(candidate)

    monkeypatch.setattr("huggingface_hub.hf_hub_download", fake_download)
    monkeypatch.setattr("lerobot.configs.policies.hf_hub_download", fake_download)

    settings = load_execution_settings("org/some-policy")

    assert "policy_preprocessor.json" in requested
    assert settings.stats["observation.state"].bounds(0) == pytest.approx((0.0, 1.0))
