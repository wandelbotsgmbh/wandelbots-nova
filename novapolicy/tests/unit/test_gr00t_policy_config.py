"""Tests for the declared GR00T deployment config and its validation against evidence."""

from __future__ import annotations

from pathlib import Path

import pytest

from novapolicy.gr00t import policy_config
from novapolicy.gr00t.inspect_policy import Evidence, validate

EXAMPLE = Path(__file__).parents[2] / "examples" / "gr00t_single_arm_pick_and_place" / "policy.toml"


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "policy.toml"
    path.write_text(text)
    return path


def test_the_example_config_loads():
    cfg = policy_config.load(EXAMPLE)
    assert cfg.policy.dt_ms == pytest.approx(1000.0 / 30.0)
    assert cfg.policy.tcp_action.position_scale_mm == 1000.0
    assert cfg.policy.gripper.closes(-1.0)
    assert not cfg.policy.gripper.closes(1.0)
    assert cfg.cell.cameras["top"] == "314522065367"


@pytest.mark.parametrize(
    ("old", "new", "message"),
    [
        ('unit = "rad"', 'unit = "TODO"', "still TODO"),
        ('unit = "rad"', 'unit = "radians"', "supported values are"),
        ("rate_hz = 30.0", 'rate_hz = "fast"', "must be float"),
        ("rate_hz = 30.0", "rate_hz = 30.0\nfps = 30", "unknown keys ['fps']"),
        (
            "rate_hz = 30.0                    # sim dt 1/120 s, decimation 4\n",
            "",
            "missing ['rate_hz']",
        ),
        ('top = "314522065367"', 'front = "314522065367"', "must map exactly the video keys"),
        ("min_tcp_z_mm = 25.0", "min_tcp_z_mm = 25.0\n", ""),
        ("workspace_x_mm = [-460.0, 275.0]", "workspace_x_mm = [275.0, -460.0]", "[min, max]"),
        ('key = "eef_delta"', 'key = "eef_pose"', "not among the declared state/action keys"),
    ],
)
def test_incomplete_or_unsupported_configs_are_rejected(tmp_path, old, new, message):
    text = EXAMPLE.read_text()
    assert old in text
    path = _write(tmp_path, text.replace(old, new, 1))
    if not message:  # a harmless edit still loads
        policy_config.load(path)
        return
    with pytest.raises(policy_config.PolicyConfigError, match=message.replace("[", r"\[")):
        policy_config.load(path)


def _evidence(**overrides: object) -> Evidence:
    ev = Evidence(
        host="host",
        port=1,
        keys={
            "state": ["arm_joints", "gripper"],
            "action": ["eef_delta", "gripper"],
            "video": ["wrist", "side", "top"],
            "language": ["annotation.human.task_description"],
        },
        horizon=16,
        action_configs={"eef_delta": {"rep": "ABSOLUTE"}},
    )
    for name, value in overrides.items():
        setattr(ev, name, value)
    return ev


def _status(checks, item):
    return next(c.status for c in checks if c.item == item)


def test_validation_passes_matching_evidence_and_skips_what_is_missing():
    checks = validate(policy_config.load(EXAMPLE), _evidence())
    assert _status(checks, "server state keys") == "PASS"
    assert _status(checks, "server language key") == "PASS"
    assert _status(checks, "tcp action raw") == "PASS"
    assert _status(checks, "checkpoint") == "SKIP"
    assert _status(checks, "training episodes") == "SKIP"
    assert not [c for c in checks if c.status == "FAIL"]


def test_validation_fails_a_server_that_serves_another_policy():
    ev = _evidence(
        keys={
            "state": ["joints"],
            "action": ["eef_delta", "gripper"],
            "video": ["wrist", "side", "top"],
            "language": ["annotation.language.language_instruction"],
        },
        horizon=8,
        action_configs={"eef_delta": {"rep": "RELATIVE"}},
    )
    checks = validate(policy_config.load(EXAMPLE), ev)
    for item in (
        "server state keys",
        "server language key",
        "server action horizon",
        "tcp action raw",
    ):
        assert _status(checks, item) == "FAIL", item


def test_validation_checks_the_meta_files():
    ev = _evidence(
        fps=15.0, images={"observation.images.top": (480, 640)}, prompts=["stack the cubes"]
    )
    checks = validate(policy_config.load(EXAMPLE), ev)
    assert _status(checks, "rate (meta)") == "FAIL"
    assert _status(checks, "image size (meta)") == "FAIL"
    assert _status(checks, "prompt (meta)") == "FAIL"
