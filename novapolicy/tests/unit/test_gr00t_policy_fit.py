"""Tests for :mod:`novapolicy.gr00t.policy_fit`: recovering an action convention from paths."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from novapolicy.gr00t.policy_fit import (
    Kinematics,
    absolute_r2,
    fit_delta_frames,
    fit_gripper,
    merge_summaries,
    path_summary,
    pose_matrix,
)

# NOVA's UR10e DH parameters (mm, rad) and the `umi_corrected` TCP of ur10e-calibration.
_UR10E_DH = [
    (0.0, math.pi / 2, 180.7, 0.0, False),
    (-612.7, 0.0, 0.0, 0.0, False),
    (-571.55, 0.0, 0.0, 0.0, False),
    (0.0, math.pi / 2, 174.15, 0.0, False),
    (0.0, -math.pi / 2, 119.85, 0.0, False),
    (0.0, 0.0, 116.55, 0.0, False),
]


def _path(commands: np.ndarray, gain: float, *, rot_z_deg: float = 0.0, tool: bool = False):
    """Poses realized when each command moves the TCP by ``gain`` x command (m -> mm)."""
    pose = pose_matrix([200.0, -600.0, 400.0], [0.0, math.pi - 0.2, 0.0])
    rz = Rotation.from_euler("z", rot_z_deg, degrees=True)
    poses = [pose]
    for cmd in commands:
        rotation = Rotation.from_matrix(pose[:3, :3])
        dp = gain * np.asarray(cmd[:3]) * 1000.0
        dr = Rotation.from_rotvec(gain * np.asarray(cmd[3:6]))
        nxt = np.eye(4)
        if tool:
            nxt[:3, 3] = pose[:3, 3] + rotation.apply(dp)
            nxt[:3, :3] = (rotation * dr).as_matrix()
        else:
            nxt[:3, 3] = pose[:3, 3] + rz.apply(dp)
            nxt[:3, :3] = (Rotation.from_rotvec(rz.apply(dr.as_rotvec())) * rotation).as_matrix()
        poses.append(nxt)
        pose = nxt
    # One command per pose transition; the last pose has no command of its own.
    return poses, np.vstack([commands, np.zeros((1, 6))])


@pytest.fixture
def commands() -> np.ndarray:
    return np.random.default_rng(0).normal(0.0, [0.03, 0.03, 0.02, 0.01, 0.01, 0.02], (120, 6))


def test_recovers_base_frame_and_gain(commands):
    poses, actions = _path(commands, 0.24)
    best = fit_delta_frames(poses, actions)[0]
    assert (best.frame, best.lag) == ("base", 0)
    assert best.pos_gain == pytest.approx(0.24, rel=1e-3)
    assert best.rot_gain == pytest.approx(0.24, rel=1e-2)
    assert best.pos_r2 > 0.99


def test_recovers_a_rotated_base_frame(commands):
    poses, actions = _path(commands, 0.5, rot_z_deg=90.0)
    best = fit_delta_frames(poses, actions)[0]
    assert best.rot_z_deg == 90
    assert best.pos_gain == pytest.approx(0.5, rel=1e-3)


def test_recovers_the_tool_frame(commands):
    poses, actions = _path(commands, 0.3, tool=True)
    best = fit_delta_frames(poses, actions)[0]
    assert best.frame == "tool"
    assert best.pos_gain == pytest.approx(0.3, rel=1e-3)


def test_absolute_targets_are_told_apart_from_deltas(commands):
    poses, deltas = _path(commands, 0.24)
    targets = np.array([[*(p[:3, 3] / 1000.0), 0.0, 0.0, 0.0] for p in poses])
    assert absolute_r2(poses, targets) > 0.99
    assert absolute_r2(poses, deltas) < 0.5


@pytest.mark.parametrize("close_value", [-1.0, 1.0])
def test_gripper_polarity_and_values(close_value):
    action = np.r_[np.full(40, -close_value), np.full(60, close_value), np.full(20, -close_value)]
    closing = np.clip(np.arange(60) * -0.002, -0.0136, 0.0)
    state = np.r_[np.zeros(40), closing, np.linspace(-0.0136, 0.0, 20)]
    grip = fit_gripper(state, action)
    assert grip is not None
    assert grip.close_when_negative == (close_value < 0)
    assert (grip.open_state, grip.close_frame, grip.open_frame) == (0.0, 40, 100)
    assert grip.holding_state == pytest.approx(-0.0136)


def test_no_gripper_switch_means_no_fit():
    assert fit_gripper(np.zeros(10), np.ones(10)) is None


def test_forward_kinematics_matches_nova_at_home():
    """At home, umi_corrected sits at the training episode's start (181.3, -691.4, 455.2) mm.

    NOVA reported (181.3, -691.3, 454.9) with joint 3 at 90.02 deg instead of 90.
    """
    tcp = SimpleNamespace(pose=SimpleNamespace(position=(-7.19, 0.0, 221.7), orientation=(0, 0, 0)))
    description = SimpleNamespace(
        dh_parameters=[
            SimpleNamespace(a=a, alpha=al, d=d, theta=th, reverse_rotation_direction=rev)
            for a, al, d, th, rev in _UR10E_DH
        ],
        tcps={"umi_corrected": tcp},
        mounting=None,
    )
    kinematics = Kinematics.from_description(description, "umi_corrected")
    home = [math.pi / 2, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
    pose = kinematics.fk(home)
    assert pose[:3, 3] == pytest.approx([181.3, -691.4, 455.15], abs=0.1)
    assert Rotation.from_matrix(pose[:3, :3]).magnitude() == pytest.approx(math.pi, abs=1e-6)


def test_path_envelope_and_merge():
    """TCP bounds and speeds are measured and merged across episodes."""
    down = [0.0, math.pi, 0.0]
    joints = np.zeros((3, 6))
    first = path_summary([pose_matrix([x, -500.0, 300.0], down) for x in (0, 10, 20)], joints, 30.0)
    assert first.speed_max == pytest.approx(300.0)
    second = path_summary(
        [pose_matrix([0.0, y, 100.0], down) for y in (-500, -480, -460)], joints, 30.0
    )
    merged = merge_summaries([first, second])
    assert merged.lo == pytest.approx([0.0, -500.0, 100.0])
    assert merged.hi == pytest.approx([20.0, -460.0, 300.0])
    assert (merged.speed_max, merged.episodes) == (pytest.approx(600.0), 2)
