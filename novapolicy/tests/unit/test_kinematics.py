"""Forward kinematics from a NOVA motion group description."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from scipy.spatial.transform import Rotation

from novapolicy.kinematics import Kinematics

UR10E_DH = [
    (0.0, math.pi / 2, 180.7, 0.0, False),
    (-612.7, 0.0, 0.0, 0.0, False),
    (-571.55, 0.0, 0.0, 0.0, False),
    (0.0, math.pi / 2, 174.15, 0.0, False),
    (0.0, -math.pi / 2, 119.85, 0.0, False),
    (0.0, 0.0, 116.55, 0.0, False),
]
HOME = [math.pi / 2, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]


def ur10e_description(tcp_offset=(-7.19, 0.0, 221.7), mounting=None) -> SimpleNamespace:
    tcp = SimpleNamespace(pose=SimpleNamespace(position=tcp_offset, orientation=(0, 0, 0)))
    return SimpleNamespace(
        dh_parameters=[
            SimpleNamespace(a=a, alpha=al, d=d, theta=th, reverse_rotation_direction=rev)
            for a, al, d, th, rev in UR10E_DH
        ],
        tcps={"umi_corrected": tcp},
        mounting=mounting,
    )


def test_forward_kinematics_matches_nova_at_home():
    """NOVA reported umi_corrected at (181.3, -691.3, 454.9) mm, joint 3 at 90.02 deg."""
    pose = Kinematics.from_description(ur10e_description(), "umi_corrected").fk(HOME)
    assert pose[:3, 3] == pytest.approx([181.3, -691.4, 455.15], abs=0.1)
    assert Rotation.from_matrix(pose[:3, :3]).magnitude() == pytest.approx(math.pi, abs=1e-6)


def test_the_mounting_moves_the_whole_robot():
    mounting = SimpleNamespace(position=(1000.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0))
    plain = Kinematics.from_description(ur10e_description(), "umi_corrected").fk(HOME)
    mounted = Kinematics.from_description(ur10e_description(mounting=mounting), "umi_corrected")
    assert mounted.fk(HOME)[:3, 3] == pytest.approx(plain[:3, 3] + [1000.0, 0.0, 0.0])


def test_an_unknown_tcp_is_named():
    with pytest.raises(ValueError, match="not defined"):
        Kinematics.from_description(ur10e_description(), "flange")
