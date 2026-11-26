"""Rotation representations and coordinate systems for TCP observations and actions."""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from novapolicy.executor import resolve_actions
from novapolicy.mapping import poses
from novapolicy.mapping.ops import Scale
from novapolicy.mapping.schema import Action, Observation, PolicySchema
from novapolicy.tests.doubles import mg as _mg
from novapolicy.types import ActionChunk

OT = poses.OrientationType
DOWN = [100.0, -500.0, 300.0, math.pi, 0.0, 0.0]


@pytest.mark.parametrize(
    "orientation_type",
    [
        OT.ROTATION_VECTOR,
        OT.QUATERNION,
        OT.EULER_ANGLES_EXTRINSIC_XYZ,
        OT.EULER_ANGLES_INTRINSIC_ZYX,
    ],
)
def test_orientation_types_round_trip(orientation_type):
    rotvec = [0.3, -0.2, 1.1]
    values = poses.from_rotvec(rotvec, orientation_type)
    assert len(values) == poses.orientation_width(orientation_type)
    assert poses.to_rotvec(values, orientation_type) == pytest.approx(rotvec)


def test_quaternions_are_scalar_last_as_in_nova():
    assert poses.from_rotvec([0.0, 0.0, 0.0], OT.QUATERNION) == pytest.approx([0, 0, 0, 1])


def test_intrinsic_and_extrinsic_euler_differ():
    rotvec = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_rotvec()
    extrinsic = poses.from_rotvec(rotvec, OT.EULER_ANGLES_EXTRINSIC_XYZ)
    intrinsic = poses.from_rotvec(rotvec, OT.EULER_ANGLES_INTRINSIC_XYZ)
    assert extrinsic == pytest.approx([0.1, 0.2, 0.3])
    assert intrinsic != pytest.approx(extrinsic)


def _state(pose: list[float]) -> SimpleNamespace:
    return SimpleNamespace(
        joints=[0.0] * 6,
        tcp="flange",
        pose=SimpleNamespace(position=pose[:3], orientation=pose[3:]),
    )


@pytest.mark.asyncio
async def test_quaternion_observations_are_named_and_converted():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.tcp(
                "eef", source=mg, orientation_type=OT.QUATERNION, position_ops=[Scale(0.001)]
            )
        ]
    )
    obs = await schema.build_observation({mg.id: _state(DOWN)})
    assert list(obs) == ["eef_x", "eef_y", "eef_z", "eef_qx", "eef_qy", "eef_qz", "eef_qw"]
    assert obs["eef_x"] == pytest.approx(0.1)
    assert [obs[k] for k in ("eef_qx", "eef_qy", "eef_qz", "eef_qw")] == pytest.approx(
        [1, 0, 0, 0], abs=1e-12
    )


def test_quaternion_actions_come_back_as_rotation_vectors():
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=mg)],
        actions=[Action.tcp("eef", target=mg, orientation_type=OT.QUATERNION)],
    )
    quat = Rotation.from_rotvec([0.0, 0.0, 0.5]).as_quat().tolist()
    chunk = schema.apply_inverse_ops(ActionChunk(tcp={mg.id: [[1.0, 2.0, 3.0, *quat]]}))
    assert chunk.tcp[mg.id][0] == pytest.approx([1.0, 2.0, 3.0, 0.0, 0.0, 0.5])


def test_operators_on_quaternion_components_are_rejected():
    with pytest.raises(ValueError, match="quaternion"):
        Action.tcp(
            "eef", target=_mg(), orientation_type=OT.QUATERNION, orientation_ops=[Scale(2.0)]
        )


def _with_coordinate_system(mg: MagicMock, systems: dict[str, tuple[list[float], str]]) -> None:
    """``systems``: id -> (pose relative to its reference, reference id)."""

    async def get_coordinate_system(*, cell, controller, coordinate_system):
        pose, reference = systems[coordinate_system]
        return SimpleNamespace(
            position=pose[:3], orientation=pose[3:], orientation_type=None,
            reference_coordinate_system=reference,
        )  # fmt: skip

    mg._api_client.controller_api.get_coordinate_system = AsyncMock(
        side_effect=get_coordinate_system
    )


@pytest.mark.asyncio
async def test_coordinate_systems_chain_their_references():
    mg = _mg()
    # table: 100 mm along x, turned 90 deg about z; fixture: 10 mm along the table's x
    _with_coordinate_system(
        mg,
        {
            "table": ([100.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2], "world"),
            "fixture": ([10.0, 0.0, 0.0, 0, 0, 0], "table"),
        },
    )
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=mg, coordinate_system="fixture")]
    )
    await schema.load_coordinate_systems()
    obs = await schema.build_observation({mg.id: _state([110.0, 10.0, 0.0, 0.0, 0.0, 0.0])})
    # fixture origin is world (100, 10, 0); the TCP is 10 mm further along world x,
    # which is the fixture's -y because the table is turned 90 deg.
    assert [obs["eef_x"], obs["eef_y"], obs["eef_z"]] == pytest.approx([0.0, -10.0, 0.0], abs=1e-9)
    assert obs["eef_rz"] == pytest.approx(-math.pi / 2)


@pytest.mark.asyncio
async def test_relative_targets_compose_in_the_coordinate_system_and_return_to_world():
    mg = _mg()
    _with_coordinate_system(mg, {"table": ([0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2], "")})
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=mg)],
        actions=[Action.tcp("eef", target=mg, mode="relative", coordinate_system="table")],
    )
    await schema.load_coordinate_systems()
    chunk = resolve_actions(
        schema, ActionChunk(tcp={mg.id: [[10.0, 0.0, 0.0, 0.0, 0.0, 0.0]]}), {mg.id: _state(DOWN)}
    )
    # +10 mm along the table's x is +10 mm along world y
    assert chunk.tcp[mg.id][0][:3] == pytest.approx([DOWN[0], DOWN[1] + 10.0, DOWN[2]])
    turned = Rotation.from_rotvec(chunk.tcp[mg.id][0][3:]) * Rotation.from_rotvec(DOWN[3:]).inv()
    assert turned.magnitude() == pytest.approx(0.0, abs=1e-9)


def test_an_unloaded_coordinate_system_is_reported():
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.tcp("eef", source=mg, coordinate_system="table")]
    )
    with pytest.raises(RuntimeError, match="load_coordinate_systems"):
        schema.tcp_observation_values(schema.tcp_mappings[0], _state(DOWN))


def test_world_is_an_exact_no_op():
    assert poses.world_to_frame(DOWN, None) == DOWN
    assert np.allclose(
        poses.frame_to_world(
            DOWN, poses.pose_matrix(DOWN) @ np.linalg.inv(poses.pose_matrix(DOWN))
        ),
        DOWN,
    )
