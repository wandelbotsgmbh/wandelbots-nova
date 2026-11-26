"""Observation.pose and its sources: a fixed Pose, an async callable, an Omniverse prim."""

from __future__ import annotations

import asyncio
import math

import httpx
import pytest

from nova.types import Pose
from novapolicy.mapping.ops import Scale
from novapolicy.mapping.pose_sources import OmniverseScene, PoseSource
from novapolicy.mapping.poses import QUATERNION
from novapolicy.mapping.schema import Observation, PolicySchema

API = "http://isaac-host:8011/omniservice/api/v2"
TOOL_DOWN = Pose((850.0, -225.0, 200.0, math.pi, 0.0, 0.0))


@pytest.mark.asyncio
async def test_a_fixed_pose_is_seen_in_the_policy_units_and_rotation():
    schema = PolicySchema(
        observations=[
            Observation.pose(
                "ee_pose",
                source=TOOL_DOWN,
                orientation_type=QUATERNION,
                position_ops=[Scale(0.001)],
            )
        ]
    )
    obs = await schema.build_observation({})
    assert list(obs) == [f"ee_pose_{s}" for s in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    assert [obs["ee_pose_x"], obs["ee_pose_y"], obs["ee_pose_z"]] == pytest.approx([
        0.85,
        -0.225,
        0.2,
    ])
    assert abs(obs["ee_pose_qx"]) == pytest.approx(1.0)  # 180 deg about x
    assert obs["ee_pose_qw"] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.asyncio
async def test_an_async_callable_is_asked_on_every_observation():
    calls = 0

    async def detector() -> Pose:
        nonlocal calls
        calls += 1
        return Pose((float(calls), 0.0, 0.0, 0.0, 0.0, 0.0))

    schema = PolicySchema(observations=[Observation.pose("cube", source=detector)])
    assert (await schema.build_observation({}))["cube_x"] == 1.0
    assert (await schema.build_observation({}))["cube_x"] == 2.0


def _scene(handler, **kwargs) -> OmniverseScene:
    client = httpx.AsyncClient(base_url=API, transport=httpx.MockTransport(handler))
    return OmniverseScene(API, client=client, **kwargs)


@pytest.mark.asyncio
async def test_a_prim_is_read_in_the_world_and_polled_in_the_background():
    requests: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        return httpx.Response(200, json={"pose": [100.0 + len(requests), 0, 50, 0, 0, 0]})

    prim = _scene(handler, poll_hz=200.0).prim("/World/Cube")
    assert isinstance(prim, PoseSource)
    await prim.start()
    assert prim.read().position.x == 101.0  # the first pose is there once start returns
    await asyncio.sleep(0.05)
    assert prim.read().position.x > 101.0
    await prim.stop()
    first = requests[0]
    assert first.url.path.endswith("/prims/poses")
    assert dict(first.url.params) == {
        "prim_path": "/World/Cube",
        "coordinate_system": "world",
        "rotation_type": "cartesian",
    }


@pytest.mark.asyncio
async def test_relative_to_asks_for_the_prim_in_the_reference_prims_frame():
    seen: list[httpx.Request] = []

    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request)
        return httpx.Response(200, json={"pose": [1, 2, 3, 0, 0, 0]})

    prim = _scene(handler).prim("/World/Cube", relative_to="/World/UR10e/base")
    await prim.start()
    await prim.stop()
    assert seen[0].url.path.endswith("/prims/poses/relative")
    assert dict(seen[0].url.params) == {
        "prim_path_1": "/World/UR10e/base",
        "prim_path_2": "/World/Cube",
        "mode": "inverse_first",
        "rotation_type": "cartesian",
    }


@pytest.mark.asyncio
async def test_a_prim_whose_polling_keeps_failing_is_refused_once_too_old():
    """Regression: a dead pose stream fed the last good pose on forever; like a stale camera
    frame, a pose older than ``max_age_s`` ends the run."""
    answers = iter([httpx.Response(200, json={"pose": [1, 2, 3, 0, 0, 0]})])

    def handler(_request: httpx.Request) -> httpx.Response:
        return next(answers, httpx.Response(503))

    prim = _scene(handler, poll_hz=200.0).prim("/World/Cube", max_age_s=0.05)
    await prim.start()
    assert prim.read().position.x == 1.0  # fresh
    await asyncio.sleep(0.1)
    with pytest.raises(RuntimeError, match="old"):
        prim.read()
    await prim.stop()


@pytest.mark.asyncio
async def test_a_prim_that_cannot_be_read_fails_at_start():
    prim = _scene(lambda _r: httpx.Response(404)).prim("/World/Missing")
    with pytest.raises(httpx.HTTPStatusError):
        await prim.start()
    with pytest.raises(RuntimeError, match="not started"):
        prim.read()


@pytest.mark.asyncio
async def test_the_schema_starts_and_stops_background_pose_sources():
    def handler(_request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, json={"pose": [10, 20, 30, 0, 0, 0]})

    prim = _scene(handler).prim("/World/Cube")
    schema = PolicySchema(observations=[Observation.pose("cube", source=prim)])
    await schema.start_sources()
    assert (await schema.build_observation({}))["cube_z"] == 30.0
    await schema.stop_sources()
    with pytest.raises(RuntimeError, match="not started"):
        await schema.build_observation({})


def test_orientation_ops_cannot_apply_to_quaternions():
    with pytest.raises(ValueError, match="quaternion"):
        Observation.pose(
            "p", source=TOOL_DOWN, orientation_type=QUATERNION, orientation_ops=[Scale(2.0)]
        )
