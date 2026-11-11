"""Contract test against a running GR00T policy server.

GR00T servers are deployed from an image that moves on (``gr00t:latest``), and a
change to their wire format once left every action undecodable without an error.
This asks a real server for one action and checks that every action it serves
decodes for this client. Run it before and after a server update:

    GR00T_HOST=<host> GR00T_PORT=<port> uv run --with pyzmq --with msgpack \\
        pytest -m integration novapolicy/tests/integration/test_gr00t_live.py

Skipped unless ``GR00T_HOST`` is set, so CI without access to a server skips it.
"""

from __future__ import annotations

import math
import os
from unittest.mock import MagicMock

import numpy as np
import pytest

from nova.types import Pose, RobotState
from novapolicy import Gr00tPolicyClient, Observation, PolicySchema

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not os.getenv("GR00T_HOST"), reason="GR00T_HOST is not set"),
]

_DOF = int(os.getenv("GR00T_DOF", "6"))
"""Joints per action group; the deployed dual-arm model drives two 6-axis arms."""


class _GreyCamera:
    async def connect(self) -> None:
        return None

    def read(self, max_age_s: float = 5.0) -> np.ndarray:
        return np.full((224, 224, 3), 128, dtype=np.uint8)

    async def disconnect(self) -> None:
        return None


@pytest.mark.asyncio
async def test_every_action_the_server_serves_decodes() -> None:
    client = Gr00tPolicyClient(
        host=os.environ["GR00T_HOST"], port=int(os.getenv("GR00T_PORT", "30555")), dt_ms=130.0
    )
    try:
        info = await client.get_server_info()
        action_keys = list(info["action_keys"])
        state_keys = list(info["state_keys"])
        video_keys = list(info["video_keys"])
        assert action_keys, f"the server serves no actions: {info}"

        groups = {key: MagicMock(id=f"0@{key}") for key in state_keys}
        camera = _GreyCamera()
        schema = PolicySchema(
            observations=[
                *(Observation.joint_positions(k, source=mg) for k, mg in groups.items()),
                *(Observation.image(key, source=camera) for key in video_keys),
                Observation.constant("language", value="Contract test."),
            ]
        )
        states = {
            mg.id: RobotState(pose=Pose((0, 0, 0, 0, 0, 0)), tcp="Flange", joints=(0.0,) * _DOF)
            for mg in groups.values()
        }
        images = {key: camera.read() for key in video_keys}

        chunk = await client.get_actions(states, schema, images=images)

        expected = {groups[key].id for key in action_keys if key in groups}
        assert set(chunk.joints) == expected, f"decoded {sorted(chunk.joints)}"
        for group_id, steps in chunk.joints.items():
            assert len(steps) == info["action_horizon"], group_id
            assert all(len(step) == _DOF for step in steps), group_id
            assert all(math.isfinite(value) for step in steps for value in step), group_id
    finally:
        await client.close()
