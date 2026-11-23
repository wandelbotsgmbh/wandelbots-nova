"""LeappPolicyClient against a reach-shaped LEAPP export (tests/unit/data/leapp_reach).

The expected joint targets are what LEAPP's own torch-based InferenceManager returns for the
same export and inputs; the client runs it with onnxruntime only.
"""

from __future__ import annotations

import math
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

pytest.importorskip("onnxruntime")

from nova.types import Pose
from novapolicy.ops import Scale
from novapolicy.poses import QUATERNION
from novapolicy.providers.leapp import LeappPipeline, LeappPolicyClient
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.tests.doubles import mg as _mg

EXPORT = Path(__file__).parents[1] / "unit" / "data" / "leapp_reach" / "reach_ur10e.yaml"
MG_ID = "0@ur10e"
HOME = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)
# InferenceManager's targets for HOME, zero velocity and the tool-down command: the first step,
# and the second one, which sees the first step's action as its last action.
T1 = [-0.000327257, -1.591848850, 1.582729697, -1.546835423, -1.578919410, -0.003035118]
T2 = [0.000401501, -1.594622254, 1.584608555, -1.546020508, -1.577775002, -0.000319857]


def _schema(mg, *, joint_vel_key: str = "joint_vel", relative: bool = False) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("joint_pos", source=mg, action=False),
            Observation.joint_velocities(joint_vel_key, source=mg),
            Observation.pose(
                "ee_pose",
                source=Pose((850.0, -225.0, 200.0, math.pi, 0.0, 0.0)),
                orientation_type=QUATERNION,
                position_ops=[Scale(0.001)],
            ),
        ],
        actions=[
            Action.joint_positions("arm", target=mg, mode="relative" if relative else "absolute")
        ],
    )


def _states() -> dict[str, SimpleNamespace]:
    return {MG_ID: SimpleNamespace(joints=HOME, pose=None)}


def test_the_export_describes_its_ports_and_rate():
    pipeline = LeappPipeline(EXPORT)
    assert pipeline.frequency == 60.0
    assert [(p.name, p.connection) for p in pipeline.inputs] == [
        ("robot_joint_pos", "state:robot:joint_pos"),
        ("robot_joint_vel", "state:robot:joint_vel"),
        ("ee_pose", "command:ee_pose"),
    ]
    assert [p.kind for p in pipeline.outputs] == ["joint/position"]


@pytest.mark.asyncio
async def test_two_steps_match_leapp_and_carry_the_last_action():
    mg = _mg()
    schema = _schema(mg)
    client = LeappPolicyClient(EXPORT)
    assert client.control_freq == 60.0
    await client.connect([MG_ID])
    await client.validate_schema(schema)

    first = await client.get_actions(_states(), schema)
    second = await client.get_actions(_states(), schema)

    assert first.joints[MG_ID][0] == pytest.approx(T1, abs=1e-6)
    assert second.joints[MG_ID][0] == pytest.approx(T2, abs=1e-6)
    assert first.dt_ms == pytest.approx(1000.0 / 60.0)


@pytest.mark.asyncio
async def test_a_new_run_starts_from_the_exported_initial_values():
    schema = _schema(_mg())
    client = LeappPolicyClient(EXPORT)
    await client.connect([MG_ID])
    await client.validate_schema(schema)
    await client.get_actions(_states(), schema)

    await client.connect([MG_ID])
    again = await client.get_actions(_states(), schema)

    assert again.joints[MG_ID][0] == pytest.approx(T1, abs=1e-6)


@pytest.mark.asyncio
async def test_an_observation_named_like_the_port_feeds_it():
    schema = _schema(_mg(), joint_vel_key="robot_joint_vel")
    client = LeappPolicyClient(EXPORT)
    await client.validate_schema(schema)
    chunk = await client.get_actions(_states(), schema)
    assert chunk.joints[MG_ID][0] == pytest.approx(T1, abs=1e-6)


@pytest.mark.asyncio
async def test_an_unfed_input_is_named_with_how_to_feed_it():
    with pytest.raises(
        ValueError,
        match=r"No observation feeds reach_policy/robot_joint_vel.*'robot_joint_vel' or 'joint_vel'",
    ):
        await LeappPolicyClient(EXPORT).validate_schema(_schema(_mg(), joint_vel_key="qd"))


@pytest.mark.asyncio
async def test_a_velocity_port_fed_by_positions_is_refused():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joint_pos", source=mg, action=False),
            Observation.joint_positions("joint_vel", source=mg, action=False),
            Observation.pose(
                "ee_pose", source=Pose((0, 0, 0, 0, 0, 0)), orientation_type=QUATERNION
            ),
        ],
        actions=[Action.joint_positions("arm", target=mg)],
    )
    with pytest.raises(
        ValueError, match=r"joint/velocity: feed it with Observation.joint_velocities"
    ):
        await LeappPolicyClient(EXPORT).validate_schema(schema)


@pytest.mark.asyncio
async def test_a_relative_joint_action_is_refused():
    with pytest.raises(ValueError, match="mode='absolute'"):
        await LeappPolicyClient(EXPORT).validate_schema(_schema(_mg(), relative=True))


@pytest.mark.asyncio
async def test_an_observation_of_the_wrong_width_is_reported():
    schema = _schema(_mg())
    client = LeappPolicyClient(EXPORT)
    await client.validate_schema(schema)
    with pytest.raises(ValueError, match="takes 6 values"):
        await client.get_actions({MG_ID: SimpleNamespace(joints=HOME[:5], pose=None)}, schema)


def _copy_export(tmp_path: Path) -> Path:
    for f in EXPORT.parent.iterdir():
        shutil.copy(f, tmp_path / f.name)
    return tmp_path / EXPORT.name


def test_a_model_that_does_not_match_its_checksum_is_refused(tmp_path: Path):
    export = _copy_export(tmp_path)
    model = tmp_path / "reach_policy.onnx"
    model.write_bytes(model.read_bytes() + b"\0")
    with pytest.raises(ValueError, match="sha256sum"):
        LeappPipeline(export)


def test_a_torch_export_is_refused_with_how_to_fix_it(tmp_path: Path):
    export = _copy_export(tmp_path)
    export.write_text(export.read_text().replace("backend: onnx", "backend: jit"))
    with pytest.raises(ValueError, match="Export with the ONNX backend"):
        LeappPipeline(export)


def _without_output_kind(tmp_path: Path, connection: str) -> Path:
    """The export with its joint output tagged only by an Isaac Lab write connection."""
    export = _copy_export(tmp_path)
    text = export.read_text()
    output = text.index("  - name: joint_pos_target")
    block_end = text.index("  - name: last_action_out")
    block = text[output:block_end].replace("      kind: joint/position\n", "")
    block = block.replace("write:robot:set_joint_position_target", connection)
    export.write_text(text[:output] + block + text[block_end:])
    return export


@pytest.mark.asyncio
async def test_the_joint_target_is_found_by_its_write_connection(tmp_path: Path):
    export = _without_output_kind(tmp_path, "write:robot:set_joint_position_target_index")
    schema = _schema(_mg())
    client = LeappPolicyClient(export)
    await client.validate_schema(schema)
    chunk = await client.get_actions(_states(), schema)
    assert chunk.joints[MG_ID][0] == pytest.approx(T1, abs=1e-6)


@pytest.mark.asyncio
async def test_an_unrecognisable_joint_target_is_found_by_the_action_name(tmp_path: Path):
    export = _without_output_kind(tmp_path, "write:robot:something_else")
    with pytest.raises(ValueError, match="name the joint action like the output"):
        await LeappPolicyClient(export).validate_schema(_schema(_mg()))
    mg = _mg()
    named = PolicySchema(
        observations=_schema(mg)._observations,
        actions=[Action.joint_positions("joint_pos_target", target=mg)],
    )
    await LeappPolicyClient(export).validate_schema(named)


@pytest.mark.asyncio
async def test_a_connection_name_two_inputs_share_feeds_neither(tmp_path: Path):
    """Two robots' joint positions both end in ``joint_pos``: only port names can tell them apart."""
    export = _copy_export(tmp_path)
    text = export.read_text().replace(
        "isaaclab_connection: state:robot:joint_vel", "isaaclab_connection: state:other:joint_pos"
    )
    export.write_text(text)
    mg = _mg()
    by_connection = _schema(mg)  # "joint_pos" would fit both inputs
    with pytest.raises(ValueError, match=r"robot_joint_pos .*: add one named 'robot_joint_pos'\n"):
        await LeappPolicyClient(export).validate_schema(by_connection)
    by_port = PolicySchema(
        observations=[
            Observation.joint_positions("robot_joint_pos", source=mg, action=False),
            Observation.joint_velocities("robot_joint_vel", source=mg),
            *_schema(mg)._observations[2:],
        ],
        actions=[Action.joint_positions("arm", target=mg)],
    )
    await LeappPolicyClient(export).validate_schema(by_port)


def _with_computed(mg, key: str) -> PolicySchema:
    async def hook(chunk, ctx):
        return None

    return PolicySchema(
        observations=_schema(mg)._observations,
        actions=[Action.joint_positions("arm", target=mg), Action.computed(hook, keys=[key])],
    )


def _edited(tmp_path: Path, old: str, new: str) -> Path:
    export = _copy_export(tmp_path)
    text = export.read_text()
    assert old in text
    export.write_text(text.replace(old, new))
    return export


@pytest.mark.asyncio
async def test_an_output_named_like_a_computed_key_lands_in_chunk_values(tmp_path: Path):
    export = _edited(
        tmp_path,
        "reach_policy: [joint_pos_target]",
        "reach_policy: [joint_pos_target, last_action_out]",
    )
    schema = _with_computed(_mg(), "last_action_out")
    client = LeappPolicyClient(export)
    await client.connect([MG_ID])
    await client.validate_schema(schema)

    chunk = await client.get_actions(_states(), schema)

    assert list(chunk.values) == ["last_action_out"]
    assert len(chunk.values["last_action_out"]) == 1  # one row: one step
    assert len(chunk.values["last_action_out"][0]) == 6


@pytest.mark.asyncio
async def test_a_computed_key_no_output_produces_is_refused():
    with pytest.raises(ValueError, match="'gripper' matches no export output"):
        await LeappPolicyClient(EXPORT).validate_schema(_with_computed(_mg(), "gripper"))


@pytest.mark.asyncio
async def test_inputs_are_fed_in_the_dtype_the_export_declares(tmp_path: Path):
    export = _edited(
        tmp_path,
        "    - name: robot_joint_pos\n      dtype: float32",
        "    - name: robot_joint_pos\n      dtype: float64",
    )
    schema = _schema(_mg())
    client = LeappPolicyClient(export)
    await client.validate_schema(schema)
    seen = {}

    def run(feeds):
        seen.update({key: str(value.dtype) for key, value in feeds.items()})
        return {"reach_policy/joint_pos_target": [list(HOME)]}

    client.pipeline.run = run  # ty: ignore[invalid-assignment]
    await client.get_actions(_states(), schema)

    assert seen["reach_policy/robot_joint_pos"] == "float64"
    assert seen["reach_policy/robot_joint_vel"] == "float32"
