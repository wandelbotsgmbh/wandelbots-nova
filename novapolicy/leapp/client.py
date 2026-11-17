"""Policy client for LEAPP exports, such as Isaac Lab reinforcement-learning policies (``isaaclab leapp export``)."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from novapolicy.leapp.pipeline import _DTYPES, LeappPipeline
from novapolicy.policy_client import DEFAULT_CONTROL_FREQ, PolicyClient
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.leapp.pipeline import Port
    from novapolicy.schema import PolicySchema

logger = logging.getLogger(__name__)

_JOINT_POSITION = "joint/position"
_JOINT_VELOCITY = "joint/velocity"
_OBSERVATION_FOR_KIND = {_JOINT_POSITION: "joint_positions", _JOINT_VELOCITY: "joint_velocities"}
"""The observation a LEAPP port of this kind has to be fed by."""


def _is_joint_target(port: Port) -> bool:
    """An output that commands joint positions: by LEAPP kind or Isaac Lab write connection."""
    return port.kind == _JOINT_POSITION or ":set_joint_position_target" in (port.connection or "")


class LeappPolicyClient(PolicyClient):
    """Runs a LEAPP export locally, one inference per step, with ``onnxruntime``.

    The export carries the observation processing, the network and the action processing of
    the training environment, so its joint output is the final joint target: declare the joint
    action absolute. The schema's names are the mapping. Each input of the export is fed from
    the observation named like the input's port (``robot_joint_vel``), or like the last part of
    its Isaac Lab connection (``state:robot:joint_vel`` -> ``joint_vel``) when no other input
    shares that name.

    The joint target is the export's output of kind ``joint/position`` or with an Isaac Lab
    ``write:<robot>:set_joint_position_target...`` connection; where that is ambiguous, the
    output named like the joint action. An output named like a key of an ``Action.computed``
    hook lands in ``chunk.values``; other outputs, such as exported gains, are not used.
    Feedback inputs such as the last action are carried from step to step and start from the
    export's initial values with every run.

    Args:
        pipeline: The export's ``.yaml`` file.
        control_freq: The policy's step rate in Hz. The export records it
            (``pipeline.configs.frequency``); pass it only for an export that does not.
    """

    def __init__(
        self,
        pipeline: str | Path,
        *,
        control_freq: float | None = None,
    ) -> None:
        self._pipeline = LeappPipeline(pipeline)
        exported = self._pipeline.frequency
        if control_freq is not None and exported is not None and control_freq != exported:
            logger.warning(
                "control_freq=%s differs from the %s Hz the export was trained at",
                control_freq,
                exported,
            )
        self._control_freq = float(control_freq or exported or DEFAULT_CONTROL_FREQ)
        if control_freq is None and exported is None:
            logger.warning(
                "%s records no frequency; using %s Hz. Pass control_freq= to set it",
                pipeline,
                self._control_freq,
            )
        if self._control_freq <= 0:
            msg = f"control_freq must be positive, got {self._control_freq}"
            raise ValueError(msg)
        self._feeds: dict[str, str] = {}  # export input key -> observation key
        self._target: tuple[Port, str] | None = None  # joint output -> joint action key
        self._computed: dict[str, Port] = {}  # Action.computed key -> export output

    @property
    def control_freq(self) -> float:
        """The policy's step rate in Hz."""
        return self._control_freq

    @property
    def pipeline(self) -> LeappPipeline:
        return self._pipeline

    async def connect(self, motion_group_ids: list[str]) -> None:  # ruff: ignore[unused-method-argument]
        """A run is an episode: feedback inputs start over from the export's initial values."""
        self._pipeline.reset()

    async def validate_schema(self, schema: PolicySchema) -> None:
        errors: list[str] = []
        self._feeds = {}
        for port in self._pipeline.inputs:
            key = self._observation_key(port, schema)
            if key is None:
                names = " or ".join(repr(n) for n in self._names_for(port))
                errors.append(
                    f"No observation feeds {port.key}"
                    + (f" ({port.connection})" if port.connection else "")
                    + f": add one named {names}"
                )
                continue
            expected = _OBSERVATION_FOR_KIND.get(port.kind or "")
            if expected is not None and schema.observation_kind(key) != expected:
                errors.append(
                    f"{port.key} is {port.kind}: feed it with Observation.{expected}({key!r}, ...)"
                )
            self._feeds[port.key] = key
        self._target = self._joint_target(schema, errors)
        self._computed = {}
        outputs = {p.name: p for p in self._pipeline.outputs}
        for key in schema.computed_action_keys:
            if key in outputs:
                self._computed[key] = outputs[key]
            else:
                errors.append(
                    f"Action.computed key {key!r} matches no export output {list(outputs)}"
                )
        if errors:
            msg = "Schema does not satisfy the LEAPP export:\n  - " + "\n  - ".join(errors)
            raise ValueError(msg)

    def _names_for(self, port: Port) -> list[str]:
        """Observation names that feed ``port``: its port name, and its connection name when
        no other input shares it (two robots' ``joint_pos`` would be ambiguous)."""
        names = [port.name]
        shared = [p for p in self._pipeline.inputs if p.connection_name == port.connection_name]
        if port.connection_name and len(shared) == 1 and port.connection_name != port.name:
            names.append(port.connection_name)
        return names

    def _observation_key(self, port: Port, schema: PolicySchema) -> str | None:
        for name in self._names_for(port):
            if schema.observation_kind(name) is not None:
                return name
        return None

    def _joint_target(self, schema: PolicySchema, errors: list[str]) -> tuple[Port, str] | None:
        outputs = self._pipeline.outputs
        actions = schema.joint_action_keys
        if len(actions) != 1:
            errors.append(f"The schema has {len(actions)} joint actions; exactly one is supported")
            return None
        key, motion_groups = actions[0]
        joint_outputs = [p for p in outputs if _is_joint_target(p)]
        if len(joint_outputs) != 1:
            joint_outputs = [p for p in outputs if p.name == key]  # named like the action
        if len(joint_outputs) != 1:
            names = [p.name for p in outputs]
            errors.append(
                f"No single joint target among the export's outputs {names}: name the joint "
                f"action like the output it should command, Action.joint_positions(<name>, ...)"
            )
            return None
        if any(mg.id in schema.relative_motion_groups() for mg in motion_groups):
            errors.append(
                f"The export outputs absolute joint targets: declare Action.joint_positions"
                f"({key!r}, ...) with mode='absolute'"
            )
        computed = set(schema.computed_action_keys)
        ignored = [
            p.key
            for p in self._pipeline.outputs
            if p is not joint_outputs[0] and p.name not in computed
        ]
        if ignored:
            logger.info("Export outputs not sent to the robot: %s", ignored)
        return joint_outputs[0], key

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,  # ruff: ignore[unused-method-argument]
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        if self._target is None:
            await self.validate_schema(schema)
        joint_output = self._target
        if joint_output is None:  # validate_schema raises before it leaves this unset
            msg = "LeappPolicyClient has no joint output to command"
            raise RuntimeError(msg)
        observation = await schema.build_observation(states, io_values)
        feeds: dict[str, np.ndarray] = {}
        for port in self._pipeline.inputs:
            key = self._feeds[port.key]
            values = schema.observation_vector(observation, key)
            if len(values) != port.size:
                msg = (
                    f"{port.key} takes {port.size} values {list(port.shape)}, "
                    f"observation {key!r} has {len(values)}"
                )
                raise ValueError(msg)
            dtype = _DTYPES.get(port.dtype, np.float32)
            feeds[port.key] = np.asarray(values, dtype=dtype).reshape(port.shape)
        outputs = await asyncio.to_thread(self._pipeline.run, feeds)
        output, action_key = joint_output
        target = [float(v) for v in np.asarray(outputs[output.key]).reshape(-1)]
        return ActionChunk(
            joints=self._split(target, action_key, schema, states),
            values={
                key: [[float(v) for v in np.asarray(outputs[port.key]).reshape(-1)]]
                for key, port in self._computed.items()
            },
            dt_ms=1000.0 / self._control_freq,
        )

    @staticmethod
    def _split(
        target: list[float], action_key: str, schema: PolicySchema, states: dict[str, RobotState]
    ) -> dict[str, list[list[float]]]:
        """The joint target per motion group of the action, in the action's order."""
        motion_groups = next(mgs for key, mgs in schema.joint_action_keys if key == action_key)
        joints: dict[str, list[list[float]]] = {}
        offset = 0
        for i, mg in enumerate(motion_groups):
            last = i == len(motion_groups) - 1
            width = len(target) - offset if last else len(states[mg.id].joints)
            joints[mg.id] = [target[offset : offset + width]]
            offset += width
        return joints
