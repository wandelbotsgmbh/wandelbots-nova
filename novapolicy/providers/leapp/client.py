"""Policy client for LEAPP exports, such as Isaac Lab reinforcement-learning policies (``isaaclab leapp export``)."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from novapolicy.providers.base import PolicyClient, resolve_control_freq
from novapolicy.providers.leapp.pipeline import _DTYPES, LeappPipeline

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.mapping.schema import PolicySchema
    from novapolicy.providers.leapp.pipeline import Port
    from novapolicy.types import ActionChunk

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

    The export's joint output is the final joint target: declare the joint action absolute.
    Each input is fed from the observation named like its port (``robot_joint_vel``), or like
    the last part of its Isaac Lab connection (``joint_vel``) when unambiguous. The joint
    target is the output of kind ``joint/position`` or with a ``set_joint_position_target``
    connection, else the one named like the joint action; outputs named like ``Action.computed``
    keys land in ``chunk.values``. Feedback inputs restart from the export's values every run.

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
        self._control_freq = resolve_control_freq(
            control_freq, owner=f"LeappPolicyClient({pipeline})", recorded=self._pipeline.frequency
        )
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
        """Observation names that feed ``port``: its port name, and its unshared connection name."""
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
        return schema.decode_actions(
            {
                action_key: [np.asarray(outputs[output.key]).reshape(-1)],
                **{key: [outputs[port.key]] for key, port in self._computed.items()},
            },
            dt_ms=1000.0 / self._control_freq,
            dof={mg_id: len(state.joints) for mg_id, state in states.items()},
            source="The LEAPP export",
        )
