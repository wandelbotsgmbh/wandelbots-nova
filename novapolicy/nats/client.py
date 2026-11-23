"""NATS policy client: robot states to the protocol's per-block proprioceptive vectors and back.

The wire protocol is ``protocol.py``; the model runs in a separate policy service on the same
broker. Velocities and the last action reset per episode (ADR 0014).
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any
import uuid

from nats.errors import NoRespondersError
import numpy as np

from nats import connect as nats_connect
from novapolicy.joint_velocities import JointVelocities
from novapolicy.nats.protocol import (
    CloseRequest,
    CloseResponse,
    PolicyInfo,
    PolicyProtocolError,
    PredictRequest,
    PredictResponse,
    ResetRequest,
    ResetResponse,
    close_subject,
    decode,
    encode,
    info_subject,
    predict_subject,
    reset_subject,
)
from novapolicy.policy_client import PolicyClient
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.nats.protocol import ActionSpec
    from novapolicy.schema import PolicySchema
    from novapolicy.state_stream import VelocitySource

logger = logging.getLogger(__name__)

PROPRIO_KEY = "proprio"
"""Observation field for the legacy single-block layout."""

ACTION_KEY = "joint_target"
"""Action name for the legacy single-block layout; named blocks use
``joint_target.<block>`` / ``proprio.<block>``."""

PROPRIO_BLOCKS = 3
"""Each proprio field is joint positions ++ joint velocities ++ last action,
each ``dof`` wide, all in radians / rad·s⁻¹."""


class _Block:
    """One bound (motion group, observation field, action head) triple."""

    __slots__ = ("action_name", "motion_group_id", "obs_field", "spec")

    def __init__(self, obs_field: str, action_name: str, spec: ActionSpec) -> None:
        self.obs_field = obs_field
        self.action_name = action_name
        self.spec = spec
        self.motion_group_id: str = ""


def _derive_blocks(info: PolicyInfo) -> tuple[list[_Block], list[str]]:
    """Map the advertised spaces into bindable motion blocks and extra action heads.

    A head without a matching observation is kept for an ``Action.computed(..., keys=[...])``
    hook. Ambiguous layouts are rejected.
    """
    if not info.actions:
        raise PolicyProtocolError("Policy declares no action space.")

    if len(info.actions) == 1 and info.actions[0].name == ACTION_KEY:
        return [_Block(PROPRIO_KEY, ACTION_KEY, info.actions[0])], []

    blocks: list[_Block] = []
    extra: list[str] = []
    for spec in info.actions:
        prefix = f"{ACTION_KEY}."
        if not spec.name.startswith(prefix):
            raise PolicyProtocolError(
                f"Cannot bind action '{spec.name}': expected '{ACTION_KEY}' (single "
                f"block) or '{ACTION_KEY}.<block>' (named blocks)."
            )
        obs_field = f"{PROPRIO_KEY}.{spec.name[len(prefix) :]}"
        if info.observation(obs_field) is None:
            extra.append(spec.name)
            continue
        blocks.append(_Block(obs_field, spec.name, spec))
    if not blocks:
        raise PolicyProtocolError("Policy declares no action block with a matching observation.")

    dt_values = {spec.dt_ms for spec in info.actions}
    if len(dt_values) > 1:
        raise PolicyProtocolError(
            f"Action heads disagree on dt_ms ({sorted(dt_values)}); one control "
            "period per policy is required."
        )
    return blocks, extra


class NatsPolicyClient(PolicyClient):
    """Policy client for policy services on the NATS policy protocol.

    Spaces are negotiated at connect. A single-block policy (``joint_target`` / ``proprio``)
    drives one motion group; with named blocks (``joint_target.<block>`` /
    ``proprio.<block>``) the i-th motion group of the schema binds to the i-th declared
    block, so declare the arms in the policy's block order.

    Args:
        policy_id: Identity in the subject tree, e.g. ``grasp`` -> ``policy.grasp.predict``.
        servers: NATS URL(s): ``ws://<host>/api/nats`` on a LAN (needs ``aiohttp``),
            ``nats://<host>:4222`` in-cluster.
        subject_prefix: Root of the subject tree; must match the policy service.
        request_timeout_s: Per-request deadline; a timeout aborts the episode.
        connect_timeout_s: Deadline for the initial broker connection.
    """

    def __init__(
        self,
        *,
        policy_id: str,
        servers: str | list[str],
        subject_prefix: str = "policy",
        request_timeout_s: float = 5.0,
        connect_timeout_s: float = 10.0,
        velocities: VelocitySource | None = None,
    ) -> None:
        self._policy_id = policy_id
        self._servers = [servers] if isinstance(servers, str) else list(servers)
        self._prefix = subject_prefix
        self._timeout_s = request_timeout_s
        self._connect_timeout_s = connect_timeout_s
        # Measured velocities where the source has them, finite differences otherwise.
        self._velocities = JointVelocities([], measured=velocities)

        self._nc: Any = None
        self._info: PolicyInfo | None = None
        self._session_id: str = ""
        self._blocks: list[_Block] = []
        self._extra_actions: list[str] = []  # action heads for Action.computed hooks
        self._motion_group_ids: list[str] = []

        # --- episode state, per motion group (reset in _begin_episode) ---
        self._step = 0
        self._t0 = 0.0
        self._last_action: dict[str, NDArray[np.float32]] = {}

    @property
    def info(self) -> PolicyInfo:
        """Spaces reported by the service. Available after :meth:`connect`."""
        if self._info is None:
            raise RuntimeError("NatsPolicyClient.connect() has not completed yet.")
        return self._info

    @property
    def bound_blocks(self) -> list[tuple[str, ActionSpec]]:
        """``(motion_group_id, action spec)`` per bound block, in binding order."""
        return [(block.motion_group_id, block.spec) for block in self._blocks]

    # -- PolicyClient ------------------------------------------------------

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Connect, fetch :class:`PolicyInfo`, bind motion groups to blocks, start a session.

        Raises:
            ValueError: Motion group count does not match the declared block count.
            PolicyProtocolError: No service answered, it speaks another protocol
                version, or its spaces are not bindable.
        """
        logger.info(
            "Connecting to policy '%s' via NATS %s (prefix=%r)",
            self._policy_id,
            self._servers,
            self._prefix,
        )
        self._nc = await nats_connect(
            servers=self._servers,
            connect_timeout=self._connect_timeout_s,
            max_reconnect_attempts=-1,
        )

        await self._velocities.start()

        info_body = await self._request(
            info_subject(self._policy_id, self._prefix),
            encode("info", {}),
            expect="info",
        )
        info = PolicyInfo.model_validate(info_body)
        self._info = info
        self._bind_blocks(motion_group_ids)

        spec = self._blocks[0].spec
        logger.info(
            "Policy '%s' v%s embodiment=%s blocks=%s horizon=%d dt=%.1fms",
            info.name,
            info.version,
            info.embodiment or "unspecified",
            [(b.action_name, b.spec.dof, b.motion_group_id) for b in self._blocks],
            spec.horizon,
            spec.dt_ms,
        )
        await self._begin_episode()

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Fail before the robot moves if the schema and the policy disagree.

        One joint action per block, each bound to one motion group of the connected set; joint
        counts are re-checked every step.
        """
        unsent = [o.key for o in (*schema.joint_velocity_mappings, *schema.pose_mappings)]
        if unsent:
            raise ValueError(
                f"Observations {unsent} are not sent to a NATS policy: its proprio carries "
                "joint positions, joint velocities and the last action itself."
            )
        converted = [m.key for m in schema.joint_mappings if m.ops]
        if converted:
            raise ValueError(
                f"Observations {converted} declare ops, but a NATS policy takes and returns "
                "joints in NOVA units (rad) in the layout it declares; remove the ops."
            )
        consumed = {f"{ACTION_KEY}.{key}" for key in schema.computed_action_keys}
        unclaimed = [name for name in self._extra_actions if name not in consumed]
        if unclaimed:
            obs = [f"{PROPRIO_KEY}.{name[len(ACTION_KEY) + 1 :]}" for name in unclaimed]
            raise ValueError(
                f"Actions {unclaimed} have no matching observation {obs} and no "
                "Action.computed(..., keys=[...]) hook consumes them."
            )
        targets = schema.joint_action_keys
        if len(targets) != len(self._blocks):
            raise ValueError(
                f"Policy '{self._policy_id}' declares {len(self._blocks)} joint "
                f"action block(s); the schema declares {len(targets)} targets."
            )
        bound: set[str] = set()
        for _, motion_groups in targets:
            if len(motion_groups) != 1:
                raise ValueError(
                    f"Each joint action target must bind exactly one motion group, "
                    f"found {len(motion_groups)}."
                )
            bound.add(motion_groups[0].id)
        if bound != set(self._motion_group_ids):
            raise ValueError(
                f"Schema binds motion groups {sorted(bound)}; the client connected "
                f"{sorted(self._motion_group_ids)}."
            )

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,  # ruff: ignore[unused-method-argument]
        images: dict[str, NDArray[Any]] | None = None,  # ruff: ignore[unused-method-argument]
        io_values: dict[str, object] | None = None,  # ruff: ignore[unused-method-argument]
    ) -> ActionChunk:
        """One inference round trip, in the policy's layout; images and IO are not sent."""
        now = time.monotonic()
        observation: dict[str, Any] = {}
        joints_by_group: dict[str, NDArray[np.float32]] = {}
        for block in self._blocks:
            state = states.get(block.motion_group_id)
            if state is None:
                raise PolicyProtocolError(
                    f"No robot state for motion group '{block.motion_group_id}'."
                )
            joints = np.asarray(state.joints, dtype=np.float32)
            joints_by_group[block.motion_group_id] = joints
            observation[block.obs_field] = self._build_proprio(block, joints, now)

        request = PredictRequest(
            session_id=self._session_id,
            step=self._step,
            t_ms=(now - self._t0) * 1000.0,
            observation=observation,
        )
        body = await self._request(
            predict_subject(self._policy_id, self._prefix),
            encode("predict", request.to_wire()),
            expect="predict",
        )
        response = PredictResponse.from_wire(body)

        self._step += 1

        return self._to_chunk(response)

    async def close(self) -> None:
        """Release the session, the velocity streams and the broker connection; idempotent."""
        await self._velocities.stop()
        if self._nc is None:
            return
        try:
            if self._session_id:
                body = await self._request(
                    close_subject(self._policy_id, self._prefix),
                    encode("close", CloseRequest(session_id=self._session_id).model_dump()),
                    expect="close",
                )
                served = CloseResponse.model_validate(body).steps
                logger.info(
                    "Closed policy session %s (%d requests sent, %d served)",
                    self._session_id,
                    self._step,
                    served,
                )
        except Exception:
            logger.warning("Policy session close failed (ignored)", exc_info=True)
        finally:
            self._session_id = ""
            nc, self._nc = self._nc, None
            try:
                await nc.drain()
            except Exception:
                logger.warning("NATS drain failed (ignored)", exc_info=True)

    # -- internals ---------------------------------------------------------

    def _bind_blocks(self, motion_group_ids: list[str]) -> None:
        assert self._info is not None
        blocks, self._extra_actions = _derive_blocks(self._info)
        if len(motion_group_ids) > len(blocks) and self._extra_actions:
            raise PolicyProtocolError(
                f"Actions {self._extra_actions} have no matching observation, so they bind "
                f"no motion group; {len(motion_group_ids)} motion groups need "
                f"{len(motion_group_ids)} blocks."
            )
        if len(motion_group_ids) != len(blocks):
            raise ValueError(
                f"Policy '{self._policy_id}' declares {len(blocks)} motion-group "
                f"block(s) ({[b.action_name for b in blocks]}), got "
                f"{len(motion_group_ids)} motion group(s): {motion_group_ids}. "
                "Declare motion groups in the policy's block order."
            )
        for block, motion_group_id in zip(blocks, motion_group_ids, strict=True):
            block.motion_group_id = motion_group_id
        self._blocks = blocks
        self._motion_group_ids = list(motion_group_ids)

    async def _begin_episode(self) -> None:
        """Open a fresh session and clear all derived episode state."""
        self._session_id = uuid.uuid4().hex
        self._step = 0
        self._t0 = time.monotonic()
        self._last_action = {}

        body = await self._request(
            reset_subject(self._policy_id, self._prefix),
            encode(
                "reset",
                ResetRequest(
                    session_id=self._session_id,
                    motion_group_ids=self._motion_group_ids,
                ).model_dump(),
            ),
            expect="reset",
        )
        # A refreshed spec on reset (a hot model swap) replaces the old one.
        self._info = ResetResponse.model_validate(body).info
        self._bind_blocks(self._motion_group_ids)
        logger.info("Policy session %s open on %s", self._session_id, self._motion_group_ids)

    def _build_proprio(
        self, block: _Block, joints: NDArray[np.float32], now: float
    ) -> NDArray[np.float32]:
        """Concatenate joint positions, velocities and last action for one block."""
        expected_dof = block.spec.dof
        if joints.size != expected_dof:
            raise PolicyProtocolError(
                f"Motion group '{block.motion_group_id}' has {joints.size} axes but "
                f"policy '{self._policy_id}' expects {expected_dof} for "
                f"'{block.action_name}'.",
                code="dof_mismatch",
            )

        velocities = np.asarray(
            self._velocities.read(block.motion_group_id, joints.tolist(), now), dtype=np.float32
        )
        # First step: "last action" is the pose we are already in, i.e. hold.
        last_action = self._last_action.get(block.motion_group_id, joints)
        proprio = np.concatenate([joints, velocities, last_action]).astype(np.float32)

        spec = self.info.observation(block.obs_field)
        if spec is not None and spec.shape and spec.shape[-1] != proprio.size:
            raise PolicyProtocolError(
                f"'{block.obs_field}' width mismatch: built {proprio.size} "
                f"({PROPRIO_BLOCKS} x {joints.size}), policy declares {spec.shape[-1]}.",
                code="obs_mismatch",
            )
        return proprio

    def _to_chunk(self, response: PredictResponse) -> ActionChunk:
        joints: dict[str, list[list[float]]] = {}
        dt_ms = response.dt_ms or self._blocks[0].spec.dt_ms
        for block in self._blocks:
            targets = response.actions.get(block.action_name)
            if targets is None:
                raise PolicyProtocolError(
                    f"Reply is missing action '{block.action_name}'; got "
                    f"{sorted(response.actions)}."
                )
            if targets.shape[1] != block.spec.dof:
                raise PolicyProtocolError(
                    f"Action '{block.action_name}' has {targets.shape[1]} values per "
                    f"step, spec declares dof={block.spec.dof}."
                )
            self._last_action[block.motion_group_id] = targets[-1].astype(np.float32)
            joints[block.motion_group_id] = targets.tolist()

        values = {
            name[len(ACTION_KEY) + 1 :]: np
            .asarray(response.actions[name])
            .reshape(len(response.actions[name]), -1)
            .tolist()
            for name in self._extra_actions
            if name in response.actions
        }
        return ActionChunk(joints=joints, values=values, dt_ms=dt_ms)

    async def _request(self, subject: str, payload: bytes, *, expect: str) -> dict[str, Any]:
        """One request/reply round trip; broker failures raise :class:`PolicyProtocolError`.

        "Nobody subscribed" and "too slow" are reported apart.
        """
        if self._nc is None:
            raise PolicyProtocolError("Not connected to NATS.")
        try:
            msg = await self._nc.request(subject, payload, timeout=self._timeout_s)
        except NoRespondersError as exc:
            raise PolicyProtocolError(
                f"Nothing is subscribed to '{subject}' — the '{self._policy_id}' policy "
                f"service is not running, or its policy id / subject prefix does not "
                f"match (client prefix={self._prefix!r}).",
                code="no_responders",
            ) from exc
        except TimeoutError as exc:
            raise PolicyProtocolError(
                f"No reply on '{subject}' within {self._timeout_s}s — the "
                f"'{self._policy_id}' policy service is subscribed but did not answer "
                f"in time.",
                code="timeout",
            ) from exc
        return decode(msg.data, expect=expect)
