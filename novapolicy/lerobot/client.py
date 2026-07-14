"""LeRobot async-inference policy client.

This client speaks LeRobot's trusted gRPC async-inference protocol and adapts
NOVA's ``PolicySchema`` observations/actions to LeRobot's flat
``observation.state`` / ``action`` tensors.

The implementation targets LeRobot policies that return a flat joint position
action vector.  Observation and action ordering is schema-derived:
``Observation.joint_positions("arm", source=mg)`` becomes ``arm_1``, ``arm_2``,
... in LeRobot's state vector, and returned actions are split back into the
schema's joint action motion groups by their current DOF.
"""

from __future__ import annotations

import asyncio
import contextlib
from enum import StrEnum
import logging
import pickle  # nosec: LeRobot async inference uses trusted pickle payloads.
import time
from typing import TYPE_CHECKING, Any, cast

import grpc
from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation
from lerobot.transport import services_pb2, services_pb2_grpc
from lerobot.transport.utils import send_bytes_in_chunks
from lerobot.utils.constants import OBS_IMAGES, OBS_STATE
import numpy as np

from novapolicy.policy_client import PolicyClient
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Mapping, Sequence

    from nova.types import RobotState
    from novapolicy.schema import PolicySchema

logger = logging.getLogger(__name__)

_IMAGE_NDIM = 3
_DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD = 0.75
_ASYNC_FROZEN_QUEUE_STEPS = 2


class AsyncQueueAggregation(StrEnum):
    """How overlapping LeRobot async-queue actions are merged by timestep."""

    WEIGHTED_AVERAGE = "weighted_average"
    LATEST_ONLY = "latest_only"
    AVERAGE = "average"
    CONSERVATIVE = "conservative"

    @property
    def old_action_weight(self) -> float:
        """Weight assigned to the queued action; the new action receives the remainder."""
        return {
            AsyncQueueAggregation.WEIGHTED_AVERAGE: 0.3,
            AsyncQueueAggregation.LATEST_ONLY: 0.0,
            AsyncQueueAggregation.AVERAGE: 0.5,
            AsyncQueueAggregation.CONSERVATIVE: 0.7,
        }[self]


class LeRobotPolicyClient(PolicyClient):
    """Policy client for LeRobot's async gRPC inference server.

    Parameters
    ----------
    server_address:
        LeRobot server address in ``"host:port"`` form.
    pretrained_name_or_path:
        Model directory or Hugging Face model id passed through to the server.
        For a remote server, this may be a server-local path that is not
        readable by the client process.
    policy_type:
        LeRobot policy type sent to the async inference server, e.g. ``"act"``.
        The client-side decoder is policy-architecture agnostic as long as the
        server returns a flat joint action vector matching the schema action DOF.
    fps:
        Dataset/control frequency used for action timing.
    playback_speed:
        Physical execution speed relative to the dataset rate. Returned
        :class:`ActionChunk` uses ``dt_ms = 1000 / (fps * playback_speed)``.
        Keep this at ``1.0`` for nominal dataset timing; values below ``1.0``
        slow physical execution without misrepresenting the dataset frequency.
    actions_per_chunk:
        Number of action steps requested from the server.  LeRobot's async
        server requires this in ``RemotePolicyConfig`` and uses it to slice the
        policy output chunk before returning actions.
    device:
        Server-side torch device string sent to the LeRobot server
        (``"cuda"``, ``"mps"``, ``"cpu"``).  The async server does not choose
        this automatically: it calls ``policy.to(device)`` with the value from
        ``RemotePolicyConfig``.  For a remote GPU server this is usually
        ``"cuda"``; use ``"mps"`` only when the LeRobot server itself runs on
        an Apple Silicon machine.  LeRobot does not expose a remote hardware
        capability RPC, so this should come from the server deployment config.
    extra_state_keys:
        Optional additional flat observation keys to append to
        ``observation.state`` after schema-derived joints and IOs.  This is a
        fallback for computed numeric observations; normal joint+gripper cases
        do not need it.
    state_overrides:
        Optional raw observation values to force before sending to LeRobot.  Use
        this for checkpoints whose state vector contains constants rather than
        measured joints, while still using schema-derived joint actions for
        decoding.  Example: ``{"arm_1": 0.0, ..., "arm_6": 0.0}``.
    timeout_s:
        gRPC deadline for individual calls.
    use_async_queue:
        Execute through LeRobot's client-side asynchronous action queue. The
        refill point is controlled by ``async_queue_refill_threshold``.
    async_queue_aggregation:
        Aggregation applied to old and new actions that target the same future
        timestep. Defaults to LeRobot's ``WEIGHTED_AVERAGE`` mode.
    async_queue_refill_threshold:
        Fraction of the previous action chunk remaining when asynchronous
        inference starts. Defaults to ``0.75`` so inference overlaps execution
        before the NOVA lookahead is close to empty.

    Notes
    -----
    LeRobot's protocol uses pickle and should only be used on a trusted network.
    The server loads the policy after ``SendPolicyInstructions``; this client
    sends those instructions lazily on the first inference call because image
    shapes and joint DOF are known then.  The upstream async-inference server
    does not expose a metadata RPC; ``Ready`` returns an empty message, and the
    server only loads the policy after ``SendPolicyInstructions``.  Therefore
    policy setup values are explicit client configuration and the model path is
    passed through unchanged to the server.
    """

    def __init__(
        self,
        server_address: str,
        pretrained_name_or_path: str,
        *,
        actions_per_chunk: int,
        policy_type: str = "act",
        fps: float = 15.0,
        playback_speed: float = 1.0,
        device: str = "cpu",
        extra_state_keys: Sequence[str] = (),
        state_overrides: Mapping[str, float] | None = None,
        timeout_s: float = 15.0,
        use_async_queue: bool = False,
        async_queue_aggregation: AsyncQueueAggregation = AsyncQueueAggregation.WEIGHTED_AVERAGE,
        async_queue_refill_threshold: float = _DEFAULT_ASYNC_QUEUE_REFILL_THRESHOLD,
    ) -> None:
        if fps <= 0:
            msg = f"fps must be positive, got {fps}"
            raise ValueError(msg)
        if playback_speed <= 0:
            msg = f"playback_speed must be positive, got {playback_speed}"
            raise ValueError(msg)
        if actions_per_chunk <= 0:
            msg = f"actions_per_chunk must be positive, got {actions_per_chunk}"
            raise ValueError(msg)
        if not 0 < async_queue_refill_threshold <= 1:
            msg = (
                "async_queue_refill_threshold must be in (0, 1], "
                f"got {async_queue_refill_threshold}"
            )
            raise ValueError(msg)

        self._server_address = server_address
        self._pretrained_name_or_path = pretrained_name_or_path
        self._policy_type = policy_type
        self._fps = fps
        self._playback_speed = playback_speed
        self._actions_per_chunk = actions_per_chunk
        self._device = device
        self._extra_state_keys = list(extra_state_keys)
        self._state_overrides = dict(state_overrides or {})
        self._timeout_s = timeout_s
        self._use_async_queue = use_async_queue
        self._async_queue_aggregation = AsyncQueueAggregation(async_queue_aggregation)
        self._async_queue_refill_threshold = async_queue_refill_threshold

        self._channel: Any | None = None
        self._stub: Any | None = None
        self._setup_sent = False
        self._timestep = 0
        self._expected_state_dim: int | None = None
        self._expected_action_dim: int | None = None
        self._expected_image_keys: set[str] | None = None
        self._logged_action_chunk_shape = False
        self._action_queue: list[tuple[int, np.ndarray[Any, Any]]] = []
        self._action_prediction_counts: dict[int, int] = {}
        self._published_actions: dict[int, np.ndarray[Any, Any]] = {}
        self._latest_action_timestep = -1
        self._action_chunk_size = -1
        self._pending_actions_task: asyncio.Task[list[Any]] | None = None
        self._trajectory_trace_enabled = False
        self._raw_action_chunk_trace: list[dict[str, object]] = []

    @property
    def dt_ms(self) -> float:
        """Physical action timestep in milliseconds."""
        return 1000.0 / (self._fps * self._playback_speed)

    @property
    def trajectory_trace(self) -> dict[str, object]:
        """Raw pre-aggregation inference chunks collected for offline analysis."""
        return {"raw_action_chunks": self._raw_action_chunk_trace}

    def enable_trajectory_trace(self) -> None:
        """Enable in-memory raw chunk collection for the next episode."""
        self._trajectory_trace_enabled = True

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        """Whether continuous chunks need a measured-state bridge before execution."""
        return self._use_async_queue

    def synchronize_action_timestep(self, timestep: int) -> None:
        """Drop queue actions whose NOVA execution timestamps have elapsed."""
        if not self._use_async_queue or timestep <= self._latest_action_timestep + 1:
            return

        previous = self._latest_action_timestep
        self._latest_action_timestep = timestep - 1
        self._action_queue = [
            (queued_timestep, action)
            for queued_timestep, action in self._action_queue
            if queued_timestep >= timestep
        ]
        retained_timesteps = {queued_timestep for queued_timestep, _action in self._action_queue}
        self._action_prediction_counts = {
            queued_timestep: count
            for queued_timestep, count in self._action_prediction_counts.items()
            if queued_timestep in retained_timesteps
        }
        logger.info(
            "Synchronized LeRobot queue to NOVA timestep %d (skipped %d actions)",
            timestep,
            timestep - previous - 1,
        )

    async def connect(self, motion_group_ids: list[str]) -> None:  # noqa: ARG002
        """Open the gRPC channel and reset the server episode state."""
        await asyncio.to_thread(self._connect_sync)

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Validate schema constraints known without loading the remote policy."""
        if self._expected_image_keys is not None:
            missing = self._expected_image_keys - set(schema.image_sources)
            if missing:
                msg = f"LeRobot policy expects image observations missing from schema: {sorted(missing)}"
                raise ValueError(msg)

        if not schema.joint_action_keys:
            msg = "LeRobotPolicyClient currently requires at least one joint action target"
            raise ValueError(msg)

    async def prepare(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,  # noqa: ARG002
    ) -> None:
        """Send LeRobot policy setup before the executor timeout starts."""
        if self._stub is None:
            await self.connect([])

        state_names = self._state_names(states, schema)
        action_slices = self._joint_action_slices(states, schema)
        io_action_slices = self._io_action_slices(action_slices, schema)
        self._validate_dimensions(state_names, action_slices, io_action_slices)
        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images)

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Send one observation to LeRobot and decode the returned action chunk."""
        if self._stub is None:
            await self.connect([])

        raw_obs = await self._build_raw_observation(states, schema, images, io_values)
        state_names = self._state_names(states, schema)
        action_slices = self._joint_action_slices(states, schema)
        io_action_slices = self._io_action_slices(action_slices, schema)
        self._validate_dimensions(state_names, action_slices, io_action_slices)

        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images)
        if self._use_async_queue:
            return await self._get_async_queue_actions(raw_obs, action_slices, io_action_slices)
        actions = await asyncio.to_thread(self._send_observation_and_get_actions, raw_obs)
        return self._decode_actions(actions, action_slices, io_action_slices)

    async def close(self) -> None:
        """Close the gRPC channel."""
        channel = self._channel
        self._channel = None
        self._stub = None
        self._setup_sent = False
        pending = self._pending_actions_task
        self._pending_actions_task = None
        if pending is not None:
            if not pending.done():
                pending.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await pending
        self._reset_action_queue()
        if channel is not None:
            close = getattr(channel, "close", None)
            if close is not None:
                await asyncio.to_thread(close)

    # ------------------------------------------------------------------
    # Connection / setup
    # ------------------------------------------------------------------

    def _connect_sync(self) -> None:
        if self._channel is not None:
            return
        channel = grpc.insecure_channel(self._server_address)
        self._channel = channel
        self._stub = services_pb2_grpc.AsyncInferenceStub(channel)
        self._stub.Ready(services_pb2.Empty(), timeout=self._timeout_s)
        self._setup_sent = False
        self._timestep = 0
        self._reset_action_queue()

    def _reset_action_queue(self) -> None:
        self._action_queue.clear()
        self._action_prediction_counts.clear()
        self._published_actions.clear()
        self._latest_action_timestep = -1
        self._action_chunk_size = -1
        self._pending_actions_task = None
        self._raw_action_chunk_trace.clear()

    def _ensure_policy_setup(
        self,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
    ) -> None:
        if self._setup_sent:
            return
        if self._stub is None:
            self._connect_sync()

        setup = RemotePolicyConfig(
            policy_type=self._policy_type,
            pretrained_name_or_path=self._pretrained_name_or_path,
            lerobot_features=self._lerobot_features(schema, state_names, images),
            actions_per_chunk=self._actions_per_chunk,
            device=self._device,
        )
        payload = pickle.dumps(setup)
        self._stub.SendPolicyInstructions(
            services_pb2.PolicySetup(data=payload),
            timeout=self._timeout_s,
        )
        self._setup_sent = True

    # ------------------------------------------------------------------
    # Observation encoding
    # ------------------------------------------------------------------

    async def _build_raw_observation(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        raw_obs = await schema.build_observation(states, io_values)
        raw_obs.update(self._state_overrides)
        if images:
            raw_obs.update(images)
        return raw_obs

    def _state_names(self, states: dict[str, RobotState], schema: PolicySchema) -> list[str]:
        """Return schema-derived LeRobot ``observation.state`` names."""
        names: list[str] = []
        for mapping in schema.joint_mappings:
            dof = 0
            for mg in mapping.sources:
                state = states.get(mg.id)
                if state is not None:
                    dof += len(state.joints)
            names.extend(f"{mapping.key}_{idx}" for idx in range(1, dof + 1))

        names.extend(mapping.key for mapping in schema.obs_io_mappings)
        names.extend(self._extra_state_keys)
        return names

    def _lerobot_features(
        self,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
    ) -> dict[str, dict[str, Any]]:
        features: dict[str, dict[str, Any]] = {
            OBS_STATE: {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            }
        }

        expected_image_keys = self._expected_image_keys or set(schema.image_sources)
        for key in sorted(expected_image_keys):
            shape = self._image_shape(key, images)
            features[f"{OBS_IMAGES}.{key}"] = {
                "dtype": "image",
                "shape": shape,
                "names": ["height", "width", "channels"],
            }
        return features

    def _image_shape(self, key: str, images: dict[str, Any] | None) -> tuple[int, int, int]:
        image = images.get(key) if images is not None else None
        if isinstance(image, np.ndarray) and image.ndim == _IMAGE_NDIM:
            return cast("tuple[int, int, int]", tuple(int(v) for v in image.shape))

        msg = (
            f"LeRobot image observation {key!r} is missing or is not an HxWxC numpy array. "
            "The client needs the first camera frame to declare LeRobot feature metadata; "
            "configure camera resolution with WebRTCCameras(..., resize=(width, height))."
        )
        raise ValueError(msg)

    # ------------------------------------------------------------------
    # Action transport / decoding
    # ------------------------------------------------------------------

    def _send_observation(
        self,
        raw_obs: dict[str, Any],
        *,
        timestep: int,
        must_go: bool,
    ) -> None:
        obs = TimedObservation(
            timestamp=time.time(),
            observation=raw_obs,
            timestep=timestep,
            must_go=must_go,
        )
        self._stub.SendObservations(
            send_bytes_in_chunks(
                pickle.dumps(obs),
                services_pb2.Observation,
                silent=True,
            ),
            timeout=self._timeout_s,
        )

    def _receive_actions(self, *, allow_empty: bool = False) -> list[Any]:
        response = self._stub.GetActions(services_pb2.Empty(), timeout=self._timeout_s)
        if not response.data:
            if allow_empty:
                return []
            msg = "LeRobot server returned an empty action response"
            raise RuntimeError(msg)
        actions = pickle.loads(response.data)  # noqa: S301  # nosec: trusted LeRobot protocol.
        if not isinstance(actions, list):
            msg = f"Expected LeRobot list[TimedAction], got {type(actions).__name__}"
            raise TypeError(msg)
        return actions

    def _send_observation_and_get_actions(
        self,
        raw_obs: dict[str, Any],
        *,
        timestep: int | None = None,
        must_go: bool = True,
        allow_empty: bool = False,
    ) -> list[Any]:
        observation_timestep = self._timestep if timestep is None else timestep
        self._send_observation(raw_obs, timestep=observation_timestep, must_go=must_go)
        actions = self._receive_actions(allow_empty=allow_empty)
        if timestep is None:
            self._timestep += len(actions)
        return actions

    async def _get_async_queue_actions(
        self,
        raw_obs: dict[str, Any],
        action_slices: list[tuple[str, slice]],
        io_action_slices: list[tuple[str, str, Any, slice]],
    ) -> ActionChunk:
        """Consume LeRobot's rolling client queue and return its current lookahead."""
        queue_updated = await self._merge_completed_action_request()

        while not self._action_queue:
            if self._pending_actions_task is None:
                self._start_action_request(raw_obs)
            queue_updated = await self._merge_completed_action_request(wait=True) or queue_updated

        timestep, current_action = self._action_queue.pop(0)
        self._action_prediction_counts.pop(timestep, None)
        self._latest_action_timestep = timestep

        # LeRobot sends one action directly to its robot each control tick.
        # NOVA instead needs a timestamped lookahead. Publish that lookahead
        # only when inference has updated the queue; repeatedly publishing a
        # shrinking tail would re-anchor old actions at "now" and make the
        # waypoint timeline jump backward.
        if self._ready_to_request_actions():
            if self._pending_actions_task is None:
                self._start_action_request(raw_obs)
            else:
                # RobotClient keeps publishing observations while the queue is
                # below the refill threshold. This lets a later observation
                # unblock GetActions when an earlier one was filtered as too
                # similar by the policy server.
                await self._send_additional_observation(raw_obs)

            # Do not wait for inference here. The selected timestep came from a
            # controller sample taken before this call; blocking for inference
            # can make it stale before the replacement is sent. NOVA keeps
            # executing the published lookahead, and the completed refill is
            # merged on the next controller-synchronized policy tick.

        preview_entries = [(timestep, current_action), *self._action_queue]

        if queue_updated:
            # The selected action is scheduled at or just after controller now.
            # Prepend its predecessor from NOVA's previously published
            # trajectory, then retain the selected action and its immutable
            # successor. The replacement request therefore carries an explicit
            # past/current/future overlap before fresh aggregated predictions.
            predecessor = self._published_actions.get(timestep - 1)
            action_timestep = timestep
            if predecessor is not None:
                preview_entries.insert(0, (timestep - 1, predecessor))
                action_timestep -= 1
            self._published_actions = dict(preview_entries)
            return self._decode_action_arrays(
                [action for _timestep, action in preview_entries],
                action_slices,
                io_action_slices,
                action_timestep=action_timestep,
                io_action_array=current_action,
            )

        # The current joint action is already present in NOVA's previously
        # published lookahead. Return only its IO component so computed actions
        # (for example gripper-triggered handover release) still run on time.
        return self._decode_action_arrays(
            [current_action],
            [],
            io_action_slices,
            action_timestep=timestep,
        )

    def _ready_to_request_actions(self) -> bool:
        if self._action_chunk_size <= 0:
            return True
        return (
            len(self._action_queue) / self._action_chunk_size < self._async_queue_refill_threshold
        )

    async def _send_additional_observation(self, raw_obs: dict[str, Any]) -> None:
        await asyncio.to_thread(
            self._send_observation,
            raw_obs,
            timestep=max(self._latest_action_timestep, 0),
            must_go=False,
        )

    def _start_action_request(self, raw_obs: dict[str, Any]) -> None:
        # Force each threshold-triggered refill through LeRobot's observation
        # similarity filter. Its default 1-radian state tolerance otherwise
        # defers ACT inference until the local queue is completely empty.
        self._pending_actions_task = asyncio.create_task(
            asyncio.to_thread(
                self._send_observation_and_get_actions,
                raw_obs,
                timestep=max(self._latest_action_timestep, 0),
                must_go=True,
                allow_empty=True,
            ),
            name="lerobot-async-action-refill",
        )

    async def _merge_completed_action_request(self, *, wait: bool = False) -> bool:
        task = self._pending_actions_task
        if task is None or (not wait and not task.done()):
            return False
        self._pending_actions_task = None
        actions = await task
        if not actions:
            return False
        return self._merge_timed_actions(actions)

    def _merge_timed_actions(self, timed_actions: list[Any]) -> bool:
        """Replace the future queue with LeRobot's timestep-aligned aggregation."""
        self._action_chunk_size = max(self._action_chunk_size, len(timed_actions))
        decoded_actions = [
            (
                int(timed_action.get_timestep()),
                self._action_to_array(timed_action.get_action()),
            )
            for timed_action in timed_actions
        ]
        if self._trajectory_trace_enabled:
            self._raw_action_chunk_trace.append(
                {
                    "first_timestep": decoded_actions[0][0],
                    "actions": [
                        {"timestep": timestep, "values": action.tolist()}
                        for timestep, action in decoded_actions
                    ],
                }
            )

        current = dict(self._action_queue)
        future: dict[int, np.ndarray[Any, Any]] = {}
        future_counts: dict[int, int] = {}
        for timestep, new_action in decoded_actions:
            if timestep <= self._latest_action_timestep:
                continue
            if timestep in current:
                # The selected action and its successor are already inside
                # NOVA's active trajectory. Keep both unchanged; together with
                # the prepended predecessor they preserve the replacement's
                # past/current/future position and velocity seam. Aggregation
                # begins only after that fixed prefix.
                previous_count = self._action_prediction_counts.get(timestep, 1)
                if timestep <= self._latest_action_timestep + _ASYNC_FROZEN_QUEUE_STEPS:
                    future[timestep] = current[timestep]
                    future_counts[timestep] = previous_count
                elif self._async_queue_aggregation is AsyncQueueAggregation.AVERAGE:
                    future[timestep] = (previous_count * current[timestep] + new_action) / (
                        previous_count + 1
                    )
                    future_counts[timestep] = previous_count + 1
                else:
                    old_weight = self._async_queue_aggregation.old_action_weight
                    future[timestep] = (
                        old_weight * current[timestep] + (1.0 - old_weight) * new_action
                    )
                    future_counts[timestep] = previous_count + 1
            else:
                future[timestep] = new_action
                future_counts[timestep] = 1
        self._action_queue = sorted(future.items())
        self._action_prediction_counts = future_counts
        return bool(future)

    @staticmethod
    def _action_to_array(action: object) -> np.ndarray[Any, Any]:
        if hasattr(action, "detach"):
            action = action.detach().cpu().numpy()
        return np.asarray(action, dtype=np.float32).reshape(-1)

    def _joint_action_slices(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
    ) -> list[tuple[str, slice]]:
        slices: list[tuple[str, slice]] = []
        offset = 0
        for _key, motion_groups in schema.joint_action_keys:
            for mg in motion_groups:
                state = states.get(mg.id)
                if state is None:
                    continue
                dof = len(state.joints)
                slices.append((mg.id, slice(offset, offset + dof)))
                offset += dof
        return slices

    def _io_action_slices(
        self,
        action_slices: list[tuple[str, slice]],
        schema: PolicySchema,
    ) -> list[tuple[str, str, Any, slice]]:
        offset = max((sl.stop for _mg_id, sl in action_slices), default=0)
        slices: list[tuple[str, str, Any, slice]] = []
        for _key, motion_group, io, mapping in schema.io_action_keys:
            slices.append((motion_group.id, io, mapping, slice(offset, offset + 1)))
            offset += 1
        return slices

    def _validate_dimensions(
        self,
        state_names: list[str],
        action_slices: list[tuple[str, slice]],
        io_action_slices: list[tuple[str, str, Any, slice]],
    ) -> None:
        if self._expected_state_dim is not None and len(state_names) != self._expected_state_dim:
            msg = (
                "LeRobot policy state dimension mismatch: "
                f"checkpoint expects {self._expected_state_dim}, schema produced {len(state_names)} "
                f"({state_names})."
            )
            raise ValueError(msg)

        action_dim = max(
            [sl.stop for _mg_id, sl in action_slices]
            + [sl.stop for _mg_id, _io, _mapping, sl in io_action_slices],
            default=0,
        )
        if self._expected_action_dim is not None and action_dim != self._expected_action_dim:
            msg = (
                "LeRobot policy action dimension mismatch: "
                f"checkpoint expects {self._expected_action_dim}, schema actions produce {action_dim}. "
                "This client decodes flat LeRobot actions as joint targets followed by IO actions."
            )
            raise ValueError(msg)

    def _decode_actions(
        self,
        actions: list[Any],
        action_slices: list[tuple[str, slice]],
        io_action_slices: list[tuple[str, str, Any, slice]],
    ) -> ActionChunk:
        return self._decode_action_arrays(
            [self._action_to_array(timed_action.get_action()) for timed_action in actions],
            action_slices,
            io_action_slices,
        )

    def _decode_action_arrays(
        self,
        action_arrays: list[np.ndarray[Any, Any]],
        action_slices: list[tuple[str, slice]],
        io_action_slices: list[tuple[str, str, Any, slice]],
        *,
        action_timestep: int = -1,
        io_action_array: np.ndarray[Any, Any] | None = None,
    ) -> ActionChunk:
        if not action_arrays:
            msg = "LeRobot returned no action steps"
            raise ValueError(msg)

        joints: dict[str, list[list[float]]] = {mg_id: [] for mg_id, _sl in action_slices}
        for action_arr in action_arrays:
            for mg_id, action_slice in action_slices:
                values = action_arr[action_slice]
                joints[mg_id].append([float(v) for v in values])

        if not self._logged_action_chunk_shape:
            logger.info(
                "First LeRobot action chunk: %d steps, action_dim=%d",
                len(action_arrays),
                int(action_arrays[0].size),
            )
            self._logged_action_chunk_shape = True

        ios: dict[str, dict[str, bool | int | float | str]] = {}
        if io_action_slices:
            io_source = action_arrays[0] if io_action_array is None else io_action_array
            for mg_id, io, mapping, action_slice in io_action_slices:
                values = io_source[action_slice]
                if values.size != 1:
                    msg = f"LeRobot IO action {io!r} expected one value, got {values.size}"
                    raise ValueError(msg)
                ios.setdefault(mg_id, {})[io] = mapping.to_hardware(float(values[0]))

        return ActionChunk(
            joints=joints,
            ios=ios or None,
            dt_ms=self.dt_ms,
            action_timestep=action_timestep,
        )
