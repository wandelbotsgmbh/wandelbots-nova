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
        Dataset/control frequency used for action timing.  Returned
        :class:`ActionChunk` uses ``dt_ms = 1000 / fps``.
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
        device: str = "cpu",
        extra_state_keys: Sequence[str] = (),
        state_overrides: Mapping[str, float] | None = None,
        timeout_s: float = 15.0,
    ) -> None:
        if fps <= 0:
            msg = f"fps must be positive, got {fps}"
            raise ValueError(msg)
        if actions_per_chunk <= 0:
            msg = f"actions_per_chunk must be positive, got {actions_per_chunk}"
            raise ValueError(msg)

        self._server_address = server_address
        self._pretrained_name_or_path = pretrained_name_or_path
        self._policy_type = policy_type
        self._fps = fps
        self._actions_per_chunk = actions_per_chunk
        self._device = device
        self._extra_state_keys = list(extra_state_keys)
        self._state_overrides = dict(state_overrides or {})
        self._timeout_s = timeout_s

        self._channel: Any | None = None
        self._stub: Any | None = None
        self._setup_sent = False
        self._timestep = 0
        self._expected_state_dim: int | None = None
        self._expected_action_dim: int | None = None
        self._expected_image_keys: set[str] | None = None

    @property
    def dt_ms(self) -> float:
        """Action timestep in milliseconds."""
        return 1000.0 / self._fps

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
        self._validate_dimensions(state_names, action_slices)

        await asyncio.to_thread(self._ensure_policy_setup, schema, state_names, images)
        actions = await asyncio.to_thread(self._send_observation_and_get_actions, raw_obs)
        return self._decode_actions(actions, action_slices)

    async def close(self) -> None:
        """Close the gRPC channel."""
        channel = self._channel
        self._channel = None
        self._stub = None
        self._setup_sent = False
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

    def _send_observation_and_get_actions(self, raw_obs: dict[str, Any]) -> list[Any]:
        obs = TimedObservation(
            timestamp=time.time(),
            observation=raw_obs,
            timestep=self._timestep,
            must_go=True,
        )
        self._stub.SendObservations(
            send_bytes_in_chunks(
                pickle.dumps(obs),
                services_pb2.Observation,
                silent=True,
            ),
            timeout=self._timeout_s,
        )
        response = self._stub.GetActions(services_pb2.Empty(), timeout=self._timeout_s)
        if not response.data:
            msg = "LeRobot server returned an empty action response"
            raise RuntimeError(msg)
        actions = pickle.loads(response.data)  # noqa: S301  # nosec: trusted LeRobot protocol.
        if not isinstance(actions, list):
            msg = f"Expected LeRobot list[TimedAction], got {type(actions).__name__}"
            raise TypeError(msg)
        self._timestep += len(actions)
        return actions

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

    def _validate_dimensions(
        self,
        state_names: list[str],
        action_slices: list[tuple[str, slice]],
    ) -> None:
        if self._expected_state_dim is not None and len(state_names) != self._expected_state_dim:
            msg = (
                "LeRobot policy state dimension mismatch: "
                f"checkpoint expects {self._expected_state_dim}, schema produced {len(state_names)} "
                f"({state_names})."
            )
            raise ValueError(msg)

        action_dim = max((sl.stop for _mg_id, sl in action_slices), default=0)
        if self._expected_action_dim is not None and action_dim != self._expected_action_dim:
            msg = (
                "LeRobot policy action dimension mismatch: "
                f"checkpoint expects {self._expected_action_dim}, schema joint actions produce "
                f"{action_dim}. This client currently decodes flat LeRobot actions as joint targets only."
            )
            raise ValueError(msg)

    def _decode_actions(
        self,
        actions: list[Any],
        action_slices: list[tuple[str, slice]],
    ) -> ActionChunk:
        if not actions:
            msg = "LeRobot returned no action steps"
            raise ValueError(msg)

        joints: dict[str, list[list[float]]] = {mg_id: [] for mg_id, _sl in action_slices}
        for timed_action in actions:
            action = timed_action.get_action()
            if hasattr(action, "detach"):
                action = action.detach().cpu().numpy()
            action_arr = np.asarray(action, dtype=np.float32).reshape(-1)
            for mg_id, action_slice in action_slices:
                values = action_arr[action_slice]
                joints[mg_id].append([float(v) for v in values])

        return ActionChunk(joints=joints, dt_ms=self.dt_ms)
