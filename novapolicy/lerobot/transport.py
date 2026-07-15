"""Trusted gRPC transport for LeRobot asynchronous inference."""

from __future__ import annotations

import pickle  # nosec: LeRobot async inference uses trusted pickle payloads.
import time
from typing import Any

import grpc
from lerobot.async_inference.helpers import TimedObservation
from lerobot.transport.utils import send_bytes_in_chunks

from lerobot.transport import services_pb2, services_pb2_grpc


class LeRobotGrpcTransport:
    """Own the LeRobot gRPC channel and protocol serialization."""

    def __init__(self, server_address: str, *, timeout_s: float) -> None:
        self._server_address = server_address
        self._timeout_s = timeout_s
        self._channel: grpc.Channel | None = None
        self._stub: services_pb2_grpc.AsyncInferenceStub | None = None

    @property
    def connected(self) -> bool:
        return self._stub is not None

    def connect(self) -> None:
        if self._channel is not None:
            return
        channel = grpc.insecure_channel(self._server_address)
        self._channel = channel
        self._stub = services_pb2_grpc.AsyncInferenceStub(channel)
        self._stub.Ready(services_pb2.Empty(), timeout=self._timeout_s)

    def configure_policy(self, config: object) -> None:
        self._require_stub().SendPolicyInstructions(
            services_pb2.PolicySetup(data=pickle.dumps(config)),
            timeout=self._timeout_s,
        )

    def send_observation(
        self,
        observation: dict[str, Any],
        *,
        timestep: int,
        must_go: bool,
    ) -> None:
        timed_observation = TimedObservation(
            timestamp=time.time(),
            observation=observation,
            timestep=timestep,
            must_go=must_go,
        )
        self._require_stub().SendObservations(
            send_bytes_in_chunks(
                pickle.dumps(timed_observation),
                services_pb2.Observation,
                silent=True,
            ),
            timeout=self._timeout_s,
        )

    def receive_actions(self, *, allow_empty: bool = False) -> list[Any]:
        response = self._require_stub().GetActions(
            services_pb2.Empty(),
            timeout=self._timeout_s,
        )
        if not response.data:
            if allow_empty:
                return []
            raise RuntimeError("LeRobot server returned an empty action response")
        actions = pickle.loads(response.data)  # noqa: S301  # nosec: trusted protocol.
        if not isinstance(actions, list):
            msg = f"Expected LeRobot list[TimedAction], got {type(actions).__name__}"
            raise TypeError(msg)
        return actions

    def infer(
        self,
        observation: dict[str, Any],
        *,
        timestep: int,
        must_go: bool,
        allow_empty: bool = False,
    ) -> list[Any]:
        self.send_observation(observation, timestep=timestep, must_go=must_go)
        return self.receive_actions(allow_empty=allow_empty)

    def close(self) -> None:
        channel = self._channel
        self._channel = None
        self._stub = None
        if channel is not None:
            channel.close()

    def _require_stub(self) -> services_pb2_grpc.AsyncInferenceStub:
        if self._stub is None:
            self.connect()
        return self._stub
