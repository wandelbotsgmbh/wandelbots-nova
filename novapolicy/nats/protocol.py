"""Wire protocol for remote policies reached over NATS. The single source of truth.

A *policy service* owns a trained model and answers inference requests. A *policy
client* (this repo's action service) owns the robot: it reads state, asks the
policy for targets, and streams them into novapolicy's jogging pipeline. Nothing
but this module defines what crosses the wire.

Transport
    Plain core-NATS **request/reply** (no JetStream): inference is a synchronous
    RPC in the hot loop, and a dropped request must fail fast rather than be
    replayed onto a moving robot later.

Encoding
    msgpack, with a small extension for numpy arrays (:func:`encode`/:func:`decode`)
    so proprioceptive vectors and camera frames travel as raw buffers instead of
    JSON number lists.

Handshake
    ``info`` returns a :class:`PolicyInfo` describing the observation and action
    spaces. The client validates its own ``PolicySchema`` against that *before*
    the first inference call, so a DOF or layout mismatch fails during connect
    instead of showing up as unexpected robot motion.

Sessions
    ``reset`` opens a session (one episode) and returns a ``session_id`` that every
    subsequent ``predict`` carries. Policies with internal state (action history, RNN
    hidden state, phase accumulators) key that state by session, and ``close``
    releases it. Stateless policies can ignore it beyond echoing it back.

.. warning::
   This module is a **copy**. Its source of truth is maintained outside this
   repository, alongside the policy-service implementation it defines the
   contract for — a policy service deploys as its own container and must not
   import from its clients, so each side carries the same file.

   Do not edit this copy to change the protocol: change it upstream, re-copy,
   and ``diff`` to confirm the two agree. Nothing checks that automatically.
   Everything below the docstring is expected to be byte-identical upstream; a
   ``diff`` that reports only this warning is a clean one.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Literal

import msgpack
import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, Field

PROTOCOL_VERSION = 1
"""Bumped on any breaking change. The client refuses a mismatching server."""

DEFAULT_SUBJECT_PREFIX = "policy"
"""Root of the subject tree. Override per deployment to isolate environments."""


# ---------------------------------------------------------------------------
# Subjects
# ---------------------------------------------------------------------------


def info_subject(policy_id: str, prefix: str = DEFAULT_SUBJECT_PREFIX) -> str:
    """``{prefix}.{policy_id}.info``: request the policy's spaces and metadata."""
    return f"{prefix}.{policy_id}.info"


def reset_subject(policy_id: str, prefix: str = DEFAULT_SUBJECT_PREFIX) -> str:
    """``{prefix}.{policy_id}.reset``: open an episode session."""
    return f"{prefix}.{policy_id}.reset"


def predict_subject(policy_id: str, prefix: str = DEFAULT_SUBJECT_PREFIX) -> str:
    """``{prefix}.{policy_id}.predict``: observation in, action chunk out."""
    return f"{prefix}.{policy_id}.predict"


def close_subject(policy_id: str, prefix: str = DEFAULT_SUBJECT_PREFIX) -> str:
    """``{prefix}.{policy_id}.close``: release session state."""
    return f"{prefix}.{policy_id}.close"


# ---------------------------------------------------------------------------
# Space description (handshake payloads, pure JSON, no arrays)
# ---------------------------------------------------------------------------

FieldKind = Literal["state"]
"""``state`` → 1-D float vector. The only kind anything in this codebase produces
or consumes; narrow rather than speculatively wide (this was ``image``/``text``
too, with no client-side consumer for either; see git history if reviving one)."""

ActionKind = Literal["joint_position"]
"""Absolute joint angles in radians, sent straight to joint jogging. The only kind
anything in this codebase emits (this was also ``joint_delta``/``tcp_pose``, with no
producer for either)."""


class FieldSpec(BaseModel):
    """One entry of the observation space."""

    name: str = Field(..., description="Key under `PredictRequest.observation`.")
    kind: FieldKind = Field(default="state")
    shape: list[int] = Field(..., description="Shape excluding any batch axis.")


class ActionSpec(BaseModel):
    """The action space. One entry per key of ``PredictResponse.actions``."""

    name: str = Field(default="joint_target", description="Key under `PredictResponse.actions`.")
    kind: ActionKind = Field(default="joint_position")
    dof: int = Field(..., description="Values per step.")
    horizon: int = Field(
        ...,
        description="Steps per chunk the policy emits. Clients may execute a prefix.",
    )
    dt_ms: float = Field(..., description="Nominal spacing between steps in milliseconds.")


class PolicyInfo(BaseModel):
    """Reply to ``info``: everything the client needs to bind safely."""

    protocol_version: int = Field(default=PROTOCOL_VERSION)
    policy_id: str = Field(..., description="Subject-tree identity, e.g. 'dummy'.")
    name: str = Field(..., description="Display name.")
    version: str = Field(default="0.0.0", description="Model/service version.")
    embodiment: str = Field(
        default="",
        description="Which robot this checkpoint was trained for, e.g. 'generic-6dof'.",
    )
    observations: list[FieldSpec] = Field(default_factory=list)
    actions: list[ActionSpec] = Field(default_factory=list)
    description: str = Field(default="")

    def observation(self, name: str) -> FieldSpec | None:
        return next((f for f in self.observations if f.name == name), None)

    @property
    def primary_action(self) -> ActionSpec:
        """The single action head. Multi-head policies (e.g. arm + gripper) will
        need the caller to walk ``actions`` instead."""
        if not self.actions:
            raise PolicyProtocolError("Policy declares no action space.")
        return self.actions[0]


class ResetRequest(BaseModel):
    """Open an episode."""

    session_id: str = Field(..., description="Client-generated episode id.")
    motion_group_ids: list[str] = Field(default_factory=list)


class ResetResponse(BaseModel):
    session_id: str
    info: PolicyInfo


class CloseRequest(BaseModel):
    session_id: str


class CloseResponse(BaseModel):
    session_id: str
    steps: int = Field(default=0, description="Inference calls served in this session.")


# ---------------------------------------------------------------------------
# Inference payloads (contain arrays, so dataclasses, not pydantic)
# ---------------------------------------------------------------------------


@dataclasses.dataclass(slots=True)
class PredictRequest:
    """One inference request. ``observation`` values are numpy arrays (``state``/
    ``image`` fields) or ``str`` (``text`` fields), keyed by ``FieldSpec.name``."""

    session_id: str
    step: int
    t_ms: float
    """Client monotonic time since ``reset``, in milliseconds."""
    observation: dict[str, Any] = dataclasses.field(default_factory=dict)

    def to_wire(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "step": self.step,
            "t_ms": self.t_ms,
            "observation": self.observation,
        }

    @classmethod
    def from_wire(cls, body: dict[str, Any]) -> PredictRequest:
        return cls(
            session_id=str(body["session_id"]),
            step=int(body["step"]),
            t_ms=float(body["t_ms"]),
            observation=dict(body.get("observation") or {}),
        )


@dataclasses.dataclass(slots=True)
class PredictResponse:
    """One action chunk. Each ``actions`` value has shape ``(horizon, dof)``."""

    session_id: str
    actions: dict[str, NDArray[np.float32]]
    dt_ms: float

    def to_wire(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "actions": self.actions,
            "dt_ms": self.dt_ms,
        }

    @classmethod
    def from_wire(cls, body: dict[str, Any]) -> PredictResponse:
        raw = body.get("actions") or {}
        actions: dict[str, NDArray[np.float32]] = {}
        for key, value in raw.items():
            arr = np.asarray(value, dtype=np.float32)
            if arr.ndim == 1:  # tolerate a single step sent unbatched
                arr = arr[np.newaxis, :]
            if arr.ndim != _CHUNK_NDIM:
                raise PolicyProtocolError(
                    f"Action '{key}' must be (horizon, dof); got shape {arr.shape}."
                )
            actions[key] = arr
        return cls(
            session_id=str(body["session_id"]),
            actions=actions,
            dt_ms=float(body.get("dt_ms", 0.0)),
        )


_CHUNK_NDIM = 2


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------


class PolicyProtocolError(RuntimeError):
    """Malformed message, version mismatch, or an error reply from the service."""

    def __init__(self, message: str, *, code: str = "protocol_error") -> None:
        super().__init__(message)
        self.code = code


# ---------------------------------------------------------------------------
# Envelope + codec
# ---------------------------------------------------------------------------
#
# The ndarray/scalar sub-encoding below matches openpi-client's
# ``msgpack_numpy.py`` (github.com/Physical-Intelligence/openpi) byte for byte:
# same tag names, same ``bytes`` keys, same ``__npgeneric__`` scalar handling.
# Nothing in this codebase talks to an openpi server today, but this is the
# closest thing robot-learning policy serving has to a lingua franca, and
# diverging from it gratuitously (e.g. by using ``str`` keys where they use
# ``bytes``) would foreclose interop for no benefit. The outer envelope
# (``{v, type, body}``) and subject scheme are still ours.

_ND_TAG = b"__ndarray__"
_SCALAR_TAG = b"__npgeneric__"

_UNSUPPORTED_DTYPE_KINDS = ("V", "O", "c")
"""Structured (``V``), object (``O``) and complex (``c``) dtypes: opaque to
``np.frombuffer`` on the far side, so packing them would silently produce
garbage on unpack instead of failing here."""


def _pack_default(obj: object) -> object:
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind in _UNSUPPORTED_DTYPE_KINDS:
        raise TypeError(f"Cannot serialize dtype {obj.dtype} over the policy protocol.")
    if isinstance(obj, np.ndarray):
        contiguous = np.ascontiguousarray(obj)
        return {
            _ND_TAG: True,
            b"dtype": contiguous.dtype.str,
            b"shape": contiguous.shape,
            b"data": contiguous.tobytes(),
        }
    if isinstance(obj, np.generic):  # numpy scalar -> tagged, dtype-preserving payload
        return {_SCALAR_TAG: True, b"dtype": obj.dtype.str, b"data": obj.item()}
    raise TypeError(f"Cannot serialize {type(obj).__name__} over the policy protocol.")


def _unpack_hook(obj: dict[str, Any]) -> object:
    if _ND_TAG in obj:
        try:
            return np.frombuffer(obj[b"data"], dtype=np.dtype(obj[b"dtype"])).reshape(obj[b"shape"])
        except (KeyError, ValueError, TypeError) as exc:
            raise PolicyProtocolError(f"Corrupt array payload: {exc}") from exc
    if _SCALAR_TAG in obj:
        try:
            return np.dtype(obj[b"dtype"]).type(obj[b"data"])
        except (KeyError, ValueError, TypeError) as exc:
            raise PolicyProtocolError(f"Corrupt scalar payload: {exc}") from exc
    return obj


def encode(msg_type: str, body: dict[str, Any]) -> bytes:
    """Wrap ``body`` in the versioned envelope and msgpack it."""
    return msgpack.packb(
        {"v": PROTOCOL_VERSION, "type": msg_type, "body": body},
        default=_pack_default,
        use_bin_type=True,
    )


def encode_error(message: str, *, code: str = "error") -> bytes:
    """Reply the service sends instead of a body when a handler fails."""
    return encode("error", {"code": code, "message": message})


def decode(payload: bytes, *, expect: str | None = None) -> dict[str, Any]:
    """Unpack an envelope and return its body.

    Raises :class:`PolicyProtocolError` on a version mismatch, an ``error`` reply,
    or (when ``expect`` is given) an unexpected message type, so callers never
    have to branch on failure shapes themselves.
    """
    if not payload:
        raise PolicyProtocolError("Empty payload.")
    try:
        envelope = msgpack.unpackb(payload, object_hook=_unpack_hook, raw=False)
    except PolicyProtocolError:
        raise
    except Exception as exc:  # msgpack raises a zoo of types
        raise PolicyProtocolError(f"Undecodable payload: {exc}") from exc

    if not isinstance(envelope, dict):
        raise PolicyProtocolError(f"Expected a message envelope, got {type(envelope).__name__}.")

    version = envelope.get("v")
    if version != PROTOCOL_VERSION:
        raise PolicyProtocolError(
            f"Policy protocol version mismatch: peer speaks v{version}, "
            f"this build speaks v{PROTOCOL_VERSION}.",
            code="version_mismatch",
        )

    msg_type = envelope.get("type")
    body = envelope.get("body") or {}
    if msg_type == "error":
        raise PolicyProtocolError(
            str(body.get("message", "unspecified policy service error")),
            code=str(body.get("code", "error")),
        )
    if expect is not None and msg_type != expect:
        raise PolicyProtocolError(f"Expected a '{expect}' reply, got '{msg_type}'.")
    return body
