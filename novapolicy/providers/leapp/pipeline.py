"""Run a LEAPP export (YAML beside its ONNX models) with ``onnxruntime`` and numpy, without torch.

As LEAPP's ``InferenceManager``: models in YAML order, each output copied to the inputs it feeds
(``data_flow``) and to the next step's (``feedback_flow``).
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import threading
from typing import Any

import numpy as np
import onnxruntime as ort
from safetensors.numpy import load_file
import yaml

_DTYPES = {
    "float32": np.float32,
    "float64": np.float64,
    "float16": np.float16,
    "int64": np.int64,
    "int32": np.int32,
    "bool": np.bool_,
}


@dataclass(frozen=True)
class Port:
    """One input or output of a model in the export."""

    node: str
    name: str
    shape: tuple[int, ...]
    dtype: str
    kind: str | None = None
    connection: str | None = None  # Isaac Lab's isaaclab_connection, e.g. "state:robot:joint_pos"

    @property
    def key(self) -> str:
        """``node/name``, as LEAPP names ports."""
        return f"{self.node}/{self.name}"

    @property
    def size(self) -> int:
        return int(np.prod(self.shape)) if self.shape else 1

    @property
    def connection_name(self) -> str | None:
        """The last part of the Isaac Lab connection: ``joint_pos``, ``ee_pose``."""
        return self.connection.rsplit(":", 1)[-1] if self.connection else None


def _port(node: str, description: dict[str, Any]) -> Port:
    shape = description["shape"]
    if isinstance(shape, str):
        shape = yaml.safe_load(shape)
    return Port(
        node=node,
        name=description["name"],
        shape=tuple(int(d) for d in shape),
        dtype=str(description["dtype"]),
        kind=description.get("kind"),
        connection=description.get("isaaclab_connection"),
    )


@dataclass
class _Model:
    name: str
    inputs: list[Port]
    outputs: list[Port]
    session: ort.InferenceSession


class LeappPipeline:
    """A LEAPP export, loaded and ready to run step by step."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        if self.path.suffix != ".yaml":
            msg = f"A LEAPP export is described by its .yaml file, got {self.path}"
            raise ValueError(msg)
        with self.path.open() as f:
            documents = [d for d in yaml.safe_load_all(f) if d]
        description: dict[str, Any] = {}
        for document in documents:
            description.update(document)
        missing = {"models", "pipeline"} - set(description)
        if missing:
            msg = f"{self.path} is not a LEAPP export: no {sorted(missing)}"
            raise ValueError(msg)
        pipeline = description["pipeline"]
        self.frequency: float | None = (pipeline.get("configs") or {}).get("frequency")
        self._models = [
            self._load_model(name, model) for name, model in description["models"].items()
        ]
        by_name = {m.name: m for m in self._models}
        self.inputs = [
            next(p for p in by_name[node].inputs if p.name == name)
            for node, names in pipeline["inputs"].items()
            for name in names
        ]
        self.outputs = [
            next(p for p in by_name[node].outputs if p.name == name)
            for node, names in pipeline["outputs"].items()
            for name in names
        ]
        self._flows: dict[str, list[str]] = {}
        for flows in (pipeline.get("data_flow") or {}, pipeline.get("feedback_flow") or {}):
            for source, targets in flows.items():
                self._flows.setdefault(source, []).extend(targets)
        self._initial_values = pipeline.get("initial_values")
        self._buffers: dict[str, np.ndarray] = {}
        self._lock = threading.Lock()  # run() may be called from a worker thread
        self.reset()

    def _load_model(self, name: str, model: dict[str, Any]) -> _Model:
        parameters = model["parameters"]
        if parameters.get("backend") != "onnx":
            msg = (
                f"Model {name!r} of {self.path} is exported as {parameters.get('backend')!r}; "
                "only ONNX exports run without torch. Export with the ONNX backend."
            )
            raise ValueError(msg)
        model_path = self.path.parent / parameters["model_path"]
        expected = parameters.get("sha256sum")
        if expected and hashlib.sha256(model_path.read_bytes()).hexdigest() != expected:
            msg = f"{model_path} does not match the sha256sum in {self.path}"
            raise ValueError(msg)
        session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        return _Model(
            name=name,
            inputs=[_port(name, d) for d in model["inputs"]],
            outputs=[_port(name, d) for d in model["outputs"]],
            session=session,
        )

    def reset(self) -> None:
        """Zero every input and load the feedback inputs' initial values: a new episode."""
        with self._lock:
            self._reset()

    def _reset(self) -> None:
        self._buffers = {
            port.key: np.zeros(port.shape, dtype=_DTYPES.get(port.dtype, np.float32))
            for model in self._models
            for port in model.inputs
        }
        if self._initial_values:
            for key, value in load_file(str(self.path.parent / self._initial_values)).items():
                if key not in self._buffers:
                    msg = f"Initial value {key!r} in {self.path} matches no model input"
                    raise ValueError(msg)
                self._buffers[key] = value

    def run(self, inputs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """One step: ``inputs`` by ``node/name``; returns the pipeline outputs by ``node/name``."""
        with self._lock:
            return self._run(inputs)

    def _run(self, inputs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        for key, value in inputs.items():
            if key not in self._buffers:
                msg = f"{key!r} is not an input of {self.path}"
                raise KeyError(msg)
            self._buffers[key] = value
        results: dict[str, np.ndarray] = {}
        for model in self._models:
            feed = {port.name: self._buffers[port.key] for port in model.inputs}
            values = model.session.run([port.name for port in model.outputs], feed)
            for port, raw in zip(model.outputs, values, strict=True):
                value = np.asarray(raw)
                results[port.key] = value
                for target in self._flows.get(port.key, []):
                    self._buffers[target] = value
        return {port.key: results[port.key] for port in self.outputs}
