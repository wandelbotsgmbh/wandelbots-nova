from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path
import time
from typing import TYPE_CHECKING

from huggingface_hub import hf_hub_download
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.utils.import_utils import register_third_party_plugins
import torch

from .observation_sources import MockObservationSource

if TYPE_CHECKING:
    from .observation_sources import ObservationSource

logger = logging.getLogger(__name__)


class LeRobotInferenceEngine:
    """Inference-only engine based on LeRobot async/policy-server loading flow."""

    def __init__(self) -> None:
        self._loaded_policy_path: str | None = None
        self._loaded_device: str | None = None
        self._policy: object | None = None
        self._preprocessor: object | None = None
        self._postprocessor: object | None = None
        self._policy_config: object | None = None
        self._control_fps: float | None = None
        self._observation_source: ObservationSource | None = None

    @property
    def loaded_policy_path(self) -> str | None:
        return self._loaded_policy_path

    @property
    def control_fps(self) -> float:
        if self._control_fps is None:
            raise RuntimeError("Control FPS is not available before policy is loaded")
        return self._control_fps

    async def ensure_loaded(self, policy_path: str, device: str) -> None:
        if self._loaded_policy_path == policy_path and self._loaded_device == device:
            return
        await asyncio.to_thread(self._load_sync, policy_path, device)

    async def warmup(self, *, task: str | None = None) -> tuple[float, list[float] | None]:
        return await asyncio.to_thread(self._infer_once_sync, task)

    async def infer_once(self, *, task: str | None = None) -> tuple[float, list[float] | None]:
        return await asyncio.to_thread(self._infer_once_sync, task)

    def _load_sync(self, policy_path: str, device: str) -> None:
        logger.info("Loading LeRobot policy: path=%s device=%s", policy_path, device)

        register_third_party_plugins()

        def _noop_init_hydra_config(*_args: object, **_kwargs: object) -> None:
            return None

        parser.init_hydra_config = _noop_init_hydra_config

        cfg = PreTrainedConfig.from_pretrained(policy_path)
        cfg.device = self._resolve_runtime_device(device)

        policy_cls = get_policy_class(cfg.type)
        policy = policy_cls.from_pretrained(policy_path)
        policy.to(cfg.device)
        policy.eval()

        overrides = {"device_processor": {"device": cfg.device}}
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy.config,
            pretrained_path=policy_path,
            preprocessor_overrides=overrides,
            postprocessor_overrides=overrides,
        )

        if hasattr(policy, "reset"):
            policy.reset()
        if hasattr(preprocessor, "reset"):
            preprocessor.reset()
        if hasattr(postprocessor, "reset"):
            postprocessor.reset()

        input_features = getattr(policy.config, "input_features", None)
        if not isinstance(input_features, dict) or len(input_features) == 0:
            raise RuntimeError("Policy config has no input_features")

        self._policy = policy
        self._preprocessor = preprocessor
        self._postprocessor = postprocessor
        self._policy_config = policy.config
        self._control_fps = self._resolve_control_fps(policy_path)
        self._loaded_policy_path = policy_path
        self._loaded_device = cfg.device
        self._observation_source = MockObservationSource(input_features=input_features)

        logger.info("LeRobot policy loaded successfully: %s", policy_path)

    def _infer_once_sync(self, task: str | None) -> tuple[float, list[float] | None]:
        if (
            self._policy is None
            or self._preprocessor is None
            or self._postprocessor is None
            or self._observation_source is None
        ):
            raise RuntimeError("Policy must be loaded before inference")

        observation = self._observation_source.next_observation(task=task)

        start = time.perf_counter()
        with torch.inference_mode():
            model_input = self._preprocessor(observation)
            action = self._policy.select_action(model_input)
            action = self._postprocessor(action)

        joint_values = self._extract_joint_values(action)
        if joint_values is not None:
            logger.info("Policy action joints=%s", [round(v, 5) for v in joint_values])

        return (time.perf_counter() - start) * 1000.0, joint_values

    @staticmethod
    def _extract_joint_values(action: object) -> list[float] | None:
        if isinstance(action, torch.Tensor):
            tensor = action.detach().to("cpu")
            if tensor.ndim > 1 and tensor.shape[0] >= 1:
                return [float(v) for v in tensor[0].tolist()]
            if tensor.ndim == 1:
                return [float(v) for v in tensor.tolist()]
        return None

    @staticmethod
    def _resolve_runtime_device(requested_device: str) -> str:
        if requested_device == "cuda" and not torch.cuda.is_available():
            if torch.backends.mps.is_available():
                logger.warning("Requested cuda device unavailable; falling back to mps")
                return "mps"
            logger.warning("Requested cuda device unavailable; falling back to cpu")
            return "cpu"
        return requested_device

    @staticmethod
    def _resolve_control_fps(policy_path: str) -> float:
        train_config_path = hf_hub_download(repo_id=policy_path, filename="train_config.json")
        train_config = json.loads(Path(train_config_path).read_text(encoding="utf-8"))

        dataset_config = train_config.get("dataset")
        if not isinstance(dataset_config, dict):
            raise ValueError("train_config.json missing dataset configuration")

        dataset_repo_id = dataset_config.get("repo_id")
        if not isinstance(dataset_repo_id, str) or not dataset_repo_id:
            raise ValueError("train_config.json dataset.repo_id must be a non-empty string")

        dataset_info_path = hf_hub_download(
            repo_id=dataset_repo_id,
            repo_type="dataset",
            filename="meta/info.json",
        )
        dataset_info = json.loads(Path(dataset_info_path).read_text(encoding="utf-8"))

        fps_value = dataset_info.get("fps")
        if isinstance(fps_value, (int, float)) and fps_value > 0:
            return float(fps_value)

        raise ValueError("dataset meta/info.json must contain a positive fps value")
