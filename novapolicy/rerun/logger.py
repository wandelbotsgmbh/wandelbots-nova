"""Main PolicyRerunLogger — thin coordinator delegating to submodules.

Initializes DH robots / visualizers, then delegates per-step logging
to focused submodules (observation, action_chunk, streaming, images).
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.rerun.streaming import StateStreamer
    from novapolicy.types import ActionChunk

logger = logging.getLogger(__name__)


class PolicyRerunLogger:
    """Logs policy execution data to Rerun.

    Uses RobotVisualizer from nova_rerun_bridge for 3D mesh rendering,
    and DH FK for action chunk TCP path visualization (no network calls).
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        camera_names: list[str] | None = None,
    ) -> None:
        self._motion_groups = motion_groups
        self._camera_names = camera_names or []
        self._dh_robots: dict[str, Any] = {}
        self._tcp_offsets: dict[str, Any] = {}  # mg_id -> 4x4 flange->TCP matrix
        self._visualizers: dict[str, Any] = {}  # mg_id -> RobotVisualizer
        self._initialized = False
        self._start_time: float = 0.0
        self._tcp_trail: dict[str, list[list[float]]] = {}  # mg_id -> [[x,y,z], ...]
        self._tcp_target_trail: dict[str, list[list[float]]] = {}
        self._max_trail_points = 500
        self._streamer: StateStreamer | None = None

    async def initialize(self) -> None:
        """Fetch DH parameters, create robot visualizers, and send blueprint."""
        try:
            from nova_rerun_bridge.dh_robot import DHRobot  # noqa: PLC0415
            from nova_rerun_bridge.model_loader import load_model_data  # noqa: PLC0415
            from nova_rerun_bridge.robot_visualizer import RobotVisualizer  # noqa: PLC0415

            import rerun as rr  # noqa: PLC0415
        except ImportError:
            logger.warning("rerun or nova_rerun_bridge not available — visualization disabled")
            return

        try:
            from nova import api  # noqa: PLC0415
            from novapolicy._sdk import get_api_gateway  # noqa: PLC0415
            from novapolicy.rerun.blueprint import send_blueprint  # noqa: PLC0415

            self._start_time = time.monotonic()

            for mg in self._motion_groups:
                description = await mg.get_description()
                model = await mg.get_model()
                model_data = await load_model_data(model, get_api_gateway(mg))
                mounting = description.mounting or api.models.Pose(
                    position=api.models.Vector3d([0, 0, 0]),
                    orientation=api.models.RotationVector([0, 0, 0]),
                )
                dh_params = description.dh_parameters or []
                dh_robot = DHRobot(dh_parameters=dh_params, mounting=mounting)
                self._dh_robots[mg.id] = dh_robot

                # Configured flange->TCP offset for the active TCP, so joint
                # action chunks can be drawn at the TCP (matching the live TCP
                # trail) instead of at the flange. Best-effort; falls back to
                # the flange if the TCP can't be resolved.
                try:
                    from novapolicy.rerun.kinematics import tcp_offset_matrix  # noqa: PLC0415

                    active_tcp = await mg.active_tcp_name()
                    if active_tcp is not None:
                        off = await mg.tcp_offset(active_tcp)
                        self._tcp_offsets[mg.id] = tcp_offset_matrix(off)
                except (OSError, RuntimeError, ValueError, TypeError, KeyError) as e:
                    logger.debug("TCP offset query failed for %s: %s", mg.id, e)

                # TCP geometries for visualization
                tcp_geometries: dict[str, api.models.Collider] = {}
                if description.safety_tool_colliders is not None:
                    for colliders in description.safety_tool_colliders.values():
                        if colliders is not None:
                            tcp_geometries = dict(colliders.root)
                            break

                # Safety link chain geometries
                robot_model_geometries: list[api.models.LinkChain] = []
                if description.safety_link_colliders is not None:
                    robot_model_geometries = [
                        api.models.LinkChain(
                            [
                                api.models.Link(link.root)
                                for link in description.safety_link_colliders
                            ]
                        )
                    ]

                self._visualizers[mg.id] = RobotVisualizer(
                    robot=dh_robot,
                    robot_model_geometries=robot_model_geometries,
                    tcp_geometries=tcp_geometries,
                    static_transform=False,
                    base_entity_path=mg.id,
                    albedo_factor=[0, 255, 100],
                    model_data=model_data,
                )
                self._tcp_trail[mg.id] = []
                self._tcp_target_trail[mg.id] = []

            send_blueprint([mg.id for mg in self._motion_groups], self._camera_names)

            # Set coordinate convention: NOVA uses right-handed Z-up
            rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            rr.log(
                "policy/status",
                rr.TextLog("Policy execution started", level=rr.TextLogLevel.INFO),
            )
            self._initialized = True
            logger.info(
                "PolicyRerunLogger initialized for %d motion groups, %d cameras",
                len(self._motion_groups),
                len(self._camera_names),
            )
        except (OSError, RuntimeError, ValueError) as e:
            logger.warning("PolicyRerunLogger initialization failed: %s", e)

    # ------------------------------------------------------------------
    # Per-step logging
    # ------------------------------------------------------------------

    def log_observation(self, states: dict[str, RobotState], step: int) -> None:
        """Log robot state: update 3D mesh positions, joint scalars, TCP trail."""
        if not self._initialized:
            return
        try:
            from novapolicy.rerun.observation import log_observation  # noqa: PLC0415

            log_observation(
                states,
                step,
                start_time=self._start_time,
                dh_robots=self._dh_robots,
                visualizers=self._visualizers,
                tcp_trail=self._tcp_trail,
                max_trail_points=self._max_trail_points,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_observation error: %s", e)

    def log_action_chunk(self, chunk: ActionChunk, step: int, *, n_action_steps: int = 0) -> None:
        """Log action chunk as TCP path line strips and inspectable text."""
        if not self._initialized:
            return
        try:
            from novapolicy.rerun.action_chunk import log_action_chunk  # noqa: PLC0415

            log_action_chunk(
                chunk,
                step,
                start_time=self._start_time,
                dh_robots=self._dh_robots,
                tcp_offsets=self._tcp_offsets,
                n_action_steps=n_action_steps,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_action_chunk error: %s", e)

    def log_target_tracking(
        self, chunk: ActionChunk, states: dict[str, RobotState], step: int
    ) -> None:
        """Log first commanded target in ``chunk`` against the latest actual state."""
        if not self._initialized:
            return
        for mg_id, steps in chunk.joints.items():
            state = states.get(mg_id)
            if steps and state is not None:
                self.log_joint_tracking(mg_id, steps[0], state, step)
        for mg_id, steps in chunk.tcp.items():
            state = states.get(mg_id)
            if steps and state is not None:
                self.log_tcp_tracking(mg_id, steps[0], state, step)

    def log_joint_tracking(
        self, mg_id: str, target: list[float], actual: RobotState, step: int
    ) -> None:
        """Log commanded/actual joint positions and tracking error."""
        if not self._initialized:
            return
        try:
            from novapolicy.rerun.target_tracking import log_joint_tracking  # noqa: PLC0415

            log_joint_tracking(
                mg_id,
                target,
                list(actual.joints),
                step,
                start_time=self._start_time,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_joint_tracking error: %s", e)

    def log_tcp_tracking(
        self, mg_id: str, target: list[float], actual: RobotState, step: int
    ) -> None:
        """Log commanded/actual TCP pose and tracking error."""
        if not self._initialized:
            return
        pose = getattr(actual, "pose", None)
        if pose is None:
            return
        try:
            from novapolicy.rerun.target_tracking import log_tcp_tracking  # noqa: PLC0415

            log_tcp_tracking(
                mg_id,
                target,
                pose,
                step,
                start_time=self._start_time,
                target_trail=self._tcp_target_trail.get(mg_id),
                max_trail_points=self._max_trail_points,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_tcp_tracking error: %s", e)

    def log_images(self, images: dict[str, Any]) -> None:
        """Log camera images to Rerun."""
        if not self._initialized or not images:
            return
        try:
            from novapolicy.rerun.images import log_images  # noqa: PLC0415

            log_images(images, start_time=self._start_time)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_images error: %s", e)

    def log_completion(self, reason: str, steps: int, duration_s: float) -> None:
        """Log execution completion."""
        if not self._initialized:
            return
        try:
            import rerun as rr  # noqa: PLC0415

            rr.log(
                "policy/status",
                rr.TextLog(
                    f"Policy finished: {reason} ({steps} steps, {duration_s:.1f}s)",
                    level=rr.TextLogLevel.INFO,
                ),
            )
        except (ImportError, OSError, RuntimeError) as e:
            # Completion logging is best-effort; never let it break execution.
            logger.debug("log_completion failed: %s", e)

    # ------------------------------------------------------------------
    # Continuous state streaming (between policy steps)
    # ------------------------------------------------------------------

    def start_streaming(self, sessions: dict[str, Any]) -> None:
        """Start background task logging robot state at ~100Hz between policy calls."""
        if not self._initialized:
            return
        from novapolicy.rerun.streaming import StateStreamer  # noqa: PLC0415

        self._streamer = StateStreamer(
            start_time=self._start_time,
            dh_robots=self._dh_robots,
            visualizers=self._visualizers,
            tcp_trail=self._tcp_trail,
            max_trail_points=self._max_trail_points,
        )
        self._streamer.start(sessions)

    async def stop_streaming(self) -> None:
        """Stop the background state streaming task."""
        if self._streamer is not None:
            await self._streamer.stop()
            self._streamer = None
