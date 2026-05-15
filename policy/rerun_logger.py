"""Rerun visualization for policy execution.

Visualizes in real-time:
- 3D robot meshes moving with actual joint positions (via RobotVisualizer)
- Action chunk targets as orange TCP line strips (via DH FK)
- Actual TCP trail as green path (tracking accuracy)
- Camera images in dedicated 2D panels
- Joint position timeseries

Usage: Pass ``viewer=nova.viewers.Rerun()`` to the ``@nova.program`` decorator.
Fully decoupled — zero overhead when no viewer is active.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.types import ActionChunk

logger = logging.getLogger(__name__)

_MIN_LINE_STEPS = 2
_MIN_TCP_COMPONENTS = 3
_TEMPORAL_FRAME_NDIM = 4
_TCP_TRAIL_COLOR = (50, 220, 100)  # green — actual TCP path
_ACTION_CHUNK_COLOR = (255, 100, 50)  # orange — target action chunk
_ACTION_TCP_COLOR = (50, 200, 255)  # cyan — TCP action targets


def _is_rerun_active() -> bool:
    """Check if a Rerun viewer is active."""
    try:
        from nova.viewers import get_viewer_manager  # noqa: PLC0415

        return get_viewer_manager().has_active_viewers
    except (ImportError, AttributeError):
        return False


class PolicyRerunLogger:
    """Logs policy execution data to Rerun.

    Uses RobotVisualizer from nova_rerun_bridge for 3D mesh rendering,
    and DH FK for action chunk TCP path visualization (no network calls).
    """

    def __init__(
        self, motion_groups: list[MotionGroup], camera_names: list[str] | None = None,
    ) -> None:
        self._motion_groups = motion_groups
        self._camera_names = camera_names or []
        self._dh_robots: dict[str, Any] = {}
        self._visualizers: dict[str, Any] = {}  # mg_id -> RobotVisualizer
        self._initialized = False
        self._start_time: float = 0.0
        self._tcp_trail: dict[str, list[list[float]]] = {}  # mg_id -> [[x,y,z], ...]
        self._max_trail_points = 500

    async def initialize(self) -> None:
        """Fetch DH parameters, create robot visualizers, and send blueprint."""
        try:
            from nova_rerun_bridge.dh_robot import DHRobot  # noqa: PLC0415
            from nova_rerun_bridge.robot_visualizer import RobotVisualizer  # noqa: PLC0415
            import rerun as rr  # noqa: PLC0415
        except ImportError:
            logger.warning("rerun or nova_rerun_bridge not available — visualization disabled")
            return

        try:
            from nova import api  # noqa: PLC0415

            self._start_time = time.monotonic()

            for mg in self._motion_groups:
                description = await mg.get_description()
                model = await mg.get_model()
                mounting = description.mounting or api.models.Pose(
                    position=api.models.Vector3d([0, 0, 0]),
                    orientation=api.models.RotationVector([0, 0, 0]),
                )
                dh_params = description.dh_parameters or []
                dh_robot = DHRobot(dh_parameters=dh_params, mounting=mounting)
                self._dh_robots[mg.id] = dh_robot

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
                            [api.models.Link(link.root) for link in description.safety_link_colliders]
                        )
                    ]

                self._visualizers[mg.id] = RobotVisualizer(
                    robot=dh_robot,
                    robot_model_geometries=robot_model_geometries,
                    tcp_geometries=tcp_geometries,
                    static_transform=False,
                    base_entity_path=mg.id,
                    albedo_factor=[0, 255, 100],
                    motion_group_model=model,
                )
                self._tcp_trail[mg.id] = []

            self._send_blueprint()

            rr.log(
                "policy/status",
                rr.TextLog("Policy execution started", level=rr.TextLogLevel.INFO),
            )
            self._initialized = True
            logger.info(
                "PolicyRerunLogger initialized for %d motion groups, %d cameras",
                len(self._motion_groups), len(self._camera_names),
            )
        except (OSError, RuntimeError, ValueError) as e:
            logger.warning("PolicyRerunLogger initialization failed: %s", e)

    def _send_blueprint(self) -> None:
        """Send a Rerun blueprint with 3D view, camera panels, and joint plots."""
        import rerun as rr  # noqa: PLC0415
        import rerun.blueprint as rrb  # noqa: PLC0415

        mg_ids = [mg.id for mg in self._motion_groups]

        # 3D view contents: robot meshes + policy overlays
        contents_3d = ["coordinate_system_world/**", "motion/**", "collision_scenes/**"]
        for mg_id in mg_ids:
            contents_3d.extend([f"{mg_id}/**", f"policy/{mg_id}/**"])

        views: list[Any] = [
            rrb.Spatial3DView(contents=contents_3d, name="3D View", background=[20, 22, 35]),
        ]

        camera_views = [
            rrb.Spatial2DView(contents=[f"policy/cameras/{n}"], name=n)
            for n in self._camera_names
        ]
        joint_views = [
            rrb.TimeSeriesView(contents=[f"policy/{mg_id}/joints/**"], name=f"Joints {mg_id}")
            for mg_id in mg_ids
        ]

        right_panels: list[Any] = []
        if camera_views:
            right_panels.append(rrb.Grid(*camera_views))
        if joint_views:
            right_panels.append(rrb.Vertical(*joint_views))

        layout = (
            rrb.Horizontal(views[0], rrb.Vertical(*right_panels), column_shares=[3, 2])
            if right_panels
            else views[0]
        )
        rr.send_blueprint(rrb.Blueprint(layout, collapse_panels=True))

    # ------------------------------------------------------------------
    # Per-step logging
    # ------------------------------------------------------------------

    def log_observation(self, states: dict[str, RobotState], step: int) -> None:
        """Log robot state: update 3D mesh positions, joint scalars, TCP trail."""
        if not self._initialized:
            return
        try:
            self._log_observation_impl(states, step)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_observation error: %s", e)

    def _log_observation_impl(self, states: dict[str, RobotState], step: int) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)
        rr.set_time("policy_step", sequence=step)

        for mg_id, state in states.items():
            if not hasattr(state, "joints"):
                continue
            joints = list(state.joints)

            # Joint timeseries
            for i, j in enumerate(joints):
                rr.log(f"policy/{mg_id}/joints/j{i}", rr.Scalars(j))

            # Update 3D robot mesh
            visualizer = self._visualizers.get(mg_id)
            if visualizer is not None:
                visualizer.log_robot_geometry(joint_position=joints)

            # TCP trail (actual path in green)
            dh_robot = self._dh_robots.get(mg_id)
            if dh_robot is not None:
                positions = dh_robot.calculate_joint_positions(joints)
                tcp_pos = positions[-1]
                trail = self._tcp_trail[mg_id]
                trail.append(tcp_pos)
                if len(trail) > self._max_trail_points:
                    trail.pop(0)
                if len(trail) >= _MIN_LINE_STEPS:
                    rr.log(
                        f"policy/{mg_id}/tcp_trail",
                        rr.LineStrips3D([trail], colors=[_TCP_TRAIL_COLOR], radii=1.0),
                    )
                rr.log(
                    f"policy/{mg_id}/tcp",
                    rr.Points3D([tcp_pos], colors=[_TCP_TRAIL_COLOR], radii=4.0),
                )

    def log_action_chunk(self, chunk: ActionChunk, step: int) -> None:
        """Log action chunk as TCP path line strips (replaced each frame)."""
        if not self._initialized:
            return
        try:
            self._log_joint_chunk(chunk, step)
            self._log_tcp_chunk(chunk, step)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_action_chunk error: %s", e)

    def _log_joint_chunk(self, chunk: ActionChunk, step: int) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)
        rr.set_time("policy_step", sequence=step)

        for mg_id, steps in chunk.joints.items():
            dh_robot = self._dh_robots.get(mg_id)
            if dh_robot is None or not steps:
                continue

            tcp_positions = [
                dh_robot.calculate_joint_positions(t)[-1] for t in steps
            ]

            if len(tcp_positions) >= _MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/action_chunk",
                    rr.LineStrips3D([tcp_positions], colors=[_ACTION_CHUNK_COLOR], radii=1.5),
                )
            elif tcp_positions:
                rr.log(
                    f"policy/{mg_id}/action_chunk",
                    rr.Points3D(tcp_positions, colors=[_ACTION_CHUNK_COLOR], radii=2.0),
                )

    def _log_tcp_chunk(self, chunk: ActionChunk, step: int) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)
        rr.set_time("policy_step", sequence=step)

        for mg_id, steps in chunk.tcp.items():
            if not steps:
                continue
            tcp_positions = [
                [s[0], s[1], s[2]] for s in steps if len(s) >= _MIN_TCP_COMPONENTS
            ]
            if len(tcp_positions) >= _MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/action_chunk_tcp",
                    rr.LineStrips3D([tcp_positions], colors=[_ACTION_TCP_COLOR], radii=1.5),
                )

    def log_images(self, images: dict[str, Any]) -> None:
        """Log camera images to Rerun."""
        if not self._initialized or not images:
            return
        try:
            self._log_images_impl(images)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_images error: %s", e)

    def _log_images_impl(self, images: dict[str, Any]) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)

        for name, frame in images.items():
            if frame is None:
                continue
            if hasattr(frame, "ndim"):
                img = frame[-1] if frame.ndim == _TEMPORAL_FRAME_NDIM else frame
                rr.log(f"policy/cameras/{name}", rr.Image(img))

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
        except (ImportError, OSError, RuntimeError):
            pass
