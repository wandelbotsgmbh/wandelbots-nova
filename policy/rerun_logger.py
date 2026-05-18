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
_ACTION_TCP_COLOR = (50, 200, 255)  # cyan — TCP action targets

# Action chunk gradient: orange (start) → yellow (end)
_CHUNK_COLOR_START = (255, 80, 20)
_CHUNK_COLOR_END = (255, 240, 60)

# Direction arrow at each waypoint
_DIRECTION_ARROW_LENGTH = 15.0  # mm
_DIRECTION_ARROW_COLOR = (255, 160, 40)  # orange
_DIRECTION_ARROW_WIDTH = 1.5

# Screen-space line widths (UI points, zoom-independent)
_TRAIL_WIDTH_UI = 2.0
_CHUNK_WIDTH_UI = 3.0

# Discarded chunk tail: dim gray (predicted but not executed)
_CHUNK_TAIL_COLOR = (100, 100, 100)
_CHUNK_TAIL_WIDTH_UI = 1.5


def _lerp_color(
    start: tuple[int, int, int], end: tuple[int, int, int], t: float
) -> tuple[int, int, int]:
    """Linearly interpolate between two RGB colors. t in [0, 1]."""
    return (
        int(start[0] + (end[0] - start[0]) * t),
        int(start[1] + (end[1] - start[1]) * t),
        int(start[2] + (end[2] - start[2]) * t),
    )


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

            # Set coordinate convention: NOVA uses right-handed Z-up (standard robotics)
            rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

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
        rr.send_blueprint(rrb.Blueprint(
            layout,
            rrb.TimePanel(state="expanded", timeline="policy_time"),
            collapse_panels=True,
        ))

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

            # TCP trail (actual path in green, screen-space width)
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
                        rr.LineStrips3D(
                            [trail],
                            colors=[_TCP_TRAIL_COLOR],
                            radii=rr.components.Radius.ui_points(_TRAIL_WIDTH_UI),
                        ),
                    )
                rr.log(
                    f"policy/{mg_id}/tcp",
                    rr.Points3D(
                        [tcp_pos],
                        colors=[_TCP_TRAIL_COLOR],
                        radii=rr.components.Radius.ui_points(4.0),
                    ),
                )

    def log_action_chunk(self, chunk: ActionChunk, step: int, *, n_action_steps: int = 0) -> None:
        """Log action chunk as TCP path line strips (replaced each frame).

        Args:
            chunk: Full predicted action chunk from the policy.
            step: Current execution step.
            n_action_steps: If >0, steps beyond this index are rendered in a
                dim color to indicate they are predicted but not executed
                (receding horizon visualization).
        """
        if not self._initialized:
            return
        try:
            self._log_joint_chunk(chunk, step, n_action_steps)
            self._log_tcp_chunk(chunk, step, n_action_steps)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_action_chunk error: %s", e)

    def _log_joint_chunk(self, chunk: ActionChunk, step: int, n_action_steps: int = 0) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)
        rr.set_time("policy_step", sequence=step)

        for mg_id, steps in chunk.joints.items():
            dh_robot = self._dh_robots.get(mg_id)
            if dh_robot is None or not steps:
                continue

            # Split into executed and discarded portions
            split = n_action_steps if 0 < n_action_steps < len(steps) else len(steps)
            executed_steps = steps[:split]
            discarded_steps = steps[split:]

            # Compute TCP positions for executed steps
            executed_positions = []
            for joint_target in executed_steps:
                positions = dh_robot.calculate_joint_positions(joint_target)
                executed_positions.append(positions[-1])

            # Log executed portion with orange→yellow gradient
            self._log_line_strip(
                f"policy/{mg_id}/action_chunk", executed_positions,
                gradient=True, width=_CHUNK_WIDTH_UI,
            )

            # Log discarded tail in dim gray
            self._log_discarded_tail(
                f"policy/{mg_id}/action_chunk_tail",
                discarded_steps, dh_robot, executed_positions,
            )

            # Log direction arrows showing travel direction at each waypoint
            if len(executed_positions) >= _MIN_LINE_STEPS:
                self._log_direction_arrows(mg_id, executed_positions)

    def _log_tcp_chunk(self, chunk: ActionChunk, step: int, n_action_steps: int = 0) -> None:
        import rerun as rr  # noqa: PLC0415

        elapsed = time.monotonic() - self._start_time
        rr.set_time("policy_time", duration=elapsed)
        rr.set_time("policy_step", sequence=step)

        for mg_id, steps in chunk.tcp.items():
            if not steps:
                continue

            split = n_action_steps if 0 < n_action_steps < len(steps) else len(steps)
            executed = steps[:split]
            discarded = steps[split:]

            executed_positions = [
                [s[0], s[1], s[2]] for s in executed if len(s) >= _MIN_TCP_COMPONENTS
            ]
            if len(executed_positions) >= _MIN_LINE_STEPS:
                n = len(executed_positions)
                colors = [
                    _lerp_color(_CHUNK_COLOR_START, _CHUNK_COLOR_END, i / max(n - 1, 1))
                    for i in range(n)
                ]
                rr.log(
                    f"policy/{mg_id}/action_chunk_tcp",
                    rr.LineStrips3D(
                        [executed_positions],
                        colors=colors,
                        radii=rr.components.Radius.ui_points(_CHUNK_WIDTH_UI),
                    ),
                )

            if discarded:
                tail_positions = [
                    [s[0], s[1], s[2]] for s in discarded if len(s) >= _MIN_TCP_COMPONENTS
                ]
                if executed_positions:
                    tail_positions = [executed_positions[-1], *tail_positions]
                if len(tail_positions) >= _MIN_LINE_STEPS:
                    rr.log(
                        f"policy/{mg_id}/action_chunk_tcp_tail",
                        rr.LineStrips3D(
                            [tail_positions],
                            colors=[_CHUNK_TAIL_COLOR],
                            radii=rr.components.Radius.ui_points(_CHUNK_TAIL_WIDTH_UI),
                        ),
                    )
            else:
                rr.log(f"policy/{mg_id}/action_chunk_tcp_tail", rr.Clear(recursive=False))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _log_line_strip(
        entity_path: str, positions: list[list[float]], *, gradient: bool, width: float,
    ) -> None:
        """Log a line strip with gradient or uniform color."""
        import rerun as rr  # noqa: PLC0415

        if len(positions) >= _MIN_LINE_STEPS:
            n = len(positions)
            colors = (
                [_lerp_color(_CHUNK_COLOR_START, _CHUNK_COLOR_END, i / max(n - 1, 1)) for i in range(n)]
                if gradient
                else [_CHUNK_COLOR_START] * n
            )
            rr.log(entity_path, rr.LineStrips3D(
                [positions], colors=colors, radii=rr.components.Radius.ui_points(width),
            ))
        elif positions:
            rr.log(entity_path, rr.Points3D(
                positions, colors=[_CHUNK_COLOR_START], radii=rr.components.Radius.ui_points(4.0),
            ))

    def _log_discarded_tail(
        self, entity_path: str, discarded_steps: list[list[float]],
        dh_robot: object, bridge_from: list[list[float]],
    ) -> None:
        """Log discarded chunk tail in dim gray, connected from last executed point."""
        import rerun as rr  # noqa: PLC0415

        if not discarded_steps:
            rr.log(entity_path, rr.Clear(recursive=False))
            return

        tail_positions = [
            dh_robot.calculate_joint_positions(jt)[-1]  # type: ignore[attr-defined]
            for jt in discarded_steps
        ]
        # Bridge from last executed point for visual continuity
        if bridge_from:
            tail_positions = [bridge_from[-1], *tail_positions]
        if len(tail_positions) >= _MIN_LINE_STEPS:
            rr.log(entity_path, rr.LineStrips3D(
                [tail_positions],
                colors=[_CHUNK_TAIL_COLOR],
                radii=rr.components.Radius.ui_points(_CHUNK_TAIL_WIDTH_UI),
            ))

    @staticmethod
    def _log_direction_arrows(mg_id: str, positions: list[list[float]]) -> None:
        """Log arrows showing travel direction at each waypoint.

        Uses central differences for interior points and forward/backward
        differences at the endpoints. Arrow length is fixed for visibility.
        """
        import math  # noqa: PLC0415

        import rerun as rr  # noqa: PLC0415

        if len(positions) < _MIN_LINE_STEPS:
            return

        origins = []
        vectors = []
        n = len(positions)

        for i in range(n):
            # Compute tangent direction via finite differences
            if i == 0:
                dx = [positions[1][j] - positions[0][j] for j in range(3)]
            elif i == n - 1:
                dx = [positions[n - 1][j] - positions[n - 2][j] for j in range(3)]
            else:
                dx = [(positions[i + 1][j] - positions[i - 1][j]) / 2.0 for j in range(3)]

            # Normalize and scale to fixed arrow length
            magnitude = math.sqrt(sum(d * d for d in dx))
            if magnitude < 1e-6:  # noqa: PLR2004
                continue
            scale = _DIRECTION_ARROW_LENGTH / magnitude
            origins.append(positions[i])
            vectors.append([d * scale for d in dx])

        if origins:
            rr.log(
                f"policy/{mg_id}/chunk_direction",
                rr.Arrows3D(
                    origins=origins,
                    vectors=vectors,
                    colors=[_DIRECTION_ARROW_COLOR],
                    radii=rr.components.Radius.ui_points(_DIRECTION_ARROW_WIDTH),
                ),
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
