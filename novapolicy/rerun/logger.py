"""Main PolicyRerunLogger — thin coordinator delegating to submodules.

Initializes DH robots / visualizers, then delegates per-step logging
to focused submodules (observation, action_chunk, streaming, images).
"""

from __future__ import annotations

import asyncio
import contextlib
from datetime import UTC, datetime, timedelta
import logging
import threading
import time
from typing import TYPE_CHECKING, Any
from uuid import uuid4

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.rerun.streaming import StateStreamer
    from novapolicy.types import ActionChunk
    from rerun import RecordingStream

from novapolicy.rerun.logtime import pinned_elapsed
from novapolicy.rerun.sink import LOG_ERRORS, AsyncLogSink

logger = logging.getLogger(__name__)

_RERUN_DISCONNECT_TIMEOUT_S = 1.0


async def _disconnect_recording(recording: RecordingStream) -> None:
    """Disconnect Rerun without allowing its synchronous client to block shutdown."""
    loop = asyncio.get_running_loop()
    completed: asyncio.Future[Exception | None] = loop.create_future()

    def disconnect() -> None:
        error: Exception | None = None
        try:
            recording.disconnect()
        except Exception as e:  # ruff: ignore[blind-except]
            error = e
        with contextlib.suppress(RuntimeError):
            loop.call_soon_threadsafe(set_result, error)

    def set_result(error: Exception | None) -> None:
        if not completed.done():
            completed.set_result(error)

    threading.Thread(target=disconnect, name="rerun-disconnect", daemon=True).start()
    try:
        error = await asyncio.wait_for(completed, timeout=_RERUN_DISCONNECT_TIMEOUT_S)
    except TimeoutError:
        logger.warning("Timed out disconnecting policy Rerun recording")
        return
    if error is not None:
        raise error


class PolicyRerunLogger:
    """Logs policy execution data to Rerun.

    Uses RobotVisualizer from nova_rerun_bridge for 3D mesh rendering,
    and DH FK for action chunk path visualization (no network calls). Policy
    data and its blueprint use a dedicated recording so NOVA planning can keep
    its own recording and blueprint.
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup],
        camera_names: list[str] | None = None,
        *,
        use_tcp_offset_for_joint_actions: bool = False,
        state_sample_interval_ms: float | None = None,
    ) -> None:
        self._motion_groups = motion_groups
        self._camera_names = camera_names or []
        self._use_tcp_offset_for_joint_actions = use_tcp_offset_for_joint_actions
        if state_sample_interval_ms is None:
            from nova.viewers import (
                Rerun,
                get_viewer_manager,
            )

            viewer = get_viewer_manager().get_viewer(Rerun)
            state_sample_interval_ms = (
                viewer.state_sample_interval_ms if viewer is not None else 1000.0 / 30.0
            )
        self._state_sample_interval_ms = state_sample_interval_ms
        self._dh_robots: dict[str, Any] = {}
        self._tcp_offsets: dict[str, Any] = {}  # mg_id -> 4x4 flange->TCP matrix
        self._visualizers: dict[str, Any] = {}  # mg_id -> RobotVisualizer
        self._initialized = False
        self._start_time: float = 0.0
        self._start_wall: datetime | None = None
        self._tcp_trail: dict[str, list[list[float]]] = {}  # mg_id -> [[x,y,z], ...]
        self._tcp_target_trail: dict[str, list[list[float]]] = {}
        self._max_trail_points = 500
        self._streamer: StateStreamer | None = None
        self._recording: RecordingStream | None = None
        # Rerun writes are handed off rather than paid for on the caller's
        # thread: for a jogging control loop the caller is the loop producing
        # the next waypoint chunk, and it cannot afford them.
        self._sink = AsyncLogSink()

    async def initialize(self) -> None:  # ruff: ignore[complex-structure]
        """Fetch DH parameters, create robot visualizers, and send blueprint."""
        try:
            from nova_rerun_bridge.dh_robot import DHRobot
            from nova_rerun_bridge.model_loader import (
                load_model_data,
            )
            from nova_rerun_bridge.robot_visualizer import (
                RobotVisualizer,
            )

            import rerun as rr
        except ImportError:
            logger.warning("rerun or nova_rerun_bridge not available — visualization disabled")
            return

        try:  # ruff: ignore[too-many-nested-blocks, too-many-statements-in-try-clause]
            from nova import api
            from novapolicy._sdk import get_api_gateway
            from novapolicy.rerun.blueprint import (
                send_blueprint,
            )

            self._start_time = time.monotonic()
            self._recording = rr.RecordingStream(
                "novapolicy",
                recording_id=f"policy_{uuid4()}",
            )
            self._recording.connect_grpc()

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

                # Joint action chunks are drawn at the flange by default. A
                # policy schema with joint actions does not declare a TCP, so
                # silently applying the controller's active TCP offset can make
                # the action marker look displaced from what is actually sent.
                if self._use_tcp_offset_for_joint_actions:
                    try:
                        from novapolicy.rerun.kinematics import (
                            tcp_offset_matrix,
                        )

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
                        api.models.LinkChain([
                            api.models.Link(link.root) for link in description.safety_link_colliders
                        ])
                    ]

                self._visualizers[mg.id] = RobotVisualizer(
                    robot=dh_robot,
                    robot_model_geometries=robot_model_geometries,
                    tcp_geometries=tcp_geometries,
                    static_transform=False,
                    base_entity_path=mg.id,
                    albedo_factor=[0, 255, 100],
                    model_data=model_data,
                    recording=self._recording,
                )
                self._tcp_trail[mg.id] = []
                self._tcp_target_trail[mg.id] = []

            send_blueprint(
                [mg.id for mg in self._motion_groups],
                self._camera_names,
                recording=self._recording,
            )

            # Set coordinate convention: NOVA uses right-handed Z-up
            rr.log(
                "/",
                rr.ViewCoordinates.RIGHT_HAND_Z_UP,
                static=True,
                recording=self._recording,
            )
            rr.log(
                "policy/status",
                rr.TextLog("Policy execution started", level=rr.TextLogLevel.INFO),
                recording=self._recording,
            )
            self._initialized = True
            self._sink.start()
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

    def _defer(self, write: Callable[[], None], *, at: float | None = None) -> None:
        """Run ``write`` on the sink, stamped when the data was produced.

        ``at`` is a monotonic instant; pass it when the data describes an
        earlier moment than now, such as a state packet that was generated
        before it was delivered.
        """
        moment = time.monotonic() if at is None else at
        if moment < self._start_time:
            # A caller handed us an instant from before the session began. The
            # monotonic epoch is machine boot, so a zero-initialised "instant"
            # lands days in the past and drags the whole timeline with it.
            # Stamp it now rather than corrupting the axis for every other view.
            # Correcting the instant itself, not just the derived elapsed value,
            # is what keeps ``log_time`` below out of the past as well.
            logger.debug(
                "Ignoring log timestamp %.3fs before session start", self._start_time - moment
            )
            moment = time.monotonic()
        elapsed = max(0.0, moment - self._start_time)

        # Rerun stamps its built-in ``log_time`` when the write happens, which
        # with writes handed to a worker is not when the data was produced: a
        # burst of state packets is written within a millisecond of each other,
        # so on that timeline they pile into a vertical step followed by a flat
        # run, where ``policy_time`` shows the same packets evenly spaced.
        # Stamping it with the produce moment keeps every timeline telling the
        # same story.
        wall = self._wall_clock_at(moment)
        recording = self._recording

        def stamped() -> None:
            import rerun as rr

            with pinned_elapsed(elapsed):
                rr.set_time("log_time", timestamp=wall, recording=recording)
                write()

        self._sink.submit(stamped)

    def _wall_clock_at(self, moment: float) -> datetime:
        """Absolute time for a monotonic instant, for stamping ``log_time``."""
        if self._start_wall is None:
            # Pin the two clocks to each other once, on first use.
            self._start_wall = datetime.now(UTC) - timedelta(
                seconds=time.monotonic() - self._start_time
            )
        return self._start_wall + timedelta(seconds=moment - self._start_time)

    def log_observation(self, states: dict[str, RobotState], step: int) -> None:
        """Log robot state: update 3D mesh positions, joint scalars, TCP trail."""
        if not self._initialized or self._recording is None:
            return
        try:
            from novapolicy.rerun.observation import (
                log_observation,
            )

            log_observation(
                states,
                step,
                start_time=self._start_time,
                dh_robots=self._dh_robots,
                visualizers=self._visualizers,
                tcp_trail=self._tcp_trail,
                max_trail_points=self._max_trail_points,
                recording=self._recording,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_observation error: %s", e)

    def log_bridge_chunk(self, chunk: ActionChunk, step: int) -> None:
        """Log an interpolated connector in Nova Violet."""
        if not self._initialized or self._recording is None:
            return
        try:
            from novapolicy.rerun.action_chunk import (
                log_bridge_chunk,
            )

            log_bridge_chunk(
                chunk,
                step,
                start_time=self._start_time,
                dh_robots=self._dh_robots,
                tcp_offsets=self._tcp_offsets,
                recording=self._recording,
            )
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_bridge_chunk error: %s", e)

    def log_action_chunk(self, chunk: ActionChunk, step: int, *, n_action_steps: int = 0) -> None:
        """Log action chunk as TCP path line strips and inspectable text."""
        if not self._initialized or self._recording is None:
            return
        recording = self._recording

        def write() -> None:
            from novapolicy.rerun.action_chunk import (
                log_action_chunk,
            )

            log_action_chunk(
                chunk,
                step,
                start_time=self._start_time,
                dh_robots=self._dh_robots,
                tcp_offsets=self._tcp_offsets,
                n_action_steps=n_action_steps,
                recording=recording,
            )

        self._defer(write)

    def log_target_tracking(
        self, chunk: ActionChunk, states: dict[str, RobotState], step: int
    ) -> None:
        """Log each first commanded target against the latest actual state."""
        for mg_id, steps in chunk.joints.items():
            state = states.get(mg_id)
            if steps and state is not None:
                self.log_joint_tracking(mg_id, steps[0], state, step)
        for mg_id, steps in chunk.tcp.items():
            state = states.get(mg_id)
            if steps and state is not None:
                self.log_tcp_tracking(mg_id, steps[0], state, step)

    def log_joint_tracking(
        self,
        mg_id: str,
        target: list[float],
        actual: RobotState,
        step: int,
        *,
        at: float | None = None,
    ) -> None:
        """Log commanded/actual joints and the derived TCP position error."""
        if not self._initialized or self._recording is None:
            return
        # Snapshot before deferring: the caller reuses its target buffers.
        #
        # Guarded, because this part runs on the caller's thread — the control
        # loop, which calls this with no try/except of its own. A state that is
        # briefly missing its joints must skip one entry, not abort execution.
        try:
            snapshot = list(target)
            joints = list(actual.joints)
            pose = getattr(actual, "pose", None)
        except LOG_ERRORS as e:
            logger.debug("log_joint_tracking snapshot failed: %s", e)
            return
        recording = self._recording
        trail = self._tcp_target_trail.get(mg_id)

        def write() -> None:
            from novapolicy.rerun.kinematics import (
                joint_tcp_position,
            )
            from novapolicy.rerun.target_tracking import (
                log_joint_tcp_tracking,
                log_joint_tracking,
            )

            log_joint_tracking(
                mg_id, snapshot, joints, step, start_time=self._start_time, recording=recording
            )
            dh_robot = self._dh_robots.get(mg_id)
            if pose is not None and dh_robot is not None:
                log_joint_tcp_tracking(
                    mg_id,
                    joint_tcp_position(dh_robot, snapshot, self._tcp_offsets.get(mg_id)),
                    pose,
                    step,
                    start_time=self._start_time,
                    recording=recording,
                    target_trail=trail,
                    max_trail_points=self._max_trail_points,
                )

        self._defer(write, at=at)

    def log_tcp_tracking(
        self,
        mg_id: str,
        target: list[float],
        actual: RobotState,
        step: int,
        *,
        at: float | None = None,
    ) -> None:
        """Log commanded/actual TCP pose and tracking error."""
        if not self._initialized or self._recording is None:
            return
        # Snapshot before deferring: the caller reuses its target buffers.
        # Guarded for the same reason as :meth:`log_joint_tracking` — this runs
        # on the control loop's own thread.
        try:
            pose = getattr(actual, "pose", None)
            if pose is None:
                return
            snapshot = list(target)
        except LOG_ERRORS as e:
            logger.debug("log_tcp_tracking snapshot failed: %s", e)
            return
        recording = self._recording
        trail = self._tcp_target_trail.get(mg_id)

        def write() -> None:
            from novapolicy.rerun.target_tracking import (
                log_tcp_tracking,
            )

            log_tcp_tracking(
                mg_id,
                snapshot,
                pose,
                step,
                start_time=self._start_time,
                recording=recording,
                target_trail=trail,
                max_trail_points=self._max_trail_points,
            )

        self._defer(write, at=at)

    def log_images(self, images: dict[str, Any]) -> None:
        """Log camera images to Rerun."""
        if not self._initialized or self._recording is None or not images:
            return
        try:
            from novapolicy.rerun.images import log_images

            log_images(images, start_time=self._start_time, recording=self._recording)
        except (OSError, RuntimeError, ValueError, TypeError) as e:
            logger.debug("log_images error: %s", e)

    def log_completion(self, reason: str, steps: int, duration_s: float) -> None:
        """Log execution completion."""
        if not self._initialized:
            return
        try:
            import rerun as rr

            rr.log(
                "policy/status",
                rr.TextLog(
                    f"Policy finished: {reason} ({steps} steps, {duration_s:.1f}s)",
                    level=rr.TextLogLevel.INFO,
                ),
                recording=self._recording,
            )
        except (ImportError, OSError, RuntimeError) as e:
            # Completion logging is best-effort; never let it break execution.
            logger.debug("log_completion failed: %s", e)

    # ------------------------------------------------------------------
    # Continuous state streaming (between policy steps)
    # ------------------------------------------------------------------

    def start_streaming(
        self,
        sessions: dict[str, Any],
        *,
        image_reader: Callable[[], dict[str, Any]] | None = None,
    ) -> None:
        """Start background state and camera-frame logging."""
        if not self._initialized:
            return
        from novapolicy.rerun.streaming import (
            StateStreamer,
        )

        self._streamer = StateStreamer(
            start_time=self._start_time,
            dh_robots=self._dh_robots,
            visualizers=self._visualizers,
            tcp_trail=self._tcp_trail,
            max_trail_points=self._max_trail_points,
            recording=self._recording,
            image_reader=image_reader,
            state_sample_interval_ms=self._state_sample_interval_ms,
        )
        self._streamer.start(sessions)

    async def stop_streaming(self) -> None:
        """Stop background streaming and disconnect the dedicated recording."""
        try:
            if self._streamer is not None:
                await self._streamer.stop()
        finally:
            # Drain before the recording goes away, or the tail of the run is
            # written to a disconnected stream. Joined off the event loop so
            # teardown of everything else can proceed meanwhile.
            await asyncio.get_running_loop().run_in_executor(None, self._sink.close)
            self._streamer = None
            recording = self._recording
            self._recording = None
            self._initialized = False
            if recording is not None:
                try:
                    await _disconnect_recording(recording)
                except (OSError, RuntimeError) as e:
                    logger.debug("Failed to disconnect policy Rerun recording: %s", e)
