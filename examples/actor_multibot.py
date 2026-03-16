"""Actor-style multi-robot coordination example.

This example ports the core idea from the Pony `multibot` prototype into
Python using in-process actors built on `asyncio.Queue` mailboxes.

Architecture:
- `OrchestratorActor`: supervises bootstrap, stage dispatch, and fail-fast shutdown
- `RobotActor`: owns one robot controller/motion group and executes stage behaviors
- `PLCActor`: grants exclusive access to shared zones through message passing

The first version intentionally stays in one event loop. CPU-heavy pure Python
pre/post-processing is isolated behind a small offload interface so future
versions can experiment with `anyio.to_thread.run_sync()` or
`anyio.to_process.run_sync()` without changing the message protocol.
"""

from __future__ import annotations

import asyncio
import logging
from collections import defaultdict, deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from time import perf_counter
from typing import Any, Protocol, TypeVar

import nova
from nova import api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.types import Pose

logger = logging.getLogger(__name__)

T = TypeVar("T")


class ComputeOffload(Protocol):
    """Abstract execution boundary for CPU-heavy local work.

    Keep NOVA client objects and other async robot I/O on the main event loop.
    Only pure Python computations should move behind this interface.
    """

    async def run(self, func: Callable[..., T], /, *args: Any, **kwargs: Any) -> T: ...


class InlineComputeOffload:
    """Default offload backend that executes inline on the event loop."""

    async def run(self, func: Callable[..., T], /, *args: Any, **kwargs: Any) -> T:
        return func(*args, **kwargs)


@dataclass(frozen=True)
class RobotPlan:
    actions: list[Any]
    tcp: str


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    controller_name: str
    manufacturer: api.models.Manufacturer
    controller_type: api.models.VirtualControllerTypes
    approach_offset: tuple[float, float, float, float, float, float]
    zone_offset: tuple[float, float, float, float, float, float]
    independent_offset: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class RobotBehaviorContext:
    spec: RobotSpec
    motion_group: Any
    tcp: str
    home_joints: tuple[float, ...]
    home_pose: Pose
    offload: ComputeOffload


BehaviorBuilder = Callable[[RobotBehaviorContext], Awaitable[RobotPlan]]


@dataclass(frozen=True)
class RobotBehavior:
    name: str
    builder: BehaviorBuilder


@dataclass(frozen=True)
class RobotStep:
    robot_id: str
    behavior: str
    zone: str | None = None


@dataclass(frozen=True)
class ParallelStage:
    name: str
    steps: tuple[RobotStep, ...]

    def participants(self) -> tuple[str, ...]:
        return tuple(step.robot_id for step in self.steps)

    def step_for(self, robot_id: str) -> RobotStep | None:
        for step in self.steps:
            if step.robot_id == robot_id:
                return step
        return None


@dataclass(frozen=True)
class CellProcess:
    stages: tuple[ParallelStage, ...]


@dataclass(frozen=True)
class Bootstrap:
    pass


@dataclass(frozen=True)
class RunStage:
    stage: ParallelStage


@dataclass(frozen=True)
class StageCompleted:
    robot_id: str
    stage_name: str
    duration_seconds: float


@dataclass(frozen=True)
class StageFailed:
    robot_id: str
    stage_name: str
    reason: str


@dataclass(frozen=True)
class RequestZone:
    zone: str
    robot_id: str
    stage_name: str
    reply_to: "Actor"


@dataclass(frozen=True)
class ZoneGranted:
    zone: str
    stage_name: str


@dataclass(frozen=True)
class ReleaseZone:
    zone: str
    robot_id: str
    stage_name: str


@dataclass(frozen=True)
class AbortRun:
    reason: str


@dataclass(frozen=True)
class _Shutdown:
    pass


@dataclass(frozen=True)
class RunEvent:
    elapsed_seconds: float
    actor: str
    event: str
    robot_id: str | None = None
    stage_name: str | None = None
    detail: str = ""


class EventRecorder:
    """Collects structured events and stage timings for the run."""

    def __init__(self):
        self._started_at = perf_counter()
        self.events: list[RunEvent] = []
        self.stage_durations: dict[tuple[str, str], float] = {}

    def record(
        self,
        *,
        actor: str,
        event: str,
        robot_id: str | None = None,
        stage_name: str | None = None,
        detail: str = "",
    ) -> None:
        elapsed_seconds = perf_counter() - self._started_at
        entry = RunEvent(
            elapsed_seconds=elapsed_seconds,
            actor=actor,
            event=event,
            robot_id=robot_id,
            stage_name=stage_name,
            detail=detail,
        )
        self.events.append(entry)
        logger.info(
            "[%7.3fs] actor=%s event=%s robot=%s stage=%s detail=%s",
            entry.elapsed_seconds,
            entry.actor,
            entry.event,
            entry.robot_id or "-",
            entry.stage_name or "-",
            entry.detail or "-",
        )

    def record_stage_duration(self, robot_id: str, stage_name: str, duration_seconds: float) -> None:
        self.stage_durations[(robot_id, stage_name)] = duration_seconds

    def render_summary(self) -> str:
        by_robot: dict[str, list[tuple[str, float]]] = defaultdict(list)
        for (robot_id, stage_name), duration_seconds in sorted(self.stage_durations.items()):
            by_robot[robot_id].append((stage_name, duration_seconds))

        lines = ["Run summary:"]
        total_duration = perf_counter() - self._started_at
        for robot_id in sorted(by_robot):
            stage_parts = ", ".join(
                f"{stage_name}={duration:.3f}s" for stage_name, duration in by_robot[robot_id]
            )
            lines.append(f"  {robot_id}: {stage_parts}")
        lines.append(f"  total_wall_time={total_duration:.3f}s")
        return "\n".join(lines)


class Actor:
    """Minimal actor abstraction backed by a single mailbox."""

    def __init__(self, name: str, recorder: EventRecorder):
        self.name = name
        self._recorder = recorder
        self._mailbox: asyncio.Queue[Any] = asyncio.Queue()

    async def send(self, message: Any) -> None:
        await self._mailbox.put(message)

    async def stop(self) -> None:
        await self.send(_Shutdown())

    async def run(self) -> None:
        self._recorder.record(actor=self.name, event="actor_started")
        while True:
            message = await self._mailbox.get()
            try:
                if isinstance(message, _Shutdown):
                    await self.on_shutdown()
                    self._recorder.record(actor=self.name, event="actor_stopped")
                    return
                await self.handle(message)
            finally:
                self._mailbox.task_done()

    async def handle(self, message: Any) -> None:
        raise NotImplementedError

    async def on_shutdown(self) -> None:
        return


class PLCActor(Actor):
    """Message-driven controller for exclusive shared-zone access."""

    def __init__(self, name: str, recorder: EventRecorder):
        super().__init__(name=name, recorder=recorder)
        self._zone_owner: dict[str, str | None] = {}
        self._zone_waiters: dict[str, deque[RequestZone]] = defaultdict(deque)

    async def handle(self, message: Any) -> None:
        if isinstance(message, RequestZone):
            await self._handle_request(message)
            return
        if isinstance(message, ReleaseZone):
            await self._handle_release(message)
            return
        if isinstance(message, AbortRun):
            self._zone_owner.clear()
            self._zone_waiters.clear()
            self._recorder.record(actor=self.name, event="abort_run", detail=message.reason)
            return
        raise TypeError(f"{self.name} received unsupported message: {type(message).__name__}")

    async def _handle_request(self, message: RequestZone) -> None:
        owner = self._zone_owner.get(message.zone)
        if owner is None:
            self._zone_owner[message.zone] = message.robot_id
            self._recorder.record(
                actor=self.name,
                event="zone_granted",
                robot_id=message.robot_id,
                stage_name=message.stage_name,
                detail=message.zone,
            )
            await message.reply_to.send(ZoneGranted(zone=message.zone, stage_name=message.stage_name))
            return

        self._zone_waiters[message.zone].append(message)
        self._recorder.record(
            actor=self.name,
            event="zone_queued",
            robot_id=message.robot_id,
            stage_name=message.stage_name,
            detail=message.zone,
        )

    async def _handle_release(self, message: ReleaseZone) -> None:
        owner = self._zone_owner.get(message.zone)
        if owner != message.robot_id:
            self._recorder.record(
                actor=self.name,
                event="zone_release_ignored",
                robot_id=message.robot_id,
                stage_name=message.stage_name,
                detail=message.zone,
            )
            return

        if self._zone_waiters[message.zone]:
            next_request = self._zone_waiters[message.zone].popleft()
            self._zone_owner[message.zone] = next_request.robot_id
            self._recorder.record(
                actor=self.name,
                event="zone_granted",
                robot_id=next_request.robot_id,
                stage_name=next_request.stage_name,
                detail=next_request.zone,
            )
            await next_request.reply_to.send(
                ZoneGranted(zone=next_request.zone, stage_name=next_request.stage_name)
            )
            return

        self._zone_owner[message.zone] = None
        self._recorder.record(
            actor=self.name,
            event="zone_released",
            robot_id=message.robot_id,
            stage_name=message.stage_name,
            detail=message.zone,
        )


class RobotActor(Actor):
    """Owns one robot and executes stage behaviors."""

    def __init__(
        self,
        *,
        cell: Any,
        spec: RobotSpec,
        orchestrator: "OrchestratorActor",
        plc: PLCActor,
        behaviors: dict[str, RobotBehavior],
        recorder: EventRecorder,
        offload: ComputeOffload,
    ):
        super().__init__(name=f"robot:{spec.robot_id}", recorder=recorder)
        self.robot_id = spec.robot_id
        self._cell = cell
        self._spec = spec
        self._orchestrator = orchestrator
        self._plc = plc
        self._behaviors = behaviors
        self._offload = offload

        self._motion_group: Any | None = None
        self._tcp: str | None = None
        self._home_joints: tuple[float, ...] | None = None
        self._home_pose: Pose | None = None

        self._current_task: asyncio.Task[None] | None = None
        self._waiting_for_zone: asyncio.Future[None] | None = None
        self._active_zone: tuple[str, str] | None = None
        self._aborting = False

    async def handle(self, message: Any) -> None:
        if isinstance(message, Bootstrap):
            self._start_task("bootstrap", self._bootstrap())
            return
        if isinstance(message, RunStage):
            self._start_task(message.stage.name, self._run_stage(message.stage))
            return
        if isinstance(message, ZoneGranted):
            if self._waiting_for_zone is not None and not self._waiting_for_zone.done():
                self._waiting_for_zone.set_result(None)
            return
        if isinstance(message, AbortRun):
            self._aborting = True
            self._recorder.record(
                actor=self.name,
                event="abort_run",
                robot_id=self.robot_id,
                detail=message.reason,
            )
            if self._waiting_for_zone is not None and not self._waiting_for_zone.done():
                self._waiting_for_zone.cancel()
            if self._current_task is not None:
                self._current_task.cancel()
            return
        raise TypeError(f"{self.name} received unsupported message: {type(message).__name__}")

    async def on_shutdown(self) -> None:
        if self._current_task is not None:
            self._current_task.cancel()
            try:
                await self._current_task
            except asyncio.CancelledError:
                pass

    def attach_orchestrator(self, orchestrator: "OrchestratorActor") -> None:
        self._orchestrator = orchestrator

    def _start_task(self, label: str, coroutine: Awaitable[None]) -> None:
        if self._current_task is not None and not self._current_task.done():
            raise RuntimeError(f"{self.name} is already executing a task")
        self._current_task = asyncio.create_task(coroutine, name=f"{self.name}:{label}")

    async def _bootstrap(self) -> None:
        started_at = perf_counter()
        self._recorder.record(actor=self.name, event="bootstrap_started", robot_id=self.robot_id)
        try:
            controller = await self._cell.controller(self._spec.controller_name)
            self._motion_group = controller[0]
            tcp_names = await self._motion_group.tcp_names()
            self._tcp = tcp_names[0]
            self._home_joints = await self._motion_group.joints()
            self._home_pose = await self._motion_group.tcp_pose(self._tcp)
            self._recorder.record(
                actor=self.name,
                event="bootstrap_ready",
                robot_id=self.robot_id,
                detail=self._tcp,
            )
            await self._orchestrator.send(
                StageCompleted(
                    robot_id=self.robot_id,
                    stage_name="bootstrap",
                    duration_seconds=perf_counter() - started_at,
                )
            )
        except Exception as exc:
            await self._orchestrator.send(
                StageFailed(
                    robot_id=self.robot_id,
                    stage_name="bootstrap",
                    reason=str(exc),
                )
            )
        finally:
            self._current_task = None

    async def _run_stage(self, stage: ParallelStage) -> None:
        started_at = perf_counter()
        step = stage.step_for(self.robot_id)
        if step is None:
            self._current_task = None
            return

        self._recorder.record(
            actor=self.name,
            event="stage_started",
            robot_id=self.robot_id,
            stage_name=stage.name,
            detail=step.behavior,
        )

        try:
            if step.zone is not None:
                self._waiting_for_zone = asyncio.get_running_loop().create_future()
                self._recorder.record(
                    actor=self.name,
                    event="zone_requested",
                    robot_id=self.robot_id,
                    stage_name=stage.name,
                    detail=step.zone,
                )
                await self._plc.send(
                    RequestZone(
                        zone=step.zone,
                        robot_id=self.robot_id,
                        stage_name=stage.name,
                        reply_to=self,
                    )
                )
                await self._waiting_for_zone
                self._active_zone = (step.zone, stage.name)
                self._recorder.record(
                    actor=self.name,
                    event="zone_granted",
                    robot_id=self.robot_id,
                    stage_name=stage.name,
                    detail=step.zone,
                )

            behavior = self._behaviors[step.behavior]
            context = self._build_behavior_context()
            plan = await behavior.builder(context)
            assert self._motion_group is not None

            self._recorder.record(
                actor=self.name,
                event="planning_started",
                robot_id=self.robot_id,
                stage_name=stage.name,
                detail=behavior.name,
            )
            trajectory = await self._motion_group.plan(plan.actions, plan.tcp)
            self._recorder.record(
                actor=self.name,
                event="plan_ready",
                robot_id=self.robot_id,
                stage_name=stage.name,
                detail=f"points={len(trajectory.joint_positions)}",
            )
            await self._motion_group.execute(trajectory, plan.tcp, actions=plan.actions)
            self._recorder.record(
                actor=self.name,
                event="execution_completed",
                robot_id=self.robot_id,
                stage_name=stage.name,
            )

            if self._active_zone is not None:
                zone_name, zone_stage = self._active_zone
                await self._plc.send(
                    ReleaseZone(
                        zone=zone_name,
                        robot_id=self.robot_id,
                        stage_name=zone_stage,
                    )
                )
                self._recorder.record(
                    actor=self.name,
                    event="zone_released",
                    robot_id=self.robot_id,
                    stage_name=stage.name,
                    detail=zone_name,
                )
                self._active_zone = None

            await self._orchestrator.send(
                StageCompleted(
                    robot_id=self.robot_id,
                    stage_name=stage.name,
                    duration_seconds=perf_counter() - started_at,
                )
            )
        except asyncio.CancelledError:
            if self._active_zone is not None:
                zone_name, zone_stage = self._active_zone
                await self._plc.send(
                    ReleaseZone(
                        zone=zone_name,
                        robot_id=self.robot_id,
                        stage_name=zone_stage,
                    )
                )
                self._active_zone = None
            self._recorder.record(
                actor=self.name,
                event="stage_cancelled",
                robot_id=self.robot_id,
                stage_name=stage.name,
            )
            raise
        except Exception as exc:
            if self._active_zone is not None:
                zone_name, zone_stage = self._active_zone
                await self._plc.send(
                    ReleaseZone(
                        zone=zone_name,
                        robot_id=self.robot_id,
                        stage_name=zone_stage,
                    )
                )
                self._active_zone = None
            self._recorder.record(
                actor=self.name,
                event="stage_failed",
                robot_id=self.robot_id,
                stage_name=stage.name,
                detail=str(exc),
            )
            if not self._aborting:
                await self._orchestrator.send(
                    StageFailed(
                        robot_id=self.robot_id,
                        stage_name=stage.name,
                        reason=str(exc),
                    )
                )
        finally:
            self._waiting_for_zone = None
            self._current_task = None

    def _build_behavior_context(self) -> RobotBehaviorContext:
        assert self._motion_group is not None
        assert self._tcp is not None
        assert self._home_joints is not None
        assert self._home_pose is not None
        return RobotBehaviorContext(
            spec=self._spec,
            motion_group=self._motion_group,
            tcp=self._tcp,
            home_joints=self._home_joints,
            home_pose=self._home_pose,
            offload=self._offload,
        )


class OrchestratorActor(Actor):
    """Coordinates bootstrap, barriers between stages, and fail-fast shutdown."""

    def __init__(
        self,
        *,
        process: CellProcess,
        robot_actors: dict[str, Actor],
        plc: PLCActor,
        recorder: EventRecorder,
    ):
        super().__init__(name="orchestrator", recorder=recorder)
        self._process = process
        self._robot_actors = robot_actors
        self._plc = plc
        self._current_stage_index = -1
        self._pending_bootstrap: set[str] = set()
        self._pending_stage: set[str] = set()
        self._finished: asyncio.Future[None] = asyncio.get_running_loop().create_future()
        self._failed = False

    async def start(self) -> None:
        self._pending_bootstrap = set(self._robot_actors)
        self._recorder.record(
            actor=self.name,
            event="run_started",
            detail=f"robots={len(self._robot_actors)} stages={len(self._process.stages)}",
        )
        for robot in self._robot_actors.values():
            await robot.send(Bootstrap())
        await self._finished

    async def handle(self, message: Any) -> None:
        if isinstance(message, StageCompleted):
            await self._handle_stage_completed(message)
            return
        if isinstance(message, StageFailed):
            await self._handle_stage_failed(message)
            return
        if isinstance(message, AbortRun):
            await self._abort_everything(message.reason)
            return
        raise TypeError(f"{self.name} received unsupported message: {type(message).__name__}")

    async def _handle_stage_completed(self, message: StageCompleted) -> None:
        self._recorder.record_stage_duration(
            message.robot_id,
            message.stage_name,
            message.duration_seconds,
        )
        self._recorder.record(
            actor=self.name,
            event="stage_completed",
            robot_id=message.robot_id,
            stage_name=message.stage_name,
            detail=f"{message.duration_seconds:.3f}s",
        )

        if message.stage_name == "bootstrap":
            self._pending_bootstrap.discard(message.robot_id)
            if not self._pending_bootstrap and not self._failed:
                await self._launch_next_stage()
            return

        self._pending_stage.discard(message.robot_id)
        if not self._pending_stage and not self._failed:
            await self._launch_next_stage()

    async def _handle_stage_failed(self, message: StageFailed) -> None:
        if self._failed:
            return
        self._failed = True
        self._recorder.record(
            actor=self.name,
            event="stage_failed",
            robot_id=message.robot_id,
            stage_name=message.stage_name,
            detail=message.reason,
        )
        await self._abort_everything(message.reason)
        if not self._finished.done():
            self._finished.set_exception(
                RuntimeError(
                    f"{message.robot_id} failed during {message.stage_name}: {message.reason}"
                )
            )

    async def _launch_next_stage(self) -> None:
        self._current_stage_index += 1
        if self._current_stage_index >= len(self._process.stages):
            self._recorder.record(actor=self.name, event="run_completed")
            if not self._finished.done():
                self._finished.set_result(None)
            return

        stage = self._process.stages[self._current_stage_index]
        self._pending_stage = set(stage.participants())
        self._recorder.record(
            actor=self.name,
            event="stage_launched",
            stage_name=stage.name,
            detail=",".join(sorted(self._pending_stage)),
        )
        for robot_id in self._pending_stage:
            await self._robot_actors[robot_id].send(RunStage(stage=stage))

    async def _abort_everything(self, reason: str) -> None:
        await self._plc.send(AbortRun(reason=reason))
        for robot in self._robot_actors.values():
            await robot.send(AbortRun(reason=reason))


def _compose_offset_pose(
    home_pose: Pose,
    offset: tuple[float, float, float, float, float, float],
) -> Pose:
    return home_pose @ Pose(offset)


async def build_approach_plan(context: RobotBehaviorContext) -> RobotPlan:
    target_pose = await context.offload.run(
        _compose_offset_pose, context.home_pose, context.spec.approach_offset
    )
    return RobotPlan(
        actions=[joint_ptp(context.home_joints), cartesian_ptp(target_pose)],
        tcp=context.tcp,
    )


async def build_shared_zone_plan(context: RobotBehaviorContext) -> RobotPlan:
    target_pose = await context.offload.run(
        _compose_offset_pose, context.home_pose, context.spec.zone_offset
    )
    return RobotPlan(
        actions=[
            joint_ptp(context.home_joints),
            cartesian_ptp(target_pose),
            joint_ptp(context.home_joints),
        ],
        tcp=context.tcp,
    )


async def build_independent_cycle_plan(context: RobotBehaviorContext) -> RobotPlan:
    target_pose = await context.offload.run(
        _compose_offset_pose, context.home_pose, context.spec.independent_offset
    )
    return RobotPlan(
        actions=[
            joint_ptp(context.home_joints),
            cartesian_ptp(target_pose),
            joint_ptp(context.home_joints),
        ],
        tcp=context.tcp,
    )


async def build_return_home_plan(context: RobotBehaviorContext) -> RobotPlan:
    return RobotPlan(actions=[joint_ptp(context.home_joints)], tcp=context.tcp)


def default_behaviors() -> dict[str, RobotBehavior]:
    return {
        "approach": RobotBehavior("approach", build_approach_plan),
        "shared_zone_pass": RobotBehavior("shared_zone_pass", build_shared_zone_plan),
        "independent_cycle": RobotBehavior("independent_cycle", build_independent_cycle_plan),
        "return_home": RobotBehavior("return_home", build_return_home_plan),
    }


def default_robot_specs() -> tuple[RobotSpec, ...]:
    robot_type = api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR5E
    manufacturer = api.models.Manufacturer.UNIVERSALROBOTS
    return (
        RobotSpec(
            robot_id="robot_a",
            controller_name="robot-a",
            manufacturer=manufacturer,
            controller_type=robot_type,
            approach_offset=(120, 0, 0, 0, 0, 0),
            zone_offset=(220, 100, 0, 0, 0, 0),
            independent_offset=(80, -80, 0, 0, 0, 0),
        ),
        RobotSpec(
            robot_id="robot_b",
            controller_name="robot-b",
            manufacturer=manufacturer,
            controller_type=robot_type,
            approach_offset=(120, 0, 0, 0, 0, 0),
            zone_offset=(220, -100, 0, 0, 0, 0),
            independent_offset=(80, 80, 0, 0, 0, 0),
        ),
        RobotSpec(
            robot_id="robot_c",
            controller_name="robot-c",
            manufacturer=manufacturer,
            controller_type=robot_type,
            approach_offset=(80, 40, 0, 0, 0, 0),
            zone_offset=(140, 0, 0, 0, 0, 0),
            independent_offset=(60, 140, 0, 0, 0, 0),
        ),
    )


def default_process() -> CellProcess:
    return CellProcess(
        stages=(
            ParallelStage(
                name="local_approach",
                steps=(
                    RobotStep("robot_a", "approach"),
                    RobotStep("robot_b", "approach"),
                    RobotStep("robot_c", "approach"),
                ),
            ),
            ParallelStage(
                name="shared_zone",
                steps=(
                    RobotStep("robot_a", "shared_zone_pass", zone="handover_zone"),
                    RobotStep("robot_b", "shared_zone_pass", zone="handover_zone"),
                    RobotStep("robot_c", "independent_cycle"),
                ),
            ),
            ParallelStage(
                name="return_home",
                steps=(
                    RobotStep("robot_a", "return_home"),
                    RobotStep("robot_b", "return_home"),
                    RobotStep("robot_c", "return_home"),
                ),
            ),
        )
    )


async def run_actor_multibot(
    *,
    cell: Any,
    robot_specs: tuple[RobotSpec, ...] | None = None,
    process: CellProcess | None = None,
    offload: ComputeOffload | None = None,
) -> EventRecorder:
    recorder = EventRecorder()
    robot_specs = robot_specs or default_robot_specs()
    process = process or default_process()
    offload = offload or InlineComputeOffload()
    behaviors = default_behaviors()

    plc = PLCActor(name="plc", recorder=recorder)
    robot_actors: dict[str, Actor] = {}
    for spec in robot_specs:
        robot_actors[spec.robot_id] = RobotActor(
            cell=cell,
            spec=spec,
            orchestrator=None,  # type: ignore[arg-type]
            plc=plc,
            behaviors=behaviors,
            recorder=recorder,
            offload=offload,
        )

    orchestrator = OrchestratorActor(
        process=process,
        robot_actors=robot_actors,
        plc=plc,
        recorder=recorder,
    )

    for actor in robot_actors.values():
        assert isinstance(actor, RobotActor)
        actor.attach_orchestrator(orchestrator)

    all_actors = [orchestrator, plc, *robot_actors.values()]
    try:
        async with asyncio.TaskGroup() as tg:
            for actor in all_actors:
                tg.create_task(actor.run(), name=actor.name)

            await orchestrator.start()

            for actor in all_actors:
                await actor.stop()
    finally:
        logger.info(recorder.render_summary())

    return recorder


_DEFAULT_SPECS = default_robot_specs()


@nova.program(
    id="actor_multibot",
    name="Actor Multibot",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=spec.controller_name,
                manufacturer=spec.manufacturer,
                type=spec.controller_type,
            )
            for spec in _DEFAULT_SPECS
        ],
        cleanup_controllers=False,
    ),
)
async def actor_multibot(ctx: nova.ProgramContext) -> None:
    """Run the actor-style multi-robot example against virtual controllers."""

    await run_actor_multibot(cell=ctx.cell)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    run_program(actor_multibot)
