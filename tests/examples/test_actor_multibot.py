import asyncio

import pytest

from examples.actor_multibot import (
    AbortRun,
    Actor,
    Bootstrap,
    CellProcess,
    EventRecorder,
    OrchestratorActor,
    PLCActor,
    ParallelStage,
    RequestZone,
    ReleaseZone,
    RobotStep,
    RunStage,
    StageCompleted,
    StageFailed,
    ZoneGranted,
)


class RecordingActor(Actor):
    def __init__(self, name: str, recorder: EventRecorder):
        super().__init__(name=name, recorder=recorder)
        self.values: list[int] = []

    async def handle(self, message):
        self.values.append(message)


class ZoneSinkActor(Actor):
    def __init__(self, name: str, recorder: EventRecorder):
        super().__init__(name=name, recorder=recorder)
        self.grants: list[ZoneGranted] = []

    async def handle(self, message):
        if isinstance(message, ZoneGranted):
            self.grants.append(message)
            return
        raise TypeError(type(message).__name__)


class FakeRobotActor(Actor):
    def __init__(
        self,
        *,
        name: str,
        robot_id: str,
        recorder: EventRecorder,
        orchestrator: OrchestratorActor,
        completion_events: dict[str, asyncio.Event] | None = None,
        fail_stage: str | None = None,
    ):
        super().__init__(name=name, recorder=recorder)
        self.robot_id = robot_id
        self._orchestrator = orchestrator
        self._completion_events = completion_events or {}
        self._fail_stage = fail_stage
        self.started_stages: list[str] = []
        self.abort_reasons: list[str] = []
        self._current_task: asyncio.Task[None] | None = None

    async def handle(self, message):
        if isinstance(message, Bootstrap):
            await self._orchestrator.send(
                StageCompleted(robot_id=self.robot_id, stage_name="bootstrap", duration_seconds=0.0)
            )
            return
        if isinstance(message, RunStage):
            self._current_task = asyncio.create_task(self._run_stage(message.stage.name))
            return
        if isinstance(message, AbortRun):
            self.abort_reasons.append(message.reason)
            if self._current_task is not None:
                self._current_task.cancel()
            return
        raise TypeError(type(message).__name__)

    async def on_shutdown(self) -> None:
        if self._current_task is not None:
            self._current_task.cancel()
            try:
                await self._current_task
            except asyncio.CancelledError:
                pass

    async def _run_stage(self, stage_name: str) -> None:
        self.started_stages.append(stage_name)
        if stage_name == self._fail_stage:
            await self._orchestrator.send(
                StageFailed(
                    robot_id=self.robot_id,
                    stage_name=stage_name,
                    reason="simulated failure",
                )
            )
            return

        gate = self._completion_events.get(stage_name)
        if gate is not None:
            await gate.wait()

        await self._orchestrator.send(
            StageCompleted(robot_id=self.robot_id, stage_name=stage_name, duration_seconds=0.0)
        )


async def stop_all(*actors: Actor) -> None:
    for actor in actors:
        await actor.stop()


async def wait_until(predicate, *, timeout: float = 1.0) -> None:
    deadline = asyncio.get_running_loop().time() + timeout
    while not predicate():
        if asyncio.get_running_loop().time() >= deadline:
            raise AssertionError("condition not met before timeout")
        await asyncio.sleep(0)


async def test_actor_mailbox_preserves_message_order():
    recorder = EventRecorder()
    actor = RecordingActor("recorder", recorder)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(actor.run())
        for value in (1, 2, 3, 4):
            await actor.send(value)
        await actor.stop()

    assert actor.values == [1, 2, 3, 4]


async def test_plc_actor_grants_zone_fifo():
    recorder = EventRecorder()
    plc = PLCActor("plc", recorder)
    first = ZoneSinkActor("first", recorder)
    second = ZoneSinkActor("second", recorder)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(plc.run())
        tg.create_task(first.run())
        tg.create_task(second.run())

        await plc.send(
            RequestZone(zone="handover_zone", robot_id="robot_a", stage_name="shared", reply_to=first)
        )
        await plc.send(
            RequestZone(zone="handover_zone", robot_id="robot_b", stage_name="shared", reply_to=second)
        )

        await wait_until(lambda: len(first.grants) == 1)
        assert [grant.zone for grant in first.grants] == ["handover_zone"]
        assert second.grants == []

        await plc.send(
            ReleaseZone(
                zone="handover_zone",
                robot_id="robot_a",
                stage_name="shared",
            )
        )
        await wait_until(lambda: len(second.grants) == 1)

        assert [grant.zone for grant in second.grants] == ["handover_zone"]
        await stop_all(plc, first, second)


async def test_orchestrator_waits_for_all_stage_participants():
    recorder = EventRecorder()
    plc = PLCActor("plc", recorder)
    stage_gate_a = asyncio.Event()
    stage_gate_b = asyncio.Event()
    process = CellProcess(
        stages=(
            ParallelStage(
                name="stage_one",
                steps=(RobotStep("robot_a", "noop"), RobotStep("robot_b", "noop")),
            ),
            ParallelStage(
                name="stage_two",
                steps=(RobotStep("robot_a", "noop"), RobotStep("robot_b", "noop")),
            ),
        )
    )

    robot_actors: dict[str, Actor] = {}
    orchestrator = OrchestratorActor(
        process=process,
        robot_actors=robot_actors,
        plc=plc,
        recorder=recorder,
    )
    robot_a = FakeRobotActor(
        name="robot_a_actor",
        robot_id="robot_a",
        recorder=recorder,
        orchestrator=orchestrator,
        completion_events={"stage_one": stage_gate_a},
    )
    robot_b = FakeRobotActor(
        name="robot_b_actor",
        robot_id="robot_b",
        recorder=recorder,
        orchestrator=orchestrator,
        completion_events={"stage_one": stage_gate_b},
    )
    robot_actors.update({"robot_a": robot_a, "robot_b": robot_b})

    async with asyncio.TaskGroup() as tg:
        for actor in (plc, orchestrator, robot_a, robot_b):
            tg.create_task(actor.run())

        start_task = asyncio.create_task(orchestrator.start())
        await wait_until(lambda: robot_a.started_stages == ["stage_one"])
        await wait_until(lambda: robot_b.started_stages == ["stage_one"])
        assert robot_a.started_stages == ["stage_one"]
        assert robot_b.started_stages == ["stage_one"]

        stage_gate_a.set()
        await asyncio.sleep(0)
        assert robot_a.started_stages == ["stage_one"]
        assert robot_b.started_stages == ["stage_one"]

        stage_gate_b.set()
        await start_task

        await wait_until(lambda: robot_a.started_stages == ["stage_one", "stage_two"])
        await wait_until(lambda: robot_b.started_stages == ["stage_one", "stage_two"])
        assert robot_a.started_stages == ["stage_one", "stage_two"]
        assert robot_b.started_stages == ["stage_one", "stage_two"]
        await stop_all(plc, orchestrator, robot_a, robot_b)


async def test_failure_triggers_abort_run():
    recorder = EventRecorder()
    plc = PLCActor("plc", recorder)
    process = CellProcess(
        stages=(ParallelStage(name="stage_one", steps=(RobotStep("robot_a", "noop"), RobotStep("robot_b", "noop"))),)
    )

    robot_actors: dict[str, Actor] = {}
    orchestrator = OrchestratorActor(
        process=process,
        robot_actors=robot_actors,
        plc=plc,
        recorder=recorder,
    )
    failing_robot = FakeRobotActor(
        name="robot_a_actor",
        robot_id="robot_a",
        recorder=recorder,
        orchestrator=orchestrator,
        fail_stage="stage_one",
    )
    passive_robot = FakeRobotActor(
        name="robot_b_actor",
        robot_id="robot_b",
        recorder=recorder,
        orchestrator=orchestrator,
        completion_events={"stage_one": asyncio.Event()},
    )
    robot_actors.update({"robot_a": failing_robot, "robot_b": passive_robot})

    async with asyncio.TaskGroup() as tg:
        for actor in (plc, orchestrator, failing_robot, passive_robot):
            tg.create_task(actor.run())

        with pytest.raises(RuntimeError, match="simulated failure"):
            await orchestrator.start()

        await wait_until(lambda: passive_robot.abort_reasons == ["simulated failure"])
        assert passive_robot.abort_reasons == ["simulated failure"]
        await stop_all(plc, orchestrator, failing_robot, passive_robot)
