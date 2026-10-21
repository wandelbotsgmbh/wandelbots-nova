# NOVA SDK mapping

How the rules map to the `wandelbots-nova` Python SDK. Written against **wandelbots-nova 6.1**.
If the project's installed SDK is newer, verify symbols in the installed package before citing
them:

```bash
uv run python -c "import nova, os; print(os.path.dirname(nova.__file__))"
```

Source: <https://github.com/wandelbotsgmbh/wandelbots-nova> (`nova/` package, `examples/`).

## Known SDK gaps (workaround required)

| Gap | Affects | Workaround |
|---|---|---|
| `MotionGroup.plan` / `execute` / `plan_and_execute` have no timeout parameter | MOT-001, ASY-002 | Wrap in `asyncio.timeout` (below) |
| `MotionGroup.stop()` is not functional (no-op, or `AttributeError` on the v2 API) | SAF-003, LIFE-001 | Cancel the execute task, or `TrajectoryCursor.pause()` |
| `nova.utils.io.wait_for_bus_io` and controller `wait_for_bool_io` have no timeout | IO-002, ASY-002 | Wrap in `asyncio.timeout` (below) |
| `Nova()` / `cell.controller()` have no connect timeout or retry | LIFE-003 | Wrap in `asyncio.timeout` + bounded retry (below) |
| `@nova.program` registry silently replaces duplicate ids | STR-001 | Give every program an explicit unique `id=` |

Report the workaround as the fix and mention the gap in one line. Do not invent SDK parameters.

## SDK examples are not production templates

`examples/*.py` and the `nova app create` template favour brevity. Patterns like
`controller[0]`, `tcp_names[0]`, literal `Pose((...))` targets, `print(...)` and missing timeouts
are intentional there. Do not accept "it is like the SDK example" as a reason to skip a finding.

## Rule → SDK

### MOT-001 · Plan / execute timeouts

```python
import asyncio

PLAN_TIMEOUT_S = 20.0
EXECUTE_TIMEOUT_S = 90.0


async def plan(mg, actions, tcp):
    async with asyncio.timeout(PLAN_TIMEOUT_S):
        return await mg.plan(actions, tcp)


async def execute(mg, trajectory, tcp, actions):
    async with asyncio.timeout(EXECUTE_TIMEOUT_S):
        await mg.execute(trajectory, tcp, actions=actions)
```

Cancelling (or timing out) the task that awaits `execute` / `plan_and_execute` stops the robot.
Treat a `TimeoutError` from `execute` as a fault: the robot stopped mid-path.

Symbols: `nova.MotionGroup.plan`, `.execute`, `.plan_and_execute` (`nova/cell/motion_group.py`,
`nova/cell/robot_cell.py`).

### SAF-003 · Stop motion on fault

```python
STOP_TIMEOUT_S = 5.0

motion_task = asyncio.create_task(execute(mg, trajectory, TCP, actions))


async def on_fault() -> None:
    motion_task.cancel()  # first: stops the robot
    try:
        async with asyncio.timeout(STOP_TIMEOUT_S):
            await asyncio.gather(motion_task, return_exceptions=True)
    except TimeoutError:
        logger.critical("[FAULT] motion task did not finish within %.1f s", STOP_TIMEOUT_S)
    # only now: IO defaults, logging, plugin cleanup
```

Do **not** call `MotionGroup.stop()`. With a `TrajectoryCursor` movement controller,
`await cursor.pause()` stops along the path and allows resuming
(`nova/cell/movement_controller/trajectory_cursor.py`).

### MOT-002 · TCP validation

```python
REQUIRED_TCPS = {cfg.TCP_GRIPPER, cfg.TCP_CAMERA}

missing = REQUIRED_TCPS - set(await mg.tcp_names())
if missing:
    raise RuntimeError(f"TCPs missing on {mg.id}: {sorted(missing)}")
```

Symbols: `MotionGroup.tcp_names()`, `.tcps()`, `.active_tcp_name()`, `.tcp_offset(tcp)`.
`tcp_names[0]` picks an arbitrary TCP and is a MOT-002 violation.

### MOT-009 · Motion group selection

```python
mg = controller.motion_group(cfg.MOTION_GROUP_ID)  # e.g. "0@ur10e"
if cfg.MOTION_GROUP_ID not in {m.id for m in await controller.motion_groups()}:
    raise RuntimeError(f"Motion group {cfg.MOTION_GROUP_ID} not found")
```

Avoid `controller[0]` (`Controller.__getitem__`).

### MOT-006 / SAF-013 · Speed profiles

`nova.types.MotionSettings` fields: `tcp_velocity_limit` (mm/s), `tcp_acceleration_limit`
(mm/s²), `tcp_orientation_velocity_limit` (rad/s), `joint_velocity_limits` (rad/s per joint),
`joint_acceleration_limits`, `blending`. Instances are immutable; derive variants with
`FAST.model_copy(update={...})`. The default `tcp_velocity_limit` is 50 mm/s.

### MOT-007 · Datasets and frames

`@nova.program(preconditions=ProgramPreconditions(dataset=remote_dataset("cell-a")))` loads the
dataset before the program runs; read it from `ctx.dataset`. `DatasetPose.as_world()` resolves
frame-relative poses locally. Example: `examples/datasets.py`, `examples/palletizing.py`.

### MOT-008 · Pose composition

`nova.types.Pose`: `target @ offset` = offset in the target/tool frame; `offset @ target` = in
the reference frame. Example: `examples/pose_transformations.py`.

### MOT-010 · Blending

`MotionSettings(blending=api.models.BlendingPosition(position_zone_radius=10))` or
`api.models.BlendingAuto(min_velocity_in_percent=...)`. `blending_radius` / `blending_auto` are
deprecated. Not supported on `collision_free` motions. Example: `examples/blending.py`.

### MOT-011 · Payload

`mg.plan(actions, tcp, payload_override="part_a")` (name of a payload registered on the
controller, or an `api.models.Payload`). Inspect with `mg.payloads()`, `mg.active_payload_name()`.
Only override when the physical controller is configured with the same payload.

### MOT-012 · Singularity handling

`mg.plan(..., singularity_handling=api.models.SingularityHandling.PALLETIZING_WRIST)`. Values:
`NONE` (default), `PALLETIZING_WRIST`, `ADAPTIVE_SAMPLING`. Experimental; pin the SDK version.

### MOT-013 · Controller limits

```python
limits = (await mg.get_description()).operation_limits.auto_limits
max_joint_velocity = [j.velocity for j in limits.joints]
max_tcp_velocity = limits.tcp.velocity if limits.tcp else None
```

### IO-002 · Bounded IO waits

```python
from nova.utils.io import wait_for_bus_io


async def wait_for_signal(io: str, expected: bool, timeout_s: float) -> None:
    try:
        async with asyncio.timeout(timeout_s):
            await wait_for_bus_io([io], on_change=lambda c: c[io].new_value is expected)
    except TimeoutError as e:
        raise TimeoutError(f"{io} did not become {expected} within {timeout_s} s") from e
```

`wait_for_bus_io` checks the current state first, so it returns immediately if the condition
already holds. It needs a connected NATS client (`nova.nats`). Controller IO equivalent:
`wait_for_bool_io(io, value)`, same wrapping.

### IO-003 / SAF-009 · Bulk IO

`nova.utils.io.get_bus_io_value([...])` reads several bus IOs in one request;
`set_bus_io_value({...})` writes several at once. Controller IO: `controller.read(key)` /
`controller.write(key, value)` are single-signal.

### PERF-002 · Push instead of poll

Bus IO: `wait_for_bus_io` (NATS push). Motion group: `mg.stream_state(response_rate_msecs)`
(one shared websocket per motion group). Controller: `controller.stream_state(rate_msecs)`.

### PERF-005 · Preplanning

`trajectory = await mg.plan(actions, tcp, start_joint_position=...)` at startup; `await
mg.execute(trajectory, tcp, actions=actions)` per cycle. Re-plan when poses, TCP, payload or
profiles change.

### PERF-008 · Path-triggered IO

```python
from nova.actions import before_target, io_write, linear

actions = [
    io_write("gripper_open", True, at=before_target(millimeters=50)),
    linear(pick_approach, settings=NORMAL),
]
```

Helpers: `after_start(seconds=|millimeters=)`, `before_target(seconds=|millimeters=)`,
`at_path_fraction(f)`. Example: `examples/path_triggers.py`.

### LIFE-003 / LIFE-004 · Connect and close

```python
async with Nova() as nova:  # always async with
    cell = nova.cell()
    for attempt in range(3):
        try:
            async with asyncio.timeout(45):
                controller = await cell.controller(cfg.CONTROLLER)
            break
        except TimeoutError:
            await asyncio.sleep(2**attempt)
    else:
        raise RuntimeError(f"Controller {cfg.CONTROLLER} not reachable")
```

`nova.exceptions.ControllerNotFound` is a configuration error; do not retry it.

### SAF-005 / ERR-004 · Fault classes

`nova.exceptions`: planning → `PlanTrajectoryFailed` (`.error`, `.to_pretty_string()`),
`NoInverseKinematicsSolutionFound` (`.pose`), `InconsistentCollisionScenes`; execution →
`InitMovementFailed`, `ErrorDuringMovement`, `LoadPlanFailed`.

### ERR-005 · Private SDK access

Private attributes such as `._api_client`, `._nova_api`, `._io_access`, `._current_motion` are not
stable. Use `nova.api` (raw generated client models) and the public `Nova.api` gateway instead,
behind one adapter module.

### STR-001 · Program ids

`@nova.program(id="pick_tray_1", name="Pick tray 1")`. Without `id=`, the function name is the id.
`nova.program.registry` replaces earlier programs with the same id without warning.

### OBS-001 / OBS-004 · Logging and cycles

`from nova.logging import logger` (level from `LOG_LEVEL`). `ctx.cycle()` returns a
`nova.events.Cycle` that publishes cycle start/finish/fail events. Example:
`examples/cycle_events.py`.

### TST-001 · Virtual cell

```python
from nova import ProgramPreconditions
from nova.cell import virtual_controller


@nova.program(
    id="pick_tray_1",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(name="ur10e", manufacturer=..., type="universalrobots-ur10e")
        ],
        cleanup_controllers=True,
    ),
)
async def pick_tray_1(ctx: nova.ProgramContext): ...
```

Run one program locally with `nova.run_program(...)`. See `docs/programs.md`.
