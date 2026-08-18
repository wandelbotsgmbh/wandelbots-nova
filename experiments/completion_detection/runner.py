"""Run one completion-detection experiment: execute a short motion while
recording everything received on the MotionGroupState stream(s).

Execution paths under test (briefing §1):
  --mode sdk   motion is executed through the NOVA Python SDK (``MotionGroup.execute``,
               move_forward movement controller + TrajectoryExecutionMachine).
  --mode api   motion is executed through ``wandelbots_api_client`` only, with a
               minimal request generator (mirrors arg3-api).

Independently of the mode, N raw-websocket observers record the state stream at
the requested rates, and a low-rate REST poller samples the pull endpoint.

Examples (from the repo root):

  # provision a virtual UR10e if missing, one complete run, step-rate + 200 ms observers
  PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
      --mode api --virtual ur10e --label dev-k8s

  # pause/resume mid trajectory (api mode only)
  PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
      --mode api --scenario pause --virtual ur10e --label dev-k8s

  # SDK execution, 3 repetitions, observers at step rate, 50 ms and 200 ms
  PYTHONPATH=. uv run python -m experiments.completion_detection.runner \
      --mode sdk --runs 3 --rates step,50,200 --virtual ur10e --label dev-k8s
"""

from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

import wandelbots_api_client.v2_pydantic as apiv2

from experiments.completion_detection.common import (
    DATA_DIR,
    EventLog,
    access_token,
    cell_name,
    nova_host,
    parse_rate,
    rate_label,
)
from experiments.completion_detection.observer import StatePoller, StateStreamObserver
from nova import Nova, api
from nova.actions import ptp
from nova.cell.controllers import virtual_controller
from nova.types.pose import Pose

# Shorthands for --virtual; any full motion-group-model string works as well
# (list them with --list-robots). Two naming styles occur: the motion-group-model
# catalog says "KUKA_KR270_R2700", while the virtual-controller API only accepts
# the legacy lowercase-dash form "kuka-kr270_r2700" (a catalog-style type is
# rejected with "not found in available configurations"). resolve_virtual
# normalizes catalog names to the accepted form.
VIRTUAL_SHORTHANDS = {
    "ur10e": "universalrobots-ur10e",
    "ur5e": "universalrobots-ur5e",
    "kr16": "kuka-kr16_r2010_2",
    "kr270": "KUKA_KR270_R2700",
    "kr240": "KUKA_KR240_R2900",
}

_PAUSE = "pause"
_RESUME = "resume"
_FINISH = "finish"


def resolve_virtual(spec: str) -> tuple[api.models.Manufacturer, str]:
    """Resolve a --virtual spec (shorthand or model string) to (manufacturer, type).

    Accepts catalog names ("KUKA_KR270_R2700") and legacy virtual-controller
    types ("kuka-kr270_r2700"); always returns the legacy form the
    add-controller API accepts.
    """
    virtual_type = VIRTUAL_SHORTHANDS.get(spec, spec)
    if "-" not in virtual_type:
        # catalog style: Manufacturer_Model_... → manufacturer-model_...
        parts = virtual_type.split("_", 1)
        if len(parts) == 2:
            virtual_type = f"{parts[0].lower()}-{parts[1].lower()}"
    prefix = virtual_type.split("-", 1)[0].lower()
    try:
        manufacturer = api.models.Manufacturer(prefix)
    except ValueError:
        valid = ", ".join(m.value for m in api.models.Manufacturer)
        raise SystemExit(
            f"Cannot derive a manufacturer from '{virtual_type}' (prefix '{prefix}'). "
            f"Expected '<Manufacturer>_<Model>' or '<manufacturer>-<model>' with manufacturer "
            f"one of: {valid}. List the instance's supported models with --list-robots."
        )
    return manufacturer, virtual_type


async def list_robots() -> None:
    """Print the motion-group models this NOVA instance supports for virtual controllers."""
    client = _make_v2_client()
    try:
        models = await apiv2.MotionGroupModelsApi(client).get_motion_group_models(
            _request_timeout=15
        )
    finally:
        await client.close()
    for model in sorted(models):
        print(model)
    print(f"\n{len(models)} models on {nova_host()} — use one as --virtual <model>")


def _git_rev() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            cwd=Path(__file__).parent,
        ).stdout.strip()
    except Exception:
        return "unknown"


def _make_v2_client() -> apiv2.ApiClient:
    configuration = apiv2.Configuration(host=f"{nova_host()}/api/v2")
    token = access_token()
    if token:
        configuration.access_token = token
    return apiv2.ApiClient(configuration)


async def execute_via_api(
    *,
    v2_client: apiv2.ApiClient,
    cell: str,
    controller: str,
    motion_group_id: str,
    trajectory: api.models.JointTrajectory,
    tcp: str | None,
    planned_duration_s: float,
    scenario: str,
    pause_at_s: float,
    pause_hold_s: float,
    post_roll_s: float,
    events: EventLog,
) -> None:
    """arg3-api-style execution: cache trajectory, then drive the execution websocket
    with a minimal request generator. Run length is time-based on purpose — the run
    must not depend on any completion detector (the thing under test)."""

    caching_api = apiv2.TrajectoryCachingApi(v2_client)
    execution_api = apiv2.TrajectoryExecutionApi(v2_client)

    events.emit("add_trajectory_sent")
    add_response = await caching_api.add_trajectory(
        cell=cell,
        controller=controller,
        add_trajectory_request=api.models.AddTrajectoryRequest(
            motion_group=motion_group_id, trajectory=trajectory, tcp=tcp
        ),
    )
    if add_response.error is not None or add_response.trajectory is None:
        raise RuntimeError(f"add_trajectory failed: {add_response}")
    trajectory_id = add_response.trajectory
    events.emit("add_trajectory_ack", trajectory_id=trajectory_id)

    command_queue: asyncio.Queue[str] = asyncio.Queue()

    async def scenario_driver() -> None:
        if scenario == "pause":
            await asyncio.sleep(pause_at_s)
            command_queue.put_nowait(_PAUSE)
            await asyncio.sleep(pause_hold_s)
            command_queue.put_nowait(_RESUME)
            await asyncio.sleep(planned_duration_s + 2.0)
        else:
            await asyncio.sleep(planned_duration_s * 1.2 + 2.0)
        # Keep the execution websocket open through the post-roll so a potential
        # connection-close side effect cannot masquerade as the terminal signal.
        await asyncio.sleep(post_roll_s)
        command_queue.put_nowait(_FINISH)

    async def request_generator(response_stream):
        driver_task = asyncio.create_task(scenario_driver())
        consumer_task: asyncio.Task | None = None
        try:
            events.emit("init_sent", trajectory_id=trajectory_id)
            yield api.models.InitializeMovementRequest(
                trajectory=api.models.TrajectoryId(id=trajectory_id),
                initial_location=api.models.Location(0),
            )
            init_ack = await anext(response_stream)
            root = init_ack.root
            events.emit(
                "init_ack",
                response_type=type(root).__name__,
                detail=root.model_dump(mode="json", exclude_none=True),
            )
            if not isinstance(root, api.models.InitializeMovementResponse):
                raise RuntimeError(f"expected InitializeMovementResponse, got {root!r}")
            # The trap from briefing §2: acknowledgements carry failures.
            if root.message or root.add_trajectory_error:
                raise RuntimeError(f"initialize failed: {root!r}")

            async def consume_responses() -> None:
                async for response in response_stream:
                    events.emit(
                        "execution_response",
                        response_type=type(response.root).__name__,
                        detail=response.root.model_dump(mode="json", exclude_none=True),
                    )

            consumer_task = asyncio.create_task(consume_responses())

            events.emit("start_sent")
            yield api.models.StartMovementRequest(direction=api.models.Direction.DIRECTION_FORWARD)

            while True:
                command = await command_queue.get()
                if command == _FINISH:
                    events.emit("finish_requested")
                    return
                if command == _PAUSE:
                    events.emit("pause_sent")
                    yield api.models.PauseMovementRequest()
                elif command == _RESUME:
                    events.emit("resume_sent")
                    yield api.models.StartMovementRequest(
                        direction=api.models.Direction.DIRECTION_FORWARD
                    )
        finally:
            for task in (driver_task, consumer_task):
                if task is not None and not task.done():
                    task.cancel()

    await execution_api.execute_trajectory(
        cell=cell, controller=controller, client_request_generator=request_generator
    )
    events.emit("execution_ws_closed")


async def run_once(args: argparse.Namespace, run_dir: Path, run_index: int) -> dict[str, Any]:
    run_dir.mkdir(parents=True, exist_ok=True)
    events = EventLog()
    rates = [parse_rate(r) for r in args.rates.split(",")]

    async with Nova() as nova:
        cell = nova.cell(args.cell)

        controller_name = args.controller
        existing = await nova._api_client.controller_api.list_robot_controllers(cell=args.cell)
        if controller_name not in existing:
            if not args.virtual:
                raise SystemExit(
                    f"Controller '{controller_name}' not found in cell '{args.cell}' "
                    f"(found: {existing}). Pass --virtual <type> to provision one."
                )
            manufacturer, virtual_type = resolve_virtual(args.virtual)
            events.emit(
                "provision_virtual_controller",
                controller=controller_name,
                controller_type=virtual_type,
            )
            await cell.ensure_controller(
                virtual_controller(
                    name=controller_name, manufacturer=manufacturer, type=virtual_type
                )
            )
        controller = await cell.controller(controller_name)

        async with controller[0] as motion_group:
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]
            start_pose = await motion_group.tcp_pose(tcp)
            start_joints = await motion_group.joints()

            offset = Pose(0, 0, args.amplitude, 0, 0, 0)
            actions = [ptp(offset @ start_pose), ptp(start_pose)] * args.cycles

            events.emit("plan_sent", tcp=tcp, actions=len(actions))
            trajectory = await motion_group.plan(actions, tcp)
            planned_duration_s = float(trajectory.times[-1])
            planned_final_joints = list(trajectory.joint_positions[-1].root)
            events.emit("plan_done", duration_s=planned_duration_s, samples=len(trajectory.times))

            # --- observers -------------------------------------------------
            observers = [
                StateStreamObserver(
                    cell=args.cell,
                    controller=controller_name,
                    motion_group=motion_group.id,
                    response_rate=rate,
                )
                for rate in rates
            ]
            observer_tasks = [asyncio.create_task(o.run()) for o in observers]
            await asyncio.gather(*(o.started.wait() for o in observers))

            poller: StatePoller | None = None
            poller_task: asyncio.Task | None = None
            poller_client: apiv2.ApiClient | None = None
            if args.poll_interval > 0:
                poller_client = _make_v2_client()
                poller = StatePoller(
                    motion_group_api=apiv2.MotionGroupApi(poller_client),
                    cell=args.cell,
                    controller=controller_name,
                    motion_group=motion_group.id,
                    interval_s=args.poll_interval,
                )
                poller_task = asyncio.create_task(poller.run())

            events.emit("observers_started", rates=[rate_label(r) for r in rates])
            await asyncio.sleep(args.baseline_s)

            # --- execute ----------------------------------------------------
            execution_error: str | None = None
            try:
                if args.mode == "sdk":
                    events.emit("sdk_execute_started")
                    timeout = planned_duration_s * 3 + 15
                    if args.scenario == "pause":
                        raise SystemExit("--scenario pause is only supported with --mode api")
                    try:
                        await asyncio.wait_for(
                            motion_group.execute(trajectory, tcp, actions=actions), timeout=timeout
                        )
                        events.emit("sdk_execute_returned")
                    except asyncio.TimeoutError:
                        # The failure mode under investigation: execute() never completes.
                        events.emit("sdk_execute_timeout", timeout_s=timeout)
                        execution_error = f"sdk execute() timed out after {timeout:.1f}s"
                    await asyncio.sleep(args.post_roll_s)
                else:
                    v2_client = _make_v2_client()
                    try:
                        await execute_via_api(
                            v2_client=v2_client,
                            cell=args.cell,
                            controller=controller_name,
                            motion_group_id=motion_group.id,
                            trajectory=trajectory,
                            tcp=tcp,
                            planned_duration_s=planned_duration_s,
                            scenario=args.scenario,
                            pause_at_s=args.pause_at * planned_duration_s,
                            pause_hold_s=args.pause_hold_s,
                            post_roll_s=args.post_roll_s,
                            events=events,
                        )
                    finally:
                        await v2_client.close()
                    await asyncio.sleep(1.0)
            except (SystemExit, asyncio.CancelledError):
                raise
            except Exception as e:
                execution_error = f"{type(e).__name__}: {e}"
                events.emit("execution_error", error=execution_error)

            # --- stop observers, write everything ---------------------------
            for task in observer_tasks:
                task.cancel()
            if poller_task:
                poller_task.cancel()
            await asyncio.gather(*observer_tasks, return_exceptions=True)
            if poller_task:
                await asyncio.gather(poller_task, return_exceptions=True)
            if poller_client is not None:
                await poller_client.close()

            stream_stats = []
            for observer in observers:
                stats = observer.write(run_dir / f"frames_{observer.label}.jsonl")
                stream_stats.append(stats)
            if poller:
                poller.write(run_dir / "poll.jsonl")

            events.write(run_dir / "events.jsonl")

            metadata = {
                "run_index": run_index,
                "created": dt.datetime.now().isoformat(timespec="seconds"),
                "label": args.label,
                "nova_host": nova_host(),
                "cell": args.cell,
                "controller": controller_name,
                "motion_group": motion_group.id,
                "mode": args.mode,
                "scenario": args.scenario,
                "rates": [rate_label(r) for r in rates],
                "poll_interval_s": args.poll_interval,
                "amplitude_mm": args.amplitude,
                "cycles": args.cycles,
                "tcp": tcp,
                "start_joints": list(start_joints),
                "planned_duration_s": planned_duration_s,
                "planned_samples": len(trajectory.times),
                "planned_final_joints": planned_final_joints,
                "execution_error": execution_error,
                "stream_stats": stream_stats,
                "sdk_git_rev": _git_rev(),
                "python": sys.version.split()[0],
            }
            (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))
            return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["sdk", "api"], default=None)
    parser.add_argument("--scenario", choices=["complete", "pause"], default="complete")
    parser.add_argument("--cell", default=cell_name())
    parser.add_argument(
        "--controller",
        default=None,
        help="controller to use (must exist, or be provisioned via --virtual); "
        "defaults to cdexp-<robot> derived from --virtual (cdexp-ur10e without it)",
    )
    parser.add_argument(
        "--virtual",
        default=None,
        metavar="MODEL",
        help="provision this virtual robot when --controller does not exist: a shorthand "
        f"({', '.join(sorted(VIRTUAL_SHORTHANDS))}) or any motion-group model string "
        "such as 'kuka-kr270_r2700' (discover with --list-robots)",
    )
    parser.add_argument(
        "--list-robots",
        action="store_true",
        help="list the motion-group models this NOVA instance supports, then exit",
    )
    parser.add_argument(
        "--rates",
        default="step,200",
        help="comma-separated observer rates: 'step' (no response_rate param) or milliseconds",
    )
    parser.add_argument("--poll-interval", type=float, default=1.0, help="REST poll (s); 0=off")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--label", default="unlabelled", help="environment label, e.g. lan-ipc")
    parser.add_argument("--amplitude", type=float, default=150.0, help="Z offset in mm")
    parser.add_argument("--cycles", type=int, default=1, help="down/up repetitions")
    parser.add_argument("--baseline-s", type=float, default=1.0)
    parser.add_argument("--post-roll-s", type=float, default=3.0)
    parser.add_argument("--pause-at", type=float, default=0.4, help="fraction of planned duration")
    parser.add_argument("--pause-hold-s", type=float, default=2.0)
    parser.add_argument("--batch", default=None, help="reuse an existing batch directory name")
    args = parser.parse_args()

    if args.list_robots:
        asyncio.run(list_robots())
        return
    if args.mode is None:
        parser.error("--mode is required (sdk|api)")
    if args.virtual:
        resolve_virtual(args.virtual)  # fail on a bad spec before touching the robot
    if args.controller is None:
        base = args.virtual or "ur10e"
        parts = re.split(r"[-_]", base, maxsplit=1)
        model = parts[1] if len(parts) > 1 else parts[0]
        args.controller = f"cdexp-{model.replace('_', '-').lower()}"

    batch = args.batch or f"{dt.datetime.now():%Y%m%d-%H%M%S}_{args.label}"
    batch_dir = DATA_DIR / batch

    for run_index in range(1, args.runs + 1):
        # never clobber an existing run when reusing a batch directory
        n = run_index
        while (batch_dir / f"run{n:02d}_{args.mode}_{args.scenario}").exists():
            n += 1
        run_dir = batch_dir / f"run{n:02d}_{args.mode}_{args.scenario}"
        print(f"→ {run_dir.relative_to(DATA_DIR.parent)}")
        metadata = asyncio.run(run_once(args, run_dir, run_index))
        status = metadata["execution_error"] or "ok"
        frames = ", ".join(f"{s['rate']}:{s['frames']}" for s in metadata["stream_stats"])
        print(f"  {status} | planned {metadata['planned_duration_s']:.2f}s | frames {frames}")

    print(f"\nBatch complete: {batch_dir}")
    print(
        "Analyze:   PYTHONPATH=. uv run python -m experiments.completion_detection.analyze "
        f"{batch_dir}"
    )
    print(
        "Visualize: PYTHONPATH=. uv run python -m experiments.completion_detection.visualize "
        f"{batch_dir}"
    )


if __name__ == "__main__":
    main()
