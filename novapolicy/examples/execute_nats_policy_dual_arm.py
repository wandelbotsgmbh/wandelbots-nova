"""Run a remote dual-arm policy served over NATS through ``PolicyExecutor``.

The model lives in a separate policy-service process reached over the instance's
broker; this script carries no model, no framework and no checkpoint. Observation
and action spaces are negotiated at connect time, so a DOF or layout mismatch
fails during connect rather than as unexpected motion.

The policy this was written against is **horizon-1**: one joint target per
predict, at a 20 ms control period. A lone waypoint is a *terminal* target the
server brakes to a standstill on, so the session tops each chunk up to
``WaypointConfig.min_chunk_horizon_ms`` by holding its final target — that
padding is the braking horizon, not commanded motion. ``--min-chunk-horizon-ms``
exposes it because the safe horizon is a property of the controller.

Run from the repository root. Review the arguments first::

    PYTHONPATH=. uv run --extra novapolicy-nats \\
        python novapolicy/examples/execute_nats_policy_dual_arm.py --help

Add ``--yes`` only when you are ready to execute robot motion::

    PYTHONPATH=. uv run --extra novapolicy-nats \\
        python novapolicy/examples/execute_nats_policy_dual_arm.py \\
        --nova-api http://<nova-host> \\
        --broker ws://<nova-host>/api/nats \\
        --policy-id <policy-id> \\
        --controller <dual-arm-controller> \\
        --yes

This attaches to an instance and a policy service that are already running; it
starts neither. The policy service must be reachable on the **same** broker —
that is where the ``policy.<id>.*`` subjects meet.

``--left`` and ``--right`` are motion-group indices on ``--controller``, and
their order is the binding contract: the first binds to the policy's first
declared block, the second to its second.
"""

from __future__ import annotations

import argparse
import asyncio
import logging

from nova import Nova, NovaConfig
from novapolicy import (
    ContinuousExecution,
    EmergencyStopError,
    MotionError,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WaypointConfig,
)
from novapolicy.nats import NatsPolicyClient


async def run(args: argparse.Namespace) -> None:
    async with Nova(config=NovaConfig(host=args.nova_api)) as nova:
        controller = await nova.cell(args.cell).controller(args.controller)
        # Declaration order becomes PolicySchema.get_motion_groups() order,
        # which becomes the policy's block binding order.
        left, right = controller[args.left], controller[args.right]

        # The keys name the schema's own action targets; the policy's observation
        # and action field names come from its PolicyInfo, not from here.
        schema = PolicySchema(
            observations=[
                Observation.joint_positions("left", source=left),
                Observation.joint_positions("right", source=right),
            ]
        )
        policy = NatsPolicyClient(
            policy_id=args.policy_id,
            servers=args.broker,
            subject_prefix=args.subject_prefix,
            request_timeout_s=args.request_timeout_s,
        )
        execution = (
            SequentialExecution() if args.sequential else ContinuousExecution(rate_hz=args.rate_hz)
        )
        executor = PolicyExecutor(
            schema,
            policy,
            execution=execution,
            # Execute every step the policy returns; for a horizon-1 policy
            # that is the single target it predicted.
            n_action_steps=0,
            timeout_s=args.duration,
            motion=WaypointConfig(min_chunk_horizon_ms=args.min_chunk_horizon_ms),
        )

        print(f"Running '{args.policy_id}' on {left.id} + {right.id} for {args.duration}s...")
        try:
            result = await executor.run()
        except MotionError as exc:
            raise SystemExit(f"Motion error: {exc}") from exc
        except EmergencyStopError as exc:
            raise SystemExit(f"Emergency stop: {exc.controller_id}") from exc
        print(
            f"Done: reason={result.reason} steps={result.steps} duration={result.duration_s:.1f}s"
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nova-api", required=True, help="NOVA instance base URL")
    parser.add_argument(
        "--broker",
        required=True,
        help="NATS broker shared with the policy service, e.g. ws://<host>/api/nats",
    )
    parser.add_argument(
        "--policy-id", required=True, help="Subject-tree identity the policy service serves under"
    )
    parser.add_argument("--cell", default="cell")
    parser.add_argument("--controller", required=True, help="Controller hosting both arms")
    parser.add_argument("--left", type=int, default=1, help="Left-arm motion-group index")
    parser.add_argument("--right", type=int, default=2, help="Right-arm motion-group index")
    parser.add_argument("--subject-prefix", default="policy")
    parser.add_argument("--request-timeout-s", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=10.0, help="Episode bound in seconds")
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Control rate; must match the policy's declared dt_ms (20 ms -> 50 Hz)",
    )
    parser.add_argument(
        "--min-chunk-horizon-ms",
        type=float,
        default=WaypointConfig().min_chunk_horizon_ms,
        help="Braking horizon a short chunk is padded to; 0 disables the padding",
    )
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="Settle each chunk to standstill before re-inferring (stop-go for a closed-loop policy)",
    )
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--yes", action="store_true", help="Confirm execution")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.yes:
        raise SystemExit(
            "This example moves the robot. Re-run with --yes after checking the arguments."
        )
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s [%(levelname)8s] %(name)s: %(message)s",
    )
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
