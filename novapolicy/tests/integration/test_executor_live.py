"""Live dual-arm tests for PolicyExecutor against a real NOVA instance.

Each test provisions its own throwaway *virtual* controllers with unique names
(the same create-on-demand pattern the core nova integration tests use) and
deletes them on teardown, so the tests are self-contained and never touch a
pre-existing controller. They drive *two* robots through the full pipeline end
to end: schema -> executor -> waypoint jogging session -> NOVA motion API ->
state stream. They require NOVA_API / NOVA_ACCESS_TOKEN and are skipped unless
run with `-m integration`.
"""

from __future__ import annotations

import contextlib
import uuid

import pytest

from novapolicy.executor import PolicyExecutor
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import ActionChunk, StopContext


async def _new_virtual_arm(cell, stack: contextlib.AsyncExitStack):
    """Provision a uniquely-named virtual UR5e and schedule its deletion.

    Returns the controller. The controller is removed from the cell when the
    ``stack`` unwinds, so each test leaves the instance as it found it.
    """
    from nova import api
    from nova.cell import virtual_controller

    name = f"policy-itest-{uuid.uuid4().hex[:8]}"
    controller = await cell.ensure_controller(
        virtual_controller(
            name=name,
            manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
            type="universalrobots-ur5e",
        )
    )

    async def _cleanup() -> None:
        with contextlib.suppress(Exception):
            await cell.delete_robot_controller(name, timeout=25)

    stack.push_async_callback(_cleanup)
    return controller


@pytest.mark.integration
@pytest.mark.asyncio
async def test_executor_jogs_both_arms_for_the_full_timeout():
    """Run a dual-arm policy for 5 s and confirm both arms jog for ~5 s.

    Proves two things end to end against a live instance: (1) a timeout_s of 5
    really keeps the robots jogging for about 5 s (the executor does not return
    early), and (2) *both* motion groups are driven — both appear in the final
    observed state. Each arm gets a small sinusoid so there is genuine motion.
    """
    import math
    import time

    from nova import Nova

    async with Nova() as nova_instance, contextlib.AsyncExitStack() as stack:
        cell = nova_instance.cell()
        controllers = [await _new_virtual_arm(cell, stack) for _ in range(2)]
        mgs = [await stack.enter_async_context(c[0]) for c in controllers]
        homes = {mg.id: list(await mg.joints()) for mg in mgs}

        async def dual_wiggle(obs):
            elapsed = obs.get("elapsed_s", 0.0)
            joints = {}
            for mg in mgs:
                home = homes[mg.id]
                steps = []
                for i in range(8):  # 8 steps * 50 ms = 400 ms per chunk
                    t = elapsed + i * 0.05
                    target = list(home)
                    target[0] = home[0] + 0.05 * math.sin(2 * math.pi * 0.3 * t)
                    steps.append(target)
                joints[mg.id] = steps
            return ActionChunk(joints=joints, dt_ms=50.0)

        schema = PolicySchema(
            observations=[
                Observation.joint_positions(f"arm{i}", source=mg) for i, mg in enumerate(mgs)
            ]
        )
        executor = PolicyExecutor(
            schema,
            CallbackPolicyClient(dual_wiggle),
            timeout_s=5.0,
        )

        wall_start = time.monotonic()
        result = await executor.run()
        wall_elapsed = time.monotonic() - wall_start

        mg_ids = {mg.id for mg in mgs}

    # Ended because the timeout elapsed, not for any other reason.
    assert result.reason == "timeout"
    # Jogged for ~5 s: the loop runs the full timeout (small poll/chunk overhang).
    assert 4.8 <= result.duration_s <= 6.5, result.duration_s
    assert wall_elapsed >= 4.8
    # Both arms were actually driven — both show up in the final observed state.
    assert result.last_state is not None
    assert set(result.last_state) == mg_ids
    assert result.steps > 0


@pytest.mark.integration
@pytest.mark.asyncio
async def test_stop_condition_ends_dual_arm_run_normally():
    """A stop condition fired during a live dual-arm run ends the run normally.

    The condition is evaluated every loop iteration; after a few ticks it
    returns True and the run ends with the condition's name in result.reason
    (no exception, no failure) while both arms were being jogged.
    """
    from nova import Nova

    async with Nova() as nova_instance, contextlib.AsyncExitStack() as stack:
        cell = nova_instance.cell()
        controllers = [await _new_virtual_arm(cell, stack) for _ in range(2)]
        mgs = [await stack.enter_async_context(c[0]) for c in controllers]
        homes = {mg.id: list(await mg.joints()) for mg in mgs}

        async def hold(obs):
            return ActionChunk(joints={mg.id: [homes[mg.id]] for mg in mgs})

        ticks = {"n": 0}

        def stop_after_a_few_ticks(ctx: StopContext) -> bool:
            ticks["n"] += 1
            return ticks["n"] >= 5

        schema = PolicySchema(
            observations=[
                Observation.joint_positions(f"arm{i}", source=mg) for i, mg in enumerate(mgs)
            ]
        )
        executor = PolicyExecutor(
            schema,
            CallbackPolicyClient(hold),
            stop_conditions=[stop_after_a_few_ticks],
            timeout_s=10.0,
        )

        result = await executor.run()
        mg_ids = {mg.id for mg in mgs}

    assert result.reason == "stop condition: stop_after_a_few_ticks"
    assert result.last_state is not None
    assert set(result.last_state) == mg_ids
