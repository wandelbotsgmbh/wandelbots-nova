"""
Isolated example: the ``computed`` hooks, shown through one concrete scenario.

Scenario: a force-compliant pick-and-place policy. The model was trained with a
wrist-mounted 6-axis force/torque sensor as an extra observation channel, and the
deployment requires every commanded action to be journaled for traceability. The
F/T sensor and the journal are both out-of-band: they are not exposed through the
robot controller's joint or I/O interface, so they cannot be declared as ordinary
observation/action sources. The ``computed`` hooks splice them into the executor's
per-step ``observe -> infer -> act`` loop:

  * Pre-inference, ``Observation.computed(read_sensor)`` is awaited and its return
    dict is merged into the observation passed to the policy. This injects the
    F/T reading under ``obs["sensor"]`` as an additional input feature.
  * Post-inference, ``Action.computed(report_action)`` is awaited with the
    ``ActionChunk`` emitted by the policy. It is a pure sink (returns ``None``) and is
    decoupled from actuation — the joint targets are dispatched to the controller
    through the normal action pipeline regardless.

This stand-in stubs both hooks (constant sensor value, hold-position policy) so it
executes against a single virtual UR5e with no physical hardware or model server.
In production, replace ``read_sensor`` with the sensor driver read and
``report_action`` with the journal sink (file, time-series DB, PLC register).

Run:
    NOVA_API=http://<instance-ip> PYTHONPATH=. python novapolicy/examples/computed/computed_action.py
"""

from typing import Any

from nova import ProgramContext, api, program, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from novapolicy import Action, ActionChunk, Observation, PolicyExecutor, PolicySchema

HOME = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)
MG_ID = "0@ur5e"


# --- IN: a computed observation -------------------------------------------
# Runs every step before the policy. Whatever dict it returns is merged into
# the observation, so the policy can read it like any other observation key.


async def read_sensor(obs: dict[str, Any]) -> dict[str, float]:
    """Read the wrist force sensor and add it to the observation.

    A real version would await a hardware/network read; here it's a constant.
    """
    return {"sensor": 42.0}


# --- OUT: a computed action -----------------------------------------------
# Runs every step after the policy, with the raw action dict it produced.

_step = 0


async def report_action(chunk: ActionChunk) -> None:
    """Side effect: log what the policy decided (the first step of its chunk)."""
    global _step
    _step += 1
    if _step % 25 == 0:
        joints = [round(j, 3) for j in chunk.joints[MG_ID][0]]
        print(f"  [computed action] step={_step} target={joints}")


# --- The policy -----------------------------------------------------------


async def mock_policy(obs: dict[str, Any]) -> ActionChunk:
    """Hold position. ``obs['sensor']`` is the wrist force from Observation.computed."""
    assert obs["sensor"] == 42.0  # the injected sensor reading is visible to the policy
    return ActionChunk(joints={MG_ID: [[obs[f"arm_{i}"] for i in range(1, 7)]]})


@program(
    id="computed-action-example",
    name="Computed Observation/Action Example",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def computed_action_example(ctx: ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e"))[0]

    fast = MotionSettings(tcp_velocity_limit=200.0)
    await mg.execute(
        await mg.plan([joint_ptp(HOME, settings=fast)], "Flange"),
        "Flange",
        actions=[joint_ptp(HOME, settings=fast)],
    )

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=mg),
            # Adds the wrist force as "sensor" to every observation, before the policy.
            Observation.computed(read_sensor),
        ],
        # Runs every step after the policy, with its raw action dict.
        actions=[Action.computed(report_action)],
    )

    executor = PolicyExecutor(schema, mock_policy, timeout_s=5.0, policy_rate_hz=20)

    print("Running for 5s — watch for [computed action] lines...")
    result = await executor.run()
    print(f"Done: reason={result.reason} steps={result.steps}")


if __name__ == "__main__":
    run_program(computed_action_example)
