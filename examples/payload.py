"""Showcase per-call payload override for trajectory planning.

The planner uses payload (mass, center of mass, moment of inertia) to compute
torque limits and acceleration scaling. By default each motion group resolves
the payload using the precedence documented in ``MotionGroup.get_setup``:

    1. caller-supplied ``payload=`` argument
    2. payload registered under the same name as the active TCP (convention)
    3. the controller's currently selected payload (``MotionGroupState.payload``)
    4. the only registered payload, if exactly one is configured
    5. otherwise no payload (planner falls back to defaults)

This example demonstrates inspecting registered payloads, planning with the
default, and overriding the payload for a specific plan call — both by
referencing a registered payload name and by passing an ad-hoc ``Payload``.
"""

from math import pi

import nova
from nova import api, run_program
from nova.actions import jnt, ptp
from nova.cell.controllers import virtual_controller
from nova.program import ProgramPreconditions
from nova.types.pose import Pose


@nova.program(
    name="Payload Override",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.cell
    controller = await cell.controller("ur10e")

    async with controller[0] as motion_group:
        # 1) Inspect what the controller knows about payloads.
        registered = await motion_group.payloads()
        print(f"Registered payloads: {list(registered)}")
        print(f"Active payload on controller: {await motion_group.active_payload_name()!r}")

        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0]
        home_joints = (0, -pi / 2, -pi / 2, -pi / 2, pi / 2, -pi / 2)
        home = (693.5, -174.1, 676.9, -3.1416, 0, 0)
        actions = [jnt(home_joints), ptp(Pose(0, 0, 200, 0, 0, 0) @ home), jnt(home_joints)]

        # 2) Default behaviour: payload resolved via the precedence above.
        await motion_group.plan_and_execute(actions, tcp)

        # 3) Override by referencing a registered payload name (if available).
        #    This raises KeyError when the name is unknown — fall back to default.
        if "heavy_gripper" in registered:
            await motion_group.plan_and_execute(actions, tcp, payload="heavy_gripper")

        # 4) Override with an ad-hoc Payload that does not need to be registered.
        custom = api.models.Payload(
            name="custom_2_5kg",
            payload=2.5,
            center_of_mass=api.models.Vector3d(root=[0.0, 0.0, 60.0]),
        )
        await motion_group.plan_and_execute(actions, tcp, payload=custom)


if __name__ == "__main__":
    run_program(main)
