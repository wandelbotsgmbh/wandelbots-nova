"""
Example: Observe a controller IO and invert another based on its value.

This example demonstrates how to monitor a digital input on a KUKA controller
and reactively set another digital output to the inverted value.

Behavior:
  - Uses `wait_for_bool_io` to block until `digital_in[2]` reaches a specific value
  - When it does, writes the inverted value to `digital_out[1]`
  - Loops, alternating the expected value each time

Prerequisites:
  - A NOVA instance
  - Run this example script:
      uv run python examples/controller_io_observer.py
  - Change the value of `digital_in[2]` (e.g. via the signals application)
    to see `digital_out[1]` update to its inverse.
  - Press Ctrl+C to stop.
"""

import asyncio

import nova
from nova import run_program
from nova.cell import kuka_controller


@nova.program(
    id="controller_io_observer",
    name="Controller IO Observer",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            kuka_controller(
                name="kuka",
                controller_ip="192.168.101.11",
                controller_port=54600,
                rsi_server_ip="192.168.102.10",
                rsi_server_port=30152,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext) -> None:
    cell = ctx.cell
    controller = await cell.controller("kuka")

    observed_io = "OUT#2"
    target_io = "OUT#1"

    # Read initial state and synchronise target
    last_value = bool(await controller.read(observed_io))
    inverted = not last_value
    await controller.write(target_io, inverted)
    print(f"Initial {observed_io}={last_value} -> {target_io}={inverted}")

    # Wait for changes and invert
    print(f"Observing {observed_io} – change it to see {target_io} toggle (Ctrl+C to stop)...")
    try:
        # We alternate: wait for the IO to become the opposite of what we last saw
        expected = not last_value
        while True:
            # await controller._io_access.wait_for_bool_io(observed_io, expected)
            await controller._io_access.wait_for_bool_io(observed_io, True)
            inverted = not expected
            await controller.write(target_io, inverted)
            print(f"{observed_io} became {expected} -> {target_io} set to {inverted}")
            expected = not expected
            await asyncio.sleep(2)
            await controller.write(observed_io, False)
    except asyncio.CancelledError:
        pass

    print("Observer stopped.")


if __name__ == "__main__":
    run_program(main)
