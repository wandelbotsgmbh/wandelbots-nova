#!/usr/bin/env python3

"""Quick test to verify that links are created when programs are registered."""

import asyncio

import nova
from nova.cell.simulation import SimulatedRobotCell
from novax.program_manager import ProgramManager


@nova.program(
    id="test_links_program",
    name="Test Links Program",
    description="A program to test links creation",
)
async def test_links_program():
    """A simple test program."""
    print("Hello from test links program!")


async def main():
    # Create a ProgramManager
    manager = ProgramManager(robot_cell_override=SimulatedRobotCell())

    # Register the program
    program_id = manager.register_program(test_links_program)
    print(f"Registered program with ID: {program_id}")

    # Get the program details
    program_details = await manager.get_program(program_id)

    if program_details:
        print(f"Program name: {program_details.name}")
        print(f"Program description: {program_details.description}")

        if program_details.links:
            print("Links created successfully:")
            print(f"  Self: {program_details.links.self}")
            print(f"  Start: {program_details.links.start}")
            print(f"  Stop: {program_details.links.stop}")
        else:
            print("ERROR: No links were created!")
    else:
        print("ERROR: Program not found!")


if __name__ == "__main__":
    asyncio.run(main())
