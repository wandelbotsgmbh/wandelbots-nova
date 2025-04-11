import asyncio
import pathlib

from nova import program
from nova.program import ProgramParameter


async def main():
    # Read the example program file
    program_file = pathlib.Path(__file__).parent / "02_plan_and_execute.py"
    program_text = program_file.read_text()

    # Create parameter instance
    params = ProgramParameter(number_of_picks=3)

    print(program_text)
    # Run the program in sandboxed environment
    await program.SandboxedProgramRunner.run(program_text, params)

if __name__ == "__main__":
    asyncio.run(main())
