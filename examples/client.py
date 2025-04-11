import asyncio
import pathlib

from nova import program


async def main():
    # Read the example program file
    program_file = pathlib.Path(__file__).parent / "02_plan_and_execute.py"
    program_text = program_file.read_text()

    print(program_text)
    # Run the program in sandboxed environment
    # await program.SandboxedProgramRunner.run(program_text, params)
    await program.run_program_endpoint(program_text, parameters={
        "number_of_picks": 3
    })

if __name__ == "__main__":
    asyncio.run(main())
