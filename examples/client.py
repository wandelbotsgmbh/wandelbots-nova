import asyncio
import pathlib

from nova.runtime import uv_runner


async def main():
    # Read the example program file
    program_file = pathlib.Path(__file__).parent / "10_standalone_program.py"
    program_text = program_file.read_text()

    print(program_text)
    # Run the program in sandboxed environment
    # await program.SandboxedProgramRunner.run(program_text, params)
    await uv_runner.run_program_endpoint(program_text, args={"number_of_picks": 3})


if __name__ == "__main__":
    asyncio.run(main())
