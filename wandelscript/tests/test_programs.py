"""Test the execution of sample programs"""

import pytest

from wandelscript.metamodel import run_program

CODE_FUNC_DEF = """
move via p2p() to (home=(0, 0.1, 0, 0, 0, 0))
movedef circle(start >--> end):
    move via arc(start :: (0.2, 0.2, 0, 0, 0, 0)) to start :: (0, 0.4, 0, 0, 0, 0)
    print('action')
    move via arc(start :: (-0.2, 0.2, 0, 0, 0, 0)) to start :: (0, 0, 0, 0, 0, 0)
move via line() to (-1.2, 0, 0)
move via circle() to (0, 0, 0)
move via line() to (1, 0, 0)
move via circle() to (0, 0, 0)
move via line() to home
"""

CODE_SAMPLES = {
    "arc_move": "move via p2p() to (0, 0, 0, 0, 0, 0)\nmove via arc((0, 2, 0, 0, 0, 0)) to (1, 1, 0, 0, 0, 0)\n",
    "movedef": CODE_FUNC_DEF,
}


@pytest.mark.asyncio
@pytest.mark.parametrize("sample_code_key", CODE_SAMPLES)
async def test_execution(sample_code_key):
    sample_code = CODE_SAMPLES[sample_code_key]
    await run_program(sample_code, default_robot="0@controller", default_tcp="Flange")
