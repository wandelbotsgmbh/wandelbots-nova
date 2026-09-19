"""Fixture module with decorated programs used to test module-based autodiscovery.

Importing this module registers its programs in the global program registry.
"""

import nova


@nova.program(id="fixture_prog_one")
async def fixture_prog_one(ctx: nova.ProgramContext):
    pass


@nova.program(id="fixture_prog_two")
async def fixture_prog_two(ctx: nova.ProgramContext):
    pass
