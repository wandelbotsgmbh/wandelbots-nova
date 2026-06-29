import nova
from nova.program import clear_registry, get_registered_programs
from novax import Novax


def test_decorator_registers_program():
    clear_registry()

    @nova.program(id="reg_prog_a")
    async def prog_a(ctx: nova.ProgramContext):
        pass

    ids = [p.program_id for p in get_registered_programs()]
    assert "reg_prog_a" in ids


def test_redecorating_same_id_replaces_entry():
    clear_registry()

    @nova.program(id="reg_dup")
    async def first(ctx: nova.ProgramContext):
        pass

    @nova.program(id="reg_dup")
    async def second(ctx: nova.ProgramContext):
        pass

    matches = [p for p in get_registered_programs() if p.program_id == "reg_dup"]
    assert len(matches) == 1


def test_auto_register_registers_all():
    clear_registry()

    @nova.program(id="auto_one")
    async def one(ctx: nova.ProgramContext):
        pass

    @nova.program(id="auto_two")
    async def two(ctx: nova.ProgramContext):
        pass

    novax = Novax(app_name="novax_auto_test")
    registered = novax.auto_register()

    assert "auto_one" in registered
    assert "auto_two" in registered
    assert novax.program_manager.has_program("auto_one")
    assert novax.program_manager.has_program("auto_two")
