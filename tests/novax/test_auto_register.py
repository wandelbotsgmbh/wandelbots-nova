import sys
from pathlib import Path

import pytest

import nova
from nova.program import Program, clear_registry, get_registered_programs
from novax import Novax
from novax.novax import _import_module


@pytest.fixture(autouse=True)
def _clean_registry():
    """Isolate each test from registry state leaked by other modules/imports."""
    clear_registry()
    yield
    clear_registry()


# --- registry -------------------------------------------------------------


def test_decorator_registers_program():
    @nova.program(id="reg_prog_a")
    async def prog_a(ctx: nova.ProgramContext):
        pass

    ids = [p.program_id for p in get_registered_programs()]
    assert "reg_prog_a" in ids


def test_bare_decorator_registers_under_function_name():
    @nova.program
    async def bare_program(ctx: nova.ProgramContext):
        pass

    ids = [p.program_id for p in get_registered_programs()]
    assert "bare_program" in ids


def test_redecorating_same_id_replaces_entry():
    @nova.program(id="reg_dup")
    async def first(ctx: nova.ProgramContext):
        pass

    @nova.program(id="reg_dup")
    async def second(ctx: nova.ProgramContext):
        pass

    matches = [p for p in get_registered_programs() if p.program_id == "reg_dup"]
    assert len(matches) == 1


def test_clear_registry_empties_registry():
    @nova.program(id="to_be_cleared")
    async def prog(ctx: nova.ProgramContext):
        pass

    assert get_registered_programs()

    clear_registry()

    assert get_registered_programs() == []


def test_get_registered_programs_returns_program_objects():
    @nova.program(id="typed_prog")
    async def prog(ctx: nova.ProgramContext):
        pass

    programs = get_registered_programs()

    assert all(isinstance(p, Program) for p in programs)
    assert {p.program_id for p in programs} == {"typed_prog"}


# --- Novax.auto_register --------------------------------------------------


def test_auto_register_registers_all():
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


def test_auto_register_empty_registry_returns_empty_list():
    novax = Novax(app_name="novax_auto_test")

    assert novax.auto_register() == []


def test_auto_register_is_idempotent():
    @nova.program(id="idem_prog")
    async def prog(ctx: nova.ProgramContext):
        pass

    novax = Novax(app_name="novax_auto_test")

    first = novax.auto_register()
    second = novax.auto_register()

    assert first == ["idem_prog"]
    assert second == ["idem_prog"]
    # Still registered exactly once.
    assert novax.program_manager.has_program("idem_prog")


def test_auto_register_updates_redecorated_program():
    @nova.program(id="update_prog", name="Original")
    async def original(ctx: nova.ProgramContext):
        pass

    novax = Novax(app_name="novax_auto_test")
    novax.auto_register()
    assert novax.program_manager._programs["update_prog"].name == "Original"

    # Re-decorate the same id with a new definition and re-run auto_register.
    @nova.program(id="update_prog", name="Updated")
    async def updated(ctx: nova.ProgramContext):
        pass

    novax.auto_register()

    assert novax.program_manager._programs["update_prog"].name == "Updated"


def test_constructor_with_app_auto_registers():
    @nova.program(id="ctor_prog")
    async def prog(ctx: nova.ProgramContext):
        pass

    app = Novax(app_name="novax_ctor_test").create_app()
    novax = Novax(app, app_name="novax_ctor_test")

    assert novax.program_manager.has_program("ctor_prog")


# --- Novax.register_module ------------------------------------------------


def test_register_module_from_module_object():
    # Drop any cached import so the decorators re-run and re-populate the registry.
    sys.modules.pop("tests.novax.fixtures_autodiscover", None)
    import tests.novax.fixtures_autodiscover as fixture_module

    novax = Novax(app_name="novax_module_test")
    registered = novax.register_module(fixture_module)

    assert "fixture_prog_one" in registered
    assert "fixture_prog_two" in registered
    assert novax.program_manager.has_program("fixture_prog_one")
    assert novax.program_manager.has_program("fixture_prog_two")


def test_register_module_from_dotted_path():
    # Drop any cached import so the decorators re-run and re-populate the registry.
    sys.modules.pop("tests.novax.fixtures_autodiscover", None)

    novax = Novax(app_name="novax_module_test")
    registered = novax.register_module("tests.novax.fixtures_autodiscover")

    assert "fixture_prog_one" in registered
    assert "fixture_prog_two" in registered


def test_register_module_from_file_path():
    # Drop any cached import so the decorators re-run and re-populate the registry.
    sys.modules.pop("fixtures_autodiscover", None)
    fixture_file = str(Path(__file__).parent / "fixtures_autodiscover.py")

    novax = Novax(app_name="novax_module_test")
    registered = novax.register_module(fixture_file)

    assert "fixture_prog_one" in registered
    assert "fixture_prog_two" in registered


# --- _import_module -------------------------------------------------------


def test_import_module_dotted_path():
    module = _import_module("tests.novax.fixtures_autodiscover")

    assert hasattr(module, "fixture_prog_one")


def test_import_module_file_path():
    fixture_file = str(Path(__file__).parent / "fixtures_autodiscover.py")

    module = _import_module(fixture_file)

    assert hasattr(module, "fixture_prog_one")


def test_import_module_file_path_reuses_cached_instance():
    fixture_file = str(Path(__file__).parent / "fixtures_autodiscover.py")

    first = _import_module(fixture_file)
    second = _import_module(fixture_file)

    assert first is second


def test_import_module_unknown_raises():
    with pytest.raises(ModuleNotFoundError):
        _import_module("tests.novax.does_not_exist_module")


# --- Novax.scan_programs --------------------------------------------------


def _write_program(directory: Path, filename: str, program_id: str) -> None:
    """Create a ``.py`` file under ``directory`` defining a single @nova.program."""
    directory.mkdir(parents=True, exist_ok=True)
    (directory / filename).write_text(
        "import nova\n\n\n"
        f"@nova.program(id={program_id!r})\n"
        "async def prog(ctx: nova.ProgramContext):\n"
        "    pass\n"
    )


def test_scan_programs_registers_files_in_directory(tmp_path):
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir, "first.py", "scan_first")
    _write_program(programs_dir, "second.py", "scan_second")

    novax = Novax(app_name="novax_scan_test", programs=str(programs_dir))
    registered = novax.scan_programs()

    assert "scan_first" in registered
    assert "scan_second" in registered
    assert novax.program_manager.has_program("scan_first")
    assert novax.program_manager.has_program("scan_second")


def test_scan_programs_skips_underscore_files(tmp_path):
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir, "visible.py", "scan_visible")
    _write_program(programs_dir, "_hidden.py", "scan_hidden")
    _write_program(programs_dir, "__init__.py", "scan_init")

    novax = Novax(app_name="novax_scan_test", programs=str(programs_dir))
    registered = novax.scan_programs()

    assert "scan_visible" in registered
    assert "scan_hidden" not in registered
    assert "scan_init" not in registered


def test_scan_programs_is_recursive(tmp_path):
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir / "nested", "deep.py", "scan_deep")

    novax = Novax(app_name="novax_scan_test", programs=str(programs_dir))
    registered = novax.scan_programs()

    assert "scan_deep" in registered


def test_scan_programs_same_filename_in_different_subdirs(tmp_path):
    # Two files sharing a stem ("prog") must both register: scanning names each
    # module after its location, so the second does not collide with the first
    # in ``sys.modules`` and get silently skipped.
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir / "a", "prog.py", "scan_a_prog")
    _write_program(programs_dir / "b", "prog.py", "scan_b_prog")

    novax = Novax(app_name="novax_scan_test", programs=str(programs_dir))
    registered = novax.scan_programs()

    assert "scan_a_prog" in registered
    assert "scan_b_prog" in registered


def test_scan_programs_missing_directory_returns_empty(tmp_path):
    novax = Novax(app_name="novax_scan_test", programs=str(tmp_path / "does_not_exist"))

    assert novax.scan_programs() == []


def test_scan_programs_disabled_when_programs_none(tmp_path):
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir, "ignored.py", "scan_ignored")

    novax = Novax(app_name="novax_scan_test", programs=None)

    assert novax.scan_programs() == []
    assert not novax.program_manager.has_program("scan_ignored")


def test_scan_programs_directory_argument_overrides_default(tmp_path):
    programs_dir = tmp_path / "custom"
    _write_program(programs_dir, "custom.py", "scan_custom")

    novax = Novax(app_name="novax_scan_test", programs=None)
    registered = novax.scan_programs(programs_dir)

    assert "scan_custom" in registered


def test_scan_programs_keeps_imported_programs_registered(tmp_path):
    # A program defined/imported outside the scanned directory.
    @nova.program(id="scan_imported")
    async def imported(ctx: nova.ProgramContext):
        pass

    programs_dir = tmp_path / "programs"
    _write_program(programs_dir, "scanned.py", "scan_scanned")

    novax = Novax(app_name="novax_scan_test", programs=str(programs_dir))
    registered = novax.scan_programs()

    assert "scan_imported" in registered
    assert "scan_scanned" in registered


def test_constructor_with_app_scans_programs_directory(tmp_path):
    programs_dir = tmp_path / "programs"
    _write_program(programs_dir, "ctor_scanned.py", "scan_ctor")

    app = Novax(app_name="novax_scan_test").create_app()
    novax = Novax(app, app_name="novax_scan_test", programs=str(programs_dir))

    assert novax.program_manager.has_program("scan_ctor")
