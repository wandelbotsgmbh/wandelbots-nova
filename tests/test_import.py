"""Guards that the top-level package imports cleanly in a fresh interpreter.

An import cycle inside `nova` (for example a submodule reaching back into
`nova.program`, which imports `from nova import Nova`) breaks a cold `import nova`
while passing both `ruff check` and `ty check`. In the test suite it shows up as
dozens of unrelated collection errors, so these tests name the failure directly.
"""

import subprocess
import sys

import pytest


def _import_in_subprocess(statement: str) -> None:
    """Run `statement` in a fresh interpreter, failing with its stderr on error."""
    result = subprocess.run(
        [sys.executable, "-c", statement], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, f"`{statement}` failed:\n{result.stderr}"


def test_importing_nova_has_no_cycle():
    _import_in_subprocess("import nova")


@pytest.mark.parametrize(
    "submodule", ["nova.datasets", "nova.program", "nova.actions", "nova.types", "nova.cell"]
)
def test_importing_a_submodule_first_has_no_cycle(submodule: str):
    """Importing a submodule before the package is the order that exposes cycles."""
    _import_in_subprocess(f"import {submodule}")
