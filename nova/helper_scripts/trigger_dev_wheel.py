"""Entry point for the ``dev-wheel`` uv script.

Runs ``scripts/trigger_dev_wheel.sh`` from the repository root, forwarding any
extra CLI arguments. This is a developer helper and expects to be run from a
checkout of the repository (the shell script is not shipped in the wheel).
"""

import subprocess
import sys
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parents[2] / "scripts" / "trigger_dev_wheel.sh"


def main() -> None:
    if not SCRIPT_PATH.is_file():
        sys.exit(f"Cannot find {SCRIPT_PATH}. Run this from a repository checkout.")
    sys.exit(subprocess.call(["bash", str(SCRIPT_PATH), *sys.argv[1:]]))


if __name__ == "__main__":
    main()
