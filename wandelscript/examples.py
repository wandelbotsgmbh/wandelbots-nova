import json
from pathlib import Path

from wandelscript.utils.serializer import loads_store

_EXAMPLE_DIR = Path(__file__).parent / "examples"


def read_examples():
    result = {}

    for a in _EXAMPLE_DIR.iterdir():
        if a.suffix == ".ws":
            code = a.open().read()
            data = loads_store((_EXAMPLE_DIR / f"{a.stem}.json").open().read())
            try:
                cell = json.load((_EXAMPLE_DIR / f"{a.stem}.cell.json").open())
            except FileNotFoundError:
                cell = json.load((_EXAMPLE_DIR / "default.cell.json").open())
            result[a.stem] = code, data, cell
    return result


EXAMPLES = read_examples()

DEFAULT, _, _ = EXAMPLES["default"]
