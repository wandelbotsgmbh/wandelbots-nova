import os
from pathlib import Path


def find_dotenv(filename: str = ".env") -> str:
    """Find .env file in current or parent directories."""
    path = os.getcwd()
    while True:
        env_path = os.path.join(path, filename)
        if os.path.isfile(env_path):
            return env_path
        parent = os.path.dirname(path)
        if parent == path:
            return ".env"
        path = parent


def set_key(key: str, value: str) -> None:
    """Set or update a key in the .env file."""
    env_path = Path(find_dotenv())
    if not env_path.exists():
        env_path.write_text(f'{key}="{value}"\n')
        return

    lines = env_path.read_text().splitlines()
    key_found = False

    for i, line in enumerate(lines):
        if line.startswith(f"{key}="):
            lines[i] = f'{key}="{value}"'
            key_found = True
            break

    if not key_found:
        lines.append(f'{key}="{value}"')

    env_path.write_text("\n".join(lines) + "\n")
