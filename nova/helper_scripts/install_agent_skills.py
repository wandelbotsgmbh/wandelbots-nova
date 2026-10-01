"""Entry point for the ``nova-agent-skills`` script.

Copies the agent skills bundled with wandelbots-nova (``nova/agent_skills/``) into a project so
coding agents (Claude Code, Cursor, GitHub Copilot, ...) can use them::

    nova-agent-skills list
    nova-agent-skills install                      # -> .agents/skills/
    nova-agent-skills install --agent claude --agent copilot
    nova-agent-skills install --agent all --force
"""

import argparse
import hashlib
import json
import sys
from collections.abc import Iterator
from importlib import resources
from importlib.metadata import PackageNotFoundError, version
from importlib.resources.abc import Traversable
from pathlib import Path

AGENT_DIRS = {
    "agents": Path(".agents/skills"),
    "claude": Path(".claude/skills"),
    "cursor": Path(".cursor/skills"),
    "copilot": Path(".github/skills"),
}
STAMP_FILE = ".installed-from.json"


def bundled_skills() -> dict[str, Traversable]:
    root = resources.files("nova") / "agent_skills"
    return {
        entry.name: entry
        for entry in root.iterdir()
        if entry.is_dir() and (entry / "SKILL.md").is_file()
    }


def _iter_files(node: Traversable, prefix: str = "") -> Iterator[tuple[str, bytes]]:
    for child in sorted(node.iterdir(), key=lambda c: c.name):
        if child.name == "__pycache__" or child.name.endswith(".pyc"):
            continue
        rel = f"{prefix}{child.name}"
        if child.is_dir():
            yield from _iter_files(child, f"{rel}/")
        else:
            yield rel, child.read_bytes()


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sdk_version() -> str:
    try:
        return version("wandelbots-nova")
    except PackageNotFoundError:
        return "unknown"


def install_skill(name: str, source: Traversable, dest: Path, force: bool) -> tuple[bool, str]:
    """Install one skill into ``dest``. Returns ``(ok, status message)``."""
    files = dict(_iter_files(source))
    digests = {rel: _sha256(data) for rel, data in files.items()}
    stamp_path = dest / STAMP_FILE
    previous: dict[str, str] = {}

    if dest.exists():
        if stamp_path.is_file():
            stamp = json.loads(stamp_path.read_text(encoding="utf-8"))
            previous = stamp.get("files", {})
            modified = sorted(
                rel
                for rel, digest in previous.items()
                if (dest / rel).is_file() and _sha256((dest / rel).read_bytes()) != digest
            )
            if modified and not force:
                return False, f"skipped, locally modified: {', '.join(modified)} (use --force)"
            if not modified and previous == digests and stamp.get("version") == _sdk_version():
                return True, "up to date"
        elif not force:
            return False, "skipped, not installed by nova-agent-skills (use --force to overwrite)"

    for rel in previous.keys() - files.keys():
        (dest / rel).unlink(missing_ok=True)
    for rel, data in files.items():
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)

    stamp = {
        "package": "wandelbots-nova",
        "version": _sdk_version(),
        "skill": name,
        "files": digests,
    }
    stamp_path.write_text(json.dumps(stamp, indent=2) + "\n", encoding="utf-8")
    return True, "installed"


def _install(args: argparse.Namespace) -> int:
    skills = bundled_skills()
    unknown = set(args.skill or []) - skills.keys()
    if unknown:
        print(f"Unknown skill(s): {', '.join(sorted(unknown))}. Available: {', '.join(skills)}")
        return 2

    agents = args.agent or ["agents"]
    if "all" in agents:
        agents = list(AGENT_DIRS)
    selected = {name: skills[name] for name in (args.skill or skills)}

    ok = True
    for agent in dict.fromkeys(agents):
        for name, source in selected.items():
            dest = args.project_dir / AGENT_DIRS[agent] / name
            installed, status = install_skill(name, source, dest, args.force)
            ok &= installed
            print(f"{status}: {dest}")
    return 0 if ok else 1


def _list(_: argparse.Namespace) -> int:
    for name in bundled_skills():
        print(name)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="nova-agent-skills",
        description="Install the coding-agent skills bundled with wandelbots-nova into a project.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="List bundled skills.").set_defaults(func=_list)

    install = sub.add_parser("install", help="Copy bundled skills into the project.")
    install.add_argument(
        "--agent",
        action="append",
        choices=[*AGENT_DIRS, "all"],
        help="Target agent folder (repeatable). Default: agents (.agents/skills).",
    )
    install.add_argument(
        "--skill", action="append", help="Skill to install (repeatable). Default: all."
    )
    install.add_argument(
        "--project-dir", type=Path, default=Path.cwd(), help="Project root. Default: cwd."
    )
    install.add_argument(
        "--force", action="store_true", help="Overwrite local edits and unmanaged folders."
    )
    install.set_defaults(func=_install)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
