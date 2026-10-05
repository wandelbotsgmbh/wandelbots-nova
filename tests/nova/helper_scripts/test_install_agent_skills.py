import hashlib
import json
from pathlib import Path

import pytest

from nova.helper_scripts.install_agent_skills import AGENT_DIRS, STAMP_FILE, bundled_skills, main

SKILL = "nova-app-review"


def _skill_dir(project: Path, agent: str = "agents") -> Path:
    return project / AGENT_DIRS[agent] / SKILL


def test_bundled_skills_contain_app_review():
    skills = bundled_skills()

    assert SKILL in skills
    assert (skills[SKILL] / "SKILL.md").read_text().startswith("---\nname: nova-app-review\n")


def test_list(capsys):
    assert main(["list"]) == 0
    assert SKILL in capsys.readouterr().out.split()


def test_install_default_target(tmp_path: Path):
    assert main(["install", "--project-dir", str(tmp_path)]) == 0

    dest = _skill_dir(tmp_path)
    assert (dest / "SKILL.md").is_file()
    assert (dest / "scripts" / "scan.py").is_file()
    assert (dest / "references" / "nova-sdk-mapping.md").is_file()
    assert not any(p.name == "__pycache__" for p in dest.rglob("*"))
    stamp = json.loads((dest / STAMP_FILE).read_text())
    assert stamp["package"] == "wandelbots-nova"
    assert stamp["skill"] == SKILL
    assert "SKILL.md" in stamp["files"]
    assert not (tmp_path / ".claude").exists()


def test_install_all_agents(tmp_path: Path):
    assert main(["install", "--project-dir", str(tmp_path), "--agent", "all"]) == 0

    for agent in AGENT_DIRS:
        assert (_skill_dir(tmp_path, agent) / "SKILL.md").is_file()


def test_reinstall_is_up_to_date(tmp_path: Path, capsys):
    main(["install", "--project-dir", str(tmp_path)])
    capsys.readouterr()

    assert main(["install", "--project-dir", str(tmp_path)]) == 0
    assert capsys.readouterr().out.startswith("up to date")


def test_local_edits_are_kept_unless_forced(tmp_path: Path):
    main(["install", "--project-dir", str(tmp_path)])
    skill_md = _skill_dir(tmp_path) / "SKILL.md"
    skill_md.write_text("my edits")

    assert main(["install", "--project-dir", str(tmp_path)]) == 1
    assert skill_md.read_text() == "my edits"

    assert main(["install", "--project-dir", str(tmp_path), "--force"]) == 0
    assert skill_md.read_text().startswith("---\nname: nova-app-review")


def test_unmanaged_folder_is_not_overwritten(tmp_path: Path):
    dest = _skill_dir(tmp_path)
    dest.mkdir(parents=True)
    (dest / "SKILL.md").write_text("someone else's skill")

    assert main(["install", "--project-dir", str(tmp_path)]) == 1
    assert (dest / "SKILL.md").read_text() == "someone else's skill"


def test_files_removed_from_the_bundle_are_deleted(tmp_path: Path):
    main(["install", "--project-dir", str(tmp_path)])
    dest = _skill_dir(tmp_path)
    stale = dest / "references" / "old.md"
    stale.write_text("old")
    stamp = json.loads((dest / STAMP_FILE).read_text())
    stamp["files"]["references/old.md"] = hashlib.sha256(b"old").hexdigest()
    stamp["version"] = "0.0.0"
    (dest / STAMP_FILE).write_text(json.dumps(stamp))

    assert main(["install", "--project-dir", str(tmp_path)]) == 0
    assert not stale.exists()


def test_unknown_skill(tmp_path: Path):
    assert main(["install", "--project-dir", str(tmp_path), "--skill", "nope"]) == 2


def test_command_is_required():
    with pytest.raises(SystemExit):
        main([])
