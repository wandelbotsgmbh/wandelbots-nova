import importlib.util
import sys
import textwrap
from pathlib import Path

import pytest

import nova

SCAN_PATH = Path(nova.__file__).parent / "agent_skills" / "nova-app-review" / "scripts" / "scan.py"


@pytest.fixture(scope="module")
def scan_module():
    spec = importlib.util.spec_from_file_location("nova_app_review_scan", SCAN_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    yield module
    sys.modules.pop(spec.name, None)


def _write(root: Path, files: dict[str, str]) -> Path:
    for rel, content in files.items():
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(textwrap.dedent(content))
    return root


VIOLATING_APP = {
    "app/programs/pick.py": '''
        import asyncio
        import time

        import nova
        from nova.actions import linear
        from nova.exceptions import PlanTrajectoryFailed
        from nova.types import MotionSettings, Pose
        from nova.utils.io import wait_for_bus_io

        SETTLE_TIME_S = 1.0


        def interlocks_ok():
            """Placeholder."""
            return True


        @nova.program(id="pick")
        async def pick(ctx: nova.ProgramContext):
            controller = await ctx.cell.controller("ur")
            mg = controller[0]
            tcp_names = await mg.tcp_names()
            tcp = tcp_names[0]
            asyncio.create_task(heartbeat(mg))
            time.sleep(1)
            await asyncio.sleep(2)
            fast = MotionSettings(tcp_velocity_limit=250)
            try:
                await mg.plan_and_execute([linear(Pose((100, 200, 300, 0, 0, 0)), settings=fast)], tcp="3")
            except PlanTrajectoryFailed:
                pass
            await wait_for_bus_io(["In.App.PickRdy"], on_change=lambda c: True)
            await mg.stop()
            print("done")


        async def heartbeat(mg):
            if VIRTUAL_PLC:
                return
            try:
                await asyncio.sleep(0)
            except asyncio.CancelledError:
                pass
            return mg._api_client
        ''',
    "app/programs/pick_copy.py": """
        import nova


        @nova.program(id="pick")
        async def pick_copy(ctx):
            ...
        """,
}

COMPLIANT_APP = {
    "app/config.py": """
        from decouple import config
        from nova.types import MotionSettings

        CONTROLLER = config("CONTROLLER", default="ur")
        MOTION_GROUP_ID = "0@ur"
        TCP = "gripper"
        PICK_READY = "In.App.PickRdy"
        PLAN_TIMEOUT_S = 20.0
        EXECUTE_TIMEOUT_S = 90.0
        FAST = MotionSettings(tcp_velocity_limit=250)
        """,
    "app/programs/pick.py": """
        import asyncio
        import logging

        import nova
        from nova.utils.io import wait_for_bus_io

        from app import config as cfg

        logger = logging.getLogger(__name__)


        async def run_motion(mg, actions):
            async with asyncio.timeout(cfg.PLAN_TIMEOUT_S):
                trajectory = await mg.plan(actions, cfg.TCP)
            async with asyncio.timeout(cfg.EXECUTE_TIMEOUT_S):
                await mg.execute(trajectory, cfg.TCP, actions=actions)


        @nova.program(id="pick")
        async def pick(ctx: nova.ProgramContext):
            controller = await ctx.cell.controller(cfg.CONTROLLER)
            mg = controller.motion_group(cfg.MOTION_GROUP_ID)
            missing = {cfg.TCP} - set(await mg.tcp_names())
            if missing:
                raise RuntimeError(f"Missing TCPs: {missing}")
            task = asyncio.create_task(run_motion(mg, []))
            try:
                await task
            except asyncio.CancelledError:
                logger.warning("pick cancelled")
                raise
            await asyncio.wait_for(
                wait_for_bus_io([cfg.PICK_READY], on_change=lambda c: True), timeout=5
            )
        """,
}


def test_violating_app_reports_expected_rules(scan_module, tmp_path: Path):
    result = scan_module.scan(_write(tmp_path, VIOLATING_APP))

    rules = {c["rule"] for c in result["candidates"]}
    assert rules >= {
        "ASY-001",
        "ASY-004",
        "ASY-005",
        "ERR-001",
        "ERR-005",
        "IO-002",
        "IO-005",
        "MOT-001",
        "MOT-002",
        "MOT-006",
        "MOT-007",
        "MOT-009",
        "OBS-001",
        "PERF-001",
        "PERF-004",
        "SAF-003",
        "SAF-005",
        "SAF-010",
        "SAF-012",
        "STR-001",
    }
    assert result["files_scanned"] == 2
    assert result["parse_errors"] == []


def test_violating_app_candidate_fields(scan_module, tmp_path: Path):
    result = scan_module.scan(_write(tmp_path, VIOLATING_APP))

    mot009 = next(c for c in result["candidates"] if c["rule"] == "MOT-009")
    assert mot009["file"] == "app/programs/pick.py"
    assert mot009["excerpt"] == "mg = controller[0]"
    assert mot009["severity"] == "warning"
    assert mot009["confidence"] == "high"
    assert mot009["path_kind"] == "app"

    duplicates = [c for c in result["candidates"] if c["rule"] == "STR-001"]
    assert {c["file"] for c in duplicates} == {"app/programs/pick.py", "app/programs/pick_copy.py"}
    assert len(result["facts"]["nova_entry_points"]) == 2


def test_compliant_app_has_no_candidates(scan_module, tmp_path: Path):
    result = scan_module.scan(_write(tmp_path, COMPLIANT_APP))

    assert result["candidates"] == []
    assert result["facts"]["queries_tcp_names"] is True
    assert result["facts"]["uses_motion_group_by_id"] is True


def test_skips_virtualenvs_and_generated_files(scan_module, tmp_path: Path):
    _write(
        tmp_path,
        {
            ".venv/lib/site.py": "import time\nasync def f():\n    time.sleep(1)\n",
            "app/io_symbols.py": "# Auto-generated from TIA export. Do not edit.\nX = 'In.App.A'\n",
        },
    )

    result = scan_module.scan(tmp_path)

    assert result["files_scanned"] == 0
    assert result["skipped_generated"] == ["app/io_symbols.py"]
    assert result["candidates"] == []


def test_test_files_are_tagged(scan_module, tmp_path: Path):
    _write(tmp_path, {"tests/test_pick.py": "print('x')\n"})

    result = scan_module.scan(tmp_path)

    assert [c["path_kind"] for c in result["candidates"]] == ["test"]


def test_main_prints_json(scan_module, tmp_path: Path, capsys):
    _write(tmp_path, {"app.py": "print('x')\n"})

    assert scan_module.main([str(tmp_path)]) == 0

    assert '"rule": "OBS-001"' in capsys.readouterr().out
