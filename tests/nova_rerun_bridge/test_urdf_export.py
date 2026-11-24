"""The bridge's use of nova2urdf's export API.

nova2urdf's ``export_controller`` returns a ``UrdfPackage``; written to disk, its
``path`` is the URDF. The bridge renders from the directory around it.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

nova2urdf = pytest.importorskip("nova2urdf")
urdf_visualizer = pytest.importorskip("nova_rerun_bridge.urdf_visualizer")


def _controller() -> SimpleNamespace:
    return SimpleNamespace(id="ur10e", configuration=SimpleNamespace(cell_id="cell"))


@pytest.fixture
def reachable(monkeypatch: pytest.MonkeyPatch) -> None:
    async def api_host(_controller: object) -> str:
        return "http://nova.test"

    monkeypatch.setattr(urdf_visualizer, "_api_host", api_host)


@pytest.mark.asyncio
@pytest.mark.usefixtures("reachable")
async def test_a_written_export_renders_from_its_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    async def export_controller(_client, controller_name, *, output_dir, **_kwargs):
        package = nova2urdf.UrdfPackage(name=controller_name)
        package.urdf_name = package.add(f"{controller_name}.urdf", "<robot name='r'/>")
        package.write(Path(output_dir) / controller_name)
        return package

    monkeypatch.setattr(nova2urdf, "export_controller", export_controller)

    directory = await urdf_visualizer._export(_controller(), tmp_path)

    assert directory == tmp_path / "ur10e"
    assert (directory / "ur10e.urdf").is_file()


@pytest.mark.asyncio
@pytest.mark.usefixtures("reachable")
async def test_unusable_meshes_leave_the_robot_unrendered(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A URDF without meshes would draw an invisible robot; no renderer is honest,
    and the program being visualized must not fail over it.
    """

    async def export_controller(*_args, **_kwargs):
        raise nova2urdf.MeshExtractionError("no geometry in the GLB")

    monkeypatch.setattr(nova2urdf, "export_controller", export_controller)

    assert await urdf_visualizer._export(_controller(), tmp_path) is None
