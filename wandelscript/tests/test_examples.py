import tempfile
from typing import Union

import numpy as np
import pydantic
import pytest
from glom import glom
from loguru import logger

import wandelscript
from nova.cell.robot_cell import ConfigurablePeriphery, RobotCell
from nova.cell.simulation import SimulatedRobotCell
from nova.types import Pose, Vector3d
from wandelscript.examples import EXAMPLES


def filter_dict(d, ignore_paths: list[str]):
    if not isinstance(d, dict):
        return d

    result = d.copy()
    for path in ignore_paths:
        try:
            # Handle both top-level keys and nested paths
            if "." in path:
                parent_path, key = path.rsplit(".", 1)
                parent = glom(result, parent_path)
                if isinstance(parent, dict):
                    parent.pop(key, None)
            else:
                result.pop(path, None)
        except (KeyError, TypeError, ValueError):
            continue

    return {
        k: filter_dict(v, ignore_paths) if isinstance(v, dict) else v for k, v in result.items()
    }


def _robot_cell_from_configuration(data) -> RobotCell:
    AnyConfiguration = Union.__getitem__(tuple(ConfigurablePeriphery.all_classes))

    class RobotCellConfiguration(pydantic.BaseModel):
        devices: list[AnyConfiguration]  # type: ignore

    config = RobotCellConfiguration(devices=data)

    return RobotCell.from_configurations(config.devices)


@pytest.mark.parametrize("example_name", EXAMPLES)
def test_example(example_name):
    # TODO: https://wandelbots.atlassian.net/browse/WP-683
    if example_name in (
        "spline",
        "interrupt",
        "tower_of_hanoi",
        "edge_pattern_manhattan",
        "edge_pattern",
        "multiple_robots2",
        "edge_pattern_line",
        "find_edge_from_4_poses",
        "frame2",
        "multiple_robots",
        "wandelchat2",
        "wandelchat3",
        "functional_pose",
        # TODO: needs to be fixed asap
        "async_write",
        # TODO: needs to be fixed asap -> fails in gh runner but works locally
        "fetch",
    ):
        return
    logger.info(f"Running example {example_name}...")
    program, data, config = EXAMPLES[example_name]
    robot_cell = _robot_cell_from_configuration(config)
    runner = wandelscript.run(
        program_id=example_name,
        program=program,
        robot_cell_override=robot_cell,
        default_tcp="Flange",
    )
    store = runner.program_run.output_data
    for key, expected in data.items():
        if isinstance(expected, list):
            expected = tuple(tuple(v) if isinstance(v, list) else v for v in expected)
        if isinstance(expected, Pose):
            assert np.allclose(expected.position, store[key]["position"], atol=1e-3, rtol=1e-3)
            assert np.allclose(
                expected.orientation, store[key]["orientation"], atol=1e-3, rtol=1e-3
            )
        elif isinstance(expected, Vector3d):
            assert np.allclose(
                expected.to_tuple(), tuple(store[key].values()), atol=1e-3, rtol=1e-3
            )
        elif isinstance(expected, dict):
            if example_name == "fetch":
                ignore_paths = ["headers", "origin", "data"]  # TODO: res_get_error.data should work
                assert filter_dict(expected, ignore_paths) == filter_dict(store[key], ignore_paths)
            else:
                assert expected == store[key]
        else:
            assert expected == store[key], f"{key=}: got: {store[key]} expected: {expected}"


@pytest.mark.skip("Do we need this?")
def test_save_and_load():
    with tempfile.TemporaryDirectory() as tempdir:
        code = f"""
a = (1, 2, 3, 4, 5, 6)
save(a, '{tempdir}/test.ws')
b = load('{tempdir}/test.ws', 'pose')
"""
        runner = wandelscript.run(code, robot_cell_override=SimulatedRobotCell())
        print(runner.execution_context.store["b"])
