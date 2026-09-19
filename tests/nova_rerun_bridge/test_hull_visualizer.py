import numpy as np
import pytest

from nova_rerun_bridge.hull_visualizer import HullVisualizer

CUBE = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ]
)
CUBE_EDGES = {
    (0, 1),
    (1, 2),
    (2, 3),
    (0, 3),
    (4, 5),
    (5, 6),
    (6, 7),
    (4, 7),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
}


def edge_set(strips: list[np.ndarray]) -> set[tuple[int, int]]:
    """Map line strips back onto CUBE vertex indices, ignoring direction."""
    lookup = {tuple(np.round(p, 6)): i for i, p in enumerate(CUBE)}
    edges = set()
    for strip in strips:
        indices = [lookup[tuple(np.round(p, 6))] for p in strip]
        for a, b in zip(indices, indices[1:]):
            edges.add((min(a, b), max(a, b)))
    return edges


def test_compute_hull_outlines_returns_hull_edges():
    """A cube outline is its 12 edges - diagonals inside a face must not show up."""
    assert edge_set(HullVisualizer.compute_hull_outlines_from_points(CUBE)) == CUBE_EDGES


def test_compute_hull_outlines_ignores_duplicated_points():
    """Convex hulls exported from CAD repeat vertices, which must not change the outline."""
    points = np.vstack([CUBE, CUBE, CUBE])

    assert edge_set(HullVisualizer.compute_hull_outlines_from_points(points)) == CUBE_EDGES


def test_compute_hull_outlines_ignores_points_inside_a_face():
    """Points on a face split it into coplanar triangles; their shared edges are not drawn.

    Such faces are what made the previous plane-grouping implementation hang: their boundary
    is not always a simple cycle.
    """
    face_points = np.array([[0.5, 0.5, 1.0], [0.25, 0.25, 1.0], [0.75, 0.25, 1.0]])
    points = np.vstack([CUBE, face_points])

    strips = HullVisualizer.compute_hull_outlines_from_points(points)

    drawn = {tuple(np.round(p, 6)) for strip in strips for p in strip}
    assert drawn == {tuple(np.round(p, 6)) for p in CUBE}
    assert edge_set(strips) == CUBE_EDGES


@pytest.mark.timeout(60)
def test_compute_hull_mesh_from_outlines():
    """The outline of a hull still hulls back into the same solid."""
    strips = HullVisualizer.compute_hull_outlines_from_points(CUBE)

    vertices, triangles, normals = HullVisualizer.compute_hull_mesh(strips)

    assert len(vertices) == 8
    assert len(triangles) == 12
    assert len(normals) == len(vertices)


def test_compute_hull_outlines_of_planar_points():
    """Fully flat input has no 3D hull; it falls back to the 2D outline."""
    square = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])

    strips = HullVisualizer.compute_hull_outlines_from_points(square)

    assert len(strips) == 1
    assert np.allclose(strips[0][0], strips[0][-1])
    assert len(strips[0]) == 5


def test_compute_hull_outlines_needs_four_points():
    assert HullVisualizer.compute_hull_outlines_from_points(CUBE[:3]) == []
