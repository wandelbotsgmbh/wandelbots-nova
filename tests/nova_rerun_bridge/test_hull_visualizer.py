import numpy as np
import pytest

from nova_rerun_bridge.hull_visualizer import HullVisualizer

# One coplanar face of a convex hull computed from CAD collision geometry. The triangles fan
# out from vertex 3 and partly only touch there, so the face boundary is not a simple cycle:
# vertex 3 has more than two boundary neighbours. Walking such a boundary while only avoiding
# the previous vertex circles the same sub-loop forever and never returns to the start vertex.
PINCHED_FACE_POINTS = np.array(
    [
        [4.9489, 0.591, -1.3391],
        [-7.8748, 15.4831, -2.4483],
        [-14.0382, 20.7235, -2.447],
        [-2.8719, 0.7592, 0.4698],
        [1.2711, -16.2252, 4.2223],
        [5.2076, -11.5949, 1.9971],
        [6.9272, -7.074, 0.3286],
        [6.4301, -2.6627, -0.7834],
    ]
)
PINCHED_FACE_SIMPLICES = [
    np.array([3, 5, 6]),
    np.array([3, 1, 2]),
    np.array([3, 5, 4]),
    np.array([3, 7, 0]),
    np.array([3, 7, 6]),
]


@pytest.mark.timeout(30)
def test_merge_coplanar_triangles_terminates_on_pinched_boundary():
    polygon = HullVisualizer.merge_coplanar_triangles_to_polygon(
        PINCHED_FACE_POINTS, PINCHED_FACE_SIMPLICES
    )

    # Every boundary edge is walked at most once, so the outline stays bounded.
    assert 0 < len(polygon) <= 2 * len(PINCHED_FACE_POINTS)


@pytest.mark.timeout(60)
def test_compute_hull_outlines_from_duplicated_points():
    """Convex hulls exported from CAD repeat vertices; outlines must still be produced."""
    cube = np.array(
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
    points = np.vstack([cube, cube, cube])

    polygons = HullVisualizer.compute_hull_outlines_from_points(points)

    assert len(polygons) == 6
    for polygon in polygons:
        assert np.allclose(polygon[0], polygon[-1])
