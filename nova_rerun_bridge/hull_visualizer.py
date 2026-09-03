from typing import Any

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError

# Adjacent hull triangles are treated as one flat face below this angle, so the edge between
# them is not part of the wireframe.
COPLANAR_ANGLE_TOLERANCE = 1e-6


class HullVisualizer:
    @staticmethod
    def compute_hull_mesh(
        polygons: list[np.ndarray],
    ) -> tuple[list[list[float]], list[list[int]], list[list[float]]]:
        """Convert polygons to mesh with optimized hull generation."""
        vertices = np.vstack(polygons)

        # Custom qhull options for better quality
        qhull_opts = trimesh.convex.QhullOptions(
            Qt=True,  # Triangulated output
            QJ=True,  # Joggled input for precision
            Qc=True,  # Keep coplanar points
            Qx=True,  # Exact pre-merges
            QbB=True,  # Scale to unit cube
            Pp=True,  # Remove precision warnings
        )

        mesh = trimesh.convex.convex_hull(vertices, qhull_options=qhull_opts, repair=True)

        return (mesh.vertices.tolist(), mesh.faces.tolist(), mesh.vertex_normals.tolist())

    @staticmethod
    def compute_hull_outlines_from_geometries(child_geometries: list[Any]) -> list[np.ndarray]:
        """Compute the wireframe outline from geometry child objects.

        Args:
            child_geometries: List of geometry objects containing convex hulls

        Returns:
            List of line strips as Nx3 numpy arrays
        """
        all_points = []
        for child in child_geometries:
            if child.convex_hull is not None:
                for v in child.convex_hull.vertices:
                    all_points.append([v.x, v.y, v.z])

        if len(all_points) < 4:
            return []

        return HullVisualizer._compute_hull_from_points(np.array(all_points))

    @staticmethod
    def compute_hull_outlines_from_points(points: np.ndarray) -> list[np.ndarray]:
        """Compute the wireframe outline of the convex hull of the given points.

        Args:
            points: List of [x,y,z] coordinates

        Returns:
            List of line strips as Nx3 numpy arrays
        """
        if len(points) < 4:
            return []

        return HullVisualizer._compute_hull_from_points(np.array(points))

    @staticmethod
    def _is_coplanar(points: np.ndarray, tolerance: float = 1e-6) -> tuple[bool, np.ndarray | None]:
        """Check if points are coplanar and return the plane normal if so.

        Returns:
            Tuple of (is_coplanar, plane_normal). plane_normal is None if not coplanar.
        """
        if len(points) < 3:
            return True, None

        # Check variance along each axis - if one is near zero, points are flat along that axis
        ranges = np.ptp(points, axis=0)  # peak-to-peak (max - min) for each dimension

        # If one dimension has zero range, points are coplanar
        min_range_idx = np.argmin(ranges)
        if ranges[min_range_idx] < tolerance:
            # Create normal vector pointing along the flat dimension
            normal = np.zeros(3)
            normal[min_range_idx] = 1.0
            return True, normal

        # Check if points are coplanar using cross product method
        p0 = points[0]
        v1 = points[1] - p0
        v2 = points[2] - p0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < tolerance:
            return True, None
        normal = normal / norm

        # Check all other points against this plane
        for i in range(3, len(points)):
            dist = abs(np.dot(points[i] - p0, normal))
            if dist > tolerance:
                return False, None

        return True, normal

    @staticmethod
    def _compute_2d_hull_polygon(points: np.ndarray, normal: np.ndarray) -> list[np.ndarray]:
        """Compute 2D convex hull for coplanar points and return as 3D polygon.

        Args:
            points: Nx3 array of coplanar 3D points
            normal: Normal vector of the plane

        Returns:
            List containing a single closed polygon as Nx3 numpy array
        """
        from scipy.spatial import ConvexHull as ConvexHull2D

        if len(points) < 3:
            return []

        # Find the dimension with minimum range (the flat dimension)
        ranges = np.ptp(points, axis=0)
        flat_dim = np.argmin(ranges)

        # Project to 2D by removing the flat dimension
        dims_2d = [i for i in range(3) if i != flat_dim]
        points_2d = points[:, dims_2d]

        try:
            hull_2d = ConvexHull2D(points_2d)
            hull_indices = hull_2d.vertices

            # Get the hull points in order and close the loop
            hull_points_3d = points[hull_indices]
            closed_loop = np.vstack([hull_points_3d, hull_points_3d[0]])
            return [closed_loop]

        except QhullError:
            # If 2D hull also fails (e.g., collinear points), return the points as a line
            if len(points) >= 2:
                closed_loop = np.vstack([points, points[0]])
                return [closed_loop]
            return []

    @staticmethod
    def _compute_hull_from_points(points: np.ndarray) -> list[np.ndarray]:
        """Internal helper to compute the hull wireframe from a numpy points array.

        The visible edges of a convex hull are the ones whose two adjacent triangles are not
        coplanar, so trimesh's face adjacency does the coplanar grouping for us. Edges are
        returned as individual two-point strips - the wireframe looks the same as closed face
        outlines, and no boundary ordering is needed. That matters because hulls of CAD
        geometry regularly have faces whose boundary is not a simple cycle.
        """
        try:
            points = np.asarray(points)
            hull = ConvexHull(points)

            # qhull does not wind the hull triangles consistently, so re-wind them against the
            # outward plane normals it reports. Without this, two coplanar triangles can appear
            # to meet at 180 degrees and their shared edge is mistaken for a hull edge.
            faces = np.array(hull.simplices)
            windings = np.cross(
                points[faces[:, 1]] - points[faces[:, 0]], points[faces[:, 2]] - points[faces[:, 0]]
            )
            flipped = np.einsum("ij,ij->i", windings, hull.equations[:, :3]) < 0
            faces[flipped] = faces[flipped][:, ::-1]

            mesh = trimesh.Trimesh(vertices=points, faces=faces, process=False)
            sharp = mesh.face_adjacency_angles > COPLANAR_ANGLE_TOLERANCE
            return list(points[mesh.face_adjacency_edges[sharp]])

        except QhullError:
            # ConvexHull failed - likely because points are coplanar
            is_coplanar, normal = HullVisualizer._is_coplanar(points)
            if is_coplanar and normal is not None:
                return HullVisualizer._compute_2d_hull_polygon(points, normal)
            # Try to compute 2D hull anyway as a fallback
            return HullVisualizer._compute_2d_hull_polygon(
                points,
                np.array([0, 1, 0]),  # default normal
            )
        except Exception:
            return []
