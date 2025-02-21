from typing import Any

import numpy as np
import trimesh
from scipy.spatial import ConvexHull


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
    def plane_from_triangle(p0, p1, p2, normal_epsilon=1e-6):
        # Compute normal
        v1 = p1 - p0
        v2 = p2 - p0
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        if norm < normal_epsilon:
            return None, None
        n = n / norm
        # Plane: n·x = d
        d = np.dot(n, p0)
        return n, d

    @staticmethod
    def group_coplanar_triangles(points, hull, angle_epsilon=1e-6, dist_epsilon=1e-6):
        # Group triangles by their plane (normal and distance)
        plane_map = {}
        for simplex in hull.simplices:
            p0, p1, p2 = points[simplex]
            n, d = HullVisualizer.plane_from_triangle(p0, p1, p2)
            if n is None:
                continue

            # Ensure a canonical representation of the plane normal
            for i_comp in range(3):
                if abs(n[i_comp]) > angle_epsilon:
                    if n[i_comp] < 0:
                        n = -n
                        d = -d
                    break

            # Round normal and distance for stable hashing
            n_rounded = tuple(np.round(n, 6))
            d_rounded = round(d, 3)

            key = (n_rounded, d_rounded)
            if key not in plane_map:
                plane_map[key] = []
            plane_map[key].append(simplex)

        return plane_map

    @staticmethod
    def merge_coplanar_triangles_to_polygon(points, simplices):
        # Extract polygon outline from coplanar triangles
        edges = {}
        for tri in simplices:
            for i in range(3):
                a = tri[i]
                b = tri[(i + 1) % 3]
                e = (min(a, b), max(a, b))
                edges[e] = edges.get(e, 0) + 1

        # Keep only outer edges (appear once)
        boundary_edges = [e for e, count in edges.items() if count == 1]
        if not boundary_edges:
            return []

        # Build adjacency
        adj = {}
        for a, b in boundary_edges:
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)

        # Walk along the boundary edges to form a closed loop
        start = boundary_edges[0][0]
        loop = [start]
        current = start
        prev = None
        while True:
            neighbors = adj[current]
            next_vertex = None
            for n in neighbors:
                if n != prev:
                    next_vertex = n
                    break
            if next_vertex is None:
                break
            loop.append(next_vertex)
            prev, current = current, next_vertex
            if next_vertex == start:
                break

        polygon_points = points[loop]
        return polygon_points

    @staticmethod
    def compute_hull_outlines_from_geometries(child_geometries: list[Any]) -> list[np.ndarray]:
        """Compute polygon outlines from geometry child objects.

        Args:
            child_geometries: List of geometry objects containing convex hulls

        Returns:
            List of closed polygons as Nx3 numpy arrays
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
    def compute_hull_outlines_from_points(points: list[list[float]]) -> list[np.ndarray]:
        """Compute polygon outlines directly from point coordinates.

        Args:
            points: List of [x,y,z] coordinates

        Returns:
            List of closed polygons as Nx3 numpy arrays
        """
        if len(points) < 4:
            return []

        return HullVisualizer._compute_hull_from_points(np.array(points))

    @staticmethod
    def _compute_hull_from_points(points: np.ndarray) -> list[np.ndarray]:
        """Internal helper to compute hull from numpy points array."""
        try:
            hull = ConvexHull(points)
            plane_map = HullVisualizer.group_coplanar_triangles(points, hull)

            polygons = []
            for simplices in plane_map.values():
                polygon_points = HullVisualizer.merge_coplanar_triangles_to_polygon(
                    points, simplices
                )
                if len(polygon_points) > 2:
                    closed_loop = np.vstack([polygon_points, polygon_points[0]])
                    polygons.append(closed_loop)
            return polygons

        except Exception:
            return []
