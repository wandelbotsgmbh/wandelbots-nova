import hashlib
import os

import numpy as np
import trimesh


class MeshExporter:
    """
    Handles the exporting of meshes for URDF, including format conversions
    and directory management.
    """

    def __init__(self, mesh_export_path="meshes"):
        """
        Initialize the mesh exporter.

        Args:
            mesh_export_path: Directory name for mesh storage relative to the export base path
        """
        self.mesh_export_path = mesh_export_path
        self.mesh_cache = {}  # Cache to avoid re-exporting identical meshes

    def ensure_mesh_directory(self, base_path):
        """
        Ensure the mesh directory exists.

        Args:
            base_path: Base directory for export

        Returns:
            Path to the mesh directory
        """
        mesh_dir = os.path.join(base_path, self.mesh_export_path)
        os.makedirs(mesh_dir, exist_ok=True)
        return mesh_dir

    def export_mesh(self, mesh, name, base_path):
        """
        Export a mesh to STL format.

        Args:
            mesh: A trimesh object to export
            name: Name for the exported mesh file
            base_path: Base directory for export

        Returns:
            Path to the exported mesh file
        """
        mesh_dir = self.ensure_mesh_directory(base_path)
        mesh_path = os.path.join(mesh_dir, f"{name}.stl")

        # Check if we've already exported this mesh
        mesh_hash = self._get_mesh_hash(mesh)
        if mesh_hash in self.mesh_cache:
            return self.mesh_cache[mesh_hash]

        # Export the mesh to STL
        mesh.export(mesh_path)
        self.mesh_cache[mesh_hash] = mesh_path
        return mesh_path

    def _get_mesh_hash(self, mesh):
        """
        Generate a hash for a mesh to identify duplicates.

        Args:
            mesh: A trimesh object

        Returns:
            A hash string representing the mesh content
        """
        if hasattr(mesh, "md5"):
            return hash(mesh.md5())

        # If md5 method is not available, create our own hash from vertices and faces
        hasher = hashlib.md5()
        if hasattr(mesh, "vertices") and mesh.vertices is not None:
            hasher.update(mesh.vertices.tobytes())
        if hasattr(mesh, "faces") and mesh.faces is not None:
            hasher.update(mesh.faces.tobytes())
        return hasher.hexdigest()

    def export_convex_hull(self, vertices, name, base_path):
        """
        Export a convex hull to STL format.

        Args:
            vertices: Array of vertices to create the hull from
            name: Name for the exported mesh file
            base_path: Base directory for export

        Returns:
            Path to the exported mesh file
        """
        mesh_dir = self.ensure_mesh_directory(base_path)
        mesh_path = os.path.join(mesh_dir, f"{name}.stl")

        # Create a convex hull from the vertices
        try:
            hull = trimesh.convex.convex_hull(np.array(vertices))
            hull.export(mesh_path)
            return mesh_path
        except (ValueError, IndexError) as e:
            print(f"Error creating convex hull: {e}")
            # Return a simple box mesh as fallback
            box = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
            box.export(mesh_path)
            return mesh_path

    def convert_glb_to_dae(self, glb_path, base_path, model_name):
        """
        Convert a GLB file to a format suitable for URDF.
        Attempts DAE first, then falls back to OBJ or the original GLB.

        Args:
            glb_path: Path to the input GLB file
            base_path: Base directory for export
            model_name: Name for the exported model

        Returns:
            Path to the converted mesh file, or original GLB if conversion failed
        """
        mesh_dir = self.ensure_mesh_directory(base_path)

        # First try DAE (though trimesh doesn't support it directly)
        dae_output_path = os.path.join(mesh_dir, f"{model_name}.dae")
        if os.path.exists(dae_output_path):
            print(f"Using existing DAE file: {dae_output_path}")
            return dae_output_path

        # Then try OBJ (which trimesh definitely supports)
        obj_output_path = os.path.join(mesh_dir, f"{model_name}.obj")
        if os.path.exists(obj_output_path):
            print(f"Using existing OBJ file: {obj_output_path}")
            return obj_output_path

        try:
            # Load the GLB file as a complete scene
            print(f"Loading GLB file: {glb_path}")
            scene = trimesh.load_scene(glb_path, file_type="glb")

            # Try to export to OBJ format (well-supported)
            print(f"Exporting to OBJ format: {obj_output_path}")
            scene.export(obj_output_path, file_type="obj")

            if os.path.exists(obj_output_path):
                print(f"Successfully converted {glb_path} to {obj_output_path}")
                return obj_output_path

        except Exception as e:
            print(f"Failed to convert GLB: {e}")

        # If all conversions fail, copy and use the original GLB
        glb_output_path = os.path.join(mesh_dir, os.path.basename(glb_path))
        if glb_path != glb_output_path:
            try:
                import shutil

                shutil.copy2(glb_path, glb_output_path)
                print(f"Using original GLB file: {glb_output_path}")
                return glb_output_path
            except Exception as e:
                print(f"Failed to copy GLB file: {e}")

        # If nothing else works, use original path
        print(f"Using original GLB file at source location: {glb_path}")
        return glb_path

    def copy_mesh_file(self, source_path, base_path):
        """
        Copy a mesh file to the mesh directory.

        Args:
            source_path: Path to the source mesh file
            base_path: Base directory for export

        Returns:
            Path to the copied mesh file
        """
        if not os.path.exists(source_path):
            print(f"Source mesh file does not exist: {source_path}")
            return None
        mesh_dir = self.ensure_mesh_directory(base_path)
        filename = os.path.basename(source_path)
        dest_path = os.path.join(mesh_dir, filename)
        # Check if file already exists
        if os.path.exists(dest_path):
            return dest_path
        # Copy the file
        try:
            import shutil

            shutil.copy2(source_path, dest_path)
            return dest_path
        except Exception as e:
            print(f"Error copying mesh file: {e}")
            return None

    def get_mesh_package_uri(self, mesh_path, package_name, base_path):
        """
        Get a URI for a mesh file.

        Args:
            mesh_path: Absolute path to the mesh file
            package_name: Name for the model (not used for package URI)
            base_path: Base directory of the exported files

        Returns:
            A file URI string (file://...)
        """
        if not mesh_path or not os.path.exists(mesh_path):
            return None
        # Return a file:// URI instead of package:// URI
        return f"file://{mesh_path}"

    def get_mesh_uri(self, mesh_path, base_path):
        """
        Get a relative URI for a mesh file.

        Args:
            mesh_path: Absolute path to the mesh file
            base_path: Base directory of the exported files

        Returns:
            A relative path string (../meshes/filename.ext)
        """
        if not mesh_path or not os.path.exists(mesh_path):
            return None

        # Get the filename only
        filename = os.path.basename(mesh_path)

        # Return the relative path
        return f"../meshes/{filename}"

    def export_mesh_for_link(self, mesh, link_name, part_idx, base_path):
        """
        Export a mesh for a specific link.

        Args:
            mesh: A trimesh object to export
            link_name: Name of the link this mesh belongs to
            part_idx: Index to identify this part of the link
            base_path: Base directory for export

        Returns:
            Path to the exported mesh file
        """
        mesh_dir = self.ensure_mesh_directory(base_path)
        filename = f"{link_name}_part_{part_idx}.stl"
        mesh_path = os.path.join(mesh_dir, filename)
        # Export the mesh to STL instead of OBJ
        mesh.export(mesh_path, file_type="stl")
        return mesh_path

    def merge_and_export_meshes(self, meshes, name, base_path):
        """
        Merge multiple meshes into a single mesh and export it.

        Args:
            meshes: List of trimesh objects to merge
            name: Name for the exported mesh file
            base_path: Base directory for export

        Returns:
            Path to the exported mesh file
        """
        mesh_dir = self.ensure_mesh_directory(base_path)
        mesh_path = os.path.join(mesh_dir, f"{name}.stl")

        # Skip if we already have this mesh
        if os.path.exists(mesh_path):
            return mesh_path

        try:
            # Merge meshes if multiple are provided
            if len(meshes) > 1:
                combined_mesh = trimesh.util.concatenate(meshes)
            else:
                combined_mesh = meshes[0]

            # Export as STL instead of OBJ
            combined_mesh.export(mesh_path, file_type="stl")
            return mesh_path
        except Exception as e:
            print(f"Error merging/exporting meshes: {e}")
            # Fallback to a simple box if merging fails
            box = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
            box.export(mesh_path, file_type="stl")
            return mesh_path
