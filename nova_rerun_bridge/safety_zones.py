import rerun as rr

from nova.api import models
from nova_rerun_bridge.hull_visualizer import HullVisualizer


def log_safety_zones(motion_group: str, optimizer_config: models.OptimizerSetup):
    """
    Log hull outlines for the safety zones defined in the optimizer configuration.
    """
    if optimizer_config.safety_setup.safety_zones is None:
        return

    for zone in optimizer_config.safety_setup.safety_zones:
        geom = zone.geometry
        zone_id = zone.id
        entity_path = f"{motion_group}/safety_zones/zone_{zone_id}"

        if geom.compound is not None:
            child_geoms = geom.compound.child_geometries
            polygons = HullVisualizer.compute_hull_outlines_from_geometries(child_geoms)
        elif geom.convex_hull is not None:

            class ChildWrapper:
                def __init__(self, convex_hull):
                    self.convex_hull = convex_hull

            c = ChildWrapper(geom.convex_hull)
            c.convex_hull = geom.convex_hull
            polygons = HullVisualizer.compute_hull_outlines_from_geometries([c])
        else:
            polygons = []

        # Log polygons as wireframe outlines
        if polygons:
            line_segments = [p.tolist() for p in polygons]
            rr.log(
                entity_path,
                rr.LineStrips3D(
                    line_segments, radii=rr.Radius.ui_points(0.75), colors=[[221, 193, 193, 255]]
                ),
                static=True,
            )
