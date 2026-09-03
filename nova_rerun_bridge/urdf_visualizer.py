"""Render a robot in Rerun from a URDF, driven by motion group joint values.

Rerun understands URDF natively: the geometry and the link tree are logged once,
and after that a pose costs one ``Transform3D`` per joint. That is around twenty
writes per step for a whole humanoid, against one per mesh for the GLB-driven
:class:`~nova_rerun_bridge.robot_visualizer.RobotVisualizer`, and the URDF also
carries the joint limits and the link names, so the viewer shows a robot rather
than a pile of meshes.

The URDF comes from ``nova2urdf``, which exports a controller's motion groups as
one tree with each part mounted where it really sits. Point
``NOVA_URDF_EXPORT_DIR`` at an export to reuse it, or let this export on demand
and cache it.

Lengths are logged in millimetres, matching everything else the bridge logs, so
a robot rendered here lines up with trajectories and TCP trails.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import TYPE_CHECKING, Any

from loguru import logger
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

from nova_rerun_bridge import collider_shapes, scene_colors

if TYPE_CHECKING:
    from collections.abc import Callable

    from rerun.recording_stream import RecordingStream
    from rerun.urdf import UrdfJoint, UrdfTree

    from nova.cell.controller import Controller

_SIDECAR = "motion_groups.json"
_EXPORT_DIR_ENV = "NOVA_URDF_EXPORT_DIR"
# nova2urdf states this as millimetres per URDF length unit.
_MILLIMETRES = 1.0
_SCENE_ROOT_FRAME = "tf#/"
"""Rerun's implicit frame for the recording root."""


@dataclass(frozen=True)
class _Collider:
    """One ``<collision>`` mesh of a link: where it sits and what it is.

    The model's own hulls, the tool's collision geometry and the safety
    controller's volumes all live in ``<collision>``; a consumer wants to tell
    them apart, and nova2urdf names them so it can. Each is drawn in its own
    colour. A safety volume that belongs to one TCP says which, because the
    URDF carries every TCP's and only one tool is mounted.
    """

    mesh: Path
    local: np.ndarray
    kind: str
    tcp: str | None = None

    @property
    def safety(self) -> bool:
        """Whether the safety controller enforces this one."""
        return self.kind == "safety"


class _UrdfKinematics:
    """Forward kinematics over a URDF's own joint tree.

    Rerun poses the robot from joint transforms but does not hand back a link's
    world transform, and the overlays need one: where a chain's base sits, and
    which frame a reported pose is stated in. Both are a walk up the fixed and
    revolute joints this reads out of the file.
    """

    def __init__(self, joints: dict[str, dict[str, Any]] | None = None) -> None:
        self._joints = joints or {}
        self._by_child = {spec["child"]: (name, spec) for name, spec in self._joints.items()}

    @classmethod
    def from_file(cls, path: Path) -> _UrdfKinematics:
        """Read the joint tree. A malformed file yields an empty one."""
        from xml.etree import ElementTree

        try:
            root = ElementTree.parse(path).getroot()
        except (OSError, ElementTree.ParseError) as exc:
            logger.warning("Could not read {} for kinematics: {}", path, exc)
            return cls()

        joints: dict[str, dict[str, Any]] = {}
        for element in root.findall("joint"):
            parent, child = element.find("parent"), element.find("child")
            if parent is None or child is None:
                continue
            origin, axis = element.find("origin"), element.find("axis")
            joints[str(element.get("name"))] = {
                "type": element.get("type", "fixed"),
                "parent": parent.get("link", ""),
                "child": child.get("link", ""),
                "origin": _transform(
                    _floats(origin, "xyz") if origin is not None else None,
                    _floats(origin, "rpy") if origin is not None else None,
                ),
                "axis": np.array(
                    _floats(axis, "xyz") or [0.0, 0.0, 1.0] if axis is not None else [0.0, 0.0, 1.0]
                ),
            }
        return cls(joints)

    def world(self, link: str, values: dict[str, float]) -> np.ndarray:
        """``link``'s transform relative to the URDF's root link."""
        if link not in self._by_child and not self._joints:
            raise KeyError(link)
        transform = np.eye(4)
        seen: set[str] = set()
        while link in self._by_child and link not in seen:
            seen.add(link)
            name, spec = self._by_child[link]
            local = spec["origin"]
            if spec["type"] in {"revolute", "continuous"}:
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec(
                    spec["axis"] * values.get(name, 0.0)
                ).as_matrix()
                local = local @ turn
            transform = local @ transform
            link = spec["parent"]
        return transform


_SAFETY_NAME = re.compile(r"safety_(link|tool)_")
"""What nova2urdf calls the safety controller's volumes in the URDF."""

_SAFETY_TOOL_NAME = re.compile(r"safety_tool_(?P<tcp>.+)")
"""A safety volume of one TCP: everything after the marker names it."""

_TOOL_NAME = re.compile(r"tool_collision_")
"""What nova2urdf calls the tool's own collision geometry."""


def _collision_geometry(urdf: Path) -> tuple[dict[str, list[_Collider]], set[str], set[str]]:
    """A URDF's collision meshes by link, which links show a mesh, which collide.

    All three come from one read. The last is every link with any ``<collision>``
    at all: Rerun's own URDF loader draws those opaque, over the model, so they
    are cleared and drawn here instead -- see :meth:`_hide_loader_collision`.
    """
    from xml.etree import ElementTree

    found: dict[str, list[_Collider]] = {}
    with_visual: set[str] = set()
    with_collision: set[str] = set()
    try:
        root = ElementTree.parse(urdf).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        logger.debug("Could not read collision geometry from {}: {}", urdf, exc)
        return found, with_visual, with_collision
    for link in root.findall("link"):
        name = link.get("name")
        if name is None:
            continue
        if link.find("visual/geometry/mesh") is not None:
            with_visual.add(name)
        if link.find("collision") is not None:
            with_collision.add(name)
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            filename = mesh.get("filename") if mesh is not None else None
            if filename is None:
                continue
            origin = collision.find("origin")
            local = _transform(
                _floats(origin, "xyz") if origin is not None else None,
                _floats(origin, "rpy") if origin is not None else None,
            )
            found.setdefault(name, []).append(
                _Collider(
                    mesh=urdf.parent / filename,
                    local=local,
                    kind=_collider_kind(collision.get("name") or ""),
                    tcp=_safety_tool_of(collision.get("name") or ""),
                )
            )
    return found, with_visual, with_collision


def _collider_kind(name: str) -> str:
    """Which kind of collision geometry a URDF element is, by its name."""
    if _SAFETY_NAME.search(name):
        return "safety"
    if _TOOL_NAME.search(name):
        return "tool"
    return "model"


def _safety_tool_of(name: str) -> str | None:
    """Which TCP a safety tool volume belongs to, read out of its name."""
    match = _SAFETY_TOOL_NAME.search(name)
    return match.group("tcp") if match else None


def _transform3d(matrix: np.ndarray) -> rr.Transform3D:
    """A 4x4 as Rerun's transform archetype."""
    return rr.Transform3D(
        translation=matrix[:3, 3].tolist(),
        quaternion=Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
    )


def _floats(element: Any, attribute: str) -> list[float] | None:
    raw = element.get(attribute)
    return [float(value) for value in raw.split()] if raw else None


def _transform(xyz: list[float] | None, rpy: list[float] | None) -> np.ndarray:
    matrix = np.eye(4)
    if rpy and any(rpy):
        matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    if xyz:
        matrix[:3, 3] = xyz
    return matrix


class UrdfRobotVisualizer:
    """A Rerun-native URDF robot, posed from motion group joint values.

    Duck-types the part of :class:`RobotVisualizer` that callers use:
    :meth:`motion_group_ids`, :meth:`add_motion_group` and
    :meth:`log_robot_geometry`. ``render_from_kinematics`` is true because the
    URDF already places every chain, so no calibration against live state is
    needed.
    """

    render_from_kinematics = True

    def __init__(
        self,
        tree: UrdfTree,
        joints_by_motion_group: dict[str, list[str]],
        *,
        entity_prefix: str = "",
        frame_prefix: str = "",
        recording: RecordingStream | None = None,
    ) -> None:
        self._tree = tree
        self._joints = joints_by_motion_group
        self._entity_prefix = entity_prefix
        self._frame_prefix = frame_prefix
        self._recording = recording
        self._logged_structure = False
        self.show_collision: bool | None = False
        """Whether the URDF's collision geometry is drawn, see-through.

        ``False`` (the default) draws none of it: the collision hulls are what
        the planner sees, not what the robot looks like, so they are shown when
        someone asks. ``True`` draws all of it; ``None`` draws it only for links
        with no visual mesh, which keeps a robot whose model is short a mesh
        looking complete.
        """
        self.show_safety = True
        """Whether the safety controller's volumes are drawn. They are their own
        thing, in their own colour, and their own switch."""
        self.active_tcp: str | None = None
        """Which tool is mounted, which the URDF cannot know: it carries the
        safety volumes of every TCP the controller has configured, and only this
        one's are drawn. Unset draws them all."""
        self._collision: dict[str, list[_Collider]] = {}
        self._links_with_visual: set[str] = set()
        self._links_with_collision: set[str] = set()
        self._collision_entities: list[tuple[str, str, np.ndarray]] | None = None
        self._entities_for: tuple[bool | None, bool, str | None] | None = None
        self._package: Path | None = None
        self._joint_cache: dict[str, UrdfJoint | None] = {}
        self._entries: dict[str, dict[str, Any]] = {}
        self._kinematics = _UrdfKinematics()
        # Last value seen per URDF joint, so a frame can be resolved even when
        # the caller names only the motion groups it commands.
        self._values: dict[str, float] = {}
        # Last value actually logged per joint, to skip writing a pose twice.
        self._logged_values: dict[str, float] = {}

    # ── construction ────────────────────────────────────────────

    @classmethod
    async def for_controller(
        cls,
        controller: Controller,
        *,
        recording: RecordingStream | None = None,
        export_dir: str | Path | None = None,
        tool_assets: dict[str, str] | None = None,
    ) -> UrdfRobotVisualizer | None:
        """Build a visualizer for every motion group of a controller.

        Returns ``None`` when no URDF can be had -- no export to reuse and
        ``nova2urdf`` not installed -- so callers can fall back.
        """
        package = await _urdf_package(controller, export_dir, tool_assets)
        if package is None:
            return None
        visualizer = cls.for_package(package, recording=recording)
        await visualizer._seed_joint_values(controller)
        return visualizer

    @classmethod
    def for_package(
        cls, package: Path, *, recording: RecordingStream | None = None
    ) -> UrdfRobotVisualizer:
        """Build a visualizer from an exported URDF package on disk.

        No controller and no instance: an export carries its own motion group
        index, so a robot can be rendered from one long after it was made.
        """
        from rerun.urdf import UrdfTree

        index = json.loads((package / _SIDECAR).read_text())
        if index.get("length_unit") != "mm":
            logger.warning(
                "URDF export at {} is in {}, but the bridge logs millimetres; "
                "the robot will be the wrong size.",
                package,
                index.get("length_unit"),
            )

        prefix = str(index.get("controller") or "robot")
        tree = UrdfTree.from_file_path(
            package / index["urdf"], entity_path_prefix=prefix, frame_prefix=f"{prefix}_"
        )
        joints = {
            entry["motion_group"]: list(entry["joints"]) for entry in index.get("motion_groups", [])
        }
        entries = {
            entry["motion_group"]: {**entry, "base_link": f"{entry.get('prefix', '')}link0_passive"}
            for entry in index.get("motion_groups", [])
        }
        logger.debug(
            "URDF visualizer for {}: {} motion groups, {} joints",
            prefix,
            len(joints),
            sum(len(names) for names in joints.values()),
        )
        visualizer = cls(
            tree, joints, entity_prefix=prefix, frame_prefix=f"{prefix}_", recording=recording
        )
        visualizer._entries = entries
        visualizer._kinematics = _UrdfKinematics.from_file(package / index["urdf"])
        (visualizer._collision, visualizer._links_with_visual, visualizer._links_with_collision) = (
            _collision_geometry(package / index["urdf"])
        )
        visualizer._package = package
        visualizer._link_root_to_scene()
        return visualizer

    @classmethod
    async def for_motion_group(
        cls,
        motion_group: Any,
        *,
        recording: RecordingStream | None = None,
        export_dir: str | Path | None = None,
        tool_assets: dict[str, str] | None = None,
    ) -> UrdfRobotVisualizer | None:
        """Build a visualizer from a motion group, covering its whole controller.

        A motion group knows which controller it belongs to but does not hand
        one over, so this assembles the controller it needs. Every motion group
        of that controller comes along: an arm cannot be placed without the part
        it rides.
        """
        from nova.cell.controller import Controller

        try:
            controller = Controller(
                configuration=Controller.Configuration(
                    cell_id=motion_group._cell,
                    controller_id=motion_group._controller_id,
                    id=motion_group._controller_id,
                    config=motion_group._api_client.config,
                )
            )
        except (AttributeError, TypeError, ValueError) as exc:
            logger.debug("Could not reach the controller of {}: {}", motion_group, exc)
            return None
        return await cls.for_controller(
            controller, recording=recording, export_dir=export_dir, tool_assets=tool_assets
        )

    async def _seed_joint_values(self, controller: Controller) -> None:
        """Record every motion group's current joints, once.

        A later caller knows only its own -- a policy drives the arms, not the
        lift unit they ride -- and a joint left unseen would be treated as zero,
        which puts the arms where the lift is not.
        """
        try:
            motion_groups = await controller.motion_groups()
        except (OSError, RuntimeError, ValueError, AttributeError) as exc:
            logger.debug("Could not read motion groups to seed joints: {}", exc)
            return
        for motion_group in motion_groups:
            names = self._joints.get(motion_group.id)
            if not names:
                continue
            try:
                state = await motion_group.get_state()
            except (OSError, RuntimeError, ValueError, TypeError, KeyError) as exc:
                logger.debug("Could not read state for {}: {}", motion_group.id, exc)
                continue
            self._values.update(zip(names, list(state.joints), strict=False))

    # ── the RobotVisualizer interface callers use ───────────────

    def _link_root_to_scene(self) -> None:
        """Connect this robot's frame graph to the scene root.

        A 3D view showing one URDF targets that robot's own root frame, so a
        single robot needs nothing. Show two and the view targets the scene root
        instead, which neither robot's graph reaches -- "No transform path from
        ... to the view's target frame" -- and both vanish. Declaring the root's
        parent closes that, and costs a single robot nothing.

        The documented ``entity_path_prefix`` + ``frame_prefix`` recipe for a
        multi-robot setup does not mention this, but without it the second robot
        is invisible.
        """
        frame = self.world_frame
        if frame is None:
            return
        rr.log(
            self._entity_prefix,
            rr.Transform3D(child_frame=frame, parent_frame=_SCENE_ROOT_FRAME),
            static=True,
            recording=self._recording,
        )

    @property
    def world_frame(self) -> str | None:
        """The coordinate frame the robot's root link defines, if any.

        A URDF puts its links in Rerun's *frame* graph, and a 3D view showing it
        adopts that graph's root as the view's target frame. An entity logged
        outside the graph gets an implicit frame of its own with no path to that
        root, so Rerun cannot relate the two and leaves it out of the view --
        which is what happens to the policy overlays. Anchoring them to this
        frame is what puts them back.
        """
        try:
            root = self._tree.root_link().name
        except (AttributeError, RuntimeError) as exc:
            logger.debug("No root link for the URDF frame: {}", exc)
            return None
        return f"{self._frame_prefix}{root}"

    @property
    def entity_root(self) -> str:
        """Entity path everything this visualizer logs lives under.

        The URDF covers a whole controller, so it is logged under the controller
        rather than under any one motion group. A viewport filtering on motion
        group ids has to be told about it or the robot is left out of the view.
        """
        return self._entity_prefix

    def motion_group_ids(self) -> list[str]:
        """Motion groups this visualizer can pose."""
        return list(self._joints)

    def add_motion_group(self, motion_group_id: str, robot: Any = None) -> bool:  # noqa: ARG002
        """Whether this visualizer already covers that motion group.

        The URDF is exported for a whole controller, so there is nothing to add:
        a motion group is either in it or belongs to a different robot.
        """
        return motion_group_id in self._joints

    def log_robot_geometry(self, joint_position: dict[str, list[float]] | list[float]) -> None:
        """Pose the named motion groups. Geometry is logged once, on first call.

        Takes a mapping of motion group id to joint values. A bare list is
        accepted for parity with :class:`RobotVisualizer`, but only means
        something when this visualizer holds a single motion group.
        """
        self._log_structure_once()
        poses = self._as_poses(joint_position)
        driven: set[str] = set()
        for motion_group_id in poses:
            driven.update(self._joints.get(motion_group_id) or [])
        self._log_undriven_joints(driven)
        for motion_group_id, values in poses.items():
            names = self._joints.get(motion_group_id)
            if names is None:
                continue
            for name, value in zip(names, values, strict=False):
                joint = self._joint(name)
                if joint is not None:
                    self._values[name] = value
                    self._logged_values[name] = value
                    rr.log(
                        f"{self._entity_prefix}/{name}",
                        joint.compute_transform(value),
                        recording=self._recording,
                    )
        self._log_collision()

    def _as_poses(
        self, joint_position: dict[str, list[float]] | list[float]
    ) -> dict[str, list[float]]:
        """Normalise a pose argument to a mapping keyed by motion group."""
        if isinstance(joint_position, dict):
            return joint_position
        if len(self._joints) == 1:
            return {next(iter(self._joints)): joint_position}
        logger.warning(
            "Joint values passed as a list, but this URDF holds {} motion "
            "groups; pass a mapping to say which one they belong to.",
            len(self._joints),
        )
        return {}

    def log_robot_geometries(
        self, trajectory: Any, times_column: Any, motion_group_id: str | None = None
    ) -> None:
        """Pose the robot across a whole trajectory in one columnar send.

        A planned trajectory is known up front, so every sample goes as one
        column per joint rather than a write per sample. ``motion_group_id``
        picks the chain when the URDF holds more than one.
        """
        names = self._joints_for(motion_group_id)
        if names is None:
            return
        self._log_structure_once()
        self._log_undriven_joints(set(names))

        samples = getattr(trajectory, "joint_positions", None) or []
        per_joint: dict[str, list[float]] = {name: [] for name in names}
        poses: list[dict[str, float]] = []
        for sample in samples:
            values = getattr(sample, "root", sample)
            pose = dict(self._values)
            for name, value in zip(names, values, strict=False):
                per_joint[name].append(float(value))
                pose[name] = float(value)
            poses.append(pose)

        for name, values in per_joint.items():
            joint = self._joint(name)
            if joint is None or not values:
                continue
            rr.send_columns(
                f"{self._entity_prefix}/{name}",
                indexes=[times_column],
                columns=joint.compute_transform_columns(values, clamp=True),
                recording=self._recording,
            )

        self._log_collision_columns(poses, times_column)
        if poses:
            self._values.update(poses[-1])

    def _log_undriven_joints(self, driven: set[str]) -> None:
        """Pose the joints this call does not drive, from what was read off the robot.

        A caller knows its own motion group: a policy drives the arms, not the
        lift unit they ride, and a trajectory drives one arm of two. The rest
        would render at zero while everything computed from a link's frame --
        the collision geometry, the overlays -- sits at the seeded pose.
        """
        for name, value in self._values.items():
            if name in driven or self._logged_values.get(name) == value:
                continue
            joint = self._joint(name)
            if joint is None:
                continue
            rr.log(
                f"{self._entity_prefix}/{name}",
                joint.compute_transform(value),
                recording=self._recording,
            )
            self._logged_values[name] = value

    def _joints_for(self, motion_group_id: str | None) -> list[str] | None:
        """This motion group's URDF joint names, or the only chain's."""
        if motion_group_id is not None:
            names = self._joints.get(motion_group_id)
            if names is None:
                logger.warning("URDF has no motion group {}", motion_group_id)
            return names
        if len(self._joints) == 1:
            return next(iter(self._joints.values()))
        logger.warning("This URDF holds {} motion groups; name the one to pose.", len(self._joints))
        return None

    def _collision_meshes(self) -> list[tuple[str, str, np.ndarray]]:
        """Log every collision mesh once; say which link each one rides on.

        The geometry stays in the URDF's ``<collision>``, which is where a
        planner looks for it, and is drawn here rather than by Rerun's loader:
        that logs a collision mesh in its link's frame, and Rerun will not
        render a mesh archetype through a frame -- an ellipsoid on the same
        frame shows, a mesh or an asset does not. Placing it by an explicit
        transform does render, so it is loaded once and moved with the robot.
        """
        if not self._collision:
            return []
        wanted = (self.show_collision, self.show_safety, self.active_tcp)
        if self._collision_entities is not None and self._entities_for == wanted:
            return self._collision_entities
        # What was drawn for another tool, or before a switch, has to go: an
        # entity nobody poses again would otherwise stay where it last was.
        previous = {entity for entity, _, _ in self._collision_entities or []}

        import trimesh

        entities: list[tuple[str, str, np.ndarray]] = []
        for link, colliders in self._collision.items():
            for index, collider in enumerate(colliders):
                if not self._draws(collider, link):
                    continue
                try:
                    mesh = trimesh.load_mesh(collider.mesh)
                except (OSError, ValueError) as exc:
                    logger.debug("Could not read {}: {}", collider.mesh, exc)
                    continue
                color = scene_colors.COLLISION_BODY[collider.kind]
                entity_path = f"{self._entity_prefix}/collision/{link}/{index}"
                rr.log(
                    entity_path,
                    rr.Mesh3D(
                        vertex_positions=mesh.vertices,
                        triangle_indices=mesh.faces,
                        albedo_factor=color,
                    ),
                    static=True,
                    recording=self._recording,
                )
                self._log_hull_edges(f"{entity_path}/edges", mesh, color)
                entities.append((entity_path, link, collider.local))
        for stale in previous - {entity for entity, _, _ in entities}:
            rr.log(stale, rr.Clear(recursive=True), recording=self._recording)
        self._collision_entities = entities
        self._entities_for = wanted
        return entities

    def _draws(self, collider: _Collider, link: str) -> bool:
        """Whether this collider is one the caller asked to see."""
        if collider.safety:
            if not self.show_safety:
                return False
            if collider.tcp is None or self.active_tcp is None:
                return True
            # The URDF holds every TCP's; only the mounted tool's belongs here.
            return collider.tcp.startswith(f"{self.active_tcp}_")
        if self.show_collision is False:
            return False
        # ``None``: only where the model gives the link no visual mesh.
        return self.show_collision is True or link not in self._links_with_visual

    def _log_hull_edges(self, entity_path: str, mesh: Any, color: tuple[int, ...]) -> None:
        """Draw a collider's edges, so its shape shows without coating the robot.

        A child of the collider's own entity, so the pose logged there moves the
        edges with it.
        """
        edges = collider_shapes.hull_edges(mesh.vertices, mesh.faces)
        if edges is None:
            return
        rr.log(
            entity_path,
            rr.LineStrips3D(
                edges,
                colors=[[*color[:3], scene_colors.OUTLINE_ALPHA]],
                radii=rr.Radius.ui_points(0.5),
            ),
            static=True,
            recording=self._recording,
        )

    @property
    def has_safety_geometry(self) -> bool:
        """Whether the export carries the safety controller's volumes.

        An export from an older ``nova2urdf`` does not, and then a caller still
        has to draw them from the API.
        """
        return any(
            collider.safety for colliders in self._collision.values() for collider in colliders
        )

    def _log_collision(self) -> None:
        """Place the collision geometry at the pose just logged."""
        for entity_path, link, local in self._collision_meshes():
            placement = self._kinematics.world(link, self._values) @ local
            rr.log(entity_path, _transform3d(placement), recording=self._recording)

    def _log_collision_columns(self, poses: list[dict[str, float]], times_column: Any) -> None:
        """Move the collision geometry along a whole trajectory, one send each."""
        entities = self._collision_meshes()
        if not entities or not poses:
            return
        frames = {
            link: [self._kinematics.world(link, pose) for pose in poses]
            for link in {link for _, link, _ in entities}
        }
        for entity_path, link, local in entities:
            placements = [frame @ local for frame in frames[link]]
            rr.send_columns(
                entity_path,
                indexes=[times_column],
                columns=rr.Transform3D.columns(
                    translation=np.array([placement[:3, 3] for placement in placements]),
                    quaternion=np.array(
                        [
                            Rotation.from_matrix(placement[:3, :3]).as_quat()
                            for placement in placements
                        ]
                    ),
                ),
                recording=self._recording,
            )

    def _log_structure_once(self) -> None:
        """Log geometry and the link tree, which never change."""
        if not self._logged_structure:
            self._tree.log_urdf_to_recording(self._recording)
            self._hide_loader_collision()
            self._logged_structure = True

    def _hide_loader_collision(self) -> None:
        """Drop the collision geometry Rerun's URDF loader draws by itself.

        The loader logs it opaque and in the model's own material, so it covers
        the visual mesh it encloses -- the robot comes out looking like its
        collision hulls. Collision belongs to this class: see-through, in its
        own colour, and only when asked for.
        """
        for link in self._links_with_collision:
            for path in self._tree.get_collision_geometry_paths(link):
                rr.log(path, rr.Clear(recursive=True), static=True, recording=self._recording)

    def chain_bases(self, joint_position: dict[str, list[float]]) -> dict[str, np.ndarray]:
        """Each motion group's base, in the robot's frame.

        A motion group's own ``DHRobot`` starts at its own base and the API
        reports its mounting as zero, so anything drawn from that robot -- TCP
        trails, action-chunk markers, safety geometry -- lands in the arm's own
        frame rather than the robot's. This is the transform that fixes them.
        """
        return self._world_frames(joint_position, lambda entry: entry["base_link"])

    def pose_frames(self, joint_position: dict[str, list[float]]) -> dict[str, np.ndarray]:
        """What takes a motion group's *reported* pose into the robot's frame.

        Nova reports a pose relative to whatever the chain is mounted on, which
        is exactly the frame of the link the chain's mount joint hangs from.
        """
        return self._world_frames(joint_position, lambda entry: entry["parent_link"])

    def _world_frames(
        self, joint_position: dict[str, list[float]], pick: Callable[[dict[str, Any]], str]
    ) -> dict[str, np.ndarray]:
        """World transform of one link per motion group, at the given pose."""
        for motion_group_id, values in self._as_poses(joint_position).items():
            names = self._joints.get(motion_group_id)
            if names:
                self._values.update(zip(names, values, strict=False))

        frames: dict[str, np.ndarray] = {}
        for motion_group_id, entry in self._entries.items():
            try:
                frames[motion_group_id] = self._kinematics.world(pick(entry), self._values)
            except KeyError as exc:
                logger.debug("No URDF link for {}: {}", motion_group_id, exc)
        return frames

    def _joint(self, name: str) -> UrdfJoint | None:
        if name not in self._joint_cache:
            joint = self._tree.get_joint_by_name(name)
            if joint is None:
                logger.warning("URDF has no joint {}", name)
            self._joint_cache[name] = joint
        return self._joint_cache[name]


def _existing_package(root: Path, controller_id: str) -> Path | None:
    """An export in *root* usable for this controller, if there is one.

    nova2urdf names its output directory after the controller with separators
    folded to underscores, so a controller called ``ur5e-left`` is exported to
    ``ur5e_left``. Look for both, and for an export handed to us directly.
    """
    sanitised = controller_id.replace("-", "_").lower()
    for candidate in (root / controller_id, root / sanitised, root):
        if (candidate / _SIDECAR).is_file():
            return candidate
    return None


async def _fingerprint(controller: Controller, tool_assets: dict[str, str] | None = None) -> str:
    """A short digest of everything an export depends on.

    The export bakes in DH parameters, joint limits, mountings, every TCP, the
    cell's stored tool colliders and any tool mesh handed to it, so a cached one
    is only good while those hold. Add a tool and the digest moves, which is
    what stops a stale robot being rendered. The exporter's own version counts
    too: a newer one writes more into the URDF from the same robot.
    """
    parts: list[str] = [
        f"exporter={_exporter_version()}",
        f"tools={sorted((tool_assets or {}).items())}",
        await _collision_store_digest(controller),
    ]
    for motion_group in await controller.motion_groups():
        try:
            description = await motion_group.get_description()
            parts.append(f"{motion_group.id}={description.model_dump_json()}")
        except (OSError, RuntimeError, ValueError, TypeError, AttributeError) as exc:
            logger.debug("Could not fingerprint {}: {}", motion_group.id, exc)
            parts.append(f"{motion_group.id}=?")
    return hashlib.sha256("|".join(parts).encode()).hexdigest()[:12]


def _exporter_version() -> str:
    """Which nova2urdf produced an export, so a newer one invalidates the cache."""
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version("nova2urdf")
    except PackageNotFoundError:
        return "?"


async def _collision_store_digest(controller: Controller) -> str:
    """What the cell's collision store holds, as far as an export copies it.

    A stored tool collider ends up in the URDF's ``<collision>``, so storing one
    has to move the export's digest; otherwise the cache keeps serving a robot
    without its tool.
    """
    gateway = getattr(controller, "_nova_api", None)
    if gateway is None:
        return "store=?"
    cell = controller.configuration.cell_id
    try:
        tools = await gateway.store_collision_components_api.list_stored_collision_tools(cell=cell)
        setups = await gateway.store_collision_setups_api.list_stored_collision_setups(cell=cell)
    except Exception as exc:  # noqa: BLE001  # a cache key is never worth failing a render
        logger.debug("Could not read the collision store of {}: {}", cell, exc)
        return "store=?"
    return f"store={tools}|{setups}"


async def _urdf_package(
    controller: Controller, export_dir: str | Path | None, tool_assets: dict[str, str] | None = None
) -> Path | None:
    """Find or produce a URDF package for a controller."""
    controller_id = controller.id
    configured = export_dir or os.environ.get(_EXPORT_DIR_ENV)
    if configured:
        # An export the caller points at is theirs to keep current.
        found = _existing_package(Path(configured).expanduser(), controller_id)
        if found is not None:
            return found
        logger.debug("No URDF export under {} for {}", configured, controller_id)

    cache = (
        Path(os.environ.get("XDG_CACHE_HOME") or Path(tempfile.gettempdir()))
        / "nova-rerun-bridge"
        / "urdf"
        / await _fingerprint(controller, tool_assets)
    )
    found = _existing_package(cache, controller_id)
    if found is not None:
        return found
    return await _export(controller, cache, tool_assets)


async def _export(
    controller: Controller, cache: Path, tool_assets: dict[str, str] | None = None
) -> Path | None:
    """Export the controller with nova2urdf, in millimetres, into *cache*.

    *tool_assets* are meshes per TCP id that the caller wants shown: the API
    serves no tool mesh, so a viewer's tool asset only reaches the robot by
    being exported onto that TCP's frame.
    """
    try:
        import httpx
        from nova2urdf.api_client import get_headers
        from nova2urdf.export_robot_urdf import export_controller
    except ImportError:
        logger.info(
            "No URDF export found and nova2urdf is not installed, so the URDF "
            "renderer is unavailable. Either install nova2urdf or export once "
            "with `nova2urdf -o <dir> export-controller {}` and point {} at it.",
            controller.id,
            _EXPORT_DIR_ENV,
        )
        return None

    host = await _api_host(controller)
    if host is None:
        logger.debug("No API host reachable from the controller; cannot export a URDF")
        return None

    # nova2urdf reads its target from the environment, like its CLI does.
    os.environ.setdefault("NOVA_API_URL", host)
    os.environ.setdefault("NOVA_CELL", controller.configuration.cell_id)

    cache.mkdir(parents=True, exist_ok=True)
    async with httpx.AsyncClient(headers=get_headers(), timeout=180, verify=False) as client:
        path = await export_controller(
            client,
            controller.id,
            output_dir=str(cache),
            length_divisor=_MILLIMETRES,
            tool_assets=tool_assets,
        )
    if path is None:
        return None
    logger.info("Exported URDF for {} to {}", controller.id, path)
    return Path(path).parent


async def _api_host(controller: Controller) -> str | None:
    """The Nova instance this controller talks to, as nova2urdf wants it."""
    motion_groups = await controller.motion_groups()
    for motion_group in motion_groups:
        host = getattr(motion_group._api_client.config, "host", "")
        if host:
            return str(host).rstrip("/")
    return None
