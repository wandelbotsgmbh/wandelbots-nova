"""Render a robot and its safety volumes in Rerun.

The robot comes from a URDF: :mod:`nova2urdf` derives one from the DH
parameters, meshes and collision model the API serves, and Rerun renders it
natively, so a pose costs one transform per joint. The collision geometry rides
along in that URDF, drawn see-through, which is why none of it is built here any
more.

Safety volumes are a separate matter and stay. They come from live controller
configuration rather than from the model, so they cannot be baked into an
exported URDF; they are drawn from the same link frames the robot uses, which is
what keeps a zone sitting on the arm it belongs to.
"""

from dataclasses import dataclass
import warnings
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import rerun as rr

from nova import api
from nova_rerun_bridge import collider_shapes, scene_colors
from nova_rerun_bridge.urdf_visualizer import UrdfRobotVisualizer

if TYPE_CHECKING:
    from nova_rerun_bridge.dh_robot import DHRobot


@dataclass
class _Volumes:
    """One motion group's safety volume sources: its chain and its colliders."""

    robot: "DHRobot"
    safety_links: dict[int, list[api.models.Collider]]
    safety_tcp: dict[str, Any]


class RobotVisualizer:
    """A robot's links and the controller's safety volumes in one entity tree.

    Build it with :meth:`for_controller`, which covers every motion group of a
    controller: a robot is one kinematic chain even where the API splits it up,
    and an arm cannot be placed without the part it rides.
    """

    def __init__(  # noqa: PLR0913  # a viewer's worth of display options
        self,
        robot: "DHRobot | None" = None,
        robot_model_geometries: list[Any] | None = None,
        tcp_geometries: dict[str, Any] | None = None,
        static_transform: bool = True,
        base_entity_path: str = "robot",
        albedo_factor: list | None = None,
        show_collision: bool | None = False,
        show_safety_link_chain: bool = True,
        collision_link_chain: Any = None,
        collision_tcp: Any = None,
        show_collision_link_chain: bool | None = None,
        show_collision_tool: bool | None = None,
        model_data: Any = None,
        recording: rr.RecordingStream | None = None,
        urdf: UrdfRobotVisualizer | None = None,
        robot_motion_group_id: str | None = None,
    ):
        """
        :param robot: DH robot for the motion group whose safety volumes are drawn.
        :param robot_model_geometries: Safety geometry per link, from the controller.
        :param tcp_geometries: Safety geometry at the TCP, keyed by collider name.
        :param static_transform: Log poses as static rather than on the timeline.
        :param base_entity_path: Entity path prefix for the safety volumes.
        :param albedo_factor: RGB tint for the safety volumes.
        :param show_collision: Draw the robot's collision geometry, see-through.
            It comes from the URDF's ``<collision>`` -- the model's own hulls and
            any tool collider the cell defines -- so it needs nothing passed in.
            ``None`` draws it only for links with no visual mesh.
        :param show_safety_link_chain: Draw the controller's safety volumes.
        :param collision_link_chain: The API's link collision volumes. Stored
            and readable as ``collision_link_geometries``, but not drawn: the
            collision geometry on screen comes from the URDF, which is also
            what a planner reads.
        :param collision_tcp: The API's tool collision volumes, as above.
        :param show_collision_link_chain: Deprecated alias of *show_collision*.
        :param show_collision_tool: Deprecated alias of *show_collision*.
        :param model_data: Accepted and ignored: a robot is rendered from its
            URDF, not from a mesh blob.
        :param recording: Recording stream that receives these entities.
        :param urdf: The URDF-backed robot. :meth:`for_controller` supplies it;
            without one only the safety volumes are drawn.
        :param robot_motion_group_id: Which motion group *robot* describes. A
            robot the API splits across several needs it: the volumes hang off
            that chain's base, and without a name there is no telling which
            base, so they land at the cell origin instead.
        """
        self.robot = robot
        self.robot_motion_group_id = robot_motion_group_id
        self.urdf = urdf
        self._volumes: dict[str, _Volumes] = {}
        self.recording = recording
        self.static_transform = static_transform
        self.base_entity_path = base_entity_path.rstrip("/")
        self.albedo_factor = albedo_factor or [255, 255, 255]
        # A caller that names a tint means it (the policy view uses its own);
        # otherwise the safety volumes take the design system's colour rather
        # than the mesh default, which is white and reads as no colour at all.
        self._volume_tint: tuple[int, ...] = (
            tuple(albedo_factor[:3]) if albedo_factor else scene_colors.SAFETY_VOLUME[:3]
        )
        self.tcp_geometries = tcp_geometries or {}
        self.show_safety_link_chain = show_safety_link_chain
        # Deprecated aliases of show_collision, readable as attributes.
        self.show_collision_link_chain = (
            show_collision_link_chain
            if show_collision_link_chain is not None
            else show_collision is not False
        )
        self.show_collision_tool = (
            show_collision_tool if show_collision_tool is not None else show_collision is not False
        )
        if show_collision_link_chain is not None or show_collision_tool is not None:
            warnings.warn(
                "show_collision_link_chain and show_collision_tool are deprecated; "
                "use show_collision, which covers both.",
                DeprecationWarning,
                stacklevel=2,
            )
            if show_collision is False:
                show_collision = bool(show_collision_link_chain or show_collision_tool)
        self.show_collision = show_collision
        if urdf is not None:
            urdf.show_collision = show_collision
            urdf.show_safety = show_safety_link_chain
        # What the API says about collision volumes, for callers that read it.
        # Drawing comes from the URDF; see the constructor's docstring.
        self.collision_link_geometries: list[Any] = (
            cast(list[Any], collision_link_chain) if collision_link_chain else []
        )
        self.collision_tcp_geometries: dict[str, api.models.Collider] = (
            collision_tcp if collision_tcp else {}
        )
        self._logged_shapes: set[str] = set()

        self.link_geometries: dict[int, list[api.models.Collider]] = {}
        for link_chain in robot_model_geometries or []:
            for link_index, link in enumerate(link_chain or []):
                self.link_geometries.setdefault(link_index, []).extend(link.values())

        if robot is not None:
            self.add_volumes(
                robot_motion_group_id,
                robot,
                safety_links=self.link_geometries,
                safety_tcp=self.tcp_geometries,
            )

    # ── kinematics ──────────────────────────────────────────────

    def compute_forward_kinematics(self, joint_positions: list[float]) -> list[np.ndarray]:
        """Link transforms at this pose, from the motion group's DH parameters."""
        if self.robot is None:
            return []
        accumulated = self.robot.pose_to_matrix(self.robot.mounting)
        transforms = [accumulated.copy()]
        for dh_param, joint_position in zip(
            self.robot.dh_parameters, joint_positions, strict=False
        ):
            transform = self.robot.dh_transform(dh_param=dh_param, joint_position=joint_position)
            accumulated = accumulated @ transform
            transforms.append(accumulated.copy())
        return transforms

    # ── construction ────────────────────────────────────────────

    @classmethod
    async def for_controller(
        cls,
        controller: Any,
        *,
        base_entity_path: str | None = None,
        recording: rr.RecordingStream | None = None,
        static_transform: bool = False,
        albedo_factor: list | None = None,
        robot: "DHRobot | None" = None,
        robot_model_geometries: list[Any] | None = None,
        tcp_geometries: dict[str, Any] | None = None,
        robot_motion_group_id: str | None = None,
        show_collision: bool | None = None,
    ) -> "RobotVisualizer":
        """Build a visualizer for every motion group of a controller.

        *robot* and the geometry arguments belong to one motion group and only
        drive its safety volumes; the robot itself comes from the URDF.
        """
        urdf = await UrdfRobotVisualizer.for_controller(controller, recording=recording)
        if urdf is None:
            msg = (
                f"No URDF available for {getattr(controller, 'id', controller)}. "
                "Install nova2urdf or point NOVA_URDF_EXPORT_DIR at an export."
            )
            raise RuntimeError(msg)
        return cls(
            base_entity_path=base_entity_path or urdf.entity_root,
            recording=recording,
            static_transform=static_transform,
            albedo_factor=albedo_factor,
            urdf=urdf,
            show_collision=show_collision,
            robot=robot,
            robot_model_geometries=robot_model_geometries,
            tcp_geometries=tcp_geometries,
            robot_motion_group_id=robot_motion_group_id,
        )

    # ── the robot itself, delegated to the URDF ─────────────────

    @property
    def entity_root(self) -> str:
        """Entity path everything this visualizer logs lives under."""
        return self.urdf.entity_root if self.urdf else self.base_entity_path

    @property
    def world_frame(self) -> str | None:
        """The coordinate frame the robot's root link defines."""
        return self.urdf.world_frame if self.urdf else None

    def motion_group_ids(self) -> list[str]:
        """The motion groups this visualizer drives."""
        return self.urdf.motion_group_ids() if self.urdf else []

    def add_motion_group(self, motion_group_id: str, robot: Any = None) -> bool:
        """Whether this visualizer already covers that motion group."""
        return bool(self.urdf and self.urdf.add_motion_group(motion_group_id, robot))

    def chain_bases(self, joint_position: dict[str, list[float]]) -> dict[str, np.ndarray]:
        """Each motion group's base, in the robot's frame."""
        return self.urdf.chain_bases(joint_position) if self.urdf else {}

    def pose_frames(self, joint_position: dict[str, list[float]]) -> dict[str, np.ndarray]:
        """What takes a motion group's reported pose into the robot's frame."""
        return self.urdf.pose_frames(joint_position) if self.urdf else {}

    def log_robot_geometry(self, joint_position: dict[str, list[float]] | list[float]) -> None:
        """Pose the robot, then its safety volumes."""
        if self.urdf is not None:
            self.urdf.log_robot_geometry(joint_position)
        if isinstance(joint_position, dict):
            self._log_volumes(joint_position)
        elif self.robot_motion_group_id is not None:
            self._log_volumes({self.robot_motion_group_id: joint_position})
        elif self._volumes:
            self._log_volumes({"": joint_position})

    def log_robot_geometries(
        self, trajectory: Any, times_column: Any, motion_group_id: str | None = None
    ) -> None:
        """Pose the robot across a whole trajectory in one columnar send."""
        if self.urdf is not None:
            self.urdf.log_robot_geometries(trajectory, times_column, motion_group_id)
        samples = getattr(trajectory, "joint_positions", None) or []
        if samples:
            last = list(getattr(samples[-1], "root", samples[-1]))
            named = motion_group_id or self.robot_motion_group_id or ""
            self._log_volumes({named: last})

    # ── safety and collision volumes ────────────────────────────

    def add_volumes(
        self,
        motion_group_id: str | None,
        robot: "DHRobot",
        *,
        safety_links: dict[int, list[api.models.Collider]] | None = None,
        safety_tcp: dict[str, Any] | None = None,
    ) -> None:
        """Register one motion group's safety volumes, so they can all be drawn.

        A robot the API splits across motion groups has a set per part, each on
        its own chain. Registering them by motion group keeps them apart -- both
        the chain their frames come from and the entity path they land on, which
        otherwise collide and leave only the last part drawn.
        """
        key = motion_group_id or ""
        self._volumes[key] = _Volumes(
            robot=robot, safety_links=safety_links or {}, safety_tcp=safety_tcp or {}
        )

    def _link_frames(
        self, robot: "DHRobot", motion_group_id: str, joint_position: list[float]
    ) -> list[np.ndarray]:
        """Link frames to hang a motion group's volumes on, in the robot's frame.

        Taken from that motion group's own DH chain, based where the URDF puts
        the chain, so a zone lands on the arm rather than at the cell origin.
        """
        base = np.eye(4)
        if self.urdf is not None:
            named = motion_group_id or None
            ids = self.urdf.motion_group_ids()
            if named is None and len(ids) == 1:
                named = ids[0]
            if named is not None:
                base = self.urdf.chain_bases({named: joint_position}).get(named, base)
        frames = [base.copy()]
        accumulated = base
        for dh_param, value in zip(robot.dh_parameters, joint_position, strict=False):
            accumulated = accumulated @ robot.dh_transform(dh_param=dh_param, joint_position=value)
            frames.append(accumulated.copy())
        return frames

    @property
    def _safety_from_urdf(self) -> bool:
        """Whether the URDF already carries the safety volumes.

        nova2urdf exports them as ``<collision>`` named ``safety_*``, and the
        URDF drives them from the link tree rather than from a pose computed
        here, so drawing them from the API too would only double them up. An
        export from an older version has none, and then this path still does.
        """
        return self.urdf is not None and self.urdf.has_safety_geometry

    def _log_volumes(self, joint_position: dict[str, list[float]]) -> None:
        """Draw the registered safety volumes for every motion group named here.

        Only for what the URDF does not already carry: see
        :attr:`_safety_from_urdf`. Everything the URDF does carry -- the model's
        collision meshes, the tool's, the safety controller's volumes -- is
        drawn from there, in :class:`UrdfRobotVisualizer`.
        """
        for motion_group_id, values in joint_position.items():
            volumes = self._volumes.get(motion_group_id) or self._volumes.get("")
            if volumes is None:
                continue
            transforms = self._link_frames(volumes.robot, motion_group_id, values)
            if not transforms:
                continue
            branch = f"{self.base_entity_path}"
            if len(self._volumes) > 1:
                branch = f"{branch}/{rr.escape_entity_path_part(motion_group_id)}"

            if not self.show_safety_link_chain or self._safety_from_urdf:
                continue
            for link_index, geometries in volumes.safety_links.items():
                if link_index >= len(transforms):
                    continue
                for position, collider in enumerate(geometries):
                    self._log_collider(
                        f"{branch}/safety_from_controller/links/"
                        f"link_{link_index}/geometry_{position}",
                        collider,
                        transforms[link_index],
                    )
            for name, collider in volumes.safety_tcp.items():
                self._log_collider(
                    f"{branch}/safety_from_controller/tcp/{rr.escape_entity_path_part(name)}",
                    collider,
                    transforms[-1],
                )

    def _log_collider(
        self, entity_path: str, collider: api.models.Collider, link_frame: np.ndarray
    ) -> None:
        """Place one collider on a link and draw its shape, see-through."""
        collider_shapes.log_collider(
            entity_path,
            collider,
            link_frame,
            color=self._volume_color(),
            recording=self.recording,
            static=self.static_transform,
            with_shape=entity_path not in self._logged_shapes,
        )
        self._logged_shapes.add(entity_path)

    def _volume_color(self) -> tuple[int, ...]:
        """The colour for a safety volume, drawn as a wireframe over the robot."""
        return (*self._volume_tint, collider_shapes.VOLUME_ALPHA)

    def _log(self, *args, **kwargs):
        rr.log(*args, recording=self.recording, **kwargs)
