from pathlib import Path
from typing import Annotated, Literal

from pydantic import ConfigDict, Field, PrivateAttr
from pydantic.dataclasses import dataclass

from nova import api
from nova.datasets.exceptions import FrameResolutionError
from nova.datasets.frames import FrameTree
from nova.types import Pose


def _detached(entry: str, frame: str) -> FrameResolutionError:
    return FrameResolutionError(
        f"{entry} is expressed in frame '{frame}' but is not attached to a dataset, so the frame "
        "cannot be resolved. Read it from a dataset loaded via nova.datasets.",
        frame=frame,
        chain=[frame],
    )


class DatasetPose(api.models.DatasetPose):
    """A dataset pose whose `pose` is an SDK `Pose` instead of the API wire model.

    `pose` is the pose as taught, relative to `frame`. Use `as_world()` to resolve it
    into world coordinates.
    """

    # `from_attributes` lets an `api.models.DatasetPose` be validated into this type as-is.
    model_config = ConfigDict(from_attributes=True)

    pose: Pose

    _tree: FrameTree | None = PrivateAttr(default=None)

    def as_world(self) -> Pose:
        """Return this pose in world coordinates.

        Raises:
            FrameResolutionError: `frame` cannot be resolved to world.
        """
        if self.frame is None:
            return self.pose
        if self._tree is None:
            raise _detached(f"Pose '{self.dataset_pose}'", self.frame)
        return self._tree.world_transform(self.frame) @ self.pose


class DatasetFrame(api.models.DatasetFrame):
    """A dataset frame whose `pose` is an SDK `Pose` instead of the API wire model.

    `pose` is the frame's pose relative to `reference_frame`. Use `as_world()` to get the
    transform from this frame into world coordinates.
    """

    model_config = ConfigDict(from_attributes=True)

    pose: Pose

    _tree: FrameTree | None = PrivateAttr(default=None)

    def as_world(self) -> Pose:
        """Return the transform from this frame into world coordinates.

        Raises:
            FrameResolutionError: The chain of reference frames cannot be resolved to world.
        """
        if self.reference_frame is None:
            return self.pose
        if self._tree is None:
            raise _detached(f"Frame '{self.frame}'", self.reference_frame)
        return self._tree.world_transform(self.frame)


class Dataset(api.models.Dataset):
    """A Dataset with its persisted poses, frames and command routines.

    Mirrors the API's `GetDatasetResponse`, but with poses, command routines
    and frames keyed by id instead of listed.
    """

    poses: dict[api.models.DatasetPoseId, DatasetPose] = Field(default_factory=dict)
    command_routines: dict[api.models.CommandRoutineId, api.models.CommandRoutine] = Field(
        default_factory=dict
    )
    frames: dict[api.models.FrameId, DatasetFrame] = Field(default_factory=dict)

    _tree: FrameTree | None = PrivateAttr(default=None)

    @classmethod
    def from_api_model(cls, api_dataset: api.models.GetDatasetResponse) -> "Dataset":
        """Convert the api datasets response into the convenience class Dataset"""
        dataset = Dataset(
            **api_dataset.model_dump(exclude={"poses", "command_routines", "frames"}),
            poses={
                pose.dataset_pose: DatasetPose.model_validate(pose) for pose in api_dataset.poses
            },
            command_routines={
                routine.command_routine: routine for routine in api_dataset.command_routines
            },
            frames={
                frame.frame: DatasetFrame.model_validate(frame) for frame in api_dataset.frames
            },
        )
        dataset.attach_frame_tree()
        return dataset

    def attach_frame_tree(self) -> None:
        """Let the poses and frames of this dataset resolve themselves to world coordinates."""
        self._tree = FrameTree(self)
        for entry in (*self.poses.values(), *self.frames.values()):
            entry._tree = self._tree

    def set_frame(
        self,
        frame: api.models.FrameId,
        pose: Pose,
        *,
        reference_frame: api.models.FrameId | None = None,
    ) -> DatasetFrame:
        """Add or replace a frame, for example one measured at runtime by a camera.

        Everything taught on the frame follows it: `as_world()` on any pose below `frame`
        returns a new result from the next call on.

        Args:
            frame: Identifier of the frame to add or replace.
            pose: The frame's pose, relative to `reference_frame`.
            reference_frame: The frame `pose` is expressed in. `None` means `world`.
        """
        if self._tree is None:
            self.attach_frame_tree()
        entry = DatasetFrame(
            frame=frame, pose=pose, reference_frame=reference_frame, dataset=self.dataset
        )
        entry._tree = self._tree
        self.frames[frame] = entry
        return entry

    @property
    def id(self) -> str:
        """Return the dataset ID, aliasing the underlying `dataset` field."""
        return self.dataset


@dataclass(frozen=True)
class LoadRemoteDatasetRequest:
    """
    Request to load a dataset that is stored on the NOVA instance.
    """

    dataset: api.models.DatasetId
    revision: int | None = None
    type: Literal["remote"] = "remote"


@dataclass(frozen=True)
class LoadLocalDatasetRequest:
    """
    Request to load a dataset from a local JSON file.

    `path` is kept exactly as written. A relative one is resolved only when the
    dataset is loaded, against the file of the ``@nova.program`` that declares it
    """

    path: Path
    type: Literal["local"] = "local"


LoadDatasetRequest = Annotated[
    LoadRemoteDatasetRequest | LoadLocalDatasetRequest, Field(discriminator="type")
]


def remote_dataset(
    dataset: api.models.DatasetId, revision: int | None = None
) -> LoadRemoteDatasetRequest:
    """Create a configuration for loading a dataset stored on the NOVA instance.

    Args:
        dataset: Identifier of the dataset to load.
        revision: Revision to load. When omitted, the latest revision is used.
    """
    return LoadRemoteDatasetRequest(dataset=dataset, revision=revision)


def local_dataset(path: str) -> LoadLocalDatasetRequest:
    """Create a configuration for loading a dataset from a local JSON file.

    `path` should be a plain string literal, e.g. ``ds.local_dataset("my_dataset.json")``
    - not an expression such as ``Path(__file__).parent / "..."``, since external
    tooling reads it directly from the program's source code.

    A relative `path` is stored as written and resolved when the dataset is loaded,
    against the file of the ``@nova.program`` that declares it.

    Raises:
        ValueError: If `path` is absolute.
    """
    if Path(path).is_absolute():
        raise ValueError(
            f"local_dataset() path must be relative, got absolute path '{path}'. Relative paths "
            "are resolved against the file of the @nova.program that declares them, so the "
            "dataset is found next to its program on any machine."
        )
    return LoadLocalDatasetRequest(path=Path(path))
