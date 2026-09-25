"""Resolution of dataset frame chains to world coordinates.

A dataset frame is expressed relative to its `reference_frame`, which is itself a dataset
frame, and so on until a frame with no reference - that one is expressed in `world`. Composing
that chain is plain pose arithmetic, so it is done locally instead of per-pose over the API.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from nova.datasets.exceptions import FrameResolutionError
from nova.types import Pose

if TYPE_CHECKING:
    from nova.datasets.types import Dataset, DatasetFrame


def _identity() -> Pose:
    # A fresh instance every time: `Pose` is not frozen, so a shared one could be mutated.
    return Pose((0, 0, 0, 0, 0, 0))


class FrameTree:
    """The frames of a dataset, resolvable to world coordinates.

    Reads the frames off the dataset on every call, so replacing a frame at runtime - a pallet
    measured by a camera, say - immediately changes what every pose on that frame resolves to.
    Chains are a few links deep, so nothing is cached.

    Resolution is lazy: a dataset with a frame reference that cannot be resolved still loads,
    and only the entries that depend on the broken chain raise.
    """

    def __init__(self, dataset: Dataset):
        self._dataset = dataset

    @property
    def _frames(self) -> dict[str, DatasetFrame]:
        return self._dataset.frames

    def world_transform(self, frame: str | None) -> Pose:
        """Return the transform from `frame` into world coordinates.

        Args:
            frame: The frame to resolve. `None` means `world`, giving the identity transform.

        Raises:
            FrameResolutionError: A frame in the chain is not defined in the dataset, or the
                chain contains a cycle.
        """
        if frame is None:
            return _identity()

        frames = self._frames
        transform = _identity()
        for frame_id in reversed(self._walk(frame)):
            transform = transform @ frames[frame_id].pose
        return transform

    def _walk(self, frame: str) -> list[str]:
        """Collect the chain from `frame` up to (but excluding) world, outermost first."""
        frames = self._frames
        chain: list[str] = []
        current: str | None = frame
        while current is not None:
            if current in chain:
                raise FrameResolutionError(
                    f"Frame '{frame}' cannot be resolved to world: frame '{current}' references "
                    "itself through a cycle of reference frames",
                    frame=frame,
                    chain=[*chain, current],
                )
            if current not in frames:
                raise FrameResolutionError(
                    f"Frame '{frame}' cannot be resolved to world: frame '{current}' is not "
                    "defined in the dataset",
                    frame=frame,
                    chain=[*chain, current],
                )
            chain.append(current)
            current = frames[current].reference_frame
        return chain
