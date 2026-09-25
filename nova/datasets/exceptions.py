class DatasetError(Exception):
    """Base class for all nova.datasets errors."""


class DatasetNotFoundError(DatasetError):
    """The requested dataset, revision, or local dataset file does not exist."""


class FrameResolutionError(DatasetError):
    """A frame could not be resolved to world coordinates.

    Raised when the chain of `reference_frame` links leaves the dataset (a frame is
    referenced but not defined) or loops back on itself.
    """

    def __init__(self, message: str, *, frame: str, chain: list[str]):
        self.frame = frame
        self.chain = chain
        # The chain stops where resolution broke, so it deliberately does not end in `world`.
        super().__init__(f"{message} (chain: {' -> '.join(chain)})")
