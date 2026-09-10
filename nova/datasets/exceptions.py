class DatasetError(Exception):
    """Base class for all nova.datasets errors."""


class DatasetNotFoundError(DatasetError):
    """The requested dataset, revision, or local dataset file does not exist."""
