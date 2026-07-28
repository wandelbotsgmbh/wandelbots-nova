from pydantic import AwareDatetime

from nova import api


# TODO: Remove this compatibility model once `DatasetPose` is available in the minimum
# supported version of `wandelbots_api_client`. Until then, keep its fields aligned with
# the API schema so SDK users can adopt dataset poses before the generated model is released.
class DatasetPose(api.models.ConfiguredPose):
    """
    A dataset pose is a taught pose that belongs to a dataset. It extends the
    shared `ConfiguredPose` with dataset-specific fields such as an identifier, human-readable
    name, timestamps, and metadata.
    """

    id: str
    """
    Unique identifier of the pose. This identifier is unique across the entire dataset.
    """
    dataset_id: str | None = None
    """
    Identifier of the dataset that contains this pose.
    """
    name: str | None = None
    """
    Human-readable name for the pose that is shown in the user interface.
    """
    description: str | None = None
    """
    Free-form text that describes the purpose of the pose.
    """
    created_at: AwareDatetime | None = None
    """
    Timestamp when the pose was created, in RFC3339 format.
    """
    updated_at: AwareDatetime | None = None
    """
    Timestamp when the pose was last updated, in RFC3339 format.
    """
    metadata: dict[str, str] | None = None
    """
    Additional key-value pairs that can be attached to the pose.
    """
