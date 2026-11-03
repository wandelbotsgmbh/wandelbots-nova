from scipy.spatial.transform import Rotation

from nova import api


def to_rotation_vector(
    orientation: api.models.Orientation, orientation_type: api.models.OrientationType
) -> api.models.RotationVector:
    if orientation_type == api.models.OrientationType.ROTATION_VECTOR:
        return orientation[0], orientation[1], orientation[2]

    if orientation_type == api.models.OrientationType.QUATERNION:
        x, y, z, w = orientation
        return tuple(Rotation.from_quat([x, y, z, w]).as_rotvec())

    name = orientation_type.value
    intrinsic = "INTRINSIC" in name
    seq = name.rsplit("_", 1)[-1]
    seq = seq.lower() if intrinsic else seq.upper()
    return tuple(Rotation.from_euler(seq, orientation, degrees=False).as_rotvec())
