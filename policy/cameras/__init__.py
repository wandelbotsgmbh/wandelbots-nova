"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol, ``CameraSet`` (WebRTC implementation),
and ``WebRTCCameraConfig``.
"""

from policy.cameras._cameras import CameraSet, CameraSource, WebRTCCameraConfig

__all__ = [
    "CameraSet",
    "CameraSource",
    "WebRTCCameraConfig",
]
