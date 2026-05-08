"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol, ``CameraSet`` (legacy),
``WebRTCCameraConfig``, and ``WebRTCCameras`` factory.
"""

from policy.cameras._cameras import CameraSet, CameraSource, WebRTCCameraConfig, WebRTCCameras

__all__ = [
    "CameraSet",
    "CameraSource",
    "WebRTCCameraConfig",
    "WebRTCCameras",
]
