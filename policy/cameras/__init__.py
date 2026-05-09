"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol, ``WebRTCCameraConfig``,
and ``WebRTCCameras`` factory.
"""

from policy.cameras._cameras import CameraSource, WebRTCCameraConfig, WebRTCCameras

__all__ = [
    "CameraSource",
    "WebRTCCameraConfig",
    "WebRTCCameras",
]
