"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol and ``WebRTCCameras`` factory.
"""

from policy.cameras._cameras import CameraSource, WebRTCCameras

__all__ = [
    "CameraSource",
    "WebRTCCameras",
]
