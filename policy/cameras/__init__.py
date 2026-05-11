"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol and ``WebRTCCameras`` factory.
"""

from policy.cameras.protocol import CameraSource
from policy.cameras.webrtc import WebRTCCameras

__all__ = [
    "CameraSource",
    "WebRTCCameras",
]
