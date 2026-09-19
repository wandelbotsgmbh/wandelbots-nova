"""Camera sources for policy observations.

Provides the ``CameraSource`` protocol and ``WebRTCCameras`` factory.
"""

from novapolicy.cameras.protocol import CameraSource
from novapolicy.cameras.webrtc import WebRTCCameras

__all__ = [
    "CameraSource",
    "WebRTCCameras",
]
