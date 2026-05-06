"""Msgpack serialization for NATS policy wire format.

This module has NO dependencies on ``nova`` or the rest of the ``policy``
package, so it can be imported by lightweight services (e.g. the mock
policy service) without pulling in the full SDK.

Wire format: always msgpack.

**Scalars** (joint positions, IO values) are sent via NATS request/reply
on the main subject.

**Images** are published separately on ``<subject>.images.<camera_name>``
as individual PNG-compressed messages (~400KB each, within NATS 1MB limit).
The policy service subscribes to these and keeps the latest frame per camera.

PNG level 1 is used: lossless, SIMD-accelerated via libpng, ~9ms encode
per 480x640 image.
"""

from __future__ import annotations

from typing import Any

import msgpack
import numpy as np

# PNG compression level 1 = fast, still lossless (~440KB for 480x640)
_PNG_LEVEL = 1


def pack(data: dict[str, Any]) -> bytes:
    """Serialize a scalar dict to msgpack bytes (no numpy arrays allowed)."""
    return msgpack.packb(data)


def unpack(data: bytes) -> dict[str, Any]:
    """Deserialize msgpack bytes to a scalar dict."""
    return msgpack.unpackb(data)


def pack_image(img: np.ndarray) -> bytes:
    """Compress a single RGB image to PNG bytes. Lossless, SIMD-accelerated."""
    import cv2  # noqa: PLC0415

    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    _, buf = cv2.imencode(".png", bgr, [cv2.IMWRITE_PNG_COMPRESSION, _PNG_LEVEL])
    return bytes(buf)


def unpack_image(data: bytes) -> np.ndarray:
    """Decompress PNG bytes back to an RGB numpy array."""
    import cv2  # noqa: PLC0415

    bgr = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
