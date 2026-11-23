"""Background pose sources for ``Observation.pose`` (docs/schema.md#object-poses)."""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Protocol, runtime_checkable

import httpx

from nova.types import Pose

logger = logging.getLogger(__name__)

DEFAULT_POLL_HZ = 60.0
_POSE_WIDTH = 6


@runtime_checkable
class PoseSource(Protocol):
    """A pose read in the background: started with the run, read without waiting."""

    async def start(self) -> None:
        """Begin reading; return once the first pose is there."""

    async def stop(self) -> None:
        """Stop reading."""

    def read(self) -> Pose:
        """The latest pose."""


class OmniverseScene:
    """An Isaac Sim scene, read through the Wandelbots NOVA extension API, one scene for any prims.

    ``api_url`` is the extension's full base URL (``http://<host>:8011/omniservice/api/v2``).
    Poses come back in mm and as a rotation vector.
    """

    def __init__(
        self,
        api_url: str,
        *,
        poll_hz: float = DEFAULT_POLL_HZ,
        timeout_s: float = 2.0,
        headers: dict[str, str] | None = None,
        client: httpx.AsyncClient | None = None,
    ) -> None:
        if poll_hz <= 0:
            msg = "poll_hz must be positive"
            raise ValueError(msg)
        self.api_url = api_url.rstrip("/")
        self.poll_hz = poll_hz
        self._timeout_s = timeout_s
        self._headers = headers or {}
        self._client = client
        self._owns_client = client is None
        self._users = 0

    def prim(self, prim_path: str, *, relative_to: str | None = None) -> OmniversePrimPose:
        """A pose source for ``prim_path``, in world or relative to prim ``relative_to``."""
        return OmniversePrimPose(self, prim_path, relative_to=relative_to)

    async def get_pose(self, prim_path: str, *, relative_to: str | None = None) -> Pose:
        """One pose request."""
        client = self._client
        if client is None:
            msg = "OmniverseScene is not open: start a prim source first"
            raise RuntimeError(msg)
        if relative_to is None:
            path = "/prims/poses"
            params = {"prim_path": prim_path, "coordinate_system": "world"}
        else:
            # ~relative_to :: prim -- the prim expressed in relative_to's frame
            path = "/prims/poses/relative"
            params = {"prim_path_1": relative_to, "prim_path_2": prim_path, "mode": "inverse_first"}
        response = await client.get(path, params={**params, "rotation_type": "cartesian"})
        response.raise_for_status()
        values = response.json().get("pose")
        if not isinstance(values, list) or len(values) != _POSE_WIDTH:
            msg = f"Unexpected pose for {prim_path!r} from {self.api_url}{path}: {values!r}"
            raise ValueError(msg)
        return Pose(tuple(float(v) for v in values))

    async def _acquire(self) -> None:
        if self._client is None:
            self._client = httpx.AsyncClient(
                base_url=self.api_url, timeout=self._timeout_s, headers=self._headers
            )
        self._users += 1

    async def _release(self) -> None:
        self._users = max(0, self._users - 1)
        if self._users == 0 and self._owns_client and self._client is not None:
            await self._client.aclose()
            self._client = None


class OmniversePrimPose:
    """The pose of one prim of an :class:`OmniverseScene`, polled in the background."""

    def __init__(self, scene: OmniverseScene, prim_path: str, *, relative_to: str | None) -> None:
        self.scene = scene
        self.prim_path = prim_path
        self.relative_to = relative_to
        self._latest: tuple[float, Pose] | None = None
        self._task: asyncio.Task[None] | None = None
        self._failing = False

    async def start(self) -> None:
        """Open the scene and read the first pose; raises if the prim cannot be read."""
        if self._task is not None:
            return
        await self.scene._acquire()
        try:
            pose = await self.scene.get_pose(self.prim_path, relative_to=self.relative_to)
        except BaseException:
            await self.scene._release()
            raise
        self._latest = (time.monotonic(), pose)
        self._task = asyncio.create_task(self._poll())

    async def stop(self) -> None:
        task, self._task = self._task, None
        if task is None:
            return
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)  # wait for the poller to end
        self._latest = None
        await self.scene._release()

    def read(self) -> Pose:
        if self._latest is None:
            msg = f"No pose of {self.prim_path!r} yet: the source is not started"
            raise RuntimeError(msg)
        return self._latest[1]

    async def _poll(self) -> None:
        period = 1.0 / self.scene.poll_hz
        while True:
            started = time.monotonic()
            try:
                pose = await self.scene.get_pose(self.prim_path, relative_to=self.relative_to)
            except (httpx.HTTPError, ValueError) as exc:
                if not self._failing:
                    logger.warning("Reading the pose of %s failed: %s", self.prim_path, exc)
                self._failing = True
            else:
                self._latest = (time.monotonic(), pose)
                if self._failing:
                    logger.info("Reading the pose of %s works again", self.prim_path)
                self._failing = False
            await asyncio.sleep(max(0.0, period - (time.monotonic() - started)))
