import asyncio
import functools
import json
import logging
import time
from abc import ABC
from collections.abc import AsyncGenerator
from typing import Any, TypeVar
from urllib.parse import quote, urlencode

import websockets
from websockets.asyncio.client import connect as websocket_connect

from nova import api
from nova.cell.robot_cell import ConfigurablePeriphery, Device
from nova.cell.state_stream import MotionGroupStateStreamRegistry, SharedMotionGroupStateStream
from nova.config import NovaConfig
from nova.exceptions import MotionGroupStateStreamClosed
from nova.version import version as pkg_version

logger = logging.getLogger(__name__)


_API_VERSION = "v2"
T = TypeVar("T")


class _Interceptor:
    def __init__(self, instance: T, gateway: "ApiGateway"):
        self._instance = instance
        self._gateway = gateway

    def __getattr__(self, name):
        original_attr = getattr(self._instance, name)
        if not callable(original_attr):
            return original_attr

        if asyncio.iscoroutinefunction(original_attr):

            @functools.wraps(original_attr)
            async def async_wrapper(*args, **kwargs):
                start = time.time()
                try:
                    return await original_attr(*args, **kwargs)
                except Exception as e:
                    logger.error(f"API CALL: {name} failed with error: {e}")
                    logger.debug(f"API CALL FAILED: {name} with args={args}, kwargs={kwargs}")
                    raise e
                finally:
                    duration = time.time() - start
                    logger.debug(f"API CALL: {name} took {duration:.2f} seconds")
                    logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

            return async_wrapper

        @functools.wraps(original_attr)
        def sync_wrapper(*args, **kwargs):
            start = time.time()
            try:
                return original_attr(*args, **kwargs)
            except Exception as e:
                logger.error(f"API CALL: {name} failed with error: {e}")
                raise e
            finally:
                duration = time.time() - start
                logger.debug(f"API CALL: {name} took {duration:.2f} seconds")
                logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

        return sync_wrapper

    # provides tab completion in python interpreter
    def __dir__(self):
        return self._instance.__dir__()


def _intercept(api_instance: T, gateway: "ApiGateway") -> T:
    # we ignore the type error here because
    # we want the return type to be the same as the original api instance to not break typing support
    return _Interceptor(api_instance, gateway)  # ty: ignore[invalid-return-type]


class ApiGateway:
    def __init__(self, config: NovaConfig):
        self.config = config
        self._init_api_client()
        # One shared state stream per motion group for every consumer of this
        # gateway. Created once here (not in _init_api_client) so a client
        # reinitialization cannot orphan running pumps; the opener resolves
        # self.motion_group_api late for the same reason.
        self._motion_group_state_streams = MotionGroupStateStreamRegistry(
            open_stream=self._open_motion_group_state_stream,
            linger_secs=config.motion_group_state_stream_linger_secs,
        )

    async def _open_motion_group_state_stream(
        self, cell: str, controller_id: str, motion_group_id: str, response_rate_msecs: int | None
    ) -> AsyncGenerator[api.models.MotionGroupState, None]:
        """Stream motion group states, raising when the server or network ends the stream.

        Replaces the generated ``stream_motion_group_state``, which swallows every
        ``ConnectionClosed`` and so hides why a stream ended. Only the consumer
        closing the generator ends it silently.
        """
        configuration = self._api_client.configuration
        host: str = configuration.host
        headers = websockets.Headers()
        if host.startswith("https://"):
            host = host.removeprefix("https://")
            if configuration.username:
                user = quote(configuration.username, safe="")
                password = quote(configuration.password or "", safe="")
                host = f"{user}:{password}@{host}"
            elif configuration.access_token:
                headers["Authorization"] = f"Bearer {configuration.access_token}"
            host = f"wss://{host}"
        else:
            host = host.replace("http://", "ws://", 1)

        path = (
            f"/cells/{quote(cell, safe='@')}/controllers/{quote(controller_id, safe='@')}"
            f"/motion-groups/{quote(motion_group_id, safe='@')}/state-stream"
        )
        url = host + path
        if response_rate_msecs is not None:
            url += "?" + urlencode({"response_rate": response_rate_msecs})

        options: dict[str, Any] = {"open_timeout": 10, "additional_headers": headers}
        options.update(getattr(configuration, "websocket_options", None) or {})
        name = f"{cell}/{controller_id}/{motion_group_id}"

        async with websocket_connect(url, **options) as websocket:
            try:
                async for message in websocket:
                    data = json.loads(message)
                    if "result" not in data:
                        raise Exception(data)
                    yield api.models.MotionGroupState.model_validate(data["result"])
            except websockets.exceptions.ConnectionClosedError as e:
                raise MotionGroupStateStreamClosed(
                    name, code=websocket.close_code, reason=websocket.close_reason, detail=str(e)
                ) from e
            raise MotionGroupStateStreamClosed(
                name, code=websocket.close_code, reason=websocket.close_reason
            )

    def motion_group_state_stream(
        self, cell: str, controller_id: str, motion_group_id: str
    ) -> SharedMotionGroupStateStream:
        """The shared motion-group state stream for one motion group of this gateway."""
        return self._motion_group_state_streams.stream(cell, controller_id, motion_group_id)

    def _init_api_client(self):
        """Initialize or reinitialize the API client with current credentials"""
        api_client_config = api.Configuration(
            host=f"{self.config.host}/api/{_API_VERSION}", access_token=self.config.access_token
        )
        api_client_config.verify_ssl = self.config.verify_ssl
        self._api_client = api.ApiClient(configuration=api_client_config)
        self._api_client.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"

        # Use the intercept function to wrap each API client
        self.system_api = _intercept(api.api.SystemApi(api_client=self._api_client), self)
        self.controller_api = _intercept(api.api.ControllerApi(api_client=self._api_client), self)
        self.controller_ios_api = _intercept(
            api.api.ControllerInputsOutputsApi(api_client=self._api_client), self
        )
        self.virtual_controller_api = _intercept(
            api.api.VirtualControllerApi(api_client=self._api_client), self
        )
        self.virtual_controller_behavior_api = _intercept(
            api.api.VirtualControllerBehaviorApi(api_client=self._api_client), self
        )
        # TODO migrate stuff in rerun bridge and then remove this
        self.virtual_robot_setup_api = self.virtual_controller_api
        self.motion_group_api = _intercept(
            api.api.MotionGroupApi(api_client=self._api_client), self
        )
        self.motion_group_jogging_api = _intercept(
            api.api.JoggingApi(api_client=self._api_client), self
        )
        self.store_collision_components_api = _intercept(
            api.api.StoreCollisionComponentsApi(api_client=self._api_client), self
        )
        self.motion_group_models_api: api.api.MotionGroupModelsApi = _intercept(
            api.api.MotionGroupModelsApi(api_client=self._api_client), self
        )
        self.store_collision_setups_api = _intercept(
            api.api.StoreCollisionSetupsApi(api_client=self._api_client), self
        )
        self.trajectory_planning_api: api.api.TrajectoryPlanningApi = _intercept(
            api.api.TrajectoryPlanningApi(api_client=self._api_client), self
        )
        self.trajectory_execution_api: api.api.TrajectoryExecutionApi = _intercept(
            api.api.TrajectoryExecutionApi(api_client=self._api_client), self
        )
        self.trajectory_caching_api: api.api.TrajectoryCachingApi = _intercept(
            api.api.TrajectoryCachingApi(api_client=self._api_client), self
        )
        self.controller_inputs_outputs_api = _intercept(
            api.api.ControllerInputsOutputsApi(api_client=self._api_client), self
        )
        self.jogging_api = _intercept(api.api.JoggingApi(api_client=self._api_client), self)
        self.store_object_api = _intercept(
            api.api.StoreObjectApi(api_client=self._api_client), self
        )
        self.kinematics_api = _intercept(api.api.KinematicsApi(api_client=self._api_client), self)
        self.cell_api = _intercept(api.api.CellApi(api_client=self._api_client), self)
        self.bus_ios_api = _intercept(
            api.api.BUSInputsOutputsApi(api_client=self._api_client), self
        )
        self.datasets_api = _intercept(api.api.DatasetsApi(api_client=self._api_client), self)

        logger.debug(f"NOVA API client initialized with user agent {self._api_client.user_agent}")

    async def close(self):
        await self._motion_group_state_streams.aclose()
        await self._api_client.close()


class NovaDevice(ConfigurablePeriphery, Device, ABC, is_abstract=True):
    class Configuration(ConfigurablePeriphery.Configuration):
        config: NovaConfig

    def __init__(
        self, configuration: Configuration, *, api_gateway: ApiGateway | None = None, **kwargs
    ):
        self._nova_config = configuration.config
        self._nova_api_gateway: ApiGateway | None = api_gateway
        # A gateway owns an api.ApiClient and with it an aiohttp session. Only close the gateway
        # if we created it ourselves; a shared one belongs to its owner (usually the Nova
        # instance) and is closed there.
        self._owns_api_gateway = api_gateway is None
        super().__init__(configuration, **kwargs)

    @property
    def _nova_api(self) -> ApiGateway:
        # Ensure we always have an API gateway instance
        # TODO ideally? we would do this in open and ensure it's not used before being opened
        if self._nova_api_gateway is None:
            self._nova_api_gateway = ApiGateway(self._nova_config)
        return self._nova_api_gateway

    async def open(self) -> None:
        return await super().open()

    async def close(self):
        await super().close()
        if self._owns_api_gateway and self._nova_api_gateway is not None:
            await self._nova_api_gateway.close()
