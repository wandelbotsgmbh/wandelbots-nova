# TODO parameter naming convention (controller -> controller_id)

from __future__ import annotations

import asyncio
import functools
import time
from abc import ABC
from enum import Enum
from typing import TypeVar

from nova import api
from nova.auth.auth_config import Auth0Config
from nova.auth.authorization import Auth0DeviceAuthorization
from nova.cell.robot_cell import ConfigurablePeriphery, Device
from nova.config import NovaConfig
from nova.utils.env_utils import set_key
from nova.version import version as pkg_version

import logging

_logger = logging.getLogger(__name__)


class ComparisonType(Enum):
    COMPARISON_TYPE_EQUAL = "COMPARATOR_EQUALS"
    COMPARISON_TYPE_GREATER = "COMPARATOR_GREATER"
    COMPARISON_TYPE_LESS = "COMPARATOR_LESS"


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
                # TODO: this should be better integrated with some error from api calls
                # and only when we get a 401 or 403 response should we refresh the token
                # instead of before every call
                await self._gateway._ensure_valid_token()
                start = time.time()
                try:
                    return await original_attr(*args, **kwargs)
                except Exception as e:
                    _logger.error(f"API CALL: {name} failed with error: {e}")
                    _logger.debug(f"API CALL FAILED: {name} with args={args}, kwargs={kwargs}")
                    raise e
                finally:
                    duration = time.time() - start
                    _logger.info(f"API CALL: {name} took {duration:.2f} seconds")
                    _logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

            return async_wrapper

        # Wrap sync callables
        @functools.wraps(original_attr)
        def sync_wrapper(*args, **kwargs):
            start = time.time()
            try:
                return original_attr(*args, **kwargs)
            except Exception as e:
                _logger.error(f"API CALL: {name} failed with error: {e}")
                raise e
            finally:
                duration = time.time() - start
                _logger.info(f"API CALL: {name} took {duration:.2f} seconds")
                _logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

        return sync_wrapper

    # provides tab completion in python interpreter
    def __dir__(self):
        return self._instance.__dir__()


def _intercept(api_instance: T, gateway: "ApiGateway") -> T:
    # we ignore the type error here because
    # we want the return type to be the same as the original api instance to not break typing support
    return _Interceptor(api_instance, gateway)  # type: ignore[return-value]


class ApiGateway:
    def __init__(self, config: NovaConfig):
        self.config = config
        self._version = "v2"
        self._validating_token = False
        self._has_valid_token = False

        self._auth0 = None
        auth0_config = Auth0Config.from_env()
        if auth0_config.is_complete():
            self._auth0 = Auth0DeviceAuthorization(auth0_config=auth0_config)

        self._init_api_client()

    def _init_api_client(self):
        """Initialize or reinitialize the API client with current credentials"""
        api_client_config = api.Configuration(
            host=f"{self.config.host}/api/v2", access_token=self.config.access_token
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

        _logger.debug(f"NOVA API client initialized with user agent {self._api_client.user_agent}")

    async def close(self):
        await self._api_client.close()

    async def _ensure_valid_token(self):
        """Ensure we have a valid access token, requesting a new one if needed"""
        if not self._auth0 or self._validating_token or self._has_valid_token:
            return
        try:
            # Test token with a direct API call without interception
            self._validating_token = True
            async with api.ApiClient(self._api_client.configuration) as client:
                system_api = api.SystemApi(client)
                await system_api.get_system_version()
                self._has_valid_token = True
        except Exception as e:
            if "401" in str(e) or "403" in str(e):
                logger.info("Access token expired, starting device authorization flow")
                self._auth0.request_device_code()
                self._auth0.display_user_instructions()

                new_token = await self._auth0.poll_token_endpoint()
                self.config.access_token = new_token

                # Store the new token in .env file
                # TODO: should we really do that
                set_key("NOVA_ACCESS_TOKEN", new_token)

                # Update the existing API client configuration with the new token
                self._api_client.configuration.access_token = new_token
                self._api_client.configuration.username = None
                self._api_client.configuration.password = None

                # Reinitialize all API clients with the new configuration
                self._init_api_client()

                _logger.info("Successfully updated access token and reinitialized API clients")
        finally:
            self._validating_token = False


class NovaDevice(ConfigurablePeriphery, Device, ABC, is_abstract=True):
    class Configuration(ConfigurablePeriphery.Configuration):
        config: NovaConfig

    def __init__(self, configuration: Configuration, **kwargs):
        self._nova_api = ApiGateway(configuration.config)
        super().__init__(configuration, **kwargs)
