# TODO parameter naming convention (controller -> controller_id)

from __future__ import annotations

import asyncio
import functools
import time
from abc import ABC
from enum import Enum
from typing import TypeVar
from urllib.parse import quote as original_quote

from nova import api
from nova.auth.auth_config import Auth0Config
from nova.auth.authorization import Auth0DeviceAuthorization
from nova.cell.robot_cell import ConfigurablePeriphery, Device
from nova.config import (  # add to the module for backward compatibility
    INTERNAL_CLUSTER_NOVA_API,  # noqa: F401
    NOVA_ACCESS_TOKEN,
    NOVA_API,
    NOVA_PASSWORD,
    NOVA_USERNAME,
)
from nova.core import logger
from nova.core.env_handler import set_key
from nova.version import version as pkg_version


def _custom_quote_for_ios(param, safe=""):
    """
    Custom quote function that preserves square brackets and hash characters for I/O names.
    This prevents double encoding of I/O names like "tool_out[0]" and KUKA names like "OUT#2".
    """
    return original_quote(param, safe="[]#")


T = TypeVar("T")


class ComparisonType(Enum):
    COMPARISON_TYPE_EQUAL = "COMPARATOR_EQUALS"
    COMPARISON_TYPE_GREATER = "COMPARATOR_GREATER"
    COMPARISON_TYPE_LESS = "COMPARATOR_LESS"


def intercept(api_instance: T, gateway: "ApiGateway") -> T:
    """Extend api interface classes to add logging and token validation"""

    class Interceptor:
        def __init__(self, instance: T):
            self._instance = instance

        def __getattr__(self, name):
            # Retrieve the original attribute
            original_attr = getattr(self._instance, name)

            # If it's not callable, return it as is
            if not callable(original_attr):
                return original_attr

            # Wrap async callables
            if asyncio.iscoroutinefunction(original_attr):

                @functools.wraps(original_attr)
                async def async_wrapper(*args, **kwargs):
                    await gateway._ensure_valid_token()
                    start = time.time()
                    try:
                        return await original_attr(*args, **kwargs)
                    except Exception as e:
                        logger.error(f"API CALL: {name} failed with error: {e}")
                        logger.debug(f"API CALL FAILED: {name} with args={args}, kwargs={kwargs}")
                        raise e
                    finally:
                        duration = time.time() - start
                        logger.info(f"API CALL: {name} took {duration:.2f} seconds")
                        logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

                return async_wrapper

            # Wrap sync callables
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
                    logger.info(f"API CALL: {name} took {duration:.2f} seconds")
                    logger.debug(f"API CALL: {name} with args={args}, kwargs={kwargs}")

            return sync_wrapper

    # we ignore the type error here because
    # we want the return type to be the same as the original api instance to not break typing support
    return Interceptor(api_instance)  # type: ignore[return-value]


class ApiGateway:
    _api_client: api.ApiClient

    def __init__(
        self,
        *,
        host: str | None = None,
        username: str | None = None,
        password: str | None = None,
        access_token: str | None = None,
        version: str = "v2",
        verify_ssl: bool = True,
        auth0_config: Auth0Config | None = None,
    ):
        host = host or NOVA_API
        access_token = access_token or NOVA_ACCESS_TOKEN
        username = username or NOVA_USERNAME
        password = password or NOVA_PASSWORD

        self._version = version
        self._verify_ssl = verify_ssl
        self._validating_token = False
        self._has_valid_token = False

        # Access token has more prio than username and password if both are provided at the same time, set username and
        # password to None
        if access_token is not None:
            username = None
            password = None

        self._auth0 = None
        auth0_config = auth0_config or Auth0Config.from_env()
        if auth0_config.is_complete():
            self._auth0 = Auth0DeviceAuthorization(auth0_config=auth0_config)

        self._host = self._host_with_prefix(host=host)
        self._access_token = access_token
        self._username = username
        self._password = password

        self._init_api_client()

    def _init_api_client(self):
        """Initialize or reinitialize the API client with current credentials"""

        # Apply monkey patch for ControllerIOsApi
        import wandelbots_api_client.api.controller_ios_api as ios_api_module

        original_quote_func = getattr(ios_api_module, "quote", None)

        if original_quote_func:
            # Store original function for potential restoration
            if not hasattr(self, "_original_quote_func"):
                self._original_quote_func = original_quote_func
            # Apply the monkey patch
            ios_api_module.quote = _custom_quote_for_ios

        stripped_host = self._host.rstrip("/")

        # init v1 api client
        api_client_config = api.Configuration(
            host=f"{stripped_host}/api/v2",
            username=self._username,
            password=self._password,
            access_token=self._access_token,
        )
        api_client_config.verify_ssl = self._verify_ssl
        self._api_client = api.ApiClient(configuration=api_client_config)
        self._api_client.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"

        # Use the intercept function to wrap each API client
        self.system_api = intercept(api.api.SystemApi(api_client=self._api_client), self)
        self.controller_api = intercept(api.api.ControllerApi(api_client=self._api_client), self)
        self.controller_ios_api = intercept(
            api.api.ControllerInputsOutputsApi(api_client=self._api_client), self
        )
        self.virtual_controller_api = intercept(
            api.api.VirtualControllerApi(api_client=self._api_client), self
        )
        self.virtual_controller_behavior_api = intercept(
            api.api.VirtualControllerBehaviorApi(api_client=self._api_client), self
        )
        # TODO migrate stuff in rerun bridge and then remove this
        self.virtual_robot_setup_api = self.virtual_controller_api
        self.motion_group_api = intercept(api.api.MotionGroupApi(api_client=self._api_client), self)
        self.motion_group_jogging_api = intercept(
            api.api.JoggingApi(api_client=self._api_client), self
        )
        self.store_collision_components_api = intercept(
            api.api.StoreCollisionComponentsApi(api_client=self._api_client), self
        )
        self.motion_group_models_api: api.api.MotionGroupModelsApi = intercept(
            api.api.MotionGroupModelsApi(api_client=self._api_client), self
        )
        self.store_collision_setups_api = intercept(
            api.api.StoreCollisionSetupsApi(api_client=self._api_client), self
        )
        self.trajectory_planning_api: api.api.TrajectoryPlanningApi = intercept(
            api.api.TrajectoryPlanningApi(api_client=self._api_client), self
        )
        self.trajectory_execution_api: api.api.TrajectoryExecutionApi = intercept(
            api.api.TrajectoryExecutionApi(api_client=self._api_client), self
        )
        self.trajectory_caching_api: api.api.TrajectoryCachingApi = intercept(
            api.api.TrajectoryCachingApi(api_client=self._api_client), self
        )
        self.controller_inputs_outputs_api = intercept(
            api.api.ControllerInputsOutputsApi(api_client=self._api_client), self
        )
        self.jogging_api = intercept(api.api.JoggingApi(api_client=self._api_client), self)
        self.store_object_api = intercept(api.api.StoreObjectApi(api_client=self._api_client), self)
        self.kinematics_api = intercept(api.api.KinematicsApi(api_client=self._api_client), self)

        logger.debug(f"NOVA API client initialized with user agent {self._api_client.user_agent}")

    async def close(self):
        # TODO: what is this ?
        # Restore the original quote function
        if hasattr(self, "_original_quote_func"):
            import wandelbots_api_client.api.controller_ios_api as ios_api_module

            ios_api_module.quote = self._original_quote_func
        await self._api_client.close()

    async def _ensure_valid_token(self):
        """Ensure we have a valid access token, requesting a new one if needed"""
        if not self._auth0 or self._validating_token or self._has_valid_token:
            return
        if self._username is not None and self._password is not None:
            return
        try:
            # Test token with a direct API call without interception
            self._validating_token = True
            async with api.ApiClient(self._api_client.configuration) as client:
                api = api.SystemApi(client)
                await api.get_system_version()
                self._has_valid_token = True
        except Exception as e:
            if "401" in str(e) or "403" in str(e):
                logger.info("Access token expired, starting device authorization flow")
                self._auth0.request_device_code()
                self._auth0.display_user_instructions()

                new_token = await self._auth0.poll_token_endpoint()
                self._access_token = new_token
                self._username = None
                self._password = None

                # Store the new token in .env file
                set_key("NOVA_ACCESS_TOKEN", new_token)

                # Update the existing API client configuration with the new token
                self._api_client.configuration.access_token = new_token
                self._api_client.configuration.username = None
                self._api_client.configuration.password = None

                # Reinitialize all API clients with the new configuration
                self._init_api_client()

                logger.info("Successfully updated access token and reinitialized API clients")
        finally:
            self._validating_token = False

    @staticmethod
    def _host_with_prefix(host: str) -> str:
        """
        The protocol prefix is required for the API client to work properly.
        This method adds the 'http://' prefix if it is missing.
        For all wandelbots.io virtual instances the prefix will 'https://'.
        """
        is_wabo_host = "wandelbots.io" in host
        if host.startswith("http") and not is_wabo_host:
            return host
        if host.startswith("http") and is_wabo_host:
            return host.replace("http://", "https://")
        if is_wabo_host:
            return f"https://{host}"
        return f"http://{host}"

    @property
    def host(self) -> str:
        return self._host

    @property
    def access_token(self) -> str | None:
        return self._access_token

    @property
    def username(self) -> str | None:
        return self._username

    @property
    def password(self) -> str | None:
        return self._password


class NovaDevice(ConfigurablePeriphery, Device, ABC, is_abstract=True):
    class Configuration(ConfigurablePeriphery.Configuration):
        nova_api: str
        nova_access_token: str | None = None
        nova_username: str | None = None
        nova_password: str | None = None

    _nova_api: ApiGateway

    def __init__(self, configuration: Configuration, **kwargs):
        self._nova_api = ApiGateway(
            host=configuration.nova_api,
            access_token=configuration.nova_access_token,
            username=configuration.nova_username,
            password=configuration.nova_password,
        )
        super().__init__(configuration, **kwargs)
