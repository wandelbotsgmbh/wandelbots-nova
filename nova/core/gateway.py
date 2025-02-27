from __future__ import annotations

import asyncio
import functools
import time
from abc import ABC
from typing import TypeVar

import wandelbots_api_client as wb
from decouple import config

from nova.auth.auth_config import Auth0Config
from nova.auth.authorization import Auth0DeviceAuthorization
from nova.core import logger
from nova.core.env_handler import set_key
from nova.core.robot_cell import ConfigurablePeriphery, Device
from nova.version import version as pkg_version

T = TypeVar("T")

INTERNAL_CLUSTER_NOVA_API = "http://api-gateway.wandelbots.svc.cluster.local:8080"


def intercept(api_instance: T, gateway: "ApiGateway"):
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

    return Interceptor(api_instance)


class ApiGateway:
    def __init__(
        self,
        *,
        host: str | None = None,
        username: str | None = None,
        password: str | None = None,
        access_token: str | None = None,
        version: str = "v1",
        verify_ssl: bool = True,
        auth0_config: Auth0Config | None = None,
    ):
        if host is None:
            host = config("NOVA_API", default=INTERNAL_CLUSTER_NOVA_API)

        if username is None:
            username = config("NOVA_USERNAME", default=None)

        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        if access_token is None:
            access_token = config("NOVA_ACCESS_TOKEN", default=None)

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
        stripped_host = self._host.rstrip("/")
        api_client_config = wb.Configuration(
            host=f"{stripped_host}/api/{version}",
            username=username,
            password=password,
            access_token=access_token,
        )
        api_client_config.verify_ssl = verify_ssl

        self._access_token = access_token
        self._username = username
        self._password = password

        self._init_api_client()

    def _init_api_client(self):
        """Initialize or reinitialize the API client with current credentials"""
        stripped_host = self._host.rstrip("/")
        api_client_config = wb.Configuration(
            host=f"{stripped_host}/api/{self._version}",
            username=self._username,
            password=self._password,
            access_token=self._access_token,
        )
        api_client_config.verify_ssl = self._verify_ssl

        self._api_client = wb.ApiClient(configuration=api_client_config)
        self._api_client.user_agent = f"Wandelbots-Nova-Python-SDK/{pkg_version}"

        # Use the intercept function to wrap each API client
        self.system_api = intercept(wb.SystemApi(api_client=self._api_client), self)
        self.controller_api = intercept(wb.ControllerApi(api_client=self._api_client), self)
        self.motion_group_api = intercept(wb.MotionGroupApi(api_client=self._api_client), self)
        self.motion_api = intercept(wb.MotionApi(api_client=self._api_client), self)
        self.motion_group_infos_api = intercept(
            wb.MotionGroupInfosApi(api_client=self._api_client), self
        )
        self.motion_group_kinematic_api = intercept(
            wb.MotionGroupKinematicApi(api_client=self._api_client), self
        )

        self.store_collision_components_api = intercept(
            wb.StoreCollisionComponentsApi(api_client=self._api_client), self
        )

        self.store_collision_scenes_api = intercept(
            wb.StoreCollisionScenesApi(api_client=self._api_client), self
        )

        self.virtual_robot_api = intercept(wb.VirtualRobotApi(api_client=self._api_client), self)
        self.virtual_robot_behavior_api = intercept(
            wb.VirtualRobotBehaviorApi(api_client=self._api_client), self
        )

        self.virtual_robot_mode_api = intercept(
            wb.VirtualRobotModeApi(api_client=self._api_client), self
        )
        self.virtual_robot_setup_api = intercept(
            wb.VirtualRobotSetupApi(api_client=self._api_client), self
        )

        self.controller_ios_api = intercept(wb.ControllerIOsApi(api_client=self._api_client), self)
        logger.debug(f"NOVA API client initialized with user agent {self._api_client.user_agent}")

    async def close(self):
        return await self._api_client.close()

    async def _ensure_valid_token(self):
        """Ensure we have a valid access token, requesting a new one if needed"""
        if not self._auth0 or self._validating_token or self._has_valid_token:
            return

        try:
            self._validating_token = True
            # Test token with a direct API call without interception
            async with wb.ApiClient(self._api_client.configuration) as client:
                api = wb.SystemApi(client)
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
