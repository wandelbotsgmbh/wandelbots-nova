import asyncio
import functools
import time
from typing import TypeVar

import wandelbots_api_client as wb
from decouple import config
from loguru import logger

T = TypeVar("T")

INTERNAL_CLUSTER_NOVA_API = "http://api-gateway.wandelbots.svc.cluster.local:8080"


def intercept(api_instance: T):
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
    ):
        if host is None:
            host = config("NOVA_API", default=INTERNAL_CLUSTER_NOVA_API)

        if username is None:
            username = config("NOVA_USERNAME", default=None)

        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        if access_token is None:
            access_token = config("NOVA_ACCESS_TOKEN", default=None)

        # Access token has more prio than username and password if both are provided at the same time, set username and
        # password to None
        if access_token is not None:
            username = None
            password = None

        stripped_host = host.rstrip("/")
        api_client_config = wb.Configuration(
            host=f"{stripped_host}/api/{version}",
            username=username,
            password=password,
            access_token=access_token,
        )
        api_client_config.verify_ssl = verify_ssl

        self._api_client = wb.ApiClient(configuration=api_client_config)
        self._host = host

        # Use the intercept function to wrap each API client
        self.controller_api = intercept(wb.ControllerApi(api_client=self._api_client))
        self.motion_group_api = intercept(wb.MotionGroupApi(api_client=self._api_client))
        self.motion_api = intercept(wb.MotionApi(api_client=self._api_client))
        self.motion_group_infos_api = intercept(wb.MotionGroupInfosApi(api_client=self._api_client))

    async def close(self):
        await self._api_client.close()
