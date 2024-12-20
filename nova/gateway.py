import asyncio
import functools
import time
from typing import TypeVar

from loguru import logger
import wandelbots_api_client as wb
from decouple import config

T = TypeVar("T")


def intercept(api_instance: T) -> T:
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
                    logger.info(f"Calling {name} with args={args}, kwargs={kwargs}")
                    start = time.time()
                    try:
                        return await original_attr(*args, **kwargs)
                    finally:
                        duration = time.time() - start
                        logger.info(f"{name} took {duration:.2f} seconds")

                return async_wrapper

            # Wrap sync callables
            @functools.wraps(original_attr)
            def sync_wrapper(*args, **kwargs):
                logger.debug(f"Calling {name} with args={args}, kwargs={kwargs}")
                start = time.time()
                try:
                    return original_attr(*args, **kwargs)
                finally:
                    duration = time.time() - start
                    logger.debug(f"{name} took {duration:.2f} seconds")

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
    ):
        if host is None:
            host = config("NOVA_HOST")

        if username is None:
            username = config("NOVA_USERNAME", default=None)

        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        if access_token is None:
            access_token = config("NOVA_ACCESS_TOKEN", default=None)

        api_client_config = wb.Configuration(
            host=f"http://{host}/api/{version}",
            username=username,
            password=password,
            access_token=access_token,
            ssl_ca_cert=False,
        )

        self._api_client = wb.ApiClient(api_client_config)

        # Use the intercept function to wrap each API client
        self.controller_api = intercept(wb.ControllerApi(api_client=self._api_client))
        self.motion_group_api = intercept(wb.MotionGroupApi(api_client=self._api_client))
        self.motion_api = intercept(wb.MotionApi(api_client=self._api_client))
        self.motion_group_infos_api = intercept(wb.MotionGroupInfosApi(api_client=self._api_client))

    async def close(self):
        await self._api_client.close()
