import asyncio
import functools
from typing import TypeVar

import wandelbots_api_client as wb
from decouple import config

import time
from loguru import logger

T = TypeVar("T")


# TODO: this is just a poc, the idea is to have a central place
#       to enhance the auto generated package without changing the type system
def enhance_with_logging(api_instance: T) -> T:
    class EnhancedApi(type(api_instance)):
        def __getattribute__(self, name):
            original_attr = api_instance.__getattribute__(name)
            if not callable(original_attr):
                return original_attr

            if asyncio.iscoroutinefunction(original_attr):
                # Handle async functions
                @functools.wraps(original_attr)
                async def timer(*args, **kwargs):
                    logger.info(f"Calling {name} with args: {args} and kwargs: {kwargs}")
                    start_time = time.time()
                    try:
                        result = await original_attr(*args, **kwargs)
                        return result
                    finally:
                        end_time = time.time()
                        logger.info(f"{name} took {end_time - start_time:.2f} seconds")

                return timer

            # Handle synchronous functions
            @functools.wraps(original_attr)
            def timer(*args, **kwargs):
                logger.debug(f"Calling {name} with args: {args} and kwargs: {kwargs}")
                start_time = time.time()
                try:
                    result = original_attr(*args, **kwargs)
                    return result
                finally:
                    end_time = time.time()
                    logger.debug(f"{name} took {end_time - start_time:.2f} seconds")

            return timer

    return EnhancedApi()


class ApiGateway:
    def __init__(
        self,
        *,
        host: str | None,
        username: str | None,
        password: str | None,
        access_token: str | None,
        version: str = "v1",
    ):
        if host is None:
            host = config("NOVA_HOST")

        if username is None:
            username = config("NOVA_USERNAME", default=None)

        if password is None:
            password = config("NOVA_PASSWORD", default=None)

        if access_token is None:
            access_token = config("NOVA_ACCESS", default=None)

        api_client_config = wb.Configuration(
            host=f"http://{host}/api/{version}",
            username=username,
            password=password,
            access_token=access_token,
            ssl_ca_cert=False,
        )
        self._api_client = wb.ApiClient(api_client_config)
        self.controller_api = enhance_with_logging(wb.ControllerApi(api_client=self._api_client))
        self.motion_group_api = enhance_with_logging(wb.MotionGroupApi(api_client=self._api_client))
        self.motion_api = enhance_with_logging(wb.MotionApi(api_client=self._api_client))
        self.motion_group_infos_api = enhance_with_logging(
            wb.MotionGroupInfosApi(api_client=self._api_client)
        )
