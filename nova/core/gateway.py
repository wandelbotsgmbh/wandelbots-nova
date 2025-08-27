from __future__ import annotations

import asyncio
import functools
import time
from abc import ABC
from enum import Enum
from typing import AsyncGenerator, TypeVar
from urllib.parse import quote as original_quote

import wandelbots_api_client as wb
from decouple import config

from nova.auth.auth_config import Auth0Config
from nova.auth.authorization import Auth0DeviceAuthorization
from nova.cell.robot_cell import ConfigurablePeriphery, Device
from nova.core import logger
from nova.core.env_handler import set_key
from nova.core.exceptions import LoadPlanFailed, PlanTrajectoryFailed
from nova.version import version as pkg_version


def _custom_quote_for_ios(param, safe=""):
    """
    Custom quote function that preserves square brackets and hash characters for I/O names.
    This prevents double encoding of I/O names like "tool_out[0]" and KUKA names like "OUT#2".
    """
    return original_quote(param, safe="[]#")


T = TypeVar("T")

INTERNAL_CLUSTER_NOVA_API = "http://api-gateway.wandelbots.svc.cluster.local:8080"


class ComparisonType(Enum):
    COMPARISON_TYPE_EQUAL = "COMPARISON_TYPE_EQUAL"
    COMPARISON_TYPE_GREATER = "COMPARISON_TYPE_GREATER"
    COMPARISON_TYPE_LESS = "COMPARISON_TYPE_LESS"


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
        self.motion_group_jogging_api = intercept(
            wb.MotionGroupJoggingApi(api_client=self._api_client), self
        )

        logger.debug(f"NOVA API client initialized with user agent {self._api_client.user_agent}")

    async def close(self):
        # Restore the original quote function
        if hasattr(self, "_original_quote_func"):
            import wandelbots_api_client.api.controller_ios_api as ios_api_module

            ios_api_module.quote = self._original_quote_func
        return await self._api_client.close()

    async def _ensure_valid_token(self):
        """Ensure we have a valid access token, requesting a new one if needed"""
        if not self._auth0 or self._validating_token or self._has_valid_token:
            return
        if self._username is not None and self._password is not None:
            return
        try:
            # Test token with a direct API call without interception
            self._validating_token = True
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

    async def stream_robot_controller_state(
        self, cell: str, controller_id: str, response_rate: int = 200
    ) -> AsyncGenerator[wb.models.RobotControllerState, None]:
        """
        Stream the robot controller state.
        """
        async for state in self.controller_api.stream_robot_controller_state(
            cell=cell, controller=controller_id, response_rate=response_rate
        ):
            yield state

    async def activate_motion_group(self, cell: str, motion_group_id: str):
        await self.motion_group_api.activate_motion_group(cell=cell, motion_group=motion_group_id)

    async def activate_all_motion_groups(self, cell: str, controller: str) -> list[str]:
        """
        Activate all motion groups for the given cell and controller.
        Returns the id of the activated motion groups.
        """
        activate_all_motion_groups_response = (
            await self.motion_group_api.activate_all_motion_groups(cell=cell, controller=controller)
        )
        motion_groups = activate_all_motion_groups_response.instances
        return [mg.motion_group for mg in motion_groups]

    async def list_controller_io_descriptions(
        self, cell: str, controller: str, ios: list[str]
    ) -> list[wb.models.IODescription]:
        if not ios:
            ios = []

        response = await self.controller_ios_api.list_io_descriptions(
            cell=cell, controller=controller, ios=ios
        )
        return response.io_descriptions

    async def read_controller_io(self, cell: str, controller: str, io: str) -> float | bool | int:
        response = await self.controller_ios_api.list_io_values(
            cell=cell, controller=controller, ios=[io]
        )

        found_io = response.io_values[0]
        if found_io.boolean_value is not None:
            return bool(found_io.boolean_value)
        if found_io.integer_value is not None:
            return int(found_io.integer_value)
        if found_io.floating_value is not None:
            return float(found_io.floating_value)
        raise ValueError(
            f"IO value for {io} is of an unexpected type. Expected bool, int or float."
        )

    async def write_controller_io(
        self, cell: str, controller: str, io: str, value: bool | int | float
    ):
        if isinstance(value, bool):
            io_value = wb.models.IOValue(io=io, boolean_value=value)
        elif isinstance(value, int):
            io_value = wb.models.IOValue(io=io, integer_value=str(value))
        elif isinstance(value, float):
            io_value = wb.models.IOValue(io=io, floating_value=value)
        else:
            raise ValueError(f"Invalid value type {type(value)}. Expected bool, int or float.")

        await self.controller_ios_api.set_output_values(
            cell=cell, controller=controller, io_value=[io_value]
        )

    async def wait_for_bool_io(self, cell: str, controller: str, io: str, value: bool):
        await self.controller_ios_api.wait_for_io_event(
            cell=cell,
            controller=controller,
            io=io,
            comparison_type=ComparisonType.COMPARISON_TYPE_EQUAL,
            boolean_value=value,
        )

    async def list_controllers(self, *, cell: str) -> list[wb.models.ControllerInstance]:
        response = await self.controller_api.list_controllers(cell=cell)
        return response.instances

    async def get_controller_instance(
        self, *, cell: str, name: str
    ) -> wb.models.ControllerInstance | None:
        controllers = await self.list_controllers(cell=cell)
        return next((c for c in controllers if c.controller == name), None)

    async def get_current_robot_controller_state(
        self, *, cell: str, controller_id: str
    ) -> wb.models.RobotControllerState:
        return await self.controller_api.get_current_robot_controller_state(
            cell=cell, controller=controller_id
        )

    async def add_robot_controller(
        self, cell: str, robot_controller: wb.models.RobotController, timeout: int = 25
    ):
        """
        Add a robot controller to the specified cell.
        Args:
            cell: The cell to add the controller to.
            robot_controller: The robot controller to add.
            timeout: The timeout in seconds for the operation.
        """
        await self.controller_api.add_robot_controller(
            cell=cell, robot_controller=robot_controller, completion_timeout=timeout
        )

    async def delete_robot_controller(
        self, *, cell: str, controller: str, completion_timeout: int = 25
    ) -> None:
        await self.controller_api.delete_robot_controller(
            cell=cell, controller=controller, completion_timeout=completion_timeout
        )

    async def wait_for_controller_ready(self, cell: str, name: str, timeout: int = 25) -> None:
        """
        Wait until the given controller has finished initializing or until timeout.
        Args:
            cell: The cell to check.
            name: The name of the controller.
            timeout: The timeout in seconds.
        """
        iteration = 0
        while iteration < timeout:
            controller = await self.get_controller_instance(cell=cell, name=name)
            if controller is None:
                logger.info(f"Controller not found: {cell}/{name}")
            elif controller.has_error:
                logger.error(f"Controller {cell}/{name} has error: {controller.error_details}")
            else:
                logger.info(f"Controller {cell}/{name} is ready")
                return

            await asyncio.sleep(1)
            iteration += 1

        raise TimeoutError(f"Timeout waiting for {cell}/{name} controller availability")

    async def plan_trajectory(
        self, cell: str, motion_group_id: str, request: wb.models.PlanTrajectoryRequest
    ) -> wb.models.JointTrajectory:
        """
        Plan a trajectory for the given motion group.
        """

        plan_trajectory_response = await self.motion_api.plan_trajectory(
            cell=cell, plan_trajectory_request=request
        )
        if isinstance(
            plan_trajectory_response.response.actual_instance,
            wb.models.PlanTrajectoryFailedResponse,
        ):
            # TODO: handle partially executable path
            raise PlanTrajectoryFailed(
                plan_trajectory_response.response.actual_instance, motion_group_id
            )
        return plan_trajectory_response.response.actual_instance

    async def load_planned_motion(
        self, cell: str, motion_group_id: str, joint_trajectory: wb.models.JointTrajectory, tcp: str
    ) -> wb.models.PlanSuccessfulResponse:
        load_plan_response = await self.motion_api.load_planned_motion(
            cell=cell,
            planned_motion=wb.models.PlannedMotion(
                motion_group=motion_group_id,
                times=joint_trajectory.times,
                joint_positions=joint_trajectory.joint_positions,
                locations=joint_trajectory.locations,
                tcp=tcp,
            ),
        )

        if (
            load_plan_response.plan_failed_on_trajectory_response is not None
            or load_plan_response.plan_failed_on_trajectory_response is not None
        ):
            raise LoadPlanFailed(load_plan_response)

        return load_plan_response.plan_successful_response

    def stream_move_to_trajectory_via_join_ptp(
        self,
        cell: str,
        motion_id: str,
        location_on_trajectory: int,
        joint_velocity_limits: wb.models.Joints | None = None,
    ) -> AsyncGenerator[wb.models.StreamMoveResponse, None]:
        return self.motion_api.stream_move_to_trajectory_via_joint_ptp(
            cell=cell,
            motion=motion_id,
            location_on_trajectory=location_on_trajectory,
            # limit_override_joint_velocity_limits_joints=joint_velocity_limits,
        )

    async def stop_motion(self, cell: str, motion_id: str):
        await self.motion_api.stop_execution(cell=cell, motion=motion_id)

    async def get_motion_group_state(
        self, cell: str, motion_group_id: str, tcp: str | None = None
    ) -> wb.models.MotionGroupStateResponse:
        return await self.motion_group_infos_api.get_current_motion_group_state(
            cell=cell, motion_group=motion_group_id, tcp=tcp
        )

    async def list_tcps(self, cell: str, motion_group_id: str) -> wb.models.ListTcpsResponse:
        return await self.motion_group_infos_api.list_tcps(cell=cell, motion_group=motion_group_id)

    async def get_active_tcp(self, cell: str, motion_group_id: str) -> wb.models.RobotTcp:
        return await self.motion_group_infos_api.get_active_tcp(
            cell=cell, motion_group=motion_group_id
        )

    async def get_optimizer_config(
        self, cell: str, motion_group_id: str, tcp: str
    ) -> wb.models.OptimizerSetup:
        return await self.motion_group_infos_api.get_optimizer_configuration(
            cell=cell, motion_group=motion_group_id, tcp=tcp
        )

    async def get_joint_number(self, cell: str, motion_group_id: str) -> int:
        spec = await self.motion_group_infos_api.get_motion_group_specification(
            cell=cell, motion_group=motion_group_id
        )
        return len(spec.mechanical_joint_limits)

    async def plan_collision_free_ptp(
        self, cell: str, motion_group_id: str, request: wb.models.PlanCollisionFreePTPRequest
    ):
        plan_result = await self.motion_api.plan_collision_free_ptp(
            cell=cell, plan_collision_free_ptp_request=request
        )

        if isinstance(plan_result.response.actual_instance, wb.models.PlanTrajectoryFailedResponse):
            raise PlanTrajectoryFailed(plan_result.response.actual_instance, motion_group_id)
        return plan_result.response.actual_instance


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
