import asyncio
import json

import nova.api as api
from nova.cell.robot_cell import RobotCell
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway
from nova.logging import logger
from nova.nats import NatsClient

# This is the default value we use to wait for add_controller API call to complete.
DEFAULT_ADD_CONTROLLER_TIMEOUT = 120

# This is the default value we use when we wait for a controller to be ready.
DEFAULT_WAIT_FOR_READY_TIMEOUT = 120


class Cell:
    """A representation of a robot cell, providing high-level operations on controllers."""

    def __init__(
        self, api_gateway: ApiGateway, cell_id: str, nats_client: NatsClient | None = None
    ):
        """
        Initializes a Cell instance.
        Args:
            api_gateway (ApiGateway): The underlying gateway for making API calls.
            cell_id (str): The unique identifier for the cell.
            nats_client (NatsClient | None): The NATS client for publishing events.
        """
        self._api_gateway = api_gateway
        self._cell_id = cell_id
        self._nats_client = nats_client

    @property
    def cell_id(self) -> str:
        """
        Returns unique identifier for this cell.
        Returns:
            str: The cell ID.
        """
        return self._cell_id

    @property
    def nats(self) -> NatsClient | None:
        """
        Returns the NATS client for this cell.
        Returns:
            NatsClient | None: The NATS client instance or None if not configured.
        """
        return self._nats_client

    def _create_controller(self, controller_id: str) -> Controller:
        return Controller(
            configuration=Controller.Configuration(
                cell_id=self._cell_id,
                controller_id=controller_id,
                id=controller_id,
                nova_api=self._api_gateway._host,
                nova_access_token=self._api_gateway._access_token,
                nova_username=self._api_gateway._username,
                nova_password=self._api_gateway._password,
            )
        )

    async def add_controller(
        self,
        robot_controller: api.models.RobotController,
        add_timeout_secs: int = DEFAULT_ADD_CONTROLLER_TIMEOUT,
        wait_for_ready_timeout_secs: int = DEFAULT_WAIT_FOR_READY_TIMEOUT,
    ) -> Controller:
        """
        Add a robot controller to the cell and wait for it to get ready.
        Args:
            robot_controller (api.models.RobotController): The robot controller to add. You can use helper functions from nova to create these configs easily,
                see :func:`nova.cell.abb_controller`, :func:`nova.cell.fanuc_controller`, :func:`nova.cell.kuka_controller`,
                :func:`nova.cell.universal_robots_controller`, :func:`nova.cell.virtual_controller`, :func:`nova.cell.yaskawa_controller`.
            add_timeout (int): The time to wait for the controller to be added (default: 25).
            wait_for_ready_timeout (int): The time to wait for the controller to be ready (default: 25).

        Returns:
            Controller: The added Controller object.
        """

        try:
            async with asyncio.TaskGroup() as tg:
                tg.create_task(
                    self._api_gateway.add_robot_controller(
                        cell=self._cell_id, robot_controller=robot_controller, timeout=None
                    )
                )
                tg.create_task(
                    self._wait_for_controller_ready(
                        cell=self._cell_id,
                        name=robot_controller.name,
                        timeout=wait_for_ready_timeout_secs,
                    )
                )
        except* asyncio.TimeoutError:
            logger.error(
                f"Timeout while adding controller {robot_controller.name} to cell {self._cell_id}"
            )
            raise TimeoutError(
                f"Timeout while adding controller {robot_controller.name} to cell {self._cell_id}"
            )

        return self._create_controller(robot_controller.name)

    async def ensure_controller(
        self,
        controller_config: api.models.RobotController,
        add_timeout: int = DEFAULT_ADD_CONTROLLER_TIMEOUT,
        wait_for_ready_timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT,
    ) -> Controller:
        """
        Ensure that a robot controller is added to the cell. If it already exists, it will be returned.
        If it doesn't exist, it will be added and waited for to be ready.
        Args:
            robot_controller (api.models.RobotController): The robot controller to add. You can use helper functions from nova to create these configs easily,
                see :func:`nova.abb_controller`, :func:`nova.fanuc_controller`, :func:`nova.kuka_controller`,
                :func:`nova.universal_robots_controller`, :func:`nova.virtual_controller`, :func:`nova.yaskawa_controller`.
            add_timeout (int): The time to wait for the controller to be added (default: 25).
            wait_for_ready_timeout (int): The time to wait for the controller to be ready (default: 25).

        Returns:
            Controller: The added Controller object.
        """
        # TODO this makes no sense if we already have the controller instance as in the robot_controller parameter
        controller = await self._api_gateway.get_controller_instance(
            cell=self.cell_id, name=controller_config.name
        )
        if controller:
            return self._create_controller(controller.name)

        return await self.add_controller(
            controller_config,
            add_timeout_secs=add_timeout,
            wait_for_ready_timeout_secs=wait_for_ready_timeout,
        )

    async def controllers(self) -> list[Controller]:
        """
        List all controllers for this cell.
        Returns:
            list[Controller]: A list of Controller objects associated with this cell.
        """
        controller_names = await self._api_gateway.controller_api.list_robot_controllers(
            cell=self._cell_id
        )
        return [self._create_controller(name) for name in controller_names]

    async def controller(self, name: str) -> Controller:
        """
        Retrieve a specific controller by name.
        Args:
            name (str): The name of the controller.
        Returns:
            Controller: The Controller object.
        Raises:
            ControllerNotFound: If no controller with the specified name exists.
        """
        controller_instance = await self._api_gateway.get_controller_instance(
            cell=self._cell_id, name=name
        )
        if not controller_instance:
            raise ControllerNotFound(controller=name)
        return self._create_controller(controller_instance.name)

    async def delete_robot_controller(self, name: str, timeout: int = 25):
        """
        Delete a robot controller from the cell.
        Args:
            name (str): The name of the controller to delete.
            timeout (int): The time to wait for the controller deletion to complete (default: 25).
        """
        await self._api_gateway.delete_robot_controller(
            cell=self._cell_id, controller=name, completion_timeout=timeout
        )

    async def get_robot_cell(self) -> RobotCell:
        """
        Return a RobotCell object containing all known controllers.
        Returns:
            RobotCell: A RobotCell initialized with the available controllers.
        """
        controllers = await self.controllers()
        return RobotCell(timer=None, **{controller.id: controller for controller in controllers})

    async def _wait_for_controller_ready(
        self, cell: str, name: str, timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT
    ) -> None:
        """
        Wait until the given controller has finished initializing or until timeout.
        Args:
            cell: The cell to check.
            name: The name of the controller.
            timeout: The timeout in seconds.
        """
        nc = self._nats_client
        nats_subject = f"nova.v2.cells.{cell}.status"
        sub = await nc.subscribe(subject=nats_subject)

        async def cell_status_consumer():
            async for msg in sub.messages:
                logger.debug(f"Received message on {msg.subject}: {msg.data}")
                data = json.loads(msg.data)
                # filter RobotControllers
                assert data[-1]["group"] == "RobotController" and data[-1]["service"] == name
                for status in data:
                    logger.debug(f"Controller status: {status}")
                    if status["service"] == name:
                        if status["status"]["code"] == "Running":
                            await (
                                sub.unsubscribe()
                            )  # TODO is this the right place to unsubscribe? is it sufficient?
                            return
                        else:
                            logger.info(f"Controller {name} status: {status['status']['code']}")

        try:
            await asyncio.wait_for(cell_status_consumer(), timeout=timeout)
        except asyncio.TimeoutError:
            logger.warning(f"Timeout waiting for {cell}/{name} controller to be ready")
            # Check if the controller exists, if not, log it
            raise TimeoutError(f"Timeout waiting for {cell}/{name} controller availability")
        finally:
            logger.debug("Cleaning up NATS subscription")
            pass
            # await sub.unsubscribe()
            # await nc.drain()  # Ensure we clean up the subscription and connection
