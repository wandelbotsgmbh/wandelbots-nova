import asyncio
import json

from icecream import ic

from nova import api
from nova.cell.robot_cell import RobotCell
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway
from nova.logging import logger
from nova.nats import NatsClient

# This is the default value we use to wait for add_controller API call to complete.
DEFAULT_ADD_CONTROLLER_TIMEOUT_SECS = 120

# This is the default value we use when we wait for a controller to be ready.
DEFAULT_WAIT_FOR_READY_TIMEOUT_SECS = 120


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
        self._api_client = api_gateway
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
                nova_api=self._api_client._host,
                nova_access_token=self._api_client._access_token,
                nova_username=self._api_client._username,
                nova_password=self._api_client._password,
            )
        )

    def _create_controller_from_config(
        self, robot_controller: api.models.RobotController
    ) -> Controller:
        return self._create_controller(controller_id=robot_controller.name)

    async def _get_controller_instance(
        self, *, cell: str, name: str
    ) -> api.models.RobotController | None:
        controller_names = await self._api_client.controller_api.list_robot_controllers(
            cell=self._cell_id
        )
        if name in controller_names:
            return await self._api_client.controller_api.get_robot_controller(
                cell=self._cell_id, controller=name
            )

        return None

    async def add_controller(
        self,
        robot_controller: api.models.RobotController,
        add_timeout_secs: int = DEFAULT_ADD_CONTROLLER_TIMEOUT_SECS,
        wait_for_ready_timeout_secs: int = DEFAULT_WAIT_FOR_READY_TIMEOUT_SECS,
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
            add_task = asyncio.create_task(
                self._api_client.controller_api.add_robot_controller(
                    cell=self._cell_id,
                    robot_controller=robot_controller,
                    completion_timeout=add_timeout_secs,
                )
            )
            wait_ready_task = asyncio.create_task(
                self._wait_for_controller_ready(
                    cell=self._cell_id,
                    name=robot_controller.name,
                    timeout=wait_for_ready_timeout_secs,
                )
            )
            await asyncio.gather(add_task, wait_ready_task)
        except (asyncio.TimeoutError, TimeoutError):
            logger.error(
                f"Timeout while adding controller {robot_controller.name} to cell {self._cell_id}"
            )
            raise TimeoutError(
                f"Timeout while adding controller {robot_controller.name} to cell {self._cell_id}"
            )

        return self._create_controller_from_config(robot_controller)

    async def ensure_controller(
        self,
        controller_config: api.models.RobotController,
        add_timeout: int = DEFAULT_ADD_CONTROLLER_TIMEOUT_SECS,
        wait_for_ready_timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT_SECS,
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
        ic(controller_config)
        ic(self.cell_id)
        controller = await self._get_controller_instance(
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
        # TODO The API returns a list of controller names as of v2, should we really offer
        # the instance listing at all?
        """
        List all controllers for this cell.
        Returns:
            list[Controller]: A list of Controller objects associated with this cell.
        """
        controller_names = await self._api_client.controller_api.list_robot_controllers(
            cell=self._cell_id
        )
        # Create tasks to get all controller instances concurrently
        async with asyncio.TaskGroup() as tg:
            tasks = [
                tg.create_task(
                    self._api_client.controller_api.get_robot_controller(
                        cell=self._cell_id, controller=name
                    )
                )
                for name in controller_names
            ]

        # Filter out None results and return the list of controller instances
        return [
            self._create_controller(result.name) for result in [task.result() for task in tasks]
        ]

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
        controller_instance = await self._get_controller_instance(cell=self._cell_id, name=name)
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
        await self._api_client.controller_api.delete_robot_controller(
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
        self, cell: str, name: str, timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT_SECS
    ) -> None:
        """
        Wait until the given controller has finished initializing or until timeout.
        Args:
            cell: The cell to check.
            name: The name of the controller.
            timeout: The timeout in seconds.
        """
        nc = self._nats_client
        if nc is None:
            raise ValueError("NATS client is not connected")

        nats_subject = f"nova.v2.cells.{cell}.status"
        controller_ready_event = asyncio.Event()

        async def on_cell_status_message(msg):
            if controller_ready_event.is_set():
                # Skip processing if controller is already ready
                return
            logger.debug(f"Received message on {msg.subject}: {msg.data}")
            data = json.loads(msg.data)
            robot_controller_service_status = [
                d for d in data if d["group"] == "RobotController" and d["service"] == name
            ]
            assert len(robot_controller_service_status) == 1, "Multiple controllers with same name?"
            if robot_controller_service_status[0]["status"]["code"] == "Running":
                controller_ready_event.set()
            else:
                logger.info(
                    f"Controller {name} status: {robot_controller_service_status[0]['status']['code']}"
                )

        # TODO currently the NotsClient does not support unsubscribing from a subject, keeping the code
        # code like this for when it is supported.
        sub = await nc.subscribe(subject=nats_subject, on_message=on_cell_status_message)
        try:
            await asyncio.wait_for(controller_ready_event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            logger.warning(f"Timeout waiting for {cell}/{name} controller to be ready")
            await sub.unsubscribe()
            raise TimeoutError(f"Timeout waiting for {cell}/{name} controller availability")
        finally:
            logger.debug("Cleaning up NATS subscription")
            pass
            # await sub.unsubscribe()
            # await nc.drain()  # Ensure we clean up the subscription and connection
        await asyncio.sleep(1)  # Give some time for any final messages to be processed
