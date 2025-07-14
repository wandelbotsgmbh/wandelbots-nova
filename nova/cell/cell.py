import nova.api as api
import asyncio
from nova.cell.robot_cell import RobotCell
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway
from nova.core.logging import logger

# This is the default value we use to wait for add_controller API call to complete.
DEFAULT_ADD_CONTROLLER_TIMEOUT = 120

# This is the default value we use when we wait for a controller to be ready.
DEFAULT_WAIT_FOR_READY_TIMEOUT = 120


class Cell:
    """A representation of a robot cell, providing high-level operations on controllers."""

    def __init__(self, api_gateway: ApiGateway, cell_id: str):
        """
        Initializes a Cell instance.
        Args:
            api_gateway (ApiGateway): The underlying gateway for making API calls.
            cell_id (str): The unique identifier for the cell.
        """
        self._api_gateway = api_gateway
        self._cell_id = cell_id

    @property
    def cell_id(self) -> str:
        """
        Returns unique identifier for this cell.
        Returns:
            str: The cell ID.
        """
        return self._cell_id

    def _create_controller(self, controller_id: str) -> Controller:
        return Controller(
            configuration=Controller.Configuration(
                nova_api=self._api_gateway.host,
                nova_access_token=self._api_gateway.access_token,
                nova_username=self._api_gateway.username,
                nova_password=self._api_gateway.password,
                cell_id=self._cell_id,
                controller_id=controller_id,
                id=controller_id,
            )
        )

    async def add_controller(
        self,
        robot_controller: api.models.RobotController,
        add_timeout: int = DEFAULT_ADD_CONTROLLER_TIMEOUT,
        wait_for_ready_timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT,
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
        logger.info(f"🔄 Adding robot controller '{robot_controller.name}' to cell '{self._cell_id}'...")

        # Start the add_robot_controller operation
        add_task = asyncio.create_task(
            self._api_gateway.add_robot_controller(
                cell=self._cell_id, robot_controller=robot_controller, timeout=add_timeout
            )
        )

        # Show progress during the add operation
        start_time = asyncio.get_event_loop().time()
        while not add_task.done():
            elapsed = int(asyncio.get_event_loop().time() - start_time)
            logger.info(f"⏳ Adding controller '{robot_controller.name}'... ({elapsed}s elapsed)")
            await asyncio.sleep(5)  # Update every 5 seconds

        # Wait for the add operation to complete
        await add_task
        logger.info(f"✅ Successfully added controller '{robot_controller.name}' to cell")

        # Now wait for the controller to be ready
        logger.info(f"🔄 Waiting for controller '{robot_controller.name}' to be ready...")

        # Start the wait_for_controller_ready operation
        wait_task = asyncio.create_task(
            self._api_gateway.wait_for_controller_ready(
                cell=self._cell_id, name=robot_controller.name, timeout=wait_for_ready_timeout
            )
        )

        # Show progress during the wait operation
        start_time = asyncio.get_event_loop().time()
        while not wait_task.done():
            elapsed = int(asyncio.get_event_loop().time() - start_time)
            logger.info(f"⏳ Waiting for controller '{robot_controller.name}' to be ready... ({elapsed}s elapsed)")
            await asyncio.sleep(5)  # Update every 5 seconds

        # Wait for the wait operation to complete
        await wait_task
        logger.info(f"✅ Controller '{robot_controller.name}' is now ready")

        return self._create_controller(robot_controller.name)

    async def ensure_controller(
        self,
        robot_controller: api.models.RobotController,
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
        logger.info(f"🔍 Checking if controller '{robot_controller.name}' already exists...")
        controller = await self._api_gateway.get_controller_instance(
            cell=self.cell_id, name=robot_controller.name
        )

        if controller:
            logger.info(f"✅ Controller '{robot_controller.name}' already exists, using existing controller")
            return self._create_controller(controller.controller)

        logger.info(f"❌ Controller '{robot_controller.name}' not found, will add new controller")
        return await self.add_controller(
            robot_controller, add_timeout=add_timeout, wait_for_ready_timeout=wait_for_ready_timeout
        )

    async def controllers(self) -> list[Controller]:
        """
        List all controllers for this cell.
        Returns:
            list[Controller]: A list of Controller objects associated with this cell.
        """
        instances = await self._api_gateway.list_controllers(cell=self._cell_id)
        return [self._create_controller(ci.controller) for ci in instances]

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
        return self._create_controller(controller_instance.controller)

    async def delete_robot_controller(self, name: str, timeout: int = 25):
        """
        Delete a robot controller from the cell.
        Args:
            name (str): The name of the controller to delete.
            timeout (int): The time to wait for the controller deletion to complete (default: 25).
        """
        logger.info(f"🗑️ Deleting robot controller '{name}' from cell '{self._cell_id}'...")

        # Start the delete operation
        delete_task = asyncio.create_task(
            self._api_gateway.delete_robot_controller(
                cell=self._cell_id, controller=name, completion_timeout=timeout
            )
        )

        # Show progress during the delete operation
        start_time = asyncio.get_event_loop().time()
        while not delete_task.done():
            elapsed = int(asyncio.get_event_loop().time() - start_time)
            logger.info(f"⏳ Deleting controller '{name}'... ({elapsed}s elapsed)")
            await asyncio.sleep(5)  # Update every 5 seconds

        # Wait for the delete operation to complete
        await delete_task
        logger.info(f"✅ Successfully deleted controller '{name}' from cell")

    async def get_robot_cell(self) -> RobotCell:
        """
        Return a RobotCell object containing all known controllers.
        Returns:
            RobotCell: A RobotCell initialized with the available controllers.
        """
        controllers = await self.controllers()
        return RobotCell(timer=None, **{controller.id: controller for controller in controllers})
