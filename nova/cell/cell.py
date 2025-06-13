import nova.api as api
from nova.cell.controllers import virtual_controller
from nova.cell.robot_cell import RobotCell
from nova.core.controller import Controller
from nova.core.exceptions import ControllerNotFound
from nova.core.gateway import ApiGateway

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

    async def add_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
        timeout: int = DEFAULT_ADD_CONTROLLER_TIMEOUT,
        wait_for_ready_timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT,
        position: str | None = None,
    ) -> Controller:
        return await self.add_controller(
            robot_controller=virtual_controller(
                name=name,
                type=controller_type,
                manufacturer=controller_manufacturer,
                position=position,
            ),
            add_timeout=timeout,
            wait_for_ready_timeout=wait_for_ready_timeout,
        )

    async def ensure_virtual_robot_controller(
        self,
        name: str,
        controller_type: api.models.VirtualControllerTypes,
        controller_manufacturer: api.models.Manufacturer,
        timeout: int = DEFAULT_ADD_CONTROLLER_TIMEOUT,
        wait_for_ready_timeout: int = DEFAULT_WAIT_FOR_READY_TIMEOUT,
    ) -> Controller:
        return await self.ensure_controller(
            robot_controller=virtual_controller(
                name=name, type=controller_type, manufacturer=controller_manufacturer
            ),
            add_timeout=timeout,
            wait_for_ready_timeout=wait_for_ready_timeout,
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
        await self._api_gateway.add_robot_controller(
            cell=self._cell_id, robot_controller=robot_controller, timeout=add_timeout
        )

        await self._api_gateway.wait_for_controller_ready(
            cell=self._cell_id, name=robot_controller.name, timeout=wait_for_ready_timeout
        )

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
        controller = await self._api_gateway.get_controller_instance(
            cell=self.cell_id, name=robot_controller.name
        )

        if controller:
            return self._create_controller(controller.controller)
        return await self.add_controller(
            robot_controller, add_timeout=add_timeout, wait_for_ready_timeout=wait_for_ready_timeout
        )

    async def ensure_virtual_tcp(
        self, tcp: api.models.RobotTcp, controller_name: str, motion_group_idx: int = 0
    ) -> api.models.RobotTcp:
        """
        Ensure that a virtual TCP with the expected configuration exists on the robot controller.
        If it doesn't exist, it will be created. If it exists but has different configuration,
        it will be updated by recreating it.

        Args:
            tcp (api.models.RobotTcp): The expected TCP configuration
            controller_name (str): The name of the controller
            motion_group_idx (int): The motion group index (default: 0)

        Returns:
            api.models.RobotTcp: The TCP configuration
        """
        controller = await self.controller(controller_name)

        async with controller[motion_group_idx] as motion_group:
            existing_tcps = await motion_group.tcps()

            existing_tcp = None
            for existing in existing_tcps:
                if existing.id == tcp.id:
                    existing_tcp = existing
                    break

            if existing_tcp:
                if self._tcp_configs_equal(existing_tcp, tcp):
                    return existing_tcp
                return existing_tcp

            await self._api_gateway.virtual_robot_setup_api.add_virtual_robot_tcp(
                cell=self._cell_id, controller=controller_name, id=motion_group_idx, robot_tcp=tcp
            )

            import asyncio

            await asyncio.sleep(1)

            return tcp

    def _tcp_configs_equal(self, tcp1: api.models.RobotTcp, tcp2: api.models.RobotTcp) -> bool:
        """Compare two TCP configurations for equality."""
        if tcp1.id != tcp2.id:
            return False

        if (
            tcp1.position.x != tcp2.position.x
            or tcp1.position.y != tcp2.position.y
            or tcp1.position.z != tcp2.position.z
        ):
            return False

        if tcp1.rotation.angles != tcp2.rotation.angles or tcp1.rotation.type != tcp2.rotation.type:
            return False

        return True

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
