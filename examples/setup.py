"""
Setup utilities for Nova examples.

This module provides functions for setting up bus IOs and other
prerequisites for Nova programs.
"""

import wandelbots_api_client.v2_pydantic as wb
from loguru import logger


async def setup_bus_ios(bus_io_api: wb.BUSInputsOutputsApi, cell_id: str, io_configs: list[dict]) -> None:
    """Setup bus IO variables for the cell.

    Args:
        api_client: Wandelbots V2 API client
        cell_id: ID of the cell to configure
        io_configs: List of IO configuration dictionaries with keys:
            - name: IO variable name (str)
            - type: IO type, either "bool" or "int" (str)
            - initial_value: Initial value for the IO (bool or int)

    Example:
        io_configs = [
            {"name": "safety", "type": "bool", "initial_value": False},
            {"name": "counter", "type": "int", "initial_value": 0},
        ]
    """
    for config in io_configs:
        io_name = config["name"]
        io_type = config["type"]
        initial_value = config["initial_value"]

        # Create profinet IO configuration
        if io_type == "bool":
            profinet_type = wb.ProfinetIOTypeEnum.PROFINET_IO_TYPE_BOOL
        elif io_type == "int":
            profinet_type = wb.ProfinetIOTypeEnum.PROFINET_IO_TYPE_INT
        else:
            raise ValueError(f"Unsupported IO type: {io_type}")

        profinet_io_data = wb.ProfinetIOData(
            description=io_name,
            type=profinet_type,
            direction=wb.ProfinetIODirection.PROFINET_IO_DIRECTION_OUTPUT,
            byte_address=800,
            bit_address=None,
        )

        # Add the profinet IO
        try:
            await bus_io_api.add_profinet_io(
                cell_id, io=profinet_io_data.description, profinet_io_data=profinet_io_data
            )
            logger.info(f"Added bus IO: {io_name} ({io_type})")
        except Exception as e:
            logger.warning(f"Could not add bus IO {io_name} (may already exist): {e}")

    # Set initial values
    io_values = []
    for config in io_configs:
        io_name = config["name"]
        io_type = config["type"]
        initial_value = config["initial_value"]

        if io_type == "bool":
            io_value = wb.IOBooleanValue(io=io_name, value=initial_value)
        elif io_type == "int":
            io_value = wb.IOIntegerValue(io=io_name, value=str(initial_value))

        io_values.append(wb.IOValue(root=io_value))
    await bus_io_api.set_bus_io_values(cell_id, io_values)
    logger.info(f"Initialized {len(io_values)} bus IO values")
