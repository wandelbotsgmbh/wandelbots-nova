# nova/experimental/api/operation_registry.py

from .list_io_values import LIST_IO_VALUES_OPERATION
from .operation_types import APIOperation
from .set_io_value import SET_IO_VALUE_OPERATION

# Dictionary: operation-name -> APIOperation
OPERATIONS: dict[str, APIOperation] = {
    LIST_IO_VALUES_OPERATION.name: LIST_IO_VALUES_OPERATION,
    SET_IO_VALUE_OPERATION.name: SET_IO_VALUE_OPERATION,
    # ... add other operations below
}
