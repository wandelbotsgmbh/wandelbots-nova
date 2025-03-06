from .client import OPCUA, OPCUAClientConfig
from .functions import opcua_read, opcua_write, opcua_call, wait_for_opcua_value

OPCUAClient = OPCUA


__all__ = [
    "OPCUAClient",
    "OPCUAClientConfig",
    "opcua_read",
    "opcua_write",
    "opcua_call",
    "wait_for_opcua_value",
]
