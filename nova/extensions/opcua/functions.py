from typing import Any

from .client import ClientConfig, OPCUAClient, SubscriptionConfig


async def opcua_write(url: str, node_id: str, value: Any, options: dict | None = None):
    """Write a value to the opcua node

    Node ids should be based on opcua standard.
    More information about how opcua node id string notation works can be found here:
    https://documentation.unified-automation.com/uasdkhp/1.4.1/html/_l2_ua_node_ids.html

    Args:
        url: the url of the opcua server
        node_id: id of the node to write the value
        value: the value to write
        options: configuration for the opcua client
            {
                # the timeout for each communication happening between opcua client and the server
                request_timeout_seconds: int,

                security_config: {
                    # the storage service path for the client certificate
                    client_certificate_path: str,

                    # the storage service path for the client private key
                    client_private_key_path: str,

                    # the security policy to use, e.g. Basic256Sha256
                    security_policy: str,

                    # the message security mode to use, e.g. SignAndEncrypt
                    message_security_mode: str
                }
            }
    """
    options = options or {}
    client_config = ClientConfig(**options)

    async with OPCUAClient(url, client_config) as opc:
        await opc.write_node(node_id, value)


async def opcua_read(url: str, node_id: str, options: dict | None = None) -> Any:
    """Reads the value of a opcua node and returns the result

    Node ids should be based on opcua standard.
    More information about how opcua node id string notation works can be found here:
    https://documentation.unified-automation.com/uasdkhp/1.4.1/html/_l2_ua_node_ids.html

    Args:
        url: the url of the opcua server
        node_id: id of the node to read the value of
        options: configuration for the opcua client
            {
                # the timeout for each communication happening between opcua client and the server
                request_timeout_seconds: int,
                security_config: {
                    # the storage service path for the client certificate
                    client_certificate_path: str,

                    # the storage service path for the client private key
                    client_private_key_path: str,

                    # the security policy to use, e.g. Basic256Sha256
                    security_policy: str,

                    # the message security mode to use, e.g. SignAndEncrypt
                    message_security_mode: str
                }
            }


    Returns:
        the value of the node

    """
    options = options or {}
    client_config = ClientConfig(**options)

    async with OPCUAClient(url, client_config) as opc:
        return await opc.read_node(node_id)


async def opcua_call(url: str, object_id: str, function_id: str, *args) -> Any:
    """executes the opcua function and returns the result

    Node ids should be based on opcua standard.
    More information about how opcua node id string notation works can be found here:
    https://documentation.unified-automation.com/uasdkhp/1.4.1/html/_l2_ua_node_ids.html

    Args:
        url: url of the opcua server
        object_id: node id of the object the function belongs to
        function_id : node id of the function
        *args: the arguments to the function,
               these parameters will be provided to OPCUA server while calling the function you specified
               the last parameter can be a wandelscript record to configure the opcua client.
                {
                    # the timeout for each communication happening between opcua client and the server
                     request_timeout_seconds: int,
                     security_config: {
                        # the storage service path for the client certificate
                        client_certificate_path: str,

                        # the storage service path for the client private key
                        client_private_key_path: str,

                        # the security policy to use, e.g. Basic256Sha256
                        security_policy: str,

                        # the message security mode to use, e.g. SignAndEncrypt
                        message_security_mode: str
                    }
                }


    Returns:
        the value returned by the opcua function
    """
    # there is no args
    if not args:
        opcua_function_args, options = (), {}
    # the last arg is a record
    elif isinstance(args[-1], dict):
        opcua_function_args, options = args[:-1], args[-1]
    # there are args but the last one is not a record
    else:
        opcua_function_args, options = args, {}

    client_config = ClientConfig(**options)
    async with OPCUAClient(url, client_config) as opc:
        return await opc.call_node(object_id, function_id, *opcua_function_args)


async def wait_for_opcua_value(url: str, node_id: str, value: Any, config: dict | None = None):
    """watches the opcua node with the given key until it matches the given value

    Node ids should be based on opcua standard.
    More information about how opcua node id string notation works can be found here:
    https://documentation.unified-automation.com/uasdkhp/1.4.1/html/_l2_ua_node_ids.html

    Args:
        url: url of the opcua server
        node_id: id of the node to watch the value of
        value: value that the node should have
        config: configuration for the subscription
    """
    config = config or {}
    subscription_config = SubscriptionConfig(**config)
    options = ClientConfig(
        request_timeout_seconds=subscription_config.request_timeout_seconds,
        security_config=subscription_config.security_config,
    )

    async with OPCUAClient(url, options) as opc:

        def condition(node_value: Any):
            return node_value == value

        await opc.watch_node_until_condition(node_id, condition, config=subscription_config)
