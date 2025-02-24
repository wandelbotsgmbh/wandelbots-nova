import asyncio
from collections.abc import Callable
from datetime import datetime
from typing import Any

import asyncua
import pydantic
from asyncua.common.subscription import DataChangeNotificationHandlerAsync
from asyncua.ua import CreateSubscriptionParameters, DataValue, Variant, VariantType


class DataChangeSubscription(DataChangeNotificationHandlerAsync):
    def __init__(self, condition: Callable[[Any], bool], print_received_messages=False):
        super().__init__()
        self._flag = asyncio.Event()
        self._condition = condition
        self._print_received_messages = print_received_messages

    async def datachange_notification(self, node, val, data) -> None:
        if self._print_received_messages:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"{current_time}: Received data change notification for node {node}, {val}")

        if self._condition(val):
            self._flag.set()

    def __await__(self):
        return self._flag.wait().__await__()


class OPCUAClientConfig(pydantic.BaseModel):
    # this is the default value of the underlying library
    request_timeout_seconds: int = 4


class SubscriptionConfig(pydantic.BaseModel):
    requested_publishing_interval: float = 100
    requested_lifetime_count: int = 10000
    max_notifications_per_publish: int = 1000
    priority: int = 0
    queue_size: int = 1
    sampling_interval: float = 0.0
    print_received_messages: bool = False
    request_timeout_seconds: int = 4

    @classmethod
    def from_dict(cls, config_dict: dict):
        return cls(**config_dict)


class OPCUA:
    """A wrapper around asyncua client"""

    def __init__(self, url=str, options: OPCUAClientConfig = OPCUAClientConfig()):
        self._client = asyncua.Client(url=url, timeout=options.request_timeout_seconds)

    async def __aenter__(self):
        await self._client.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self._client.disconnect()

    async def read_node(self, key: str) -> Any:
        node = await self.get_node(key)
        val = await node.read_value()
        return val

    async def write_node(self, key: str, val: Any):
        node = await self.get_node(key)
        data_type_variant = await node.read_data_type_as_variant_type()
        data_value = DataValue(Variant(val, data_type_variant))
        await node.write_value(data_value)

    async def call_node(self, parent_key: str, function_key: str, *args):
        node = await self.get_node(parent_key)
        method_node = await self.get_node(function_key)

        mapped_args = await self._map_arguments(method_node, *args)
        result = await node.call_method(method_node.nodeid, *mapped_args)
        return result

    async def watch_node_until_condition(self, key: str, condition: Callable[[Any], bool], config: SubscriptionConfig):
        data_change_sub = DataChangeSubscription(condition, print_received_messages=config.print_received_messages)

        create_sub_params = CreateSubscriptionParameters(
            RequestedPublishingInterval=config.requested_publishing_interval,
            RequestedLifetimeCount=config.requested_lifetime_count,
            RequestedMaxKeepAliveCount=self._client.get_keepalive_count(config.requested_publishing_interval),
            MaxNotificationsPerPublish=config.max_notifications_per_publish,
            PublishingEnabled=True,
            Priority=config.priority,
        )
        sub = await self._client.create_subscription(create_sub_params, data_change_sub)
        self._compare_subscription_parameters(create_sub_params, sub.parameters)

        node = self._client.get_node(key)
        await sub.subscribe_data_change(
            nodes=node, queuesize=config.queue_size, sampling_interval=config.sampling_interval
        )

        await data_change_sub
        await sub.delete()

    async def _map_arguments(self, function_node, *args):
        input_arguments_node = await function_node.get_child("0:InputArguments")
        required_arguments = await input_arguments_node.get_value()
        if len(args) != len(required_arguments):
            raise Exception(  # pylint: disable=broad-exception-raised
                f"you gave: {len(args)} number of argument but the opcua function expects: {len(required_arguments)}"
            )

        mapped_input_argument_values = []
        for i, argument in enumerate(required_arguments):
            data_type_identifier = argument.DataType.Identifier
            mapped_input_argument_values.append(Variant(args[i], VariantType(data_type_identifier)))
        return mapped_input_argument_values

    async def get_node(self, key: str) -> asyncua.Node:
        return self._client.get_node(key)

    def _compare_subscription_parameters(self, create_sub_params, revised_sub_params):
        if (
            create_sub_params.RequestedPublishingInterval == revised_sub_params.RequestedPublishingInterval
            and create_sub_params.RequestedLifetimeCount == revised_sub_params.RequestedLifetimeCount
            and create_sub_params.RequestedMaxKeepAliveCount == revised_sub_params.RequestedMaxKeepAliveCount
        ):
            return

        # this print statement is here to inform Wandelscript user about the differences between the subscription parameters
        print(f"Revised values returned differ from subscription values: {revised_sub_params}")


from pyjectory import datatypes as dts


async def opcua_write(url: str, node_id: str, value: Any, options: dts.Record | None = None):
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
                request_timeout_seconds: int
            }
    """
    options = options or dts.Record()
    options = OPCUAClientConfig(**options.to_dict())

    async with OPCUA(url, options) as opc:
        await opc.write_node(node_id, value)


async def opcua_read(url: str, node_id: str, options: dts.Record | None = None) -> Any:
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
                request_timeout_seconds: int
            }


    Returns:
        the value of the node

    """
    options = options or dts.Record()
    options = OPCUAClientConfig(**options.to_dict())

    async with OPCUA(url, options) as opc:
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
                     request_timeout_seconds: int
                }


    Returns:
        the value returned by the opcua function
    """
    # there is no args
    if not args:
        opcua_function_args, options = (), dts.Record()
    # the last arg is a record
    elif isinstance(args[-1], dts.Record):
        opcua_function_args, options = args[:-1], args[-1]
    # there are args but the last one is not a record
    else:
        opcua_function_args, options = args, dts.Record()

    options = OPCUAClientConfig(**options.to_dict())
    async with OPCUA(url, options) as opc:
        return await opc.call_node(object_id, function_id, *opcua_function_args)


async def wait_for_opcua_value(url: str, node_id: str, value: Any, config: dts.Record | None = None):
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
    config = config or dts.Record()
    subscription_config = SubscriptionConfig(**config.to_dict())
    options = OPCUAClientConfig(request_timeout=subscription_config.request_timeout_seconds)

    async with OPCUA(url, options) as opc:

        def condition(node_value: Any):
            return node_value == value

        await opc.watch_node_until_condition(node_id, condition, config=subscription_config)
