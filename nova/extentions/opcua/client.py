import asyncio
import os.path
import tempfile
from collections.abc import Callable
from datetime import datetime
from typing import Any

import asyncua
import httpx
import pydantic
from asyncua import ua
from asyncua.common import Node
from asyncua.common.subscription import DataChangeNotif, DataChangeNotificationHandlerAsync
from asyncua.crypto import security_policies
from asyncua.ua import CreateSubscriptionParameters, DataValue, Variant, VariantType
from decouple import config

from nova import logger


class DataChangeSubscription(DataChangeNotificationHandlerAsync):
    """
    This class extends the DataChangeNotificationHandlerAsync class from the asyncua library.
    It is used to create a subscription that listens for data change notifications on a node.
    It supports a condition function that is used to determine when the subscription should be completed.
    """

    def __init__(self, condition: Callable[[Any], bool], print_received_messages=False):
        super().__init__()
        self._flag = asyncio.Event()
        self._condition = condition
        self._print_received_messages = print_received_messages

    async def datachange_notification(self, node: Node, val, data: DataChangeNotif) -> None:
        if self._print_received_messages:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"{current_time}: Received data change notification for node {node}, {val}")

        if self._condition(val):
            self._flag.set()

    def __await__(self):
        return self._flag.wait().__await__()


class SecurityConfig(pydantic.BaseModel):
    security_policy: str
    message_security_mode: str
    client_certificate_path: str
    client_private_key_path: str


class ClientConfig(pydantic.BaseModel):
    # this is the default value of the underlying library
    request_timeout_seconds: int = 4
    security_config: SecurityConfig | None = None


class SubscriptionConfig(pydantic.BaseModel):
    requested_publishing_interval: float = 100
    requested_lifetime_count: int = 10000
    max_notifications_per_publish: int = 1000
    priority: int = 0
    queue_size: int = 1
    sampling_interval: float = 0.0
    print_received_messages: bool = False
    request_timeout_seconds: int = 4
    security_config: SecurityConfig | None = None

    @classmethod
    def from_dict(cls, config_dict: dict):
        return cls(**config_dict)


async def fetch_certificate(certificate_path: str, private_key_path: str) -> tuple[bytes, bytes]:
    cell = config("K8S_NAMESPACE", default="cell")

    cert_url = f"http://api-gateway:8080/api/v1/cells/{cell}/store/objects/{certificate_path}"
    private_key_url = (
        f"http://api-gateway:8080/api/v1/cells/{cell}/store/objects/{private_key_path}"
    )

    async with httpx.AsyncClient() as client:
        response = await client.get(cert_url)
        response.raise_for_status()
        cert = response.content

        response = await client.get(private_key_url)
        response.raise_for_status()
        private_key = response.content

    return private_key, cert


async def certificate_files(cert_path: str, private_key_path: str) -> tuple[str, str]:
    private_key, cert = await fetch_certificate(cert_path, private_key_path)
    cert_suffix = os.path.splitext(cert_path)[1]
    private_key_suffix = os.path.splitext(private_key_path)[1]

    with tempfile.NamedTemporaryFile(delete=False, mode="wb", suffix=cert_suffix) as temp_file:
        temp_file.write(cert)
        cert_path = temp_file.name
    with tempfile.NamedTemporaryFile(
        delete=False, mode="wb", suffix=private_key_suffix
    ) as temp_file:
        temp_file.write(private_key)
        private_key_path = temp_file.name

    return private_key_path, cert_path


class OPCUAClient:
    """A wrapper around asyncua client"""

    def __init__(self, url: str, options: ClientConfig = ClientConfig()):
        self._options = options
        self._client = asyncua.Client(url=url, timeout=options.request_timeout_seconds)

    async def _extend_client_with_certificate(self):
        if not self._options.security_config:
            raise Exception("Security configuration is missing")

        # Taken from self._client.set_security_string
        policy_class = getattr(
            security_policies, f"SecurityPolicy{self._options.security_config.security_policy}"
        )
        mode = getattr(ua.MessageSecurityMode, self._options.security_config.message_security_mode)

        self._temp_private_key_file, self._temp_cert_file = await certificate_files(
            self._options.security_config.client_certificate_path,
            self._options.security_config.client_private_key_path,
        )
        await self._client.set_security(
            policy_class, self._temp_cert_file, self._temp_private_key_file, None, None, mode
        )

    def _delete_certificate_files(self):
        files = []
        if hasattr(self, "_temp_cert_file") and os.path.exists(self._temp_cert_file):
            files.append(self._temp_cert_file)
        if hasattr(self, "_temp_private_key_file") and os.path.exists(self._temp_private_key_file):
            files.append(self._temp_private_key_file)

        for file_path in files:
            try:
                os.remove(file_path)
            except Exception as cleanup_err:
                logger.error(f"Error cleaning up temporary file {file_path}: {cleanup_err}")

    async def __aenter__(self):
        try:
            if self._options.security_config:
                await self._extend_client_with_certificate()

            await self._client.connect()
        except:
            self._delete_certificate_files()
            raise
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self._delete_certificate_files()
        await self._client.disconnect()

    async def read_node(self, key: str) -> Any:
        """
        Reads the value of a opcua node and returns the result
        Args:
            key: opc ua node id

        Returns: the value of the node

        """
        node = await self.get_node(key)
        val = await node.read_value()
        return val

    async def write_node(self, key: str, val: Any):
        """
        Writes the value to a opcua node
        Args:
            key: opc ua node id
            val: the value to write
        """
        node = await self.get_node(key)
        data_type_variant = await node.read_data_type_as_variant_type()
        data_value = DataValue(Variant(val, data_type_variant))
        await node.write_value(data_value)

    async def call_node(self, parent_key: str, function_key: str, *args):
        """
        Calls a method on a opcua node
        Args:
            parent_key: the parent node id which contains the method
            function_key: the method node id
            *args: arguments to the opc ua method

        Returns:
            the response returned by the opcua method

        """
        node = await self.get_node(parent_key)
        method_node = await self.get_node(function_key)

        mapped_args = await self._map_arguments(method_node, *args)
        result = await node.call_method(method_node.nodeid, *mapped_args)
        return result

    async def watch_node_until_condition(
        self, key: str, condition: Callable[[Any], bool], config: SubscriptionConfig
    ):
        """
        Creates a subscription that listens for data change notifications on a node until a condition is met
        Args:
            key: the node to listed
            condition: the condition that should be met to stop the subscription
                       every time asyncua library receives a data change notification
                       it will check if the condition is met
            config: configuration parameters for the subscription.
                    https://reference.opcfoundation.org/Core/Part4/v104/docs/5.13.1
        """
        data_change_sub = DataChangeSubscription(
            condition, print_received_messages=config.print_received_messages
        )

        create_sub_params = CreateSubscriptionParameters(
            RequestedPublishingInterval=config.requested_publishing_interval,
            RequestedLifetimeCount=config.requested_lifetime_count,
            RequestedMaxKeepAliveCount=self._client.get_keepalive_count(
                config.requested_publishing_interval
            ),
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
            create_sub_params.RequestedPublishingInterval
            == revised_sub_params.RequestedPublishingInterval
            and create_sub_params.RequestedLifetimeCount
            == revised_sub_params.RequestedLifetimeCount
            and create_sub_params.RequestedMaxKeepAliveCount
            == revised_sub_params.RequestedMaxKeepAliveCount
        ):
            return

        # this print statement is here to inform Wandelscript user about the differences between the subscription parameters
        logger.warning(
            f"Revised values returned differ from subscription values: {revised_sub_params}"
        )
        print(f"Revised values returned differ from subscription values: {revised_sub_params}")
