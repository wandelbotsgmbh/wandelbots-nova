import asyncio

import httpx
import pydantic

from nova.auth.auth_config import Auth0Config


class Auth0DeviceCodeInfo(pydantic.BaseModel):
    """
    Model to store device code information.

    Attributes:
        device_code (str): The device code.
        user_code (str): The user code.
        verification_uri (str): The verification URI.
        expires_in (int): The expiration time in seconds.
        interval (int): The interval time in seconds (default is 5).
    """

    device_code: str
    user_code: str
    verification_uri: str
    expires_in: int
    interval: int = pydantic.Field(default=5)


class Auth0TokenInfo(pydantic.BaseModel):
    """
    Model to store token information.

    Attributes:
        access_token (str): The access token.
        refresh_token (str, optional): The refresh token.
    """

    access_token: str
    refresh_token: str | None = None


class Auth0Parameters(pydantic.BaseModel):
    """
    Model to store Auth0 parameters.

    Attributes:
        auth0_domain (str): The Auth0 domain.
        auth0_client_id (str): The Auth0 client ID.
        auth0_audience (str): The Auth0 audience.
    """

    auth0_domain: str
    auth0_client_id: str
    auth0_audience: str


class Auth0DeviceAuthorization:
    """
    Class to handle Auth0 device authorization.

    Methods:
        __init__(auth0_domain: str, auth0_client_id: str, auth0_audience: str):
            Initializes the Auth0DeviceAuthorization instance with the given parameters.
        request_device_code():
            Requests a device code from Auth0.
        display_user_instructions():
            Displays instructions for the user to authenticate.
        poll_token_endpoint():
            Polls the token endpoint to obtain an access token.
        refresh_access_token(refresh_token: str):
            Refreshes the access token using the refresh token.
    """

    def __init__(self, auth0_config: Auth0Config | None = None):
        """
        Initialize with Auth0Config from env vars or passed config.

        Args:
            auth0_config: Optional Auth0Config object. If not provided,
                         will be created from environment variables.
        """
        try:
            self.params = auth0_config or Auth0Config.from_env()
            if not self.params.is_complete():
                raise ValueError("Auth0 configuration is incomplete")

            domain, client_id, audience = self.params.get_validated_config()
            self.auth0_domain = domain
            self.auth0_client_id = client_id
            self.auth0_audience = audience

        except ValueError as e:
            raise ValueError(f"Error: Auth0 configuration is invalid: {e}")

        self.headers = {"content-type": "application/x-www-form-urlencoded"}
        self.device_code_info: Auth0DeviceCodeInfo | None = None
        self.interval = 5
        self.attempts = 10

    def request_device_code(self):
        """
        Requests a device code from Auth0.

        Returns:
            Auth0DeviceCodeInfo: The device code information.

        Raises:
            Exception: If there is an error requesting the device code.
        """
        device_code_url = f"https://{self.auth0_domain}/oauth/device/code"
        data = {
            "client_id": self.auth0_client_id,
            "scope": "openid profile email",
            "audience": self.auth0_audience,
        }

        response = httpx.post(device_code_url, headers=self.headers, data=data)
        if response.status_code == 200:
            self.device_code_info = Auth0DeviceCodeInfo(**response.json())
            self.interval = self.device_code_info.interval
            return self.device_code_info
        raise Exception("Error requesting device code:", response.json())

    def get_device_code_info(self) -> Auth0DeviceCodeInfo | None:
        """
        Returns the device code information.

        Returns:
            Auth0DeviceCodeInfo | None: The device code information.
        """
        return self.device_code_info

    def display_user_instructions(self) -> None:
        """
        Displays instructions for the user to authenticate.

        Raises:
            Exception: If device code information is not available.
        """
        if self.device_code_info:
            verification_uri = f"{self.device_code_info.verification_uri}?user_code={self.device_code_info.user_code}"
            user_code = self.device_code_info.user_code
            print(
                f"Please visit {verification_uri} and validate the code {user_code} to authenticate."
            )
        else:
            raise Exception("Device code information is not available.")

    async def poll_token_endpoint(self):
        """
        Polls the token endpoint to obtain an access token.

        Returns:
            str: The access token.

        Raises:
            Exception: If there is an error polling the token endpoint.
        """
        if not self.device_code_info:
            raise Exception("Device code information is not available.")

        token_url = f"https://{self.auth0_domain}/oauth/token"
        token_data = {
            "grant_type": "urn:ietf:params:oauth:grant-type:device_code",
            "device_code": self.device_code_info.device_code,
            "client_id": self.auth0_client_id,
        }

        async with httpx.AsyncClient() as client:
            while self.attempts > 0:
                self.attempts = self.attempts - 1
                token_response = await client.post(token_url, headers=self.headers, data=token_data)
                if token_response.status_code == 200:
                    # If the response status is 200, it means the access token is successfully obtained.
                    token_info = Auth0TokenInfo(**token_response.json())
                    self.refresh_token = token_info.refresh_token
                    return token_info.access_token
                if token_response.status_code == 400:
                    # If the response status is 400, check the error type.
                    error = token_response.json().get("error")
                    if error == "authorization_pending":
                        # If the error is 'authorization_pending', it means the user has not yet authorized.
                        await asyncio.sleep(self.interval)
                    elif error == "slow_down":
                        # If the error is 'slow_down', it means the server requests to slow down polling.
                        self.interval += 5
                        await asyncio.sleep(self.interval)
                elif token_response.status_code == 403:
                    # If the response status is 403, it means the request is forbidden, wait and retry.
                    await asyncio.sleep(self.interval)
                else:
                    # For other status codes, raise an exception with the error details.
                    raise Exception("Error:", token_response.status_code, token_response.json())
            raise Exception("Error: It was not able to authenticate. Please try again.")

    def refresh_access_token(self, refresh_token: str):
        """
        Refreshes the access token using the refresh token.

        Args:
            refresh_token (str): The refresh token.

        Returns:
            str: The new access token.

        Raises:
            Exception: If there is an error refreshing the access token.
        """
        if not refresh_token:
            raise Exception("Refresh token is not available.")

        token_url = f"https://{self.auth0_domain}/oauth/token"
        token_data = {
            "grant_type": "refresh_token",
            "client_id": self.auth0_client_id,
            "refresh_token": refresh_token,
        }

        response = httpx.post(token_url, headers=self.headers, data=token_data)

        if response.status_code == 200:
            token_info = Auth0TokenInfo(**response.json())
            return token_info.access_token
        raise Exception("Error refreshing access token:", response.json())
