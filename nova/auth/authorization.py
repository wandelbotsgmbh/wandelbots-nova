import time
import httpx
from pydantic import BaseModel, Field, ValidationError


class DeviceCodeInfo(BaseModel):
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
    interval: int = Field(default=5)


class TokenInfo(BaseModel):
    """
    Model to store token information.

    Attributes:
        access_token (str): The access token.
        refresh_token (str, optional): The refresh token.
    """

    access_token: str
    refresh_token: str | None = None


class Auth0Parameters(BaseModel):
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

    def __init__(self, auth0_domain: str, auth0_client_id: str, auth0_audience: str):
        """
        Initializes the Auth0DeviceAuthorization instance with the given parameters.

        Args:
            auth0_domain (str): The Auth0 domain.
            auth0_client_id (str): The Auth0 client ID.
            auth0_audience (str): The Auth0 audience.

        Raises:
            ValueError: If the parameters are not set correctly.
        """
        try:
            self.params = Auth0Parameters(
                auth0_domain=auth0_domain,
                auth0_client_id=auth0_client_id,
                auth0_audience=auth0_audience,
            )
        except ValidationError as e:
            raise ValueError(f"Error: The following parameters are not set correctly: {e}")

        self.auth0_domain = auth0_domain
        self.auth0_client_id = auth0_client_id
        self.auth0_audience = auth0_audience
        self.headers = {"content-type": "application/x-www-form-urlencoded"}
        self.device_code_info: DeviceCodeInfo | None = None
        self.interval = 5

    def request_device_code(self):
        """
        Requests a device code from Auth0.

        Returns:
            DeviceCodeInfo: The device code information.

        Raises:
            Exception: If there is an error requesting the device code.
        """
        device_code_url = f"https://{self.auth0_domain}/oauth/device/code"
        data = {
            "client_id": self.auth0_client_id,
            "scope": "openid profile email offline_access",
            "audience": self.auth0_audience,
        }

        response = httpx.post(device_code_url, headers=self.headers, data=data)
        if response.status_code == 200:
            self.device_code_info = DeviceCodeInfo(**response.json())
            self.interval = self.device_code_info.interval
            return self.device_code_info
        else:
            raise Exception("Error requesting device code:", response.json())

    def display_user_instructions(self):
        """
        Displays instructions for the user to authenticate.

        Raises:
            Exception: If device code information is not available.
        """
        if self.device_code_info:
            verification_uri = self.device_code_info.verification_uri
            user_code = self.device_code_info.user_code
            print(
                f"Please visit {verification_uri} and enter the code {user_code} to authenticate."
            )
        else:
            raise Exception("Device code information is not available.")

    def poll_token_endpoint(self):
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

        while True:
            token_response = httpx.post(token_url, headers=self.headers, data=token_data)
            if token_response.status_code == 200:
                # If the response status is 200, it means the access token is successfully obtained.
                token_info = TokenInfo(**token_response.json())
                self.refresh_token = token_info.refresh_token
                print("Access Token received:", token_info.access_token)
                print("Refresh Token received:", self.refresh_token)
                return token_info.access_token
            elif token_response.status_code == 400:
                # If the response status is 400, check the error type.
                error = token_response.json().get("error")
                if error == "authorization_pending":
                    # If the error is 'authorization_pending', it means the user has not yet authorized.
                    print("Waiting for user confirmation...")
                    time.sleep(self.interval)
                elif error == "slow_down":
                    # If the error is 'slow_down', it means the server requests to slow down polling.
                    print("Server requests to slow down polling. Increasing wait time.")
                    self.interval += 5
                    time.sleep(self.interval)
            elif token_response.status_code == 403:
                # If the response status is 403, it means the request is forbidden, wait and retry.
                print("Waiting...")
                time.sleep(self.interval)
            else:
                # For other status codes, raise an exception with the error details.
                raise Exception("Error:", token_response.status_code, token_response.json())

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
            token_info = TokenInfo(**response.json())
            print("New Access Token received:", token_info.access_token)
            return token_info.access_token
        else:
            raise Exception("Error refreshing access token:", response.json())
