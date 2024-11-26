from urllib.parse import urlencode, urlparse
from loguru import logger
import wandelbots_api_client as wb


def use_nova_api(host) -> wb.ApiClient:
    base_url = f"http://{host}/api/v1"
    config = wb.Configuration(host=base_url, ssl_ca_cert=False)
    return wb.ApiClient(config)


# TODO: what is we read the instance from env varaibles?
class Instance:
    def __init__(
        self, host="http://api-gateway.wandelbots.svc.cluster.local:8080", user=None, password=None, access_token=None
    ):
        self._api_version = "v1"
        self.access_token = access_token
        self.user = user
        self.password = password
        self.host = self._parse_host(host)

    def _parse_host(self, host: str) -> str:
        """remove any trailing slashes and validate scheme"""
        _url = host.rstrip("/")
        parsed_url = urlparse(_url)

        if self.has_access_token() and self.has_basic_auth():
            raise ValueError("please choose either user and password or access token access")

        if _url.startswith("https"):
            if not self.has_auth():
                raise ValueError("Access token or user and password are required for https connections")
        elif _url.startswith("http"):
            if self.has_auth():
                raise ValueError("Access token and/or user and password are not required for http connections")
        elif parsed_url.hostname and parsed_url.hostname.endswith(".wandelbots.io"):
            _url = "https://" + _url
        else:  # assume http
            _url = "http://" + _url
        return _url

    @property
    def socket_uri(self):
        return self.host.replace("http", "ws").replace("https", "wss")

    def get_socket_uri_with_auth(self, additional_params: dict = None, url: str = None):
        if self.has_basic_auth():
            _uri = self.socket_uri.replace("wss://", f"wss://{self.user}:{self.password}@")
        else:
            _uri = self.socket_uri
        params = {}
        if self.has_access_token():
            params["token"] = self.access_token

        if additional_params:
            params.update(additional_params)

        query_string = urlencode(params)
        return f"{_uri}/api/{self._api_version}/{url}?{query_string}"

    def _connect(self):
        logger.info(f"Connecting to {self.host}")
        # do some connection stuff

    def has_auth(self):
        return self.has_basic_auth() or self.has_access_token()

    def has_basic_auth(self):
        return self.user is not None and self.password is not None

    def has_access_token(self):
        return self.access_token is not None
