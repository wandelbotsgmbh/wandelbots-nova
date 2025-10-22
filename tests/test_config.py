import os
from unittest.mock import patch

from nova.config import NovaConfig


def test_when_user_provides_nats_config_explicitly():
    config = NovaConfig(nats_client_config={"servers": "nats://custom-server:4222"})

    assert config.nats_client_config["servers"] == "nats://custom-server:4222"


def test_when_no_user_input_use_env_var():
    with patch.dict(os.environ, {"NATS_BROKER": "nats://env-broker:4222"}):
        config = NovaConfig()
        assert config.nats_client_config["servers"] == "nats://env-broker:4222"


def test_when_no_user_input_no_env_var_but_token_and_host_https():
    config = NovaConfig(host="https://api.example.com", access_token="test-token")
    assert config.nats_client_config["servers"] == "wss://test-token@api.example.com:443/api/nats"


def test_when_no_user_input_no_env_var_but_only_host_http():
    config = NovaConfig(host="http://localhost:8080")
    assert config.nats_client_config["servers"] == "ws://localhost:8080/api/nats"


def test_when_host_and_token_prioritize_env_var():
    with patch.dict(os.environ, {"NATS_BROKER": "nats://env-broker:4222"}):
        config = NovaConfig(host="https://api.example.com", access_token="test-token")
        assert config.nats_client_config["servers"] == "nats://env-broker:4222"


def test_when_everything_prioritize_user_input():
    with patch.dict(os.environ, {"NATS_BROKER": "nats://env-broker:4222"}):
        config = NovaConfig(
            host="https://api.example.com",
            access_token="test-token",
            nats_client_config={"servers": "nats://custom-server:4222"},
        )
        assert config.nats_client_config["servers"] == "nats://custom-server:4222"

def test_when_scheme_not_set_use_https():
    config = NovaConfig(host="example.com", access_token="token")
    assert config.nats_client_config["servers"] == "wss://token@example.com:443/api/nats"