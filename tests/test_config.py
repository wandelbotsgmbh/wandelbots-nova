import nova.config as config_module
from nova.config import NovaConfig


def test_when_user_provides_nats_config_explicitly():
    config = NovaConfig(nats_client_config={"servers": "nats://custom-server:4222"})

    assert config.nats_client_config["servers"] == "nats://custom-server:4222"


def test_when_no_user_input_use_env_var(monkeypatch):
    monkeypatch.setattr(config_module, "NATS_BROKER", "nats://env-broker:4222")
    config = NovaConfig()
    assert config.nats_client_config["servers"] == "nats://env-broker:4222"


def test_when_no_user_input_no_env_var_but_token_and_host_https():
    config = NovaConfig(host="https://api.example.com", access_token="test-token")
    assert config.nats_client_config["servers"] == "wss://test-token@api.example.com:443/api/nats"


def test_when_no_user_input_no_env_var_but_only_host_http():
    config = NovaConfig(host="http://localhost:8080")
    assert config.nats_client_config["servers"] == "ws://localhost:8080/api/nats"


def test_when_host_and_token_prioritize_env_var(monkeypatch):
    monkeypatch.setattr(config_module, "NATS_BROKER", "nats://env-broker:4222")
    config = NovaConfig(host="https://api.example.com", access_token="test-token")
    assert config.nats_client_config["servers"] == "nats://env-broker:4222"


def test_when_everything_prioritize_user_input(monkeypatch):
    monkeypatch.setattr(config_module, "NATS_BROKER", "nats://env-broker:4222")
    config = NovaConfig(
        host="https://api.example.com",
        access_token="test-token",
        nats_client_config={"servers": "nats://custom-server:4222"},
    )
    assert config.nats_client_config["servers"] == "nats://custom-server:4222"


def test_when_host_has_trailing_slash_it_is_normalized():
    config = NovaConfig(host="https://api.example.com/")
    assert config.host == "https://api.example.com"


def test_host_is_trimmed():
    config = NovaConfig(host=" https://api.example.com/   ")
    assert config.host == "https://api.example.com"