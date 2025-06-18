from unittest.mock import AsyncMock, patch

import httpx
import pytest

from wandelscript.builtins.fetch import _create_response_object, fetch


@pytest.fixture
def mock_response():
    """Create a mock httpx.Response for testing."""
    response = AsyncMock(spec=httpx.Response)
    response.status_code = 200
    response.reason_phrase = "OK"
    response.headers = {"Content-Type": "application/json"}
    response.url = "https://example.com"
    response.history = []
    response.json.return_value = {"test": "data"}
    response.text = "test response text"
    response.content = b"test response content"
    return response


@pytest.mark.asyncio
async def test_fetch_basic_get():
    """Test basic GET request functionality."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 200
        mock_response.reason_phrase = "OK"
        mock_response.headers = {"Content-Type": "application/json"}
        mock_response.url = "https://example.com"
        mock_response.history = []

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch("https://example.com")

        assert result["status"] == 200
        assert result["ok"] is True
        assert result["statusText"] == "OK"
        assert result["url"] == "https://example.com"
        assert result["redirected"] is False


@pytest.mark.asyncio
async def test_fetch_post_with_json_body():
    """Test POST request with JSON body."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 201
        mock_response.reason_phrase = "Created"
        mock_response.headers = {"Content-Type": "application/json"}
        mock_response.url = "https://example.com"
        mock_response.history = []

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch(
            "https://example.com", {"method": "POST", "body": {"name": "test", "value": 123}}
        )

        assert result["status"] == 201
        assert result["ok"] is True

        mock_client.return_value.__aenter__.return_value.request.assert_called_once()
        call_args = mock_client.return_value.__aenter__.return_value.request.call_args
        assert call_args[0][0] == "POST"  # method
        assert call_args[0][1] == "https://example.com"  # url
        assert "json" in call_args[1]  # json body should be used


@pytest.mark.asyncio
async def test_fetch_form_data_body():
    """Test POST request with form data body."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 200
        mock_response.headers = {"Content-Type": "application/json"}
        mock_response.url = "https://example.com"
        mock_response.history = []

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch(
            "https://example.com",
            {
                "method": "POST",
                "body": {"username": "test", "password": "secret"},
                "headers": {"Content-Type": "application/x-www-form-urlencoded"},
            },
        )

        assert result["status"] == 200

        call_args = mock_client.return_value.__aenter__.return_value.request.call_args
        assert "data" in call_args[1]


@pytest.mark.asyncio
async def test_fetch_text_body():
    """Test POST request with plain text body."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 200
        mock_response.headers = {"Content-Type": "text/plain"}
        mock_response.url = "https://example.com"
        mock_response.history = []

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch("https://example.com", {"method": "POST", "body": "Hello, World!"})

        assert result["status"] == 200

        call_args = mock_client.return_value.__aenter__.return_value.request.call_args
        assert "content" in call_args[1]


@pytest.mark.asyncio
async def test_fetch_options_validation():
    """Test validation of Fetch API options."""
    with pytest.raises(ValueError, match="Unsupported method"):
        await fetch("https://example.com", {"method": "INVALID"})

    with pytest.raises(ValueError, match="Unsupported mode"):
        await fetch("https://example.com", {"mode": "invalid"})

    with pytest.raises(ValueError, match="Unsupported credentials"):
        await fetch("https://example.com", {"credentials": "invalid"})

    with pytest.raises(ValueError, match="Unsupported cache"):
        await fetch("https://example.com", {"cache": "invalid"})

    with pytest.raises(ValueError, match="Unsupported redirect"):
        await fetch("https://example.com", {"redirect": "invalid"})


@pytest.mark.asyncio
async def test_fetch_redirect_handling():
    """Test redirect handling options."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 200
        mock_response.headers = {"Content-Type": "application/json"}
        mock_response.url = "https://example.com"
        mock_response.history = [AsyncMock()]  # Simulate redirect

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch("https://example.com", {"redirect": "follow"})
        assert result["redirected"] is True

        mock_client.assert_called_with(follow_redirects=True)


@pytest.mark.asyncio
async def test_fetch_http_error_status():
    """Test that HTTP error statuses return Response objects, not exceptions."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_response = AsyncMock(spec=httpx.Response)
        mock_response.status_code = 404
        mock_response.reason_phrase = "Not Found"
        mock_response.headers = {"Content-Type": "text/html"}
        mock_response.url = "https://example.com"
        mock_response.history = []

        mock_client.return_value.__aenter__.return_value.request.return_value = mock_response

        result = await fetch("https://example.com")

        assert result["status"] == 404
        assert result["ok"] is False
        assert result["statusText"] == "Not Found"


@pytest.mark.asyncio
async def test_fetch_network_error():
    """Test that network errors raise exceptions."""
    with patch("httpx.AsyncClient") as mock_client:
        mock_client.return_value.__aenter__.return_value.request.side_effect = httpx.RequestError(
            "Network error"
        )

        with pytest.raises(httpx.RequestError):
            await fetch("https://example.com")


def test_response_object_properties(mock_response):
    """Test Response object properties."""
    response_obj = _create_response_object(mock_response)

    assert response_obj["status"] == 200
    assert response_obj["statusText"] == "OK"
    assert response_obj["ok"] is True
    assert response_obj["headers"] == {"Content-Type": "application/json"}
    assert response_obj["url"] == "https://example.com"
    assert response_obj["redirected"] is False
    assert response_obj["type"] == "basic"
    assert callable(response_obj["bodyUsed"])
    assert callable(response_obj["json"])
    assert callable(response_obj["text"])
    assert callable(response_obj["blob"])
    assert callable(response_obj["arrayBuffer"])
    assert callable(response_obj["formData"])
    assert callable(response_obj["clone"])


def test_response_json_method(mock_response):
    """Test Response json() method."""
    response_obj = _create_response_object(mock_response)

    assert response_obj["bodyUsed"]() is False
    json_data = response_obj["json"]()
    assert json_data == {"test": "data"}
    assert response_obj["bodyUsed"]() is True


def test_response_text_method(mock_response):
    """Test Response text() method."""
    response_obj = _create_response_object(mock_response)

    assert response_obj["bodyUsed"]() is False
    text_data = response_obj["text"]()
    assert text_data == "test response text"
    assert response_obj["bodyUsed"]() is True


def test_response_blob_method(mock_response):
    """Test Response blob() method."""
    response_obj = _create_response_object(mock_response)

    assert response_obj["bodyUsed"]() is False
    blob_data = response_obj["blob"]()
    assert blob_data == b"test response content"
    assert response_obj["bodyUsed"]() is True


def test_response_array_buffer_method(mock_response):
    """Test Response arrayBuffer() method (alias for blob)."""
    response_obj = _create_response_object(mock_response)

    buffer_data = response_obj["arrayBuffer"]()
    assert buffer_data == b"test response content"


def test_response_body_consumption_tracking(mock_response):
    """Test that body consumption is properly tracked."""
    response_obj = _create_response_object(mock_response)

    assert response_obj["bodyUsed"]() is False

    response_obj["json"]()
    assert response_obj["bodyUsed"]() is True

    with pytest.raises(RuntimeError, match="Body has already been consumed"):
        response_obj["text"]()


def test_response_clone_method(mock_response):
    """Test Response clone() method."""
    response_obj = _create_response_object(mock_response)
    cloned_obj = response_obj["clone"]()

    assert cloned_obj["status"] == response_obj["status"]
    assert cloned_obj["ok"] == response_obj["ok"]
    assert cloned_obj["url"] == response_obj["url"]

    assert cloned_obj is not response_obj

    response_obj["json"]()
    assert response_obj["bodyUsed"]() is True
    assert cloned_obj["bodyUsed"]() is False


def test_response_form_data_method():
    """Test Response formData() method."""
    mock_response = AsyncMock(spec=httpx.Response)
    mock_response.status_code = 200
    mock_response.headers = {"Content-Type": "application/x-www-form-urlencoded"}
    mock_response.text = "key1=value1&key2=value2&key3=value3a&key3=value3b"

    response_obj = _create_response_object(mock_response)
    form_data = response_obj["formData"]()

    expected = {
        "key1": "value1",
        "key2": "value2",
        "key3": ["value3a", "value3b"],  # Multiple values become list
    }
    assert form_data == expected
