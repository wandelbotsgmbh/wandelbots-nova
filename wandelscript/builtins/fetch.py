from typing import Any
from urllib.parse import parse_qs

import httpx

from wandelscript.datatypes import as_builtin_type
from wandelscript.metamodel import register_builtin_func


@register_builtin_func()
async def fetch(url: str, options: dict | None = None) -> dict:
    """Fetch data from a URL with JavaScript Fetch API compatibility.

    Args:
        url: The URL to fetch data from.
        options: Additional options for the fetch operation.
            {
                # The HTTP method to use (GET, POST, PUT, DELETE, etc.). Default is GET.
                method: str,
                # The body of the request. Can be dict (JSON), str (text), bytes, or FormData/URLSearchParams as dict.
                body: Any,
                # Additional headers to include in the request. Default is None.
                headers: dict,
                mode: str,
                credentials: str,
                cache: str,
                redirect: str,
                referrer: str,
                referrerPolicy: str,
                integrity: str,
                keepalive: bool,
                signal: Any,
            }

    Raises:
        ValueError: If the method is not supported or options are invalid.
        httpx.RequestError: For network-related errors.

    Returns:
        A Response object as a dict with JavaScript Fetch API compatibility:
        {
            status: int,           # HTTP status code
            statusText: str,       # HTTP status message
            ok: bool,             # True if status is 200-299
            headers: dict,        # Response headers
            url: str,             # Final URL after redirects
            redirected: bool,     # True if redirected
            type: str,            # Response type ("basic", "cors", etc.)
            bodyUsed: bool,       # True if body has been consumed
            json: callable,       # Function to get JSON data
            text: callable,       # Function to get text data
            blob: callable,       # Function to get binary data
            arrayBuffer: callable, # Function to get binary data (alias for blob)
            formData: callable,   # Function to get form data
            clone: callable,      # Function to clone the response
        }

    """
    options = options or {}

    method = options.get("method", "GET").upper()
    body = options.get("body")
    headers = options.get("headers", {})
    mode = options.get("mode", "cors")
    credentials = options.get("credentials", "same-origin")
    cache = options.get("cache", "default")
    redirect = options.get("redirect", "follow")
    referrer = options.get("referrer", "client")  # noqa: F841
    referrer_policy = options.get("referrerPolicy", "strict-origin-when-cross-origin")  # noqa: F841
    integrity = options.get("integrity")  # noqa: F841
    keepalive = options.get("keepalive", False)
    signal = options.get("signal")  # noqa: F841

    valid_methods = {"GET", "POST", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"}
    if method not in valid_methods:
        raise ValueError(f"Unsupported method: {method}")

    valid_modes = {"cors", "no-cors", "same-origin", "navigate"}
    if mode not in valid_modes:
        raise ValueError(f"Unsupported mode: {mode}")

    valid_credentials = {"omit", "same-origin", "include"}
    if credentials not in valid_credentials:
        raise ValueError(f"Unsupported credentials: {credentials}")

    valid_cache = {"default", "no-store", "reload", "no-cache", "force-cache", "only-if-cached"}
    if cache not in valid_cache:
        raise ValueError(f"Unsupported cache: {cache}")

    valid_redirect = {"follow", "error", "manual"}
    if redirect not in valid_redirect:
        raise ValueError(f"Unsupported redirect: {redirect}")

    if headers is None:
        headers = {}
    elif not isinstance(headers, dict):
        headers = dict(headers) if hasattr(headers, "items") else {}

    try:
        client_kwargs: dict[str, Any] = {}
        
        if redirect == "error":
            client_kwargs["follow_redirects"] = False
        elif redirect == "manual":
            client_kwargs["follow_redirects"] = False
        else:  # "follow"
            client_kwargs["follow_redirects"] = True
        
        if keepalive:
            client_kwargs["timeout"] = None
        
        request_kwargs: dict[str, Any] = {"headers": headers}

        # Handle different body types and set appropriate content-type
        if body is not None:
            if isinstance(body, dict):
                has_files = any(hasattr(v, "read") for v in body.values() if v is not None)
                if has_files:
                    request_kwargs["files"] = body
                else:
                    if all(
                        isinstance(v, (str, int, float, bool)) or v is None for v in body.values()
                    ):
                        request_kwargs["data"] = body
                        if "content-type" not in {k.lower() for k in headers.keys()}:
                            headers["Content-Type"] = "application/x-www-form-urlencoded"
                    else:
                        request_kwargs["json"] = body
            elif isinstance(body, str):
                request_kwargs["content"] = body
                if "content-type" not in {k.lower() for k in headers.keys()}:
                    headers["Content-Type"] = "text/plain"
            elif isinstance(body, bytes):
                request_kwargs["content"] = body
                if "content-type" not in {k.lower() for k in headers.keys()}:
                    headers["Content-Type"] = "application/octet-stream"
            else:
                request_kwargs["json"] = body

        async with httpx.AsyncClient(**client_kwargs) as client:
            response = await client.request(method, url, **request_kwargs)

        return _create_response_object(response)

    except httpx.RequestError as exc:
        raise exc
    except httpx.HTTPStatusError as exc:
        if exc.response:
            return _create_response_object(exc.response)
        else:
            raise exc
    except Exception as exc:
        raise exc


def _create_response_object(response: httpx.Response) -> dict:
    """Create a JavaScript Fetch API compatible Response object as a dict."""

    # Track if body has been consumed
    body_used = False

    def mark_body_used():
        nonlocal body_used
        body_used = True

    def check_body_used():
        if body_used:
            raise RuntimeError("Body has already been consumed")

    def json_method():
        check_body_used()
        mark_body_used()
        try:
            data = response.json()
            return as_builtin_type(data)
        except Exception as e:
            raise ValueError(f"Failed to parse JSON: {e}")

    def text_method():
        check_body_used()
        mark_body_used()
        return response.text

    def blob_method():
        check_body_used()
        mark_body_used()
        return response.content

    def array_buffer_method():
        return blob_method()

    def form_data_method():
        check_body_used()
        mark_body_used()
        content_type = response.headers.get("Content-Type", "").lower()
        if "application/x-www-form-urlencoded" in content_type:
            # Parse URL-encoded form data
            parsed = parse_qs(response.text)
            result = {}
            for key, values in parsed.items():
                result[key] = values[0] if len(values) == 1 else values
            return result
        else:
            return response.text

    def clone_method():
        return _create_response_object(response)

    response_type = "basic"  # Simplified for Wandelscript context

    return {
        "status": response.status_code,
        "statusText": response.reason_phrase or "",
        "ok": 200 <= response.status_code < 300,
        "headers": dict(response.headers),
        "url": str(response.url),
        "redirected": len(response.history) > 0,
        "type": response_type,
        "bodyUsed": lambda: body_used,
        "json": json_method,
        "text": text_method,
        "blob": blob_method,
        "arrayBuffer": array_buffer_method,
        "formData": form_data_method,
        "clone": clone_method,
    }
