import httpx

from wandelscript.datatypes import as_builtin_type
from wandelscript.metamodel import register_builtin_func


@register_builtin_func()
async def fetch(url: str, options: dict | None = None) -> dict:
    """Fetch data from a URL.

    Args:
        url: The URL to fetch data from.
        options: Additional options for the fetch operation.
            {
                # The HTTP method to use (GET, POST, PUT, DELETE). Default is GET.
                method: str,
                # The body of the request. Default is None.
                body: record,
                # Additional headers to include in the request. Default is None.
                headers: record,
            }

    Raises:
        ValueError: If the method is not supported.

    Returns:
        The fetched data as a record with the following fields:
        {
            data: Any,  # The fetched data.
            status_code: int,  # The HTTP status code of the response
        }

    """
    options = options or {}
    method = options.get("method", "GET")
    body = options.get("body")
    headers = options.get("headers")

    try:
        # Select the appropriate HTTP method using httpx
        async with httpx.AsyncClient() as client:
            match method:
                case "GET":
                    response = await client.get(url, headers=headers)
                case "POST":
                    response = await client.post(url, json=body, headers=headers)
                case "PUT":
                    response = await client.put(url, json=body, headers=headers)
                case "DELETE":
                    response = await client.delete(url, headers=headers)
                case _:
                    raise ValueError(f"Unsupported method: {method}")

        # Handle different content types based on headers
        content_type = response.headers.get("Content-Type", "").lower()
        if "application/json" in content_type:
            try:
                data = response.json()
            except ValueError:
                data = response.text  # Fall back to text if JSON parsing fails
        elif "text/" in content_type:
            data = response.text
        else:
            data = response.content  # Fall back to raw bytes

        parsed_data = as_builtin_type(data)
        return {"data": parsed_data, "status_code": response.status_code}

    except httpx.HTTPStatusError as exc:
        status_code = exc.response.status_code if exc.response else 500
        return {"error": str(exc), "status_code": status_code}
