import os


def get_host_address():
    """Get the current host address, supporting VS Code webviewer environments."""
    # Check for VS Code environment variables
    if "VSCODE_PROXY_URI" in os.environ:
        # Extract host and path from VS Code proxy URI
        # Example: 'http://172.31.11.183/cell/visual-studio-code/proxy/{{port}}/'
        proxy_uri = os.environ["VSCODE_PROXY_URI"]
        if "://" in proxy_uri:
            # Split into protocol and rest
            protocol, rest = proxy_uri.split("://", 1)
            # Split rest into host and path
            parts = rest.split("/")
            host = parts[0]
            # Find cell and vscode parts
            if len(parts) >= 3 and "proxy" in parts:
                proxy_index = parts.index("proxy")
                if proxy_index >= 2:
                    cell_name = parts[proxy_index - 2]
                    return f"{protocol}://{host}/{cell_name}"
            return host
    return "localhost"


def get_rerun_address():
    return f"{get_host_address()}/visual-studio-code/rerun/?url={get_host_address()}/visual-studio-code/nova.rrd"
