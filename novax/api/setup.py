"""
Nova Application Setup Module

This module provides high-level functions to set up a complete Nova application
with proper separation of concerns between container setup, service registration,
and FastAPI application creation.
"""

from typing import Optional

from fastapi import FastAPI

from novax.container import NovaContainer
from novax.discovery import setup_program_discovery

from .factory import add_health_endpoints, configure_cors, create_nova_api_app
from .services_registry import initialize_api_services, register_api_services


def setup_nova_container() -> NovaContainer:
    """
    Create and configure a Nova container with all necessary dependencies.

    Returns:
        Configured NovaContainer instance
    """
    container = NovaContainer()

    # Register API services with the container
    register_api_services(container)

    # Initialize the services
    initialize_api_services(container)

    return container


def wire_container_with_api(container: NovaContainer) -> None:
    """
    Wire the container with the API modules for dependency injection.

    Args:
        container: The NovaContainer instance to wire
    """
    container.wire(modules=["novax.api.programs", "novax.api.runs"])


def print_database_stats(container: NovaContainer) -> None:
    """
    Print database statistics for debugging and monitoring.

    Args:
        container: The NovaContainer instance with database connection
    """
    db_connection = container.stores.database_connection()
    stats = db_connection.get_database_stats()
    print(
        f"Database loaded: {stats['template_count']} templates, {stats['instance_count']} instances"
    )


def create_complete_nova_app(
    discovery_package_name: Optional[str] = None,
    include_health_endpoints: bool = True,
    enable_cors: bool = True,
    cors_origins: list = None,
    print_stats: bool = True,
) -> FastAPI:
    """
    Create a complete Nova application with all features configured.

    This is the main entry point for creating a fully functional Nova API application.
    It handles all setup steps in the correct order:
    1. Program discovery
    2. Container setup and service registration
    3. Container wiring
    4. FastAPI app creation with all features

    Args:
        discovery_package_name: Package name for auto-discovery
        include_health_endpoints: Whether to include health check endpoints
        enable_cors: Whether to enable CORS middleware
        cors_origins: List of allowed CORS origins
        print_stats: Whether to print database statistics

    Returns:
        Fully configured FastAPI application ready to run
    """
    # Step 1: Setup program discovery
    setup_program_discovery(discovery_package_name)

    # Step 2: Create and configure the container
    container = setup_nova_container()

    # Step 3: Wire the container with API modules
    wire_container_with_api(container)

    # Step 4: Print database stats if requested
    if print_stats:
        print_database_stats(container)

    # Step 5: Create the FastAPI application
    app = create_nova_api_app(container)

    # Step 6: Add optional features
    if include_health_endpoints:
        add_health_endpoints(app)

    if enable_cors:
        configure_cors(app, cors_origins)

    return app


def create_minimal_nova_app(discovery_package_name: Optional[str] = None) -> FastAPI:
    """
    Create a minimal Nova application with basic features only.

    This creates an app without health endpoints or CORS, suitable for
    production environments where these features are handled elsewhere.

    Args:
        discovery_package_name: Package name for auto-discovery

    Returns:
        Minimal FastAPI application
    """
    return create_complete_nova_app(
        discovery_package_name=discovery_package_name,
        include_health_endpoints=False,
        enable_cors=False,
        print_stats=False,
    )


def create_development_nova_app(discovery_package_name: Optional[str] = None) -> FastAPI:
    """
    Create a Nova application optimized for development.

    This includes all development-friendly features like health endpoints,
    permissive CORS, and debug output.

    Args:
        discovery_package_name: Package name for auto-discovery

    Returns:
        Development-friendly FastAPI application
    """
    return create_complete_nova_app(
        discovery_package_name=discovery_package_name,
        include_health_endpoints=True,
        enable_cors=True,
        cors_origins=["*"],  # Allow all origins in development
        print_stats=True,
    )
