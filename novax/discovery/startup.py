"""
Startup Discovery Module

This module provides legacy compatibility functions for program discovery.
These functions create a default discovery service and delegate to it.

For new code, use the ProgramDiscoveryService directly through dependency injection.
"""

from typing import Optional

from ..database import template_store
from ..services.program_discovery_service import ProgramDiscoveryService


def _get_default_discovery_service():
    """Get a default discovery service instance for legacy functions"""
    return ProgramDiscoveryService(template_store)


def initialize_discovery(discovery_package_name: Optional[str] = None) -> None:
    """
    Initialize program discovery during application startup.

    This function is designed to be called automatically during application startup
    to discover and register all program templates.

    Args:
        discovery_package_name: Package name for auto-discovery. If None, defaults to 'nova_python_app.programs'
    """
    # Use default package if none specified
    if discovery_package_name is None:
        discovery_package_name = "nova_python_app.programs"

    print(f"🔍 Initializing auto-discovery for package: {discovery_package_name}")

    try:
        service = _get_default_discovery_service()
        service.discover_programs(discovery_package_name)
        print("✅ Program discovery completed successfully")
    except Exception as e:
        print(f"❌ Error during program discovery: {e}")
        # Still try to sync any templates that were already registered
        print("🔄 Attempting to sync existing templates...")
        try:
            service = _get_default_discovery_service()
            service.sync_templates_only()
        except Exception as sync_error:
            print(f"❌ Error syncing templates: {sync_error}")


def setup_program_discovery(discovery_package_name: Optional[str] = None) -> None:
    """
    Handle program discovery and template synchronization.

    Args:
        discovery_package_name: Package name for auto-discovery, if None uses import-based discovery
    """
    service = _get_default_discovery_service()

    if discovery_package_name:
        print(f"Starting auto-discovery for package: {discovery_package_name}")
        service.discover_programs(discovery_package_name)
    else:
        print("Using import-based program discovery. Ensure programs are imported.")
        # Still sync templates to database even without auto-discovery
        service.sync_templates_only()


def initialize_discovery_with_service(
    discovery_service, package_name: Optional[str] = None
) -> bool:
    """
    Initialize program discovery using a discovery service from the DI container.

    This is the preferred approach as it allows users to customize the discovery service.

    Args:
        discovery_service: The discovery service instance from the DI container
        package_name: Package name for auto-discovery, if None uses service default

    Returns:
        True if discovery completed successfully
    """
    return discovery_service.initialize_discovery(package_name)


# Legacy functions for backward compatibility
def auto_discover_programs(package_name: str) -> int:
    """
    Legacy function for backward compatibility.

    Args:
        package_name: Package name for auto-discovery

    Returns:
        Number of programs discovered
    """
    service = _get_default_discovery_service()
    return service.discover_programs(package_name)


def sync_templates_to_database(templates=None) -> None:
    """
    Legacy function for backward compatibility.

    Args:
        templates: List of templates to sync (optional)
    """
    service = _get_default_discovery_service()
    service.sync_templates_only(templates)
