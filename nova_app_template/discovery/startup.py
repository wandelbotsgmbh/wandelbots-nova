"""
Startup Discovery Module

This module handles automatic initialization of program discovery during application startup.
It provides both the legacy direct function approach and the new service-based approach.
"""

from typing import Optional
from .auto_discovery import auto_discover_programs, sync_templates_to_database


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
        auto_discover_programs(discovery_package_name)
        print("✅ Program discovery completed successfully")
    except Exception as e:
        print(f"❌ Error during program discovery: {e}")
        # Still try to sync any templates that were already registered
        print("🔄 Attempting to sync existing templates...")
        try:
            sync_templates_to_database()
        except Exception as sync_error:
            print(f"❌ Error syncing templates: {sync_error}")


def setup_program_discovery(discovery_package_name: Optional[str] = None) -> None:
    """
    Handle program discovery and template synchronization.
    
    Args:
        discovery_package_name: Package name for auto-discovery, if None uses import-based discovery
    """
    if discovery_package_name:
        print(f"Starting auto-discovery for package: {discovery_package_name}")
        auto_discover_programs(discovery_package_name)
    else:
        print("Using import-based program discovery. Ensure programs are imported.")
        # Still sync templates to database even without auto-discovery
        sync_templates_to_database()


def initialize_discovery_with_service(discovery_service, package_name: Optional[str] = None) -> bool:
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
