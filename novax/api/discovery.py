"""
Discovery API Endpoints

This module provides API endpoints for program discovery operations.
Users can trigger discovery, get discovery status, and view discovered programs.
"""

from typing import Dict, Any, Optional
from fastapi import APIRouter, Depends, HTTPException
from dependency_injector.wiring import Provide, inject

from ..container import NovaContainer
from ..interfaces import ProgramDiscoveryServiceInterface


router = APIRouter(prefix="/discovery", tags=["discovery"])


@router.post("/discover", response_model=Dict[str, Any])
@inject
def discover_programs(
    package_name: Optional[str] = None,
    discovery_service: ProgramDiscoveryServiceInterface = Depends(
        Provide[NovaContainer.services.program_discovery_service]
    )
) -> Dict[str, Any]:
    """
    Trigger program discovery for a specific package.
    
    Args:
        package_name: Package name to discover from (optional, uses default if not provided)
        discovery_service: Discovery service injected from container
        
    Returns:
        Discovery result with statistics
    """
    try:
        success = discovery_service.initialize_discovery(package_name)
        stats = discovery_service.get_discovery_stats()
        
        return {
            "success": success,
            "message": "Discovery completed successfully" if success else "Discovery encountered issues",
            "package_name": package_name or "default",
            "statistics": stats
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Discovery failed: {str(e)}")


@router.get("/status", response_model=Dict[str, Any])
@inject  
def get_discovery_status(
    discovery_service: ProgramDiscoveryServiceInterface = Depends(
        Provide[NovaContainer.services.program_discovery_service]
    )
) -> Dict[str, Any]:
    """
    Get current discovery status and statistics.
    
    Args:
        discovery_service: Discovery service injected from container
        
    Returns:
        Current discovery status and statistics
    """
    try:
        stats = discovery_service.get_discovery_stats()
        discovered_programs = discovery_service.get_discovered_programs()
        
        return {
            "status": "active",
            "statistics": stats,
            "discovered_programs": list(discovered_programs.keys()),
            "total_discovered": len(discovered_programs)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get discovery status: {str(e)}")


@router.get("/programs", response_model=Dict[str, Any])
@inject
def get_discovered_programs(
    discovery_service: ProgramDiscoveryServiceInterface = Depends(
        Provide[NovaContainer.services.program_discovery_service]
    )
) -> Dict[str, Any]:
    """
    Get all currently discovered programs.
    
    Args:
        discovery_service: Discovery service injected from container
        
    Returns:
        Dictionary of discovered program templates
    """
    try:
        discovered_programs = discovery_service.get_discovered_programs()
        
        return {
            "programs": discovered_programs,
            "total_count": len(discovered_programs),
            "program_names": list(discovered_programs.keys())
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get discovered programs: {str(e)}")


@router.post("/sync", response_model=Dict[str, Any])
@inject
def sync_discovered_programs(
    discovery_service: ProgramDiscoveryServiceInterface = Depends(
        Provide[NovaContainer.services.program_discovery_service]
    )
) -> Dict[str, Any]:
    """
    Manually sync discovered programs to the database.
    
    Args:
        discovery_service: Discovery service injected from container
        
    Returns:
        Sync operation result
    """
    try:
        synced_count = discovery_service.sync_discovered_programs()
        
        return {
            "success": True,
            "message": "Programs synced successfully",
            "synced_count": synced_count
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Sync failed: {str(e)}")
