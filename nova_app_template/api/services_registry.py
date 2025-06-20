"""
Service Registry for API Services

This module handles the registration of all API-related services
with the NovaContainer, separating service registration from FastAPI creation.
"""

from typing import TYPE_CHECKING
from dependency_injector import providers

if TYPE_CHECKING:
    from nova_python_app.backbone.container import NovaContainer


def register_api_services(container: "NovaContainer") -> None:
    """
    Register all API services with the container.
    
    This function separates service registration from FastAPI app creation,
    allowing for better modularity and testability.
    
    Args:
        container: The NovaContainer instance to register services with
    """
    from .services import (
        ProgramAPIService,
        ProgramRunAPIService,
        ProgramExecutionService
    )
    
    # Register Program API Service
    container.program_api_service.override(
        providers.Factory(
            ProgramAPIService,
            template_store=container.stores.provided.program_template_store,
            instance_store=container.stores.provided.program_instance_store
        )
    )
    
    # Register Program Run API Service
    container.program_run_api_service.override(
        providers.Factory(
            ProgramRunAPIService,
            run_store=container.stores.provided.program_run_store,
            instance_store=container.stores.provided.program_instance_store
        )
    )
    
    # Register Program Execution Service
    container.program_execution_service.override(
        providers.Factory(
            ProgramExecutionService,
            processor=container.processors.provided.current_processor,
            template_store=container.stores.provided.program_template_store,
            instance_store=container.stores.provided.program_instance_store,
            run_store=container.stores.provided.program_run_store
        )
    )


def initialize_api_services(container: "NovaContainer") -> None:
    """
    Initialize all registered API services.
    
    This calls the initialize method on each service that has one.
    
    Args:
        container: The NovaContainer instance with registered services
    """
    # Initialize services that have an initialize method
    services_to_initialize = [
        container.program_api_service(),
        container.program_run_api_service(),
    ]
    
    for service in services_to_initialize:
        if hasattr(service, 'initialize'):
            service.initialize()


def get_service_health_status(container: "NovaContainer") -> dict:
    """
    Get health status of all registered API services.
    
    Args:
        container: The NovaContainer instance with registered services
        
    Returns:
        Dictionary with service health status information
    """
    health_status = {
        "services": {},
        "overall_status": "healthy"
    }
    
    service_names = [
        "program_api_service",
        "program_run_api_service", 
        "program_execution_service"
    ]
    
    for service_name in service_names:
        try:
            service = getattr(container, service_name)()
            health_status["services"][service_name] = {
                "status": "healthy",
                "type": type(service).__name__
            }
        except Exception as e:
            health_status["services"][service_name] = {
                "status": "unhealthy",
                "error": str(e)
            }
            health_status["overall_status"] = "degraded"
    
    return health_status
