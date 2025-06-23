"""
Nova Python App Main Entry Point

This module handles:
- Container creation and configuration
- Program discovery and template synchronization
- FastAPI app creation and container wiring
- Application startup
"""
import uvicorn

from nova_app_template import create_container, create_nova_api_app
from nova_app_template.interfaces import ProgramDiscoveryServiceInterface

class ProgramDiscoveryService(ProgramDiscoveryServiceInterface):
    pass

if __name__ == "__main__":
    # Step 1: Create the configured container with all services
    print("🚀 Starting Nova Python Framework...")
    container = create_container()
    
    # Step 2: Initialize program discovery using the service from the container
    discovery_service = container.services.program_discovery_service()
    discovery_service.discover_programs()
        
    # Step 4: Create the FastAPI app with container integration
    app = create_nova_api_app(container)
    
    
    # Step 6: Start the server
    print("Starting Nova Python Framework API server...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
