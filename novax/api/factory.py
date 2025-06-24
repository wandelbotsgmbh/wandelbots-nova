"""
FastAPI Application Factory

This module contains factory functions for creating FastAPI applications
with proper separation of concerns between app creation and service registration.
"""

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

from novax.container import NovaContainer

from .discovery import router as discovery_router
from .programs import register_program_routes
from .runs import register_program_run_routes


def create_base_fastapi_app(
    title: str = "Nova Python Framework API",
    description: str = "API for managing and executing Nova Python programs",
    version: str = "2.0.0",
    include_docs: bool = True,
) -> FastAPI:
    """
    Create a base FastAPI application with standard configuration.

    Args:
        title: API title
        description: API description
        version: API version
        include_docs: Whether to include API documentation endpoints

    Returns:
        Configured FastAPI application instance
    """
    app_config = {"title": title, "description": description, "version": version}

    if not include_docs:
        app_config.update({"redoc_url": None, "docs_url": None, "openapi_url": None})
    else:
        app_config.update(
            {
                "redoc_url": None,  # Disable redoc, use custom UI
                "openapi_url": "/openapi.json",
            }
        )

    app = FastAPI(**app_config)

    # Add default root endpoint with Stoplight Elements UI
    @app.get("/", summary="Opens the Stoplight UI", response_class=HTMLResponse)
    async def root():
        return """
        <!doctype html>
            <html lang="en">
              <head>
                <meta charset="utf-8">
                <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
                <title>Nova Python Framework API</title>
                <!-- Embed elements Elements via Web Component -->
                <script src="https://unpkg.com/@stoplight/elements/web-components.min.js"></script>
                <link rel="stylesheet" href="https://unpkg.com/@stoplight/elements/styles.min.css">
              </head>
              <body>

                <elements-api
                  apiDescriptionUrl="/openapi.json"
                  router="hash"
                  layout="sidebar"
                  tryItCredentialsPolicy="same-origin"
                />

              </body>
        </html>
        """

    return app


def register_api_routes(app: FastAPI) -> None:
    """
    Register all API routes with the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    register_program_routes(app)
    register_program_run_routes(app)
    app.include_router(discovery_router)


def create_nova_api_app(
    container: NovaContainer,
    title: str = "Nova Python Framework API",
    description: str = "API for managing and executing Nova Python programs",
    version: str = "2.0.0",
    include_docs: bool = True,
) -> FastAPI:
    """
    Create a complete Nova API application with container integration.

    This is the main factory function that creates a fully configured
    FastAPI application with all Nova framework features.

    Args:
        container: Configured NovaContainer instance
        title: API title
        description: API description
        version: API version
        include_docs: Whether to include API documentation endpoints

    Returns:
        Fully configured FastAPI application
    """
    # Create base FastAPI app
    app = create_base_fastapi_app(
        title=title, description=description, version=version, include_docs=include_docs
    )

    # Store container reference for dependency injection
    app.container = container

    # Register all API routes
    register_api_routes(app)

    return app


def add_health_endpoints(app: FastAPI) -> None:
    """
    Add health check endpoints to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    from .services_registry import get_service_health_status

    @app.get("/health", tags=["health"])
    async def health_check():
        """Basic health check endpoint"""
        return {"status": "healthy", "service": "nova-api"}

    @app.get("/health/services", tags=["health"])
    async def services_health_check():
        """Detailed health check for all services"""
        container = getattr(app, "container", None)
        if not container:
            return {"status": "unhealthy", "error": "Container not available"}

        return get_service_health_status(container)


def configure_cors(app: FastAPI, allowed_origins: list = None) -> None:
    """
    Configure CORS settings for the FastAPI application.

    Args:
        app: FastAPI application instance
        allowed_origins: List of allowed origins for CORS
    """
    from fastapi.middleware.cors import CORSMiddleware

    if allowed_origins is None:
        allowed_origins = ["*"]  # Allow all origins in development

    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
