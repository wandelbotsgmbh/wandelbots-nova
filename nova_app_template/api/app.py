"""
FastAPI Application Creation

This module contains functions for creating FastAPI applications.
It ONLY handles FastAPI app creation and configuration - nothing else.
"""

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

from nova_app_template.container import NovaContainer

from .programs import register_program_routes
from .runs import register_program_run_routes


def create_fast_api_app(
    title: str = "Nova Python Framework API",
    description: str = "API for managing and executing Nova Python programs",
    version: str = "2.0.0",
) -> FastAPI:
    """
    Create a basic FastAPI application with documentation UI.

    Args:
        title: API title
        description: API description
        version: API version

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title=title,
        description=description,
        version=version,
        redoc_url=None,
        openapi_url="/openapi.json",
    )

    # Add documentation UI endpoint
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


def create_nova_api_app(container: NovaContainer) -> FastAPI:
    """
    Create a Nova API application with container integration.

    Args:
        container: NovaContainer instance for dependency injection

    Returns:
        FastAPI application with Nova routes and container integration
    """
    # Create the base FastAPI app
    app = create_fast_api_app()

    # Attach container for dependency injection
    app.container = container

    # Register API routes
    register_program_routes(app)
    register_program_run_routes(app)

    return app


# Legacy aliases for backward compatibility
create_web_app = create_nova_api_app
create_nova_web_app = create_nova_api_app
