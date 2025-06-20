"""
API package for Nova Python App

This package handles ONLY FastAPI application creation and route registration.
All business logic services are in the separate 'services' package.
Container creation and wiring should be done in __main__.py.
"""

# FastAPI application creation functions
from .app import (
    create_fast_api_app,
    create_nova_api_app,
    create_web_app,  # Legacy alias
    create_nova_web_app  # Legacy alias
)

# Route registration functions
from .programs import register_program_routes
from .runs import register_program_run_routes

__all__ = [
    # FastAPI app creation
    'create_fast_api_app',
    'create_nova_api_app',
    'create_web_app',  # Legacy
    'create_nova_web_app',  # Legacy
    
    # Route registration
    'register_program_routes',
    'register_program_run_routes',
]
