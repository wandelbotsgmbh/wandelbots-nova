from novax.api.app import create_nova_api_app
from novax.container import create_container


class Novax:
    def __init__(self):
        self.container = create_container()

    def discover_programs(self):
        """
        Discover programs using the program discovery service.
        """
        discovery_service = self.container.services.program_discovery_service()
        discovery_service.discover_programs()

    def create_app(self):
        """
        Create the FastAPI application with the Nova container.

        Returns:
            FastAPI application instance
        """
        app = create_nova_api_app(self.container)
        return app
