"""
Interfaces for the Nova Python Framework

These interfaces define the contracts that can be implemented by users
to customize the behavior of the framework.
"""

from abc import ABC, abstractmethod
from contextlib import contextmanager
from typing import Any, Optional

# Import the processor interface from the processors package


class DatabaseConnectionInterface(ABC):
    """Interface for database connections"""

    @abstractmethod
    def init_database(self) -> None:
        """Initialize the database schema"""
        pass

    @abstractmethod
    @contextmanager
    def get_connection(self):
        """Get a database connection context manager"""
        pass

    @abstractmethod
    def get_database_stats(self) -> dict[str, int]:
        """Get database statistics"""
        pass

    @abstractmethod
    def backup_database(self, backup_path: str) -> bool:
        """Create a backup of the database"""
        pass


class ProgramTemplateStoreInterface(ABC):
    """Interface for program template storage"""

    @abstractmethod
    def save(self, template: Any) -> bool:
        """Save or update a program template"""
        pass

    @abstractmethod
    def get(self, name: str) -> Optional[dict[str, Any]]:
        """Get a program template by name"""
        pass

    @abstractmethod
    def get_all(self) -> list[dict[str, Any]]:
        """Get all program templates"""
        pass

    @abstractmethod
    def delete(self, name: str) -> bool:
        """Delete a program template"""
        pass


class ProgramInstanceStoreInterface(ABC):
    """Interface for program instance storage"""

    @abstractmethod
    def save(self, instance: Any) -> bool:
        """Save or update a program instance"""
        pass

    @abstractmethod
    def get(self, name: str) -> Optional[dict[str, Any]]:
        """Get a program instance by name"""
        pass

    @abstractmethod
    def get_all(self) -> list[dict[str, Any]]:
        """Get all program instances"""
        pass

    @abstractmethod
    def get_by_template(self, template_name: str) -> list[dict[str, Any]]:
        """Get all program instances for a specific template"""
        pass

    @abstractmethod
    def delete(self, name: str) -> bool:
        """Delete a program instance"""
        pass

    @abstractmethod
    def update_data(self, name: str, data: dict[str, Any]) -> bool:
        """Update only the data field of a program instance"""
        pass


class ProgramRunStoreInterface(ABC):
    """Interface for program run storage"""

    @abstractmethod
    def save(self, run: Any) -> bool:
        """Save or update a program run"""
        pass

    @abstractmethod
    def get(self, run_id: str) -> Optional[dict[str, Any]]:
        """Get a program run by ID"""
        pass

    @abstractmethod
    def get_all(self) -> list[dict[str, Any]]:
        """Get all program runs"""
        pass

    @abstractmethod
    def get_by_program(self, program_name: str) -> list[dict[str, Any]]:
        """Get all runs for a specific program"""
        pass

    @abstractmethod
    def update_status(self, run_id: str, status: str, **kwargs) -> bool:
        """Update the status of a program run"""
        pass

    @abstractmethod
    def delete(self, run_id: str) -> bool:
        """Delete a program run"""
        pass


class APIServiceInterface(ABC):
    """Interface for API services that handle business logic"""

    @abstractmethod
    def initialize(self) -> None:
        """Initialize the service"""
        pass


class ProgramAPIServiceInterface(APIServiceInterface):
    """Interface for program API service"""

    @abstractmethod
    def get_programs(self) -> list[dict[str, Any]]:
        """Get all programs"""
        pass

    @abstractmethod
    def get_program(self, name: str) -> Optional[dict[str, Any]]:
        """Get a specific program"""
        pass

    @abstractmethod
    def create_program(self, program_data: dict[str, Any]) -> bool:
        """Create a new program"""
        pass

    @abstractmethod
    def update_program(self, name: str, program_data: dict[str, Any]) -> bool:
        """Update an existing program"""
        pass

    @abstractmethod
    def delete_program(self, name: str) -> bool:
        """Delete a program"""
        pass


class ProgramRunAPIServiceInterface(APIServiceInterface):
    """Interface for program run API service"""

    @abstractmethod
    def get_runs(self, program_name: Optional[str] = None) -> list[dict[str, Any]]:
        """Get runs, optionally filtered by program name"""
        pass

    @abstractmethod
    def get_run(self, run_id: str) -> Optional[dict[str, Any]]:
        """Get a specific run"""
        pass

    @abstractmethod
    def create_run(self, run_data: dict[str, Any]) -> Optional[str]:
        """Create a new run"""
        pass

    @abstractmethod
    def update_run_status(self, run_id: str, status: str, **kwargs) -> bool:
        """Update run status"""
        pass


class ProgramExecutionServiceInterface(ABC):
    """Interface for program execution service"""

    @abstractmethod
    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        parameters: dict[str, Any],
        environment_variables: dict[str, str],
    ) -> dict[str, Any]:
        """Execute a program using the configured processor"""
        pass


class ProgramDiscoveryServiceInterface(ABC):
    """Interface for program discovery services"""

    @abstractmethod
    def discover_programs(self, package_name: Optional[str] = None) -> int:
        """
        Discover programs from a specified package or use default.

        Args:
            package_name: Package name to discover from, None for default

        Returns:
            Number of programs discovered
        """
        pass

    @abstractmethod
    def sync_discovered_programs(self) -> int:
        """
        Sync discovered programs to persistent storage.

        Returns:
            Number of programs synced
        """
        pass

    @abstractmethod
    def initialize_discovery(self, package_name: Optional[str] = None) -> bool:
        """
        Initialize program discovery during application startup.

        Args:
            package_name: Package name to discover from, None for default

        Returns:
            True if discovery completed successfully
        """
        pass

    @abstractmethod
    def get_discovered_programs(self) -> dict[str, Any]:
        """
        Get currently discovered programs.

        Returns:
            Dictionary of discovered program templates
        """
        pass
