"""
Program Discovery Service

Service for automatic program discovery and template synchronization.
This service can be customized by users to implement their own discovery logic.
"""

import importlib
import pkgutil
import sys
from typing import Any, Optional

from nova.runtime.function import Function as NovaFunction

from ..interfaces import ProgramDiscoveryServiceInterface, ProgramTemplateStoreInterface
from ..store.models import BaseProgramModel, ProgramTemplate


class ProgramDiscoveryService(ProgramDiscoveryServiceInterface):
    """Default implementation of program discovery service"""

    def __init__(
        self,
        template_store: ProgramTemplateStoreInterface,
        default_package: str = "nova_python_app.programs",
    ):
        """
        Initialize the discovery service.

        Args:
            template_store: Store for persisting program templates
            default_package: Default package to discover programs from
        """
        self.template_store = template_store
        self.default_package = default_package
        self._last_discovery_count = 0
        self._last_sync_count = 0

    def discover_programs(self, package_name: Optional[str] = None) -> int:
        """
        Discover programs from a specified package or use default.

        Args:
            package_name: Package name to discover from, None for default

        Returns:
            Number of programs discovered
        """
        target_package = package_name or self.default_package

        print(f"🔍 Discovering programs from package: {target_package}")

        try:
            discovered_count = self._auto_discover_programs(target_package)
            self._last_discovery_count = discovered_count

            print(f"✅ Discovered {discovered_count} program templates")
            return discovered_count

        except Exception as e:
            print(f"❌ Error during program discovery: {e}")
            self._last_discovery_count = 0
            return 0

    def sync_discovered_programs(self) -> int:
        """
        Sync discovered programs to persistent storage.

        Note: With the new architecture, programs are automatically synced
        during discovery, so this method mainly serves for compatibility
        and can be used to re-sync if needed.

        Returns:
            Number of programs in the store
        """
        print("🔄 Checking programs in database...")

        try:
            # Get all templates from the store
            templates = self.template_store.list()
            count = len(templates)

            print(f"📊 Found {count} templates in database")
            self._last_sync_count = count
            return count

        except Exception as e:
            print(f"❌ Error accessing template store: {e}")
            self._last_sync_count = 0
            return 0

    def initialize_discovery(self, package_name: Optional[str] = None) -> bool:
        """
        Initialize program discovery during application startup.

        Args:
            package_name: Package name to discover from, None for default

        Returns:
            True if discovery completed successfully
        """
        target_package = package_name or self.default_package
        print(f"🚀 Initializing program discovery for package: {target_package}")

        try:
            # Discover programs
            discovered_count = self.discover_programs(target_package)

            # Sync to database
            synced_count = self.sync_discovered_programs()

            if discovered_count > 0 and synced_count > 0:
                print(
                    f"✨ Discovery initialization completed successfully: {synced_count}/{discovered_count} programs synced"
                )
                return True
            elif discovered_count == 0:
                print("⚠️  No programs discovered - this might be expected for empty packages")
                return True
            else:
                print(
                    f"⚠️  Discovery completed but some programs failed to sync: {synced_count}/{discovered_count}"
                )
                return False

        except Exception as e:
            print(f"💥 Critical error during discovery initialization: {e}")
            return False

    def get_discovered_programs(self) -> dict[str, Any]:
        """
        Get currently discovered programs from the template store.

        Returns:
            Dictionary of discovered program templates
        """
        try:
            templates = self.template_store.list()
            return {template.name: template for template in templates}
        except Exception:
            return {}

    def get_discovery_stats(self) -> dict[str, int]:
        """
        Get statistics about the last discovery operation.

        Returns:
            Dictionary with discovery statistics
        """
        try:
            current_count = len(self.template_store.list())
        except Exception:
            current_count = 0

        return {
            "last_discovery_count": self._last_discovery_count,
            "last_sync_count": self._last_sync_count,
            "currently_registered": current_count,
        }

    def _auto_discover_programs(self, package_name: str) -> int:
        """
        Auto-discover programs by walking the specified package and importing/reloading its modules.
        Looks for functions tagged with @nova.program decorator.
        """
        discovered_templates = []

        try:
            # Attempt to import the top-level package first to ensure it exists
            # and to handle the case where package_name itself is the module to discover.
            # We reload it if it's already imported to ensure its own decorators run.
            if package_name in sys.modules:
                package = importlib.reload(sys.modules[package_name])
            else:
                package = importlib.import_module(package_name)

            # If it's a package (has __path__), walk its modules
            if hasattr(package, "__path__"):
                for _, modname, _ in pkgutil.walk_packages(
                    path=package.__path__,
                    prefix=package.__name__ + ".",
                    onerror=lambda x: None,  # Or handle errors as needed
                ):
                    if modname in sys.modules:
                        module = importlib.reload(
                            sys.modules[modname]
                        )  # Reload if already imported
                    else:
                        module = importlib.import_module(modname)  # Import if new

                    # Inspect the module for @nova.program decorated functions
                    module_templates = self._discover_nova_programs_in_module(module)
                    discovered_templates.extend(module_templates)

            # If it's not a package (no __path__), it means package_name was a single module.
            # Inspect it for @nova.program decorated functions
            else:
                module_templates = self._discover_nova_programs_in_module(package)
                discovered_templates.extend(module_templates)

        except ImportError:
            print(f"Error: Could not import package/module {package_name} for auto-discovery.")

        # Sync discovered templates to database
        self._sync_templates_to_database(discovered_templates)

        return len(discovered_templates)

    def _discover_nova_programs_in_module(self, module):
        """
        Discover functions decorated with @nova.program in a module and return them as templates.
        """
        discovered_templates = []

        for attr_name in dir(module):
            attr = getattr(module, attr_name)

            # Check if this is a Function object created by @nova.program
            if isinstance(attr, NovaFunction):
                # Get the wrapped function
                wrapped_func = attr._wrapped

                # Create a NovaX program template
                template_name = attr.name

                # Create a basic schema from the Function's input model
                try:
                    schema = attr.input_schema
                except Exception:
                    schema = {}

                # Create the program template
                template = ProgramTemplate(
                    name=template_name,
                    model_class=BaseProgramModel,
                    function=wrapped_func,
                    schema=schema,
                )

                discovered_templates.append(template)
                print(
                    f"Discovered @nova.program function: {template_name} in module {module.__name__}"
                )

        return discovered_templates

    def _sync_templates_to_database(self, templates):
        """Sync discovered program templates to the database"""
        for template in templates:
            success = self.template_store.save(template)
            if success:
                print(f"Synced template '{template.name}' to database")

    def sync_templates_only(self, templates=None):
        """
        Sync a list of templates to the database.
        Used by legacy startup functions for compatibility.

        Args:
            templates: List of templates to sync. If None, gets templates from store.
        """
        if templates is None:
            # For compatibility, try to get all existing templates
            try:
                templates = self.template_store.list()
            except Exception:
                templates = []

        self._sync_templates_to_database(templates)
