import importlib
import pkgutil
import sys
from nova_python_app.backbone.decorators import REGISTERED_PROGRAM_TEMPLATES
from nova_python_app.backbone.database import template_store


def auto_discover_programs(package_name: str):
    """
    Clears existing registered programs and auto-discovers programs
    by walking the specified package and importing/reloading its modules.
    """
    REGISTERED_PROGRAM_TEMPLATES.clear()  # Clear previously registered programs for auto-discovery
    try:
        # Attempt to import the top-level package first to ensure it exists
        # and to handle the case where package_name itself is the module to discover.
        # We reload it if it's already imported to ensure its own decorators run.
        if package_name in sys.modules:
            package = importlib.reload(sys.modules[package_name])
        else:
            package = importlib.import_module(package_name)

        # If it's a package (has __path__), walk its modules
        if hasattr(package, '__path__'):
            for _, modname, _ in pkgutil.walk_packages(
                path=package.__path__,
                prefix=package.__name__ + '.',
                onerror=lambda x: None,  # Or handle errors as needed
            ):
                if modname in sys.modules:
                    importlib.reload(sys.modules[modname])  # Reload if already imported
                else:
                    importlib.import_module(modname)  # Import if new
        # If it's not a package (no __path__), it means package_name was a single module.
        # It has already been imported or reloaded above.
        # No further action needed here for the single module case as it's covered.

    except ImportError:
        print(f"Error: Could not import package/module {package_name} for auto-discovery.")
    # Removed the specific AttributeError catch for package.__path__ as the logic
    # now handles single modules and packages more uniformly. If import_module fails,
    # the ImportError above will catch it. If it succeeds, hasattr check distinguishes.
    
    # Sync discovered templates to database
    sync_templates_to_database()


def sync_templates_to_database():
    """Sync in-memory program templates to the database"""
    for template_name, template in REGISTERED_PROGRAM_TEMPLATES.items():
        success = template_store.save(template)
        if success:
            print(f"Synced template '{template_name}' to database")
        else:
            print(f"Failed to sync template '{template_name}' to database")
