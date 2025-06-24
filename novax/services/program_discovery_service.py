"""
Program Discovery Service

Service for automatic program discovery and template synchronization.
This service can be customized by users to implement their own discovery logic.
"""

from typing import Dict, Optional, Any
from ..interfaces import ProgramDiscoveryServiceInterface, ProgramTemplateStoreInterface
from ..decorators import REGISTERED_PROGRAM_TEMPLATES
from ..discovery.auto_discovery import auto_discover_programs


class ProgramDiscoveryService(ProgramDiscoveryServiceInterface):
    """Default implementation of program discovery service"""
    
    def __init__(self, template_store: ProgramTemplateStoreInterface, default_package: str = "nova_python_app.programs"):
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
        
        # Clear existing registered programs
        REGISTERED_PROGRAM_TEMPLATES.clear()
        
        try:
            # Use the existing auto_discovery logic
            auto_discover_programs(target_package)
            
            discovered_count = len(REGISTERED_PROGRAM_TEMPLATES)
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
        
        Returns:
            Number of programs synced
        """
        print("🔄 Syncing discovered programs to database...")
        
        sync_count = 0
        for template_name, template in REGISTERED_PROGRAM_TEMPLATES.items():
            try:
                success = self.template_store.save(template)
                if success:
                    print(f"✅ Synced template '{template_name}' to database")
                    sync_count += 1
                else:
                    print(f"❌ Failed to sync template '{template_name}' to database")
            except Exception as e:
                print(f"❌ Error syncing template '{template_name}': {e}")
        
        self._last_sync_count = sync_count
        print(f"📊 Synced {sync_count} templates successfully")
        return sync_count
    
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
                print(f"✨ Discovery initialization completed successfully: {synced_count}/{discovered_count} programs synced")
                return True
            elif discovered_count == 0:
                print("⚠️  No programs discovered - this might be expected for empty packages")
                return True
            else:
                print(f"⚠️  Discovery completed but some programs failed to sync: {synced_count}/{discovered_count}")
                return False
                
        except Exception as e:
            print(f"💥 Critical error during discovery initialization: {e}")
            return False
    
    def get_discovered_programs(self) -> Dict[str, Any]:
        """
        Get currently discovered programs.
        
        Returns:
            Dictionary of discovered program templates
        """
        return dict(REGISTERED_PROGRAM_TEMPLATES)
    
    def get_discovery_stats(self) -> Dict[str, int]:
        """
        Get statistics about the last discovery operation.
        
        Returns:
            Dictionary with discovery statistics
        """
        return {
            "last_discovery_count": self._last_discovery_count,
            "last_sync_count": self._last_sync_count,
            "currently_registered": len(REGISTERED_PROGRAM_TEMPLATES)
        }
