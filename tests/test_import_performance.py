#!/usr/bin/env python3
"""
Performance tests for nova module import speed using pyinstrument.
"""

import sys
import time
from pathlib import Path

from pyinstrument import Profiler


def test_nova_import_performance():
    """Test that nova module imports in reasonable time."""
    modules_to_remove = [name for name in sys.modules.keys() if name.startswith("nova")]
    for module in modules_to_remove:
        del sys.modules[module]

    start_time = time.time()
    import nova  # noqa: F401

    import_time = time.time() - start_time

    print(f"Nova import took {import_time:.2f}s")

    assert import_time < 1.0, f"Nova import took {import_time:.2f}s, expected < 1.0s"


def test_nova_import_profile():
    """Profile nova import to identify bottlenecks."""
    modules_to_remove = [name for name in sys.modules.keys() if name.startswith("nova")]
    for module in modules_to_remove:
        del sys.modules[module]

    profiler = Profiler()
    profiler.start()

    import nova  # noqa: F401

    profiler.stop()

    print("\n" + "=" * 80)
    print("NOVA IMPORT PROFILING RESULTS")
    print("=" * 80)
    print(profiler.output_text(unicode=True, color=True))

    profile_path = Path("nova_import_profile.html")
    with open(profile_path, "w") as f:
        f.write(profiler.output_html())

    print(f"\nDetailed HTML profile saved to: {profile_path.absolute()}")


def test_api_module_lazy_loading():
    """Test that api module components are accessible after lazy loading implementation."""
    from nova import api

    assert hasattr(api, "models"), "api.models should be accessible"
    assert hasattr(api, "api"), "api.api should be accessible"
    assert hasattr(api, "api_client"), "api.api_client should be accessible"
    assert hasattr(api, "configuration"), "api.configuration should be accessible"
    assert hasattr(api, "exceptions"), "api.exceptions should be accessible"

    models = api.models
    assert models is not None, "api.models should not be None"

    print("All api module components are accessible after lazy loading")


def test_backward_compatibility():
    """Test that existing import patterns still work."""
    from nova.api import models

    assert models is not None, "from nova.api import models should work"

    from nova import api

    assert api.models is not None, "nova.api.models should work"

    print("Backward compatibility maintained for existing import patterns")


if __name__ == "__main__":
    print("Running nova import performance tests...")
    print("-" * 50)

    test_nova_import_performance()

    test_nova_import_profile()

    test_api_module_lazy_loading()

    test_backward_compatibility()

    print("\nAll tests completed successfully!")
