# Bundled wheels

`nova2urdf` derives a URDF from the DH parameters and meshes the Nova API
serves, and `nova_rerun_bridge` renders robots from that URDF. It is not on
PyPI yet, so the wheel is bundled here and resolved through
`[tool.uv.sources]` in `pyproject.toml`.

**This is interim.** `[tool.uv.sources]` is honoured by uv for this project
only; pip and anyone installing the published `wandelbots-nova` wheel ignore
it, so the `nova-rerun-bridge` extra cannot resolve `nova2urdf` until the
package is on an index.

When it is published, delete this directory and the `nova2urdf` entry from
`[tool.uv.sources]`. The dependency declaration in the extra already names the
version, so nothing else changes.

Rebuild after a nova2urdf change:

    cd <nova2urdf checkout> && uv build
    cp dist/nova2urdf-<version>-py3-none-any.whl <here>/
    cd <here>/.. && uv lock && uv sync --all-extras
