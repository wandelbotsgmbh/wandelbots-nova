"""On-demand robot model downloading using the NOVA API."""

from pathlib import Path
from typing import TYPE_CHECKING

from loguru import logger

from nova_rerun_bridge.helper_scripts.download_models import get_project_root

if TYPE_CHECKING:
    from nova.core.gateway import ApiGateway


def get_models_dir() -> Path:
    """Get the models directory path."""
    return Path(get_project_root()) / "models"


def model_exists(model_name: str) -> bool:
    """Check if a model file exists locally.

    Args:
        model_name: The model name (e.g., "Yaskawa_HC10DTP")

    Returns:
        True if the model file exists, False otherwise
    """
    models_dir = get_models_dir()
    model_path = models_dir / f"{model_name}.glb"
    return model_path.exists()


async def download_model(model_name: str, api_gateway: "ApiGateway") -> Path:
    """Download a robot model GLB file from the NOVA API.

    Args:
        model_name: The model name (e.g., "Yaskawa_HC10DTP")
        api_gateway: The NOVA API gateway instance

    Returns:
        Path to the downloaded model file

    Raises:
        Exception: If download fails
    """
    models_dir = get_models_dir()
    models_dir.mkdir(parents=True, exist_ok=True)

    model_path = models_dir / f"{model_name}.glb"

    # Skip if already exists
    if model_path.exists():
        logger.debug(f"Model {model_name} already exists at {model_path}")
        return model_path

    logger.info(f"Downloading robot model: {model_name}")

    # Use the motion group models API to download the GLB
    glb_data = await api_gateway.motion_group_models_api.get_motion_group_glb_model(model_name)

    # Write to file
    with open(model_path, "wb") as f:
        f.write(glb_data)

    logger.info(f"Downloaded model to: {model_path} ({len(glb_data)} bytes)")
    return model_path


async def ensure_model_available(model_name: str, api_gateway: "ApiGateway") -> Path | None:
    """Ensure a robot model is available locally, downloading if necessary.

    This is the main entry point for on-demand model downloading.

    Args:
        model_name: The model name (e.g., "Yaskawa_HC10DTP")
        api_gateway: The NOVA API gateway instance

    Returns:
        Path to the model file, or None if download failed
    """
    if not model_name:
        return None

    if model_exists(model_name):
        return get_models_dir() / f"{model_name}.glb"

    try:
        return await download_model(model_name, api_gateway)
    except Exception as e:
        logger.warning(f"Failed to download model {model_name}: {e}")
        return None
