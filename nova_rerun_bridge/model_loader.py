"""Load robot models directly from the NOVA API."""

from loguru import logger

from nova.core.gateway import ApiGateway


async def load_model_data(model_name: str, api_gateway: ApiGateway) -> bytes | None:
    """Load a robot model's GLB data directly from the NOVA API.

    Args:
        model_name: The model name (e.g., "Yaskawa_HC10DTP")
        api_gateway: The NOVA API gateway instance

    Returns:
        GLB data as bytes, or None if loading failed
    """
    if not model_name:
        return None

    try:
        return await api_gateway.motion_group_models_api.get_motion_group_glb_model(model_name)
    except Exception as e:
        logger.warning(f"Failed to load model {model_name}: {e}")
        return None
