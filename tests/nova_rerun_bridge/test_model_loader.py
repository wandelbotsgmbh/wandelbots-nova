"""Unit tests for model_loader module.

Tests cover on-demand robot model loading from the NOVA API.
"""

from unittest.mock import AsyncMock, Mock

import pytest

from nova_rerun_bridge.model_loader import load_model_data


class TestLoadModelData:
    """Test load_model_data function."""

    @pytest.mark.asyncio
    async def test_returns_glb_data_on_success(self):
        """Should return GLB bytes when API call succeeds."""
        fake_glb_data = b"fake GLB binary data"
        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=fake_glb_data
        )

        result = await load_model_data("Yaskawa_HC10DTP", mock_api_gateway)

        assert result == fake_glb_data
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_called_once_with(
            "Yaskawa_HC10DTP"
        )

    @pytest.mark.asyncio
    async def test_returns_none_for_empty_model_name(self):
        """Should return None if model name is empty."""
        mock_api_gateway = Mock()

        result = await load_model_data("", mock_api_gateway)

        assert result is None
        # API should not be called
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_not_called()

    @pytest.mark.asyncio
    async def test_returns_none_for_none_model_name(self):
        """Should return None if model name is None."""
        mock_api_gateway = Mock()

        result = await load_model_data(None, mock_api_gateway)

        assert result is None

    @pytest.mark.asyncio
    async def test_returns_none_on_api_error(self):
        """Should return None and log warning when API call fails."""
        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            side_effect=Exception("API Error: Model not found")
        )

        result = await load_model_data("InvalidModel", mock_api_gateway)

        assert result is None

    @pytest.mark.asyncio
    async def test_handles_network_errors_gracefully(self):
        """Should handle network errors without raising exceptions."""
        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            side_effect=ConnectionError("Network unreachable")
        )

        result = await load_model_data("SomeModel", mock_api_gateway)

        assert result is None
