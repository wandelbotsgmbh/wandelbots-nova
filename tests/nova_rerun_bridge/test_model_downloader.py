"""Unit tests for model_downloader module.

Tests cover on-demand robot model downloading functionality.
"""

from pathlib import Path
from unittest.mock import AsyncMock, Mock, patch

import pytest

from nova_rerun_bridge.model_downloader import (
    download_model,
    ensure_model_available,
    get_models_dir,
    model_exists,
)


class TestGetModelsDir:
    """Test get_models_dir function."""

    @patch("nova_rerun_bridge.model_downloader.get_project_root")
    def test_returns_models_subdirectory(self, mock_get_root):
        """Should return models subdirectory of project root."""
        mock_get_root.return_value = Path("/fake/project")

        result = get_models_dir()

        assert result == Path("/fake/project/models")

    @patch("nova_rerun_bridge.model_downloader.get_project_root")
    def test_returns_path_object(self, mock_get_root):
        """Should return a Path object."""
        mock_get_root.return_value = Path("/fake/project")

        result = get_models_dir()

        assert isinstance(result, Path)


class TestModelExists:
    """Test model_exists function."""

    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    def test_returns_true_when_model_file_exists(self, mock_get_models_dir, tmp_path):
        """Should return True when the model GLB file exists."""
        mock_get_models_dir.return_value = tmp_path

        # Create a fake model file
        model_file = tmp_path / "TestRobot_Model.glb"
        model_file.write_bytes(b"fake glb data")

        result = model_exists("TestRobot_Model")

        assert result is True

    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    def test_returns_false_when_model_file_missing(self, mock_get_models_dir, tmp_path):
        """Should return False when the model GLB file does not exist."""
        mock_get_models_dir.return_value = tmp_path

        result = model_exists("NonExistentModel")

        assert result is False

    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    def test_returns_false_when_models_dir_missing(self, mock_get_models_dir, tmp_path):
        """Should return False when the models directory does not exist."""
        mock_get_models_dir.return_value = tmp_path / "nonexistent"

        result = model_exists("SomeModel")

        assert result is False

    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    def test_checks_correct_file_extension(self, mock_get_models_dir, tmp_path):
        """Should check for .glb extension specifically."""
        mock_get_models_dir.return_value = tmp_path

        # Create a file with wrong extension
        wrong_ext_file = tmp_path / "TestRobot_Model.gltf"
        wrong_ext_file.write_bytes(b"fake data")

        result = model_exists("TestRobot_Model")

        assert result is False


class TestDownloadModel:
    """Test download_model function."""

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_downloads_and_saves_model(self, mock_get_models_dir, tmp_path):
        """Should download model from API and save to disk."""
        mock_get_models_dir.return_value = tmp_path

        fake_glb_data = b"fake GLB binary data for robot model"
        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=fake_glb_data
        )

        result = await download_model("Yaskawa_HC10DTP", mock_api_gateway)

        # Should return path to saved file
        assert result == tmp_path / "Yaskawa_HC10DTP.glb"

        # File should exist with correct content
        assert result.exists()
        assert result.read_bytes() == fake_glb_data

        # Should have called the API
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_called_once_with(
            "Yaskawa_HC10DTP"
        )

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_creates_models_directory_if_missing(self, mock_get_models_dir, tmp_path):
        """Should create the models directory if it doesn't exist."""
        models_dir = tmp_path / "models"
        mock_get_models_dir.return_value = models_dir

        assert not models_dir.exists()

        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=b"glb data"
        )

        await download_model("TestModel", mock_api_gateway)

        assert models_dir.exists()

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_skips_download_if_model_exists(self, mock_get_models_dir, tmp_path):
        """Should skip download if model file already exists."""
        mock_get_models_dir.return_value = tmp_path

        # Pre-create the model file
        model_file = tmp_path / "ExistingModel.glb"
        original_content = b"original content"
        model_file.write_bytes(original_content)

        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=b"new content"
        )

        result = await download_model("ExistingModel", mock_api_gateway)

        # Should return path without calling API
        assert result == model_file
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_not_called()

        # File should retain original content
        assert model_file.read_bytes() == original_content

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_raises_on_api_error(self, mock_get_models_dir, tmp_path):
        """Should raise exception when API call fails."""
        mock_get_models_dir.return_value = tmp_path

        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            side_effect=Exception("API Error: Model not found")
        )

        with pytest.raises(Exception, match="API Error"):
            await download_model("InvalidModel", mock_api_gateway)


class TestEnsureModelAvailable:
    """Test ensure_model_available function."""

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.model_exists")
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_returns_path_when_model_exists(self, mock_get_models_dir, mock_model_exists):
        """Should return path immediately if model already exists."""
        mock_get_models_dir.return_value = Path("/fake/models")
        mock_model_exists.return_value = True

        mock_api_gateway = Mock()

        result = await ensure_model_available("ExistingModel", mock_api_gateway)

        assert result == Path("/fake/models/ExistingModel.glb")
        # Should not call the API
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_not_called()

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.download_model")
    @patch("nova_rerun_bridge.model_downloader.model_exists")
    async def test_downloads_when_model_missing(self, mock_model_exists, mock_download):
        """Should download model if it doesn't exist."""
        mock_model_exists.return_value = False
        mock_download.return_value = Path("/fake/models/NewModel.glb")

        mock_api_gateway = Mock()

        result = await ensure_model_available("NewModel", mock_api_gateway)

        assert result == Path("/fake/models/NewModel.glb")
        mock_download.assert_called_once_with("NewModel", mock_api_gateway)

    @pytest.mark.asyncio
    async def test_returns_none_for_empty_model_name(self):
        """Should return None if model name is empty."""
        mock_api_gateway = Mock()

        result = await ensure_model_available("", mock_api_gateway)

        assert result is None

    @pytest.mark.asyncio
    async def test_returns_none_for_none_model_name(self):
        """Should return None if model name is None."""
        mock_api_gateway = Mock()

        result = await ensure_model_available(None, mock_api_gateway)

        assert result is None

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.download_model")
    @patch("nova_rerun_bridge.model_downloader.model_exists")
    async def test_returns_none_on_download_failure(self, mock_model_exists, mock_download):
        """Should return None and log warning if download fails."""
        mock_model_exists.return_value = False
        mock_download.side_effect = Exception("Network error")

        mock_api_gateway = Mock()

        with patch("nova_rerun_bridge.model_downloader.logger") as mock_logger:
            result = await ensure_model_available("FailingModel", mock_api_gateway)

            assert result is None
            mock_logger.warning.assert_called_once()
            assert "Failed to download model" in str(mock_logger.warning.call_args)

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_integration_full_download_flow(self, mock_get_models_dir, tmp_path):
        """Integration test: full flow from missing model to downloaded."""
        mock_get_models_dir.return_value = tmp_path

        fake_glb_data = b"realistic robot model GLB data"
        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=fake_glb_data
        )

        # Model doesn't exist yet
        assert not (tmp_path / "UniversalRobots_UR10e.glb").exists()

        result = await ensure_model_available("UniversalRobots_UR10e", mock_api_gateway)

        # Should return path to new file
        assert result == tmp_path / "UniversalRobots_UR10e.glb"

        # File should exist with correct content
        assert result.exists()
        assert result.read_bytes() == fake_glb_data

    @pytest.mark.asyncio
    @patch("nova_rerun_bridge.model_downloader.get_models_dir")
    async def test_integration_existing_model_not_redownloaded(
        self, mock_get_models_dir, tmp_path
    ):
        """Integration test: existing model should not be re-downloaded."""
        mock_get_models_dir.return_value = tmp_path

        # Pre-create model file
        model_file = tmp_path / "KUKA_KR10.glb"
        original_content = b"original model data"
        model_file.write_bytes(original_content)

        mock_api_gateway = Mock()
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model = AsyncMock(
            return_value=b"new data that should not be written"
        )

        result = await ensure_model_available("KUKA_KR10", mock_api_gateway)

        # Should return existing path
        assert result == model_file

        # API should not have been called
        mock_api_gateway.motion_group_models_api.get_motion_group_glb_model.assert_not_called()

        # File content should be unchanged
        assert model_file.read_bytes() == original_content
