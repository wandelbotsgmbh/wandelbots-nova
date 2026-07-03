"""Tests for the motion group model generation script.

Covers the pure conversion helpers (to_enum_value, generate_enum_source) and
an integration test that mocks the NOVA API and runs main() end-to-end.
"""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from wandelbots_api_client.v2_pydantic.exceptions import NotFoundException, ServiceException

from scripts.create_motion_group_models import generate_enum_source, main, to_enum_value


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _mock_nova(return_value=None, side_effect=None) -> MagicMock:
    """Create a mocked Nova context manager with a configurable API response."""
    mock = MagicMock()
    mock.__aenter__ = AsyncMock(return_value=mock)
    mock.__aexit__ = AsyncMock(return_value=False)
    mock.api.motion_group_models_api.get_motion_group_models = AsyncMock(
        return_value=return_value, side_effect=side_effect
    )
    return mock


# ---------------------------------------------------------------------------
# to_enum_value
# ---------------------------------------------------------------------------


class TestToEnumValue:
    def test_converts_valid_strings(self):
        assert to_enum_value("ABB_15000_0_95_5") == "abb-15000_0_95_5"
        assert to_enum_value("KUKA_KR250_R2700_2") == "kuka-kr250_r2700_2"
        assert to_enum_value("Yaskawa_AR1440") == "yaskawa-ar1440"

    def test_preserves_numbers(self):
        assert to_enum_value("FANUC_CRX10IA_L") == "fanuc-crx10ia_l"


# ---------------------------------------------------------------------------
# generate_enum_source
# ---------------------------------------------------------------------------


class TestGenerateEnumSource:
    def test_generates_valid_python(self):
        models = ["KUKA_KR16_R2010_2", "ABB_1200_07_7"]
        source = generate_enum_source(models)

        assert "class MotionGroupModel(StrEnum):" in source
        assert "from enum import StrEnum" in source
        assert 'ABB_1200_07_7 = "abb-1200_07_7"' in source
        assert 'KUKA_KR16_R2010_2 = "kuka-kr16_r2010_2"' in source

    def test_output_is_sorted(self):
        models = ["KUKA_KR16_R2010_2", "ABB_1200_07_7", "FANUC_CRX10IA_L"]
        source = generate_enum_source(models)

        abb_pos = source.index("ABB_1200_07_7")
        fanuc_pos = source.index("FANUC_CRX10IA_L")
        kuka_pos = source.index("KUKA_KR16_R2010_2")
        assert abb_pos < fanuc_pos < kuka_pos

    def test_empty_models_list(self):
        source = generate_enum_source([])
        assert "class MotionGroupModel(StrEnum):" in source
        assert '= "' not in source

    def test_contains_auto_generated_header(self):
        source = generate_enum_source(["KUKA_KR16_R2010_2"])
        assert "AUTO-GENERATED" in source


# ---------------------------------------------------------------------------
# Integration: mock the API and run main() end-to-end
# ---------------------------------------------------------------------------


class TestMainIntegration:
    @pytest.mark.asyncio
    async def test_main_writes_generated_file(self, tmp_path, caplog):
        output_file = tmp_path / "motion_group_models.py"
        mock = _mock_nova(return_value=["KUKA_KR16_R2010_2", "ABB_1200_07_7"])

        with (
            patch("scripts.create_motion_group_models.Nova", return_value=mock),
            patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file),
        ):
            await main()

        content = output_file.read_text()
        assert "class MotionGroupModel(StrEnum):" in content
        assert 'ABB_1200_07_7 = "abb-1200_07_7"' in content
        assert 'KUKA_KR16_R2010_2 = "kuka-kr16_r2010_2"' in content
        assert "Wrote 2 models" in caplog.text

    @pytest.mark.asyncio
    async def test_main_with_empty_api_response_does_not_write(self, tmp_path, caplog):
        output_file = tmp_path / "motion_group_models.py"
        mock = _mock_nova(return_value=[])

        with (
            patch("scripts.create_motion_group_models.Nova", return_value=mock),
            patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file),
        ):
            await main()

        assert not output_file.exists()
        assert "API returned no models" in caplog.text

    @pytest.mark.asyncio
    async def test_main_api_raises_exception(self, tmp_path):
        output_file = tmp_path / "motion_group_models.py"
        mock = _mock_nova(side_effect=RuntimeError("API unreachable"))

        with (
            patch("scripts.create_motion_group_models.Nova", return_value=mock),
            patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file),
        ):
            with pytest.raises(RuntimeError, match="API unreachable"):
                await main()

        assert not output_file.exists()

    @pytest.mark.asyncio
    async def test_main_api_returns_404(self, tmp_path):
        output_file = tmp_path / "motion_group_models.py"
        mock = _mock_nova(side_effect=NotFoundException(status=404, reason="Not found"))

        with (
            patch("scripts.create_motion_group_models.Nova", return_value=mock),
            patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file),
        ):
            with pytest.raises(NotFoundException):
                await main()

        assert not output_file.exists()

    @pytest.mark.asyncio
    async def test_main_api_returns_500(self, tmp_path):
        output_file = tmp_path / "motion_group_models.py"
        mock = _mock_nova(side_effect=ServiceException(status=500, reason="Internal server error"))

        with (
            patch("scripts.create_motion_group_models.Nova", return_value=mock),
            patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file),
        ):
            with pytest.raises(ServiceException):
                await main()

        assert not output_file.exists()


# ---------------------------------------------------------------------------
# Integration: real API (only runs in the pipeline)
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestMainIntegrationRealAPI:
    @pytest.mark.asyncio
    async def test_main_writes_generated_file_from_real_api(self, tmp_path):
        output_file = tmp_path / "motion_group_models.py"

        with patch("scripts.create_motion_group_models.OUTPUT_PATH", output_file):
            await main()

        content = output_file.read_text()
        assert "class MotionGroupModel(StrEnum):" in content
        # The real API should return at least one model
        assert '= "' in content
