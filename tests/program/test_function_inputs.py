from unittest.mock import Mock

import pytest
from pydantic import Field, ValidationError

import nova
from nova.cell import Cell
from nova.program.function import ProgramContext, ProgramPreconditions


class _FakeApiClient:
    def __init__(self):
        self.is_connected = False


class FakeNova:
    """Minimal Nova stand-in to avoid network calls in tests."""

    def __init__(self):
        self._api_client = _FakeApiClient()
        self._is_connected = False

    def is_connected(self):
        return self._is_connected

    async def open(self):
        self._is_connected = True

    async def close(self):
        self._is_connected = False

    def cell(self):  # pragma: no cover - simple stub
        """Return a fake Cell-like object for ProgramContext."""
        return Mock(spec=Cell, id="test-cell-1")


@pytest.mark.asyncio
async def test_ctx_first_and_implicit_on_call():
    @nova.program(
        name="sample_inputs",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def sample(ctx, count: int, repeat: bool = False):
        # ctx should always be injected and available
        assert isinstance(ctx, ProgramContext)
        return count if not repeat else count * 2

    # Programs must be executed with a connected Nova instance when called directly.
    connected_nova = FakeNova()
    await connected_nova.open()
    assert await sample(count=2, nova=connected_nova) == 2
    assert await sample(count=2, repeat=True, nova=connected_nova) == 4

    # ctx can be overridden for tests via nova=...
    connected_nova2 = FakeNova()
    await connected_nova2.open()
    assert await sample(count=3, nova=connected_nova2) == 3


@pytest.mark.asyncio
async def test_missing_ctx_parameter_raises():
    with pytest.raises(TypeError):

        @nova.program(
            name="no_ctx",
            preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        )
        async def no_ctx(count: int):  # type: ignore[unused-ignore]
            return count


@pytest.mark.asyncio
async def test_extra_fields_rejected_by_input_model():
    @nova.program(preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False))
    async def only_value(ctx, value: int = Field(..., ge=0)):
        assert isinstance(ctx, ProgramContext)
        return value

    with pytest.raises(Exception):
        connected_nova = FakeNova()
        await connected_nova.open()
        await only_value(value=1, extra="nope", nova=connected_nova)


@pytest.mark.asyncio
async def test_ctx_only_signature_allowed():
    @nova.program(
        name="ctx_only",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def ctx_only(ctx: ProgramContext):
        return ctx.nova is not None

    connected_nova = FakeNova()
    await connected_nova.open()
    assert await ctx_only(nova=connected_nova) is True


@pytest.mark.asyncio
async def test_no_extra_kwargs_when_no_inputs():
    @nova.program(
        name="ctx_only_no_inputs",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def ctx_only_no_inputs(ctx):
        return "ok"

    connected_nova = FakeNova()
    await connected_nova.open()
    assert await ctx_only_no_inputs(nova=connected_nova) == "ok"

    with pytest.raises(TypeError):
        connected_nova2 = FakeNova()
        await connected_nova2.open()
        await ctx_only_no_inputs(nova=connected_nova2, unexpected=1)


@pytest.mark.asyncio
async def test_missing_nova_or_ctx_raises_helpful_error():
    @nova.program(
        name="needs_nova",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def needs_nova(ctx, value: int):
        return value

    with pytest.raises(RuntimeError) as excinfo:
        await needs_nova(value=1)

    assert "run_program" in str(excinfo.value)


@pytest.mark.asyncio
async def test_positional_args_not_allowed():
    @nova.program(
        name="positional_not_allowed",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def positional_not_allowed(ctx, value: int):
        return value

    with pytest.raises(TypeError):
        await positional_not_allowed(1)
