import pytest
from pydantic import Field, ValidationError

import nova
from nova.program.function import ProgramContext, ProgramPreconditions


class _FakeApiClient:
    async def close(self):
        pass


class FakeNova:
    """Minimal Nova stand-in to avoid network calls in tests."""

    def __init__(self):
        self._api_client = _FakeApiClient()

    def is_connected(self) -> bool:  # pragma: no cover - simple passthrough
        return False


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

    # ctx is optional at call-site, similar to 'self'
    assert await sample(count=2) == 2
    assert await sample(count=2, repeat=True) == 4

    # ctx can be overridden for tests via nova=...
    assert await sample(count=3, nova=FakeNova()) == 3


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

    with pytest.raises(ValidationError):
        await only_value(value=1, extra="nope", nova=FakeNova())


@pytest.mark.asyncio
async def test_ctx_only_signature_allowed():
    @nova.program(
        name="ctx_only",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def ctx_only(ctx: ProgramContext):
        return ctx.nova is not None

    assert await ctx_only(nova=FakeNova()) is True


@pytest.mark.asyncio
async def test_no_extra_kwargs_when_no_inputs():
    @nova.program(
        name="ctx_only_no_inputs",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def ctx_only_no_inputs(ctx):
        return "ok"

    assert await ctx_only_no_inputs(nova=FakeNova()) == "ok"

    with pytest.raises(TypeError):
        await ctx_only_no_inputs(nova=FakeNova(), unexpected=1)


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
