import pytest
from pydantic import BaseModel, Field, ValidationError

import nova
from nova.program.function import NovaProgramContext, ProgramPreconditions


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
async def test_inputs_accessible_via_ctx_inputs():
    class InputModel(BaseModel):
        count: int = Field(..., ge=1)

    @nova.program(
        name="sample_inputs",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=InputModel,
    )
    async def sample(inputs: InputModel, ctx: NovaProgramContext):
        return inputs.count

    result = await sample(count=3, nova=FakeNova())
    assert result == 3


@pytest.mark.asyncio
async def test_inputs_reject_extra_fields():
    class SimpleInput(BaseModel):
        value: int = Field(..., ge=0)

    @nova.program(
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=SimpleInput,
    )
    async def only_value(inputs: SimpleInput, ctx: NovaProgramContext):
        return inputs.value

    with pytest.raises(ValidationError):
        await only_value(value=1, extra="nope", nova=FakeNova())


@pytest.mark.asyncio
async def test_inputs_accepts_basemodel_instance():
    class Pair(BaseModel):
        left: int
        right: int

    pair_instance = Pair(left=2, right=5)

    @nova.program(
        name="pair_sum",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=Pair,
    )
    async def pair_sum(inputs: Pair, ctx: NovaProgramContext):
        return inputs.left + inputs.right

    result = await pair_sum(pair_instance, nova=FakeNova())
    assert result == 7


@pytest.mark.asyncio
async def test_ctx_only_signature_injected():
    @nova.program(
        name="ctx_only",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
    )
    async def ctx_only(ctx: NovaProgramContext):
        return ctx.nova is not None

    assert await ctx_only(nova=FakeNova()) is True


@pytest.mark.asyncio
async def test_inputs_and_ctx_keyword_only_order():
    class InputModel(BaseModel):
        value: int

    @nova.program(
        name="kw_only_ctx",
        preconditions=ProgramPreconditions(controllers=[], cleanup_controllers=False),
        input_model=InputModel,
    )
    async def kw_only_ctx(inputs: InputModel, *, ctx: NovaProgramContext | None = None):
        assert ctx is not None
        return inputs.value + 1

    assert await kw_only_ctx(value=2, nova=FakeNova()) == 3
