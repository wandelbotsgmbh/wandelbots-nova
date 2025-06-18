from dataclasses import dataclass

from pydantic import BaseModel

from wandelscript.ffi import foreign_function, is_foreign_function


class CustomTypePydantic(BaseModel):
    str_attr: str
    float_attr: float


@dataclass
class CustomTypeDataclass:
    str_attr: str
    int_attr: int


def test_decorator_autoconversion():
    @foreign_function()
    def auto_convert_pydantic(data: CustomTypePydantic, test_data: dict):
        assert isinstance(data, CustomTypePydantic)
        assert isinstance(test_data, dict)
        for key, value in test_data.items():
            assert hasattr(data, key)
            assert getattr(data, key) == value
        return data

    @foreign_function()
    def auto_convert_dataclass(data: CustomTypeDataclass, test_data: dict):
        assert isinstance(data, CustomTypeDataclass)
        assert isinstance(test_data, dict)
        for key, value in test_data.items():
            assert hasattr(data, key)
            assert getattr(data, key) == value
        return data

    @foreign_function()
    def auto_convert_list(data: list[CustomTypePydantic]):
        assert isinstance(data, list)
        for item in data:
            assert isinstance(item, CustomTypePydantic)
        return data

    assert is_foreign_function(auto_convert_pydantic)

    result = auto_convert_pydantic(*[dict(str_attr="test", float_attr=1.0)] * 2)
    assert isinstance(result, dict)
    assert is_foreign_function(auto_convert_dataclass)

    result = auto_convert_dataclass(*[dict(str_attr="test", int_attr=42)] * 2)
    assert isinstance(result, dict)

    result = auto_convert_list(
        [CustomTypePydantic(str_attr=f"test {i}", float_attr=1.0 * i) for i in range(3)]
    )
    assert isinstance(result, tuple)  # no lists in Wandelscript
    assert len(result) == 3
    for item in result:
        assert isinstance(item, dict)
        for field in CustomTypePydantic.model_fields:
            assert field in item


def test_decorator_no_autoconversion():
    @foreign_function(autoconvert_types=False)
    def no_auto_convert(data: CustomTypePydantic):
        assert not isinstance(data, CustomTypePydantic)
        return data  # type: ignore # mypy thinks this is unreachable because of the type assertion above

    assert is_foreign_function(no_auto_convert)
    result = no_auto_convert(dict(str_attr="test", float_attr=1.0))
    assert isinstance(result, dict)


# check if the function has bears the attribute _wandelscript_foreign_function and is a ForeignFunction
