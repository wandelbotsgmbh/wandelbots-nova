import pytest

from wandelscript.antlrvisitor import parse_code
from wandelscript.exception import ProgramSyntaxError


def test_parse_error():
    code = """a = 0
1"""
    with pytest.raises(ProgramSyntaxError) as error:
        parse_code(code)
    assert error.value.location.start.line == 2
    assert error.value.location.start.column == 0


def test_parse_error_ident():
    code = "   a = [0, 1, 2] + [0, 0, 3]\na=1\n  a=1"
    with pytest.raises(ProgramSyntaxError) as error:
        parse_code(code)
    print(error.value.message)
    assert error.value.location.start.line == 1
    assert error.value.location.start.column == 3
