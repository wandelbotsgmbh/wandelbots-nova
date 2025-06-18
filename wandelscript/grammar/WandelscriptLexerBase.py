import re
import sys
from typing import TextIO

from antlr4 import InputStream, Lexer
from antlr4.Token import CommonToken

from .wandelscriptParser import wandelscriptParser


def get_indentation_count(whitespace: str):
    count = 0
    for c in whitespace:
        if c == "\t":
            count += 8 - count % 8
        else:
            count += 1
    return count


class WandelscriptLexerBase(Lexer):
    NEWLINE_MATCHER = re.compile("[\r\n\f]+")
    NOT_NEWLINE_MATCHER = re.compile("[^\r\n\f]+")

    def __init__(self, input_stream: InputStream, output: TextIO = sys.stdout):
        super().__init__(input_stream, output)
        self.tokens = []
        self.indents = []
        self.opened = 0

    def reset(self):
        self.tokens = []
        self.indents = []
        self.opened = 0
        super().reset()

    def emitToken(self, token):
        self._token = token
        self.tokens.append(token)

    def nextToken(self):
        # Check if the end-of-file is ahead and there are still some DEDENTS expected.
        if self._input.LA(1) == wandelscriptParser.EOF and len(self.indents) != 0:
            # Remove any trailing EOF tokens from our buffer.
            self.tokens = [token for token in self.tokens if token.type != wandelscriptParser.EOF]

            # First emit an extra line break that serves as the end of the statement.
            self.emitToken(self.common_token(wandelscriptParser.NEWLINE, "\n"))

            # Now emit as much DEDENT tokens as needed.
            while len(self.indents) != 0:
                self.emitToken(self.create_dedent())
                self.indents.pop()

            # Put the EOF back on the token stream.
            self.emitToken(self.common_token(wandelscriptParser.EOF, "<EOF>"))

        next_ = super().nextToken()
        return next_ if len(self.tokens) == 0 else self.tokens.pop(0)

    def create_dedent(self):
        return self.common_token(wandelscriptParser.DEDENT, "")

    def common_token(self, type_: int, text: str):
        stop = self.getCharIndex() - 1
        start = stop if text == "" else stop - len(text) + 1
        return CommonToken(
            self._tokenFactorySourcePair, type_, Lexer.DEFAULT_TOKEN_CHANNEL, start, stop
        )

    def at_start_of_input(self):
        return self.getCharIndex() == 0

    def open_bracket(self):
        self.opened += 1

    def close_bracket(self):
        self.opened -= 1

    def on_newline(self):
        # the NEWLINE lexer rule captures WS after the newline char (char sequence)
        new_line = self.NOT_NEWLINE_MATCHER.sub("", self.text)
        spaces = self.NEWLINE_MATCHER.sub("", self.text)

        # Strip newlines inside open clauses except if we are near EOF. We keep NEWLINEs near EOF to
        # satisfy the final newline needed by the single_put rule used by the REPL.
        next_ = self._input.LA(1)
        next_next = self._input.LA(2)

        (LF, CR, COMMENT) = map(ord, "\n\r#")  # (10, 13, 35)
        if self.opened > 0 or (next_next != wandelscriptParser.EOF and next_ in (LF, CR, COMMENT)):
            self.skip()
        else:
            self.emitToken(self.common_token(wandelscriptParser.NEWLINE, new_line))
            indent = get_indentation_count(spaces)
            previous = 0 if len(self.indents) == 0 else self.indents[-1]

            if indent == previous:
                self.skip()
            elif indent > previous:
                self.indents.append(indent)
                self.emitToken(self.common_token(wandelscriptParser.INDENT, spaces))
            else:
                while len(self.indents) > 0 and self.indents[-1] > indent:
                    self.emitToken(self.create_dedent())
                    self.indents.pop()
