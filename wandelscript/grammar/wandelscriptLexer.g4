/**
 * Wandelscript syntax grammar in ANTLR4 notation
 */

lexer grammar wandelscriptLexer;

tokens  { INDENT, DEDENT }

options {
    superClass = WandelscriptLexerBase;
}

OPEN_PARENS:          '(' {self.open_bracket()};
CLOSE_PARENS:         ')' {self.close_bracket()};
OPEN_BRACKET:         '[' {self.open_bracket()};
CLOSE_BRACKET:        ']' {self.close_bracket()};
OPEN_BRACE:           '{' {self.open_bracket()};
CLOSE_BRACE:          '}' {self.close_bracket()};
DOT:                  '.';
COMMA:                ',';
COLON:                ':';
PLUS:                 '+';
MINUS:                '-';
STAR:                 '*';
DIV:                  '/';
BITWISE_OR:           '|';
TILDE:                '~';
LT:                   '<';
GT:                   '>';
OP_ASSIGNMENT:        '=';
DOUBLE_COLON:         '::';
OP_EQ:                '==';
OP_NE:                '!=';
OP_LE:                '<=';
OP_GE:                '>=';
OP_RANGE:             '..';
ORIENTATION_ELLIPSIS: '...';

MOVE: 'move';
VIA: 'via';
TO: 'to';

MOVEDEF: 'movedef';
MOVEDEV_ARROW: '>-->';

DEF: 'def';
RETURN: 'return';
PASS: 'pass';

IF: 'if';
ELIF: 'elif';
ELSE: 'else:';

SWITCH: 'switch';
CASE: 'case';
DEFAULT: 'default:';

WITH: 'with';

FOR: 'for';
IN: 'in';

REPEAT: 'repeat';
WHILE: 'while';

BREAK: 'break';
STOP: 'stop';

NOT: 'not';

AND: 'and';
OR: 'or';

DO: 'do';
SYNC: 'sync';
EXCEPT: 'except';

INTERRUPT: 'interrupt';
WHEN: 'when';

ACTIVATE_OR_DEACTIVATE: ('activate' | 'deactivate');

WAIT: 'wait';
RAISE: 'raise';

PRINT: 'print';

READ: 'read';
WRITE: 'write';
CALL: 'call';

TRUE: 'True';
FALSE: 'False';
INT: [0-9]+;
FLOAT: ([0-9]+ '.' [0-9]+) | 'pi' | 'inf';
STRING: '\'' ( ~[\\\r\n\f'] )* '\'' | '"' (  ~[\\\r\n\f"] )* '"';

// this rule must be after all tokens matching keywords because it would also match these
NAME: ([a-z] | [A-Z] | '_') ([a-z] | [A-Z] | [0-9] | '_')*;

NEWLINE
  : ( {self.at_start_of_input()}? WS
    | ('\r'? '\n' | '\r' | '\f') WS?
    )
    {self.on_newline()}
  ;

SKIP_: ( WS | COMMENT ) -> channel(HIDDEN);
fragment WS: [ \t]+;
fragment COMMENT: '#' ~[\r\n\f]*;
