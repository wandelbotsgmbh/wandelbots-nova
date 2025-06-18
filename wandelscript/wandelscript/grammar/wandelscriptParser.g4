/**
 * Wandelscript syntax grammar in ANTLR4 notation
 */

parser grammar wandelscriptParser;

options {
    tokenVocab = wandelscriptLexer;
}

program
  : (NEWLINE | statement)* EOF
  ;

identifier
  : NAME
  ;

suite
  : statement
  | NEWLINE INDENT statement+ DEDENT
  ;

statement
  : simpleStatement NEWLINE
  | complexStatement
  ;

simpleStatement
  : print_
  | stop
  | wait
  | motion
  | write
  | call
  | break_
  | return_
  | functionCall
  | assignment
  | raiseException
  | pass_
  | switchInterrupt
  ;

break_
  : BREAK
  ;

return_
  : RETURN result=expression
  ;

stop
  : STOP
  ;

pass_
  : PASS
  ;

motion
  : 'move' (frame_relation=frameRelation | tcp=expression)? ('via' via=connector)? 'to' end=expression (WITH modifier)?
  ;

switchInterrupt
  : action=ACTIVATE_OR_DEACTIVATE name=identifier
  ;

wait: WAIT duration=expression;
raiseException: RAISE exception=expression;

complexStatement
  : forLoop
  | repeatLoop
  | switch_
  | functionDefinition
  | moveDefinition
  | conditional
  | whileLoop
  | context
  | interrupt
  | robotContext
  | syncContext
  ;

whileLoop
  : WHILE condition=expression ':' body=suite
  ;

functionDefinition
  : DEF name=identifier '(' parameters=identifierList ')' ':' suite
  ;

moveDefinition
  : 'movedef' name=identifier '(' start=identifier '>-->' end=identifier
    (',' parameters=identifierList)? ')' ':' body=suite
  ;

forLoop
  : FOR name=identifier (IN | '=') range_ ':' body=suite
  ;

range_
  : start=expression '..' (interval_type='<')? end=expression
  ;

repeatLoop
  : REPEAT count=expression ':' body=suite
  ;

conditional
  : 'if' condition=expression ':' body=suite
    ('elif' elif_condition=expression ':' elif_body=suite)*
    ('else:' else_body=suite)?
  ;

switch_
  : 'switch' switch_expression=expression ':' NEWLINE
    ('case' case_expressions=expression ':' case_bodies=suite)*
    'default:' default_body=suite
  ;

interrupt
  : INTERRUPT name=identifier '(' parameters=identifierList ')'
    WHEN condition=identifier '(' arguments=expressionList ')' ':' body=suite
  ;

context
  : WITH modifier ':' body=suite
  ;

syncContext
  : ('do' ':' do_body=suite)? 'sync' ((':' sync_body=suite) | NEWLINE)
    ('except' ':' handler=suite)?
  ;

robotContext
  : ('do' 'with' robot=expression ':' do_body=suite) ('and' 'do' 'with' robot=expression ':' do_body=suite)*
  ;

unaryOperator
  : sign | inverse | not
  ;

expression
  : left=expression multiplicationOperator right=expression
  | left=expression additionOperator right=expression
  | left=expression comparisonOperator right=expression
  | left=expression logicalOperator right=expression
  |	atom
  ;

atom
  : functionCall
  | position
  | orientation
  | pose
  | float_
  | integer
  | bool_
  | string
  | read
  | call
  | array
  | record
  | unary
  | reference
  | frameRelation
  | atom '[' key=expression ']'    // Access using bracket notation
  | atom '.' property=identifier   // Access using dot notation
  | '(' assignment ')'
  | '(' expression ')'
  ;

unary
  : operation=unaryOperator operand=atom
  ;

functionCall
  : identifier '(' arguments=expressionList ')'
  ;

assignment
  : (frameRelation | (identifier  (',' identifier)*)) '=' value=expression
  ;

frameRelation
  : '[' target=expression '|' source=expression ']'
  ;

reference
  : identifier
  ;

float_
  : FLOAT
  ;

integer
  : INT
  ;

bool_
  : TRUE | FALSE
  ;

array
  : '[' (value=expression (',' value=expression)*)? ']'
  ;

keyValuePair
  : identifier ':' value=expression
  ;

record
  : '{' (value=keyValuePair (',' value=keyValuePair)* ','?)? '}'
  ;

pose
  : '(' position_=vector ',' orientation_=vector ')'
  ;

position
  : '(' vector ')'
  ;

orientation
  : '(' '...' ',' vector ')'
  ;

vector
  : x=expression ',' y=expression ',' z=expression
  ;

string
  : STRING
  ;

connector
  : name=identifier '(' arguments=expressionList ')'
  ;

print_
  : PRINT '(' text=expression ')'
  ;

read
  : READ '(' device=expression ',' key=expression ')'
  ;

write
  : WRITE '(' device=expression ',' key=expression ',' value=expression ')'
  ;

call
  : CALL '(' device=expression ',' key=expression ',' arguments=expressionList ')'
  ;

identifierList
  : (identifier (',' identifier)*)?
  ;

modifier
  : (functionCall (',' functionCall)*)?
  ;

expressionList
  : (expression (',' expression)*)?
  ;

additionOperator
  : ('+'|'-')
  ;

sign
  : ('+'|'-')
  ;

inverse
  : '~'
  ;

not
  : NOT
  ;

multiplicationOperator
  : ('*'|'/'|'::')
  ;

comparisonOperator
  : ('<'|'>'|'=='|'<='|'>='|'!=')
  ;

logicalOperator
  : AND | OR
  ;
