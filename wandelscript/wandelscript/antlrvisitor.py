import functools
import math

from antlr4 import CommonTokenStream, InputStream
from antlr4.error.ErrorListener import ErrorListener
from antlr4.error.Errors import ParseCancellationException

from wandelscript import metamodel, operators
from wandelscript.exception import ProgramSyntaxError, TextPosition, TextRange
from wandelscript.grammar.wandelscriptLexer import wandelscriptLexer
from wandelscript.grammar.wandelscriptParser import wandelscriptParser
from wandelscript.grammar.wandelscriptParserVisitor import wandelscriptParserVisitor
from wandelscript.metamodel import Rule
from wandelscript.utils.runtime import ensure_trailing_newline


def tracked(func):
    def wrapped(self, ctx, *args, **kwargs):
        result = func(self, ctx, *args, **kwargs)
        try:
            location = TextRange(
                TextPosition(ctx.start.line, ctx.start.column),
                TextPosition(ctx.stop.line, ctx.stop.column),
            )
        except:  # noqa: E722
            location = None
        if isinstance(result, Rule):
            result.set_location(location)
        else:
            result.location = location
        return result

    return wrapped


class Visitor(wandelscriptParserVisitor):  # pylint: disable=too-many-public-methods
    def visit(self, tree):
        result = tree.accept(self)
        assert result is not None
        return result

    def defaultResult(self):
        return []

    def aggregateResult(self, aggregate, nextResult):
        return aggregate.append(nextResult)

    @tracked
    def visitArray(self, ctx):
        elements = [self.visit(node) for node in ctx.expression()]
        return metamodel.Array(elements)

    @tracked
    def visitKeyValuePair(self, ctx):
        key = self.visit(ctx.identifier())
        value = self.visit(ctx.expression())
        return metamodel.KeyValuePair(key, value)

    @tracked
    def visitRecord(self, ctx):
        # Visit all key-value pairs within the object
        items = tuple(self.visit(pairCtx) for pairCtx in ctx.keyValuePair())
        return metamodel.Record(items)

    @tracked
    def visitBreak_(self, ctx):
        return metamodel.Break()

    @tracked
    def visitSwitchInterrupt(self, ctx):
        action = ctx.ACTIVATE_OR_DEACTIVATE().getText()
        name = self.visit(ctx.name)
        return metamodel.SwitchInterrupt(action, name)

    @tracked
    def visitWhileLoop(self, ctx):
        condition = self.visit(ctx.condition)
        body = self.visit(ctx.body)
        return metamodel.WhileLoop(condition, body)

    @tracked
    def visitPass_(self, ctx):
        return metamodel.Pass()

    @tracked
    def visitStop(self, ctx):
        return metamodel.Stop()

    @tracked
    def visitReturn_(self, ctx):
        return metamodel.Return(self.visit(ctx.result))

    @tracked
    def visitConditional(self, ctx):
        expression = map(self.visit, ctx.expression())
        suite = map(self.visit, ctx.suite())
        condition = next(expression)
        body = next(suite)
        elif_condition = list(expression)
        elif_body = list(suite)
        if len(elif_body) > len(elif_condition):
            *elif_body, else_body = elif_body
        else:
            else_body = None
        return metamodel.Conditional(
            condition=condition,
            body=body,
            elif_condition=elif_condition,
            elif_body=elif_body,
            else_body=else_body,
        )

    @tracked
    def visitSyncContext(self, ctx):
        do_body = self.visit(ctx.do_body) if ctx.do_body else None
        sync_body = self.visit(ctx.sync_body) if ctx.sync_body else None
        exception_handler = self.visit(ctx.handler) if ctx.handler else None
        return metamodel.SyncContext(
            do_body=do_body, sync_body=sync_body, exception_handler=exception_handler
        )

    @tracked
    def visitRobotContext(self, ctx):
        robots = tuple(map(self.visit, ctx.expression()))
        bodies = tuple(map(self.visit, ctx.suite()))
        return metamodel.RobotContext(robots=robots, bodies=bodies)

    @tracked
    def visitSwitch_(self, ctx):
        switch_expression, *case_expressions = map(self.visit, ctx.expression())
        (*case_bodies,) = map(self.visit, ctx.suite())
        if len(case_expressions) < len(case_bodies):
            *case_bodies, default_body = case_bodies
        else:
            default_body = None
        return metamodel.Switch(switch_expression, case_expressions, case_bodies, default_body)

    @tracked
    def visitWait(self, ctx):
        return metamodel.Wait(self.visit(ctx.duration))

    @tracked
    def visitRange_(self, ctx):
        return metamodel.Range(
            start=self.visit(ctx.start),
            interval_type=bool(ctx.interval_type),
            end=self.visit(ctx.end),
        )

    @tracked
    def visitForLoop(self, ctx):
        return metamodel.ForLoop(
            name=self.visit(ctx.name), range=self.visit(ctx.range_()), body=self.visit(ctx.body)
        )

    @tracked
    def visitInterrupt(self, ctx):
        name = self.visit(ctx.name)
        condition = self.visit(ctx.condition)
        arguments = self.visit(ctx.arguments)
        body = self.visit(ctx.body)
        parameters = self.visit(ctx.parameters)
        return metamodel.Interrupt(name, parameters, condition, arguments, body)

    @tracked
    def visitMotion(self, ctx):
        connector = self.visit(ctx.connector()) if ctx.connector() is not None else None
        end = self.visit(ctx.end)
        if ctx.modifier():
            modifier = self.visit(ctx.modifier())
        else:
            modifier = None
        tcp = self.visit(ctx.tcp) if ctx.tcp is not None else None
        fr = self.visit(ctx.frame_relation) if ctx.frame_relation is not None else None

        return metamodel.Motion(connector, end, modifier, tcp, fr)

    @tracked
    def visitModifier(self, ctx):
        return metamodel.Modifier([self.visit(node) for node in ctx.functionCall()])

    @tracked
    def visitContext(self, ctx):
        return metamodel.Context(modifier=self.visit(ctx.modifier()), body=self.visit(ctx.suite()))

    @tracked
    def visitReference(self, ctx):
        return metamodel.Reference(self.visit(ctx.identifier()))

    @tracked
    def visitSuite(self, ctx):
        return metamodel.Block([self.visit(node) for node in ctx.statement()])

    @tracked
    def visitProgram(self, ctx):
        statements = [self.visit(node) for node in ctx.statement()]
        block = metamodel.Block(statements)
        return metamodel.RootBlock(block)

    @tracked
    def visitBlock(self, ctx):
        return metamodel.Block(statements=list(map(self.visit, ctx.statement())))

    @tracked
    def visitFrameRelation(self, ctx):
        return metamodel.FrameRelation(self.visit(ctx.target), self.visit(ctx.source))

    def visitAlternative(self, ctx):
        it = ctx.getChildren()
        a = next(it)
        return self.visit(a)

    def visitAtom(self, ctx):
        node = (
            ctx.position()
            or ctx.orientation()
            or ctx.pose()
            or ctx.float_()
            or ctx.integer()
            or ctx.bool_()
            or ctx.string()
            or ctx.functionCall()
            or ctx.array()
            or ctx.record()
            or ctx.unary()
            or ctx.reference()
            or ctx.frameRelation()
            or ctx.assignment()
            or ctx.read()
            or ctx.call()
            or ctx.expression()
        )
        if ctx.key is not None:
            return metamodel.IndexAccess(self.visit(ctx.atom()), self.visit(ctx.expression()))
        if ctx.property_ is not None:
            return metamodel.PropertyAccess(self.visit(ctx.atom()), self.visit(ctx.property_))
        return self.visit(node)

    visitUnaryOperator = visitAlternative
    visitStatement = visitAlternative
    visitSimpleStatement = visitAlternative
    visitComplexStatement = visitAlternative
    visitConstant = visitAlternative

    def visitAssignmentStatement(self, ctx):
        return self.visit(ctx.assignment())

    @tracked
    def visitAssignment(self, ctx):
        names = tuple(self.visit(node) for node in ctx.identifier())
        value = self.visit(ctx.value)
        if not names:
            frame_relation = self.visit(ctx.frameRelation())
            return metamodel.Assignment(frame_relation, value)
        if len(names) == 1:
            return metamodel.Assignment(names[0], value)
        return metamodel.Assignment(names, value)

    def visitIdentifier(self, ctx):
        return ctx.getText()

    @tracked
    def visitInverse(self, ctx):
        return metamodel.Inverse.inv

    @tracked
    def visitNot(self, ctx):
        return metamodel.Not.not_

    @tracked
    def visitRaiseException(self, ctx):
        return metamodel.RaiseException(self.visit(ctx.exception))

    @tracked
    def visitMoveDefinition(self, ctx):
        name = self.visit(ctx.name)
        start = self.visit(ctx.start)
        end = self.visit(ctx.end)
        if ctx.parameters:
            parameters = self.visit(ctx.parameters)
        else:
            parameters = metamodel.Parameters([])
        body = self.visit(ctx.body)
        return metamodel.MoveDefinition(name, start, end, body, parameters)

    @tracked
    def visitConnector(self, ctx):
        name = self.visit(ctx.identifier())
        arguments = self.visit(ctx.arguments)
        return metamodel.Connector(name, arguments.data)

    def visitExpression(self, ctx):
        atom = ctx.atom()
        if atom is not None:
            return self.visit(atom)
        left = self.visit(ctx.left)
        right = self.visit(ctx.right)
        selected_node = (
            ctx.multiplicationOperator()
            or ctx.additionOperator()
            or ctx.comparisonOperator()
            or ctx.logicalOperator()
        )
        operation = self.visit(selected_node)
        return operation(left, right)

    def visitSign(self, ctx):
        return operators.Sign(ctx.getText())

    def visitConstantAtom(self, ctx):
        return self.visit(ctx.constant())

    @tracked
    def visitInteger(self, ctx):
        return metamodel.ConstantInt(int(ctx.INT().getText()))

    @tracked
    def visitString(self, ctx):
        return metamodel.String(ctx.STRING().getText()[1:-1])

    @tracked
    def visitFloat_(self, ctx):
        string = ctx.FLOAT().getText()
        return metamodel.ConstantFloat(float(string) if string != "pi" else math.pi)

    @tracked
    def visitBool_(self, ctx):
        value = {"True": True, "False": False}[ctx.getText()]
        return metamodel.Bool(value)

    @tracked
    def visitPose(self, ctx):
        position = self.visit(ctx.position_)
        orientation = self.visit(ctx.orientation_)
        return metamodel.ExpressionsList(position + orientation)

    @tracked
    def visitPosition(self, ctx):
        position = self.visit(ctx.vector())
        return metamodel.ExpressionsList(position)

    def visitUnary(self, ctx):
        operation = self.visit(ctx.operation)
        operand = self.visit(ctx.operand)
        return operation(operand)

    def visitVector(self, ctx):
        return [self.visit(ctx.x), self.visit(ctx.y), self.visit(ctx.z)]

    @tracked
    def visitRepeatLoop(self, ctx):
        count = self.visit(ctx.expression())
        body = self.visit(ctx.suite())
        return metamodel.RepeatLoop(count, body)

    @tracked
    def visitRead(self, ctx):
        identifier = self.visit(ctx.device)
        key = self.visit(ctx.key)
        return metamodel.Read(identifier, key)

    @tracked
    def visitWrite(self, ctx):
        sensor = self.visit(ctx.device)
        key = self.visit(ctx.key)
        value = self.visit(ctx.value)
        return metamodel.Write(sensor, key, value)

    @tracked
    def visitCall(self, ctx):
        identifier = self.visit(ctx.device)
        key = self.visit(ctx.key)
        arguments = self.visit(ctx.arguments)
        return metamodel.Call(identifier, key, arguments)

    def visitAdditionOperator(self, ctx):
        return operators.AdditionOperator(ctx.getText())

    def visitMultiplicationOperator(self, ctx):
        return operators.MultiplicationOperator(ctx.getText())

    def visitComparisonOperator(self, ctx):
        return operators.ComparisonOperator(ctx.getText())

    def visitLogicalOperator(self, ctx):
        return operators.LogicalOperator(ctx.getText())

    @tracked
    def visitIdentifierList(self, ctx):
        return metamodel.Parameters(list(map(self.visit, ctx.identifier())))

    @tracked
    def visitExpressionList(self, ctx):
        return metamodel.Arguments(list(map(self.visit, ctx.expression())))

    @tracked
    def visitFunctionDefinition(self, ctx):
        name = self.visit(ctx.identifier())
        parameters = self.visit(ctx.identifierList())
        body = self.visit(ctx.suite())
        return metamodel.FunctionDefinition(name, parameters, body)

    @tracked
    def visitFunctionCall(self, ctx):
        name = self.visit(ctx.identifier())
        arguments = self.visit(ctx.arguments)
        return metamodel.FunctionCall(name, arguments)

    @tracked
    def visitPrint_(self, ctx):
        return metamodel.Print(self.visit(ctx.text))


class ThrowingErrorListener(ErrorListener):
    def __init__(self, code):
        super().__init__()
        self.code = code.split("\n")

    def syntaxError(self, recognizer, offendingSymbol, line, column, msg, e):  # pylint: disable=too-many-positional-arguments
        ex = ParseCancellationException("")
        ex.line = line
        ex.column = column
        ex.message = msg
        raise ex


@functools.cache
def parse_code(code: str):
    try:
        preprocessed_code = ensure_trailing_newline(code)
        lexer = wandelscriptLexer(InputStream(preprocessed_code))
        lexer.removeErrorListeners()
        lexer.addErrorListener(ThrowingErrorListener(code))
        stream = CommonTokenStream(lexer)
        parser = wandelscriptParser(stream)
        parser.removeErrorListeners()
        parser.addErrorListener(ThrowingErrorListener(code))
        tree = parser.program()
        model = Visitor().visit(tree)
        result = metamodel.Program(body=model)
    except ParseCancellationException as error:
        location = TextRange(
            TextPosition(error.line, error.column), TextPosition(error.line, error.column + 1)
        )
        raise ProgramSyntaxError(location=location, text=error.message) from error
    return result


metamodel.Program.from_code = staticmethod(parse_code)  # type: ignore
