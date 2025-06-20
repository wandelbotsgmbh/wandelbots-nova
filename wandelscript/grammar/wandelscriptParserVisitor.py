# Generated from wandelscriptParser.g4 by ANTLR 4.13.2
from antlr4 import *

if "." in __name__:
    from .wandelscriptParser import wandelscriptParser
else:
    from wandelscriptParser import wandelscriptParser

# This class defines a complete generic visitor for a parse tree produced by wandelscriptParser.


class wandelscriptParserVisitor(ParseTreeVisitor):
    # Visit a parse tree produced by wandelscriptParser#program.
    def visitProgram(self, ctx: wandelscriptParser.ProgramContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#identifier.
    def visitIdentifier(self, ctx: wandelscriptParser.IdentifierContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#suite.
    def visitSuite(self, ctx: wandelscriptParser.SuiteContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#statement.
    def visitStatement(self, ctx: wandelscriptParser.StatementContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#simpleStatement.
    def visitSimpleStatement(self, ctx: wandelscriptParser.SimpleStatementContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#break_.
    def visitBreak_(self, ctx: wandelscriptParser.Break_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#return_.
    def visitReturn_(self, ctx: wandelscriptParser.Return_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#stop.
    def visitStop(self, ctx: wandelscriptParser.StopContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#pass_.
    def visitPass_(self, ctx: wandelscriptParser.Pass_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#motion.
    def visitMotion(self, ctx: wandelscriptParser.MotionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#switchInterrupt.
    def visitSwitchInterrupt(self, ctx: wandelscriptParser.SwitchInterruptContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#wait.
    def visitWait(self, ctx: wandelscriptParser.WaitContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#raiseException.
    def visitRaiseException(self, ctx: wandelscriptParser.RaiseExceptionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#complexStatement.
    def visitComplexStatement(self, ctx: wandelscriptParser.ComplexStatementContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#whileLoop.
    def visitWhileLoop(self, ctx: wandelscriptParser.WhileLoopContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#functionDefinition.
    def visitFunctionDefinition(self, ctx: wandelscriptParser.FunctionDefinitionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#moveDefinition.
    def visitMoveDefinition(self, ctx: wandelscriptParser.MoveDefinitionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#forLoop.
    def visitForLoop(self, ctx: wandelscriptParser.ForLoopContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#range_.
    def visitRange_(self, ctx: wandelscriptParser.Range_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#repeatLoop.
    def visitRepeatLoop(self, ctx: wandelscriptParser.RepeatLoopContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#conditional.
    def visitConditional(self, ctx: wandelscriptParser.ConditionalContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#switch_.
    def visitSwitch_(self, ctx: wandelscriptParser.Switch_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#interrupt.
    def visitInterrupt(self, ctx: wandelscriptParser.InterruptContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#context.
    def visitContext(self, ctx: wandelscriptParser.ContextContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#syncContext.
    def visitSyncContext(self, ctx: wandelscriptParser.SyncContextContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#robotContext.
    def visitRobotContext(self, ctx: wandelscriptParser.RobotContextContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#unaryOperator.
    def visitUnaryOperator(self, ctx: wandelscriptParser.UnaryOperatorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#expression.
    def visitExpression(self, ctx: wandelscriptParser.ExpressionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#atom.
    def visitAtom(self, ctx: wandelscriptParser.AtomContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#unary.
    def visitUnary(self, ctx: wandelscriptParser.UnaryContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#functionCall.
    def visitFunctionCall(self, ctx: wandelscriptParser.FunctionCallContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#assignment.
    def visitAssignment(self, ctx: wandelscriptParser.AssignmentContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#frameRelation.
    def visitFrameRelation(self, ctx: wandelscriptParser.FrameRelationContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#reference.
    def visitReference(self, ctx: wandelscriptParser.ReferenceContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#float_.
    def visitFloat_(self, ctx: wandelscriptParser.Float_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#integer.
    def visitInteger(self, ctx: wandelscriptParser.IntegerContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#bool_.
    def visitBool_(self, ctx: wandelscriptParser.Bool_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#array.
    def visitArray(self, ctx: wandelscriptParser.ArrayContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#keyValuePair.
    def visitKeyValuePair(self, ctx: wandelscriptParser.KeyValuePairContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#record.
    def visitRecord(self, ctx: wandelscriptParser.RecordContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#pose.
    def visitPose(self, ctx: wandelscriptParser.PoseContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#position.
    def visitPosition(self, ctx: wandelscriptParser.PositionContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#orientation.
    def visitOrientation(self, ctx: wandelscriptParser.OrientationContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#vector.
    def visitVector(self, ctx: wandelscriptParser.VectorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#string.
    def visitString(self, ctx: wandelscriptParser.StringContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#connector.
    def visitConnector(self, ctx: wandelscriptParser.ConnectorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#print_.
    def visitPrint_(self, ctx: wandelscriptParser.Print_Context):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#read.
    def visitRead(self, ctx: wandelscriptParser.ReadContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#write.
    def visitWrite(self, ctx: wandelscriptParser.WriteContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#call.
    def visitCall(self, ctx: wandelscriptParser.CallContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#identifierList.
    def visitIdentifierList(self, ctx: wandelscriptParser.IdentifierListContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#modifier.
    def visitModifier(self, ctx: wandelscriptParser.ModifierContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#expressionList.
    def visitExpressionList(self, ctx: wandelscriptParser.ExpressionListContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#additionOperator.
    def visitAdditionOperator(self, ctx: wandelscriptParser.AdditionOperatorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#sign.
    def visitSign(self, ctx: wandelscriptParser.SignContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#inverse.
    def visitInverse(self, ctx: wandelscriptParser.InverseContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#not.
    def visitNot(self, ctx: wandelscriptParser.NotContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#multiplicationOperator.
    def visitMultiplicationOperator(self, ctx: wandelscriptParser.MultiplicationOperatorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#comparisonOperator.
    def visitComparisonOperator(self, ctx: wandelscriptParser.ComparisonOperatorContext):
        return self.visitChildren(ctx)

    # Visit a parse tree produced by wandelscriptParser#logicalOperator.
    def visitLogicalOperator(self, ctx: wandelscriptParser.LogicalOperatorContext):
        return self.visitChildren(ctx)


del wandelscriptParser
