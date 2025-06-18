# Generated from wandelscriptParser.g4 by ANTLR 4.13.2
from antlr4 import *

if "." in __name__:
    from .wandelscriptParser import wandelscriptParser
else:
    from wandelscriptParser import wandelscriptParser


# This class defines a complete listener for a parse tree produced by wandelscriptParser.
class wandelscriptParserListener(ParseTreeListener):
    # Enter a parse tree produced by wandelscriptParser#program.
    def enterProgram(self, ctx: wandelscriptParser.ProgramContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#program.
    def exitProgram(self, ctx: wandelscriptParser.ProgramContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#identifier.
    def enterIdentifier(self, ctx: wandelscriptParser.IdentifierContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#identifier.
    def exitIdentifier(self, ctx: wandelscriptParser.IdentifierContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#suite.
    def enterSuite(self, ctx: wandelscriptParser.SuiteContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#suite.
    def exitSuite(self, ctx: wandelscriptParser.SuiteContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#statement.
    def enterStatement(self, ctx: wandelscriptParser.StatementContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#statement.
    def exitStatement(self, ctx: wandelscriptParser.StatementContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#simpleStatement.
    def enterSimpleStatement(self, ctx: wandelscriptParser.SimpleStatementContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#simpleStatement.
    def exitSimpleStatement(self, ctx: wandelscriptParser.SimpleStatementContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#break_.
    def enterBreak_(self, ctx: wandelscriptParser.Break_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#break_.
    def exitBreak_(self, ctx: wandelscriptParser.Break_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#return_.
    def enterReturn_(self, ctx: wandelscriptParser.Return_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#return_.
    def exitReturn_(self, ctx: wandelscriptParser.Return_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#stop.
    def enterStop(self, ctx: wandelscriptParser.StopContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#stop.
    def exitStop(self, ctx: wandelscriptParser.StopContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#pass_.
    def enterPass_(self, ctx: wandelscriptParser.Pass_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#pass_.
    def exitPass_(self, ctx: wandelscriptParser.Pass_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#motion.
    def enterMotion(self, ctx: wandelscriptParser.MotionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#motion.
    def exitMotion(self, ctx: wandelscriptParser.MotionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#switchInterrupt.
    def enterSwitchInterrupt(self, ctx: wandelscriptParser.SwitchInterruptContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#switchInterrupt.
    def exitSwitchInterrupt(self, ctx: wandelscriptParser.SwitchInterruptContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#wait.
    def enterWait(self, ctx: wandelscriptParser.WaitContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#wait.
    def exitWait(self, ctx: wandelscriptParser.WaitContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#raiseException.
    def enterRaiseException(self, ctx: wandelscriptParser.RaiseExceptionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#raiseException.
    def exitRaiseException(self, ctx: wandelscriptParser.RaiseExceptionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#complexStatement.
    def enterComplexStatement(self, ctx: wandelscriptParser.ComplexStatementContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#complexStatement.
    def exitComplexStatement(self, ctx: wandelscriptParser.ComplexStatementContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#whileLoop.
    def enterWhileLoop(self, ctx: wandelscriptParser.WhileLoopContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#whileLoop.
    def exitWhileLoop(self, ctx: wandelscriptParser.WhileLoopContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#functionDefinition.
    def enterFunctionDefinition(self, ctx: wandelscriptParser.FunctionDefinitionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#functionDefinition.
    def exitFunctionDefinition(self, ctx: wandelscriptParser.FunctionDefinitionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#moveDefinition.
    def enterMoveDefinition(self, ctx: wandelscriptParser.MoveDefinitionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#moveDefinition.
    def exitMoveDefinition(self, ctx: wandelscriptParser.MoveDefinitionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#forLoop.
    def enterForLoop(self, ctx: wandelscriptParser.ForLoopContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#forLoop.
    def exitForLoop(self, ctx: wandelscriptParser.ForLoopContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#range_.
    def enterRange_(self, ctx: wandelscriptParser.Range_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#range_.
    def exitRange_(self, ctx: wandelscriptParser.Range_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#repeatLoop.
    def enterRepeatLoop(self, ctx: wandelscriptParser.RepeatLoopContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#repeatLoop.
    def exitRepeatLoop(self, ctx: wandelscriptParser.RepeatLoopContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#conditional.
    def enterConditional(self, ctx: wandelscriptParser.ConditionalContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#conditional.
    def exitConditional(self, ctx: wandelscriptParser.ConditionalContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#switch_.
    def enterSwitch_(self, ctx: wandelscriptParser.Switch_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#switch_.
    def exitSwitch_(self, ctx: wandelscriptParser.Switch_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#interrupt.
    def enterInterrupt(self, ctx: wandelscriptParser.InterruptContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#interrupt.
    def exitInterrupt(self, ctx: wandelscriptParser.InterruptContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#context.
    def enterContext(self, ctx: wandelscriptParser.ContextContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#context.
    def exitContext(self, ctx: wandelscriptParser.ContextContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#syncContext.
    def enterSyncContext(self, ctx: wandelscriptParser.SyncContextContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#syncContext.
    def exitSyncContext(self, ctx: wandelscriptParser.SyncContextContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#robotContext.
    def enterRobotContext(self, ctx: wandelscriptParser.RobotContextContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#robotContext.
    def exitRobotContext(self, ctx: wandelscriptParser.RobotContextContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#unaryOperator.
    def enterUnaryOperator(self, ctx: wandelscriptParser.UnaryOperatorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#unaryOperator.
    def exitUnaryOperator(self, ctx: wandelscriptParser.UnaryOperatorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#expression.
    def enterExpression(self, ctx: wandelscriptParser.ExpressionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#expression.
    def exitExpression(self, ctx: wandelscriptParser.ExpressionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#atom.
    def enterAtom(self, ctx: wandelscriptParser.AtomContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#atom.
    def exitAtom(self, ctx: wandelscriptParser.AtomContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#unary.
    def enterUnary(self, ctx: wandelscriptParser.UnaryContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#unary.
    def exitUnary(self, ctx: wandelscriptParser.UnaryContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#functionCall.
    def enterFunctionCall(self, ctx: wandelscriptParser.FunctionCallContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#functionCall.
    def exitFunctionCall(self, ctx: wandelscriptParser.FunctionCallContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#assignment.
    def enterAssignment(self, ctx: wandelscriptParser.AssignmentContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#assignment.
    def exitAssignment(self, ctx: wandelscriptParser.AssignmentContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#frameRelation.
    def enterFrameRelation(self, ctx: wandelscriptParser.FrameRelationContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#frameRelation.
    def exitFrameRelation(self, ctx: wandelscriptParser.FrameRelationContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#reference.
    def enterReference(self, ctx: wandelscriptParser.ReferenceContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#reference.
    def exitReference(self, ctx: wandelscriptParser.ReferenceContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#float_.
    def enterFloat_(self, ctx: wandelscriptParser.Float_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#float_.
    def exitFloat_(self, ctx: wandelscriptParser.Float_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#integer.
    def enterInteger(self, ctx: wandelscriptParser.IntegerContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#integer.
    def exitInteger(self, ctx: wandelscriptParser.IntegerContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#bool_.
    def enterBool_(self, ctx: wandelscriptParser.Bool_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#bool_.
    def exitBool_(self, ctx: wandelscriptParser.Bool_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#array.
    def enterArray(self, ctx: wandelscriptParser.ArrayContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#array.
    def exitArray(self, ctx: wandelscriptParser.ArrayContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#keyValuePair.
    def enterKeyValuePair(self, ctx: wandelscriptParser.KeyValuePairContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#keyValuePair.
    def exitKeyValuePair(self, ctx: wandelscriptParser.KeyValuePairContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#record.
    def enterRecord(self, ctx: wandelscriptParser.RecordContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#record.
    def exitRecord(self, ctx: wandelscriptParser.RecordContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#pose.
    def enterPose(self, ctx: wandelscriptParser.PoseContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#pose.
    def exitPose(self, ctx: wandelscriptParser.PoseContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#position.
    def enterPosition(self, ctx: wandelscriptParser.PositionContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#position.
    def exitPosition(self, ctx: wandelscriptParser.PositionContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#orientation.
    def enterOrientation(self, ctx: wandelscriptParser.OrientationContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#orientation.
    def exitOrientation(self, ctx: wandelscriptParser.OrientationContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#vector.
    def enterVector(self, ctx: wandelscriptParser.VectorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#vector.
    def exitVector(self, ctx: wandelscriptParser.VectorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#string.
    def enterString(self, ctx: wandelscriptParser.StringContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#string.
    def exitString(self, ctx: wandelscriptParser.StringContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#connector.
    def enterConnector(self, ctx: wandelscriptParser.ConnectorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#connector.
    def exitConnector(self, ctx: wandelscriptParser.ConnectorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#print_.
    def enterPrint_(self, ctx: wandelscriptParser.Print_Context):
        pass

    # Exit a parse tree produced by wandelscriptParser#print_.
    def exitPrint_(self, ctx: wandelscriptParser.Print_Context):
        pass

    # Enter a parse tree produced by wandelscriptParser#read.
    def enterRead(self, ctx: wandelscriptParser.ReadContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#read.
    def exitRead(self, ctx: wandelscriptParser.ReadContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#write.
    def enterWrite(self, ctx: wandelscriptParser.WriteContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#write.
    def exitWrite(self, ctx: wandelscriptParser.WriteContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#call.
    def enterCall(self, ctx: wandelscriptParser.CallContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#call.
    def exitCall(self, ctx: wandelscriptParser.CallContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#identifierList.
    def enterIdentifierList(self, ctx: wandelscriptParser.IdentifierListContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#identifierList.
    def exitIdentifierList(self, ctx: wandelscriptParser.IdentifierListContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#modifier.
    def enterModifier(self, ctx: wandelscriptParser.ModifierContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#modifier.
    def exitModifier(self, ctx: wandelscriptParser.ModifierContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#expressionList.
    def enterExpressionList(self, ctx: wandelscriptParser.ExpressionListContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#expressionList.
    def exitExpressionList(self, ctx: wandelscriptParser.ExpressionListContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#additionOperator.
    def enterAdditionOperator(self, ctx: wandelscriptParser.AdditionOperatorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#additionOperator.
    def exitAdditionOperator(self, ctx: wandelscriptParser.AdditionOperatorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#sign.
    def enterSign(self, ctx: wandelscriptParser.SignContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#sign.
    def exitSign(self, ctx: wandelscriptParser.SignContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#inverse.
    def enterInverse(self, ctx: wandelscriptParser.InverseContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#inverse.
    def exitInverse(self, ctx: wandelscriptParser.InverseContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#not.
    def enterNot(self, ctx: wandelscriptParser.NotContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#not.
    def exitNot(self, ctx: wandelscriptParser.NotContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#multiplicationOperator.
    def enterMultiplicationOperator(self, ctx: wandelscriptParser.MultiplicationOperatorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#multiplicationOperator.
    def exitMultiplicationOperator(self, ctx: wandelscriptParser.MultiplicationOperatorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#comparisonOperator.
    def enterComparisonOperator(self, ctx: wandelscriptParser.ComparisonOperatorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#comparisonOperator.
    def exitComparisonOperator(self, ctx: wandelscriptParser.ComparisonOperatorContext):
        pass

    # Enter a parse tree produced by wandelscriptParser#logicalOperator.
    def enterLogicalOperator(self, ctx: wandelscriptParser.LogicalOperatorContext):
        pass

    # Exit a parse tree produced by wandelscriptParser#logicalOperator.
    def exitLogicalOperator(self, ctx: wandelscriptParser.LogicalOperatorContext):
        pass


del wandelscriptParser
