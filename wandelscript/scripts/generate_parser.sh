#!/bin/bash
# Clean up old ANTLR-generated files, regenerate the lexer and parser in Python3 with the visitor
# pattern from grammar files located in a specified directory and provide feedback on success or
# failure.

# Directory to output generated files (optional)
GRAMMAR_DIR="${1:-wandelscript/grammar}"

cd "$GRAMMAR_DIR" || { >&2 echo "Failed to access $GRAMMAR_DIR"; exit 1; }

echo "Cleaning up old generated files..."
rm -f wandelscriptLexer.interp
rm -f wandelscriptLexer.py
rm -f wandelscriptLexer.tokens
rm -f wandelscriptParser.interp
rm -f wandelscriptParser.py
rm -f wandelscriptParser.tokens
rm -f wandelscriptParserListener.py
rm -f wandelscriptParserVisitor.py

# Run ANTLR to generate Python3 files with the visitor pattern
echo "Running ANTLR to generate parser and lexer..."
antlr -Dlanguage=Python3 -visitor ./*.g4

if [ $? -eq 0 ]; then
  echo "ANTLR generation complete. Files are located in $GRAMMAR_DIR."
else
  echo "ANTLR generation failed. Please check for errors."
fi

cd - > /dev/null || exit 1
