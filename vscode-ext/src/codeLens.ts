import * as vscode from 'vscode'

import { COMMAND_DEBUG_NOVA_PROGRAM, COMMAND_RUN_NOVA_PROGRAM } from './consts.js'

/**
 * CodeLens provider for Nova programs
 * Detects @nova.program decorators and adds run/debug buttons
 */
export class NovaCodeLensProvider implements vscode.CodeLensProvider {
  private readonly _onDidChangeCodeLenses = new vscode.EventEmitter<void>()
  public readonly onDidChangeCodeLenses: vscode.Event<void> =
    this._onDidChangeCodeLenses.event

  /**
   * Provide CodeLens for a document
   */
  provideCodeLenses(
    document: vscode.TextDocument,
    _token: vscode.CancellationToken,
  ): vscode.ProviderResult<vscode.CodeLens[]> {
    console.log(
      `CodeLens provider called for: ${document.fileName}, language: ${document.languageId}`,
    )

    // Check if CodeLens is enabled
    const config = vscode.workspace.getConfiguration('wandelbots-nova')
    const isEnabled = config.get<boolean>('enableNovaCodeLens', true)

    if (!isEnabled) {
      console.log('Nova CodeLens is disabled in configuration')
      return []
    }

    const codeLenses: vscode.CodeLens[] = []

    // Only process Python files
    if (document.languageId !== 'python') {
      console.log('Not a Python file, skipping CodeLens')
      return codeLenses
    }

    const text = document.getText()
    const lines = text.split(/\r?\n/)

    // Track if we're inside a multi-line decorator
    let insideNovaDecorator = false
    let decoratorStartLine = -1
    let decoratorParenthesesCount = 0

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i]
      const trimmed = line.trim()

      // Check for start of @nova.program decorator
      if (!insideNovaDecorator && this.isNovaProgram(line)) {
        console.log(
          `Found @nova.program decorator at line ${i + 1}: ${trimmed}`,
        )
        insideNovaDecorator = true
        decoratorStartLine = i

        // Count parentheses to track multi-line decorators
        decoratorParenthesesCount = this.countParentheses(line)
        console.log(`Initial parentheses count: ${decoratorParenthesesCount}`)

        // Check if this is a complete decorator on one line
        if (
          decoratorParenthesesCount === 0 ||
          trimmed === '@nova.program' ||
          this.isCompleteSingleLineDecorator(trimmed)
        ) {
          console.log(`Single-line decorator detected`)
          insideNovaDecorator = false
          this.processNovaDecorator(
            lines,
            decoratorStartLine,
            i,
            codeLenses,
            document,
          )
          // Reset state to look for more decorators
          decoratorStartLine = -1
          decoratorParenthesesCount = 0
        }
        continue
      }

      // If we're inside a multi-line decorator, count parentheses
      if (insideNovaDecorator) {
        const lineParenCount = this.countParentheses(line)
        decoratorParenthesesCount += lineParenCount
        console.log(
          `Line ${
            i + 1
          }: "${trimmed}" - line paren count: ${lineParenCount}, total: ${decoratorParenthesesCount}`,
        )

        // If parentheses are balanced (count reaches 0), we've reached the end
        if (decoratorParenthesesCount <= 0) {
          console.log(
            `End of multi-line decorator at line ${i + 1}, parentheses balanced`,
          )
          insideNovaDecorator = false
          this.processNovaDecorator(
            lines,
            decoratorStartLine,
            i,
            codeLenses,
            document,
          )
          // Reset state to look for more decorators
          decoratorStartLine = -1
          decoratorParenthesesCount = 0
        }
      }
    }

    console.log(`Returning ${codeLenses.length} CodeLens items`)
    return codeLenses
  }

  /**
   * Check if a decorator is complete on a single line
   */
  private isCompleteSingleLineDecorator(line: string): boolean {
    const trimmed = line.trim()
    return (
      trimmed === '@nova.program' ||
      (trimmed.startsWith('@nova.program(') && trimmed.endsWith(')'))
    )
  }

  /**
   * Count the net parentheses in a line (opening - closing)
   */
  private countParentheses(line: string): number {
    let count = 0
    for (const char of line) {
      if (char === '(') count++
      if (char === ')') count--
    }
    return count
  }

  /**
   * Process a complete nova.program decorator and add CodeLens
   */
  private processNovaDecorator(
    lines: string[],
    startLine: number,
    endLine: number,
    codeLenses: vscode.CodeLens[],
    document: vscode.TextDocument,
  ): void {
    // Find the function definition that follows this decorator
    const functionLine = this.findFunctionDefinition(lines, endLine)

    if (functionLine !== -1) {
      const functionName = this.extractFunctionName(lines[functionLine])
      console.log(`Found function: ${functionName} at line ${functionLine + 1}`)

      if (functionName) {
        const range = new vscode.Range(
          startLine,
          0,
          startLine,
          lines[startLine]?.length ?? 0,
        )

        // Add "Run" button
        const runCommand: vscode.Command = {
          title: '▶️ Run Program',
          command: COMMAND_RUN_NOVA_PROGRAM,
          arguments: [document.uri, functionName, startLine],
        }

        // Add "Debug" button
        const debugCommand: vscode.Command = {
          title: '🐛 Debug Program',
          command: COMMAND_DEBUG_NOVA_PROGRAM,
          arguments: [document.uri, functionName, startLine],
        }

        codeLenses.push(
          new vscode.CodeLens(range, runCommand),
          new vscode.CodeLens(range, debugCommand),
        )

        console.log(`Added CodeLens for function: ${functionName}`)
      }
    } else {
      console.log('No function definition found after decorator')
    }
  }

  /**
   * Check if a line contains a nova.program decorator
   */
  private isNovaProgram(line: string): boolean {
    const trimmed = line.trim()
    // Handle various decorator formats
    const patterns = [
      '@nova.program',
      '@nova.program(',
      '@ nova.program',
      '@ nova.program(',
    ]

    for (const pattern of patterns) {
      if (trimmed.startsWith(pattern) || trimmed.includes(pattern)) {
        console.log(
          `Matched nova.program pattern: "${pattern}" in line: "${trimmed}"`,
        )
        return true
      }
    }

    return false
  }

  /**
   * Find the function definition that follows a decorator
   * @returns Line index of function definition, or -1 if not found
   */
  private findFunctionDefinition(
    lines: string[],
    decoratorIndex: number,
  ): number {
    console.log(
      `Looking for function definition after decorator at line ${
        decoratorIndex + 1
      }`,
    )

    // Look for the function definition in the next few lines
    for (
      let i = decoratorIndex + 1;
      i < Math.min(decoratorIndex + 30, lines.length);
      i++
    ) {
      const line = lines[i]?.trim() ?? ''
      console.log(`Checking line ${i + 1}: "${line}"`)

      // Skip empty lines and other decorators
      if (line === '' || line.startsWith('@')) {
        console.log(`Skipping line ${i + 1} (empty or decorator)`)
        continue
      }

      // Check if this is a function definition
      if (line.startsWith('def ') || line.startsWith('async def ')) {
        console.log(`Found function definition at line ${i + 1}: "${line}"`)
        return i
      }

      // Skip comments and docstrings
      if (
        line.startsWith('#') ||
        line.startsWith('"""') ||
        line.startsWith("'''")
      ) {
        console.log(`Skipping comment/docstring at line ${i + 1}`)
        continue
      }

      // If we hit something else substantial, we might have missed the function
      if (line !== '' && !line.startsWith('@')) {
        console.log(
          `Found non-function content at line ${i + 1}, stopping search`,
        )
        break
      }
    }

    console.log('No function definition found')
    return -1
  }

  /**
   * Extract function name from a function definition line
   */
  private extractFunctionName(line: string): string | null {
    // Match patterns like "def function_name(" or "async def function_name("
    const match = line.match(/(?:async\s+)?def\s+(\w+)\s*\(/)
    return match ? match[1] : null
  }

  /**
   * Refresh CodeLens (called when document changes)
   */
  public refresh(): void {
    this._onDidChangeCodeLenses.fire()
  }

  public dispose(): void {
    this._onDidChangeCodeLenses.dispose()
  }
}
