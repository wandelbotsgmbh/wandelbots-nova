---
mode: agent
model: Claude Sonnet 4
tools: ['codebase', 'usages', 'vscodeAPI', 'problems', 'changes', 'testFailure', 'terminalSelection', 'terminalLastCommand', 'openSimpleBrowser', 'fetch', 'findTestFiles', 'searchResults', 'githubRepo', 'extensions', 'runTests', 'editFiles', 'runNotebooks', 'search', 'new', 'runCommands', 'runTasks', 'getPythonEnvironmentInfo', 'getPythonExecutableCommand', 'installPythonPackage', 'configurePythonEnvironment']
description: "Develop a new robotics program"
---
Help the user to develop a new robotics program by guiding them through the necessary steps and providing relevant information.

- Use simple python code as instructed in the copilot-instructions.
- Follow best practices for developing robotics programs. Especially check the examples here: https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples
- Aim for simplicity and clarity in your code.
- Aim for well documented code so the user can easily understand and modify it in the future.
- unless explicity stated, DO NOT WRITE TESTS or demos or documentation. Only write the `program` and register it in the app.py