---
mode: agent
model: Claude Sonnet 4
tools: ["runCommands", 'terminalSelection', 'terminalLastCommand']
description: "Starts a server you can test your programs locally"
---
Start local server by running the followin command:
```bash
uv run python -m app_name
```

Wait until you see successful program start logs.
If you see any error that caused program to not start inform user.