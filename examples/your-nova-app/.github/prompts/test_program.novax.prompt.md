---
mode: agent
model: Claude Sonnet 4
tools: ["runCommands", 'terminalSelection', 'terminalLastCommand']
description: "Starts a server you can test your programs locally"
---

When user wants to test the robotics programs they wrote.
Follow these steps:
1. Make sure no server is running in the port. Check the __main__.py file to see which port is used
2. If the server is running, idetify the process and kill it.
3. Start the new server in the background.
IMPORTANT: DO NOT START THE SERVER LIKE THIS:
```bash
uv run python -m app_name
```

but this doesn't block the terminal.
```bash
uv run python -m app_name  > app.log 2>&1 &
```

So use `uv run python -m app_name` command in a way that it doesn't blocks the terminal.

4. Check the log file to see if the server started successfully.
If not, inform the user about the failure and provide the relevant error logs.

5. Ask the user to provide the program name they want to test
6. query the /programs API to understand the data structure it expects.
7. Prompt the user for the input data required by the program.
8. Start the program by using the /programs/<program_name>/start endpoint with the provided input data.
IGNORE THE PROGRAM RUN, DO NOT CALL runs api
9. Conguratulate the user on successfully starting the program. 
10. End the conversation.