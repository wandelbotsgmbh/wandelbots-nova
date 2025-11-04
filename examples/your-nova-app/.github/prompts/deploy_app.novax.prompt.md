---
mode: agent
model: Claude Sonnet 4
tools: ["runCommands", 'terminalSelection', 'terminalLastCommand']
---
Users want to deploy their application to a Nova instance.
Follow these steps to help user.

## Steps to deploy the application

### Collect necessary information for the deployment
1. Make sure nova cli is installed by running the command:
```bash
nova version
```
2. Run this command to understand which Nova instance we are going to do the deployment to:
```bash
nova config view
```
3. If the config doesn't have a host configuration, ask user to configure it by running this command
```bash
nova config set host <host>
```
4. If the user doesn't have an image-registry configured, ask them to follow the documentation here to do the setup:
   https://docs.wandelbots.io/25.6/developing-cli

5. If the user has all the necessary information configured. Then construct an image tag like this:
   ```
   <image-registry>/<image-name>/<app-name>:latest
   ```
IMPORTANT:
To understand what the <app-name> is read the app field from .nova file.
DO NOT USE THE pyproject.toml for this purpose.

6. Prompt the user with the following information and ask for confirmation.
   - Nova instance to which we will deploy the application
   - The image tag you constructed above


### User approved the deployment
1. Run this command:
```bash
docker buildx build --load -t <image-tag> --platform linux/amd64 .
```
2. Inform the user this command is temporary and will not be needed in a future version of nova cli.
3. Run 
```bash
nova app install
```


### User did NOT approve the deployment
1. Inform the user that the deployment has been canceled.