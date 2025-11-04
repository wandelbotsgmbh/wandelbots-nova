# Getting Started with the Nova Python SDK

## Table of contents

- [Fast track](#fast-track)
- [More configuration options](#more-configuration-options)

## Fast track

The fastest way to use the Python SDK is to run your code inside Nova using the Visual Studio Code workspace. In that environment everything is preconfigured, so using the SDK is as easy as:

```python
from nova import Nova

async with Nova() as nova:
    ...
```

This works because the Nova platform provides all of the required connection settings automatically. You do not need to supply hostnames, tokens, or certificates—just create the instance and start coding.

Ready for more control? Explore the options below.

## More configuration options

You can use the SDK with different Nova environments such as the Wandelbots Portal, an IPC, or Nova running in a virtual machine. If you're staying inside the default Nova workspace, you can skip this section. The `NovaConfig` class holds the details for each environment when you need them:

```python
from nova.config import NovaConfig

portal_config = NovaConfig(
    host="https://xxxxx.instance.wandelbots.io",
    access_token="your_access_token",
)

ipc_config = NovaConfig(
    host="http://192.168.0.10",
)
```

Pass the configuration into `Nova` when you create it:

```python
async with Nova(config=portal_config) as nova:
    print("Connected to the Portal instance")

async with Nova(config=ipc_config) as nova:
    print("Connected to the IPC instance")
```

### Use environment variables

Instead of hard-coding connection details, you can store them in environment variables. This is handy when you deploy code or share it with teammates.

Create a `.env` file in your Python project and add the following entries:

```
NOVA_API=https://xxxxx.instance.wandelbots.io
NOVA_ACCESS_TOKEN=your_access_token
```

Once the variables are in place, create a Nova instance as usual—they are picked up automatically:

```python
from nova import Nova

async with Nova() as nova:
    print("Connected using environment variables")
```

When you run inside the Nova platform (for example, an App Store application), the platform injects these variables for you. Local projects just need a matching `.env` file.

-----

If you want to learn more about how these configurations work, check the `config.py` file in the nova package.
