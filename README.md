# wandelbots-nova

[![PyPI version](https://badge.fury.io/py/wandelbots-nova.svg)](https://badge.fury.io/py/wandelbots-nova)
[![License](https://img.shields.io/github/license/wandelbotsgmbh/wandelbots-nova.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/LICENSE)
[![Build Status](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/release.yaml/badge.svg)](https://github.com/wandelbotsgmbh/wandelbots-nova/actions/workflows/release.yaml)

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services on top of NOVA and makes programming a robot as easy as possible.

## Requirements

This library requires

- Python >=3.10

## Installation

To use the library, first install it using the following command

```bash
pip install wandelbots-nova
```

### Optional: Install with `nova-rerun-bridge`

We recommend installing the library with the `nova-rerun-bridge` extra to make usage of the visualization tool [rerun](https://rerun.io/).
See the [README.md](nova_rerun_bridge/README.md) for further details.

```bash
pip install "wandelbots-nova[nova-rerun-bridge]"
```

Or add to your pyproject.toml:

```bash
wandelbots-nova = { version = ">=0.12", extras = ["nova-rerun-bridge"] }
```

You need to download the robot models to visualize the robot models in the rerun viewer. You can download the models by running the following command:

```bash
poetry run download-models
```

## Usage

Import the library in your code to get started.

```python
from nova import Nova
```

The SDK also includes an auto-generated API client for the NOVA API. You can access the API client using the `api` module.

```python
from nova import api
```

Checkout the [01_basic](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/01_basic.py) and [02_plan_and_execute](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples/02_plan_and_execute.py) examples to learn how to use the library.

In the [this](https://github.com/wandelbotsgmbh/wandelbots-nova/tree/main/examples) directory are more examples to explain the advanced usage of the SDK.

## Development

To install the development dependencies, run the following command

```bash
poetry install
```

### Using Branch Versions For Testing

When having feature branches or forks, or might be helpful to test the library as dependency in other projects first.
Poetry allows to pull the library from different sources. See the [Poetry Doc](https://python-poetry.org/docs/dependency-specification/#git-rev-project) for more information.

Poetry Version < 2:
```toml
wandelbots-nova = { git = "https://github.com/wandelbotsgmbh/wandelbots-nova.git", branch = "fix/http-prefix" }
```

Poetry Version >=2
```toml
wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@fix/http-prefix
```

### Environment Variables for NOVA Configuration

1. **Copy the Template:** Make a copy of the provided `.env.template` file and rename it to `.env` with `cp .env.template .env`.
2. **Fill in the Values:** Open the `.env` file in a text editor and provide the necessary values for each variable. The table below describes each variable and its usage.

| Variable            | Description                                                               | Required | Default | Example                                          |
| ------------------- | ------------------------------------------------------------------------- | -------- | ------- | ------------------------------------------------ |
| `NOVA_API`          | The base URL or hostname of the NOVA server instance.                     | Yes      | None    | `https://nova.example.com` or `http://172.0.0.1` |
| `NOVA_USERNAME`     | The username credential used for authentication with the NOVA service.    | Yes\*    | None    | `my_username`                                    |
| `NOVA_PASSWORD`     | The password credential used in conjunction with `NOVA_USERNAME`.         | Yes\*    | None    | `my_password`                                    |
| `NOVA_ACCESS_TOKEN` | A pre-obtained access token for NOVA if using token-based authentication. | Yes\*    | None    | `eyJhbGciOi...`                                  |

> **Note on Authentication:**
> You can authenticate with NOVA using either **username/password** credentials or a pre-obtained **access token**, depending on your setup and security model:
>
> - If using **username/password**: Ensure both `NOVA_USERNAME` and `NOVA_PASSWORD` are set, and leave `NOVA_ACCESS_TOKEN` unset.
> - If using an **access token**: Ensure `NOVA_ACCESS_TOKEN` is set, and leave `NOVA_USERNAME` and `NOVA_PASSWORD` unset.
>
> **Only one method should be used at a time.** If both methods are set, the token-based authentication (`NOVA_ACCESS_TOKEN`) will typically take precedence.
