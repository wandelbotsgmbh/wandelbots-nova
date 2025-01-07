# wandelbots-nova

This library provides an SDK for the Wandelbots NOVA API.

The SDK will help you to build your own apps and services on top of NOVA and makes programming a robot as easy as possible.

## Requirements

This library requires
* Python >=3.10

## Installation

To use the library, first install it using the following command

```bash
pip install wandelbots-nova
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

### Environment Variables for NOVA Configuration

1. **Copy the Template:** Make a copy of the provided `.env.template` file and rename it to `.env` with `cp .env.template .env`.
2. **Fill in the Values:** Open the `.env` file in a text editor and provide the necessary values for each variable. The table below describes each variable and its usage.

| Variable            | Description                                                               | Required | Default | Example                    |
|---------------------|---------------------------------------------------------------------------|----------|---------|----------------------------|
| `NOVA_API`         | The base URL or hostname of the NOVA server instance.                     | Yes      | None    | `https://nova.example.com` |
| `NOVA_USERNAME`     | The username credential used for authentication with the NOVA service.    | Yes*     | None    | `my_username`              |
| `NOVA_PASSWORD`     | The password credential used in conjunction with `NOVA_USERNAME`.         | Yes*     | None    | `my_password`              |
| `NOVA_ACCESS_TOKEN` | A pre-obtained access token for NOVA if using token-based authentication. | Yes*     | None    | `eyJhbGciOi...`            |

> **Note on Authentication:**
> You can authenticate with NOVA using either **username/password** credentials or a pre-obtained **access token**, depending on your setup and security model:
> - If using **username/password**: Ensure both `NOVA_USERNAME` and `NOVA_PASSWORD` are set, and leave `NOVA_ACCESS_TOKEN` unset.
> - If using an **access token**: Ensure `NOVA_ACCESS_TOKEN` is set, and leave `NOVA_USERNAME` and `NOVA_PASSWORD` unset.
>
> **Only one method should be used at a time.** If both methods are set, the token-based authentication (`NOVA_ACCESS_TOKEN`) will typically take precedence.



