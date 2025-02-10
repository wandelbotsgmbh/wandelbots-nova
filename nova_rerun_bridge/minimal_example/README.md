## Example

This example demonstrates how to use the Nova Rerun Bridge library together with Wandelbots-Nova. The `test.py` file contains the necessary code to get started.

## Files

```
test_rerun/
├── README.md
├── test.py
```

## Running the Example

1. Ensure you have the required dependencies installed with `poetry install`
2. Download the robot models by running `poetry run download-models`
3. Add an .env file with the necessary environment variables:
   ```sh
   NOVA_API=<your-url-instance>
   NOVA_ACCESS_TOKEN=<your-nova-access-token>
   ```
4. Run the `test.py` script:
   ```sh
   poetry run python test.py
   ```
