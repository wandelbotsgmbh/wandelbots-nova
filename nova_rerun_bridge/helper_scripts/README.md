## Installing Robot Models

After installing the library, you need to download the robot models:

```bash
# If installed via uv
uv run download-models

# If installed via pip
python -m nova_rerun_bridge.models.download_models

# manually
Copy the models from [the react component lib](https://github.com/wandelbotsgmbh/wandelbots-js-react-components/tree/main/public/models) into the draco folder.
Run the unpack_models.sh script to unpack the models.

This is necessary as trimesh does not support draco compressed models.
```
