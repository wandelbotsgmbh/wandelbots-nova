# wandelbots-nova

The Wandelbots NOVA extension for VS Code.

## What this extension does
ƒ
- **CodeLens actions**: Adds buttons above each `@nova.program` to quickly run NOVA programs: **Run**, **Debug**, and **Tune path**.
- **Trajectory fine-tuning (Trajectory tuner)**: Interactively step through trajectories, jog the robot, and save adjusted poses.
- **Instance selection**: Configure which NOVA instance to connect to via extension settings.

## Using the extension

### Configure the NOVA instance

1. Open VS Code settings and search for Wandelbots NOVA.
2. Set **NOVA Instance Host** to the host of your NOVA instance.

### Run and debug from your code

The extension scans your Python files for `@nova.program` and displays CodeLens buttons above each decorated function:

- **Run**: Execute the program on the configured NOVA instance.
- **Debug**: Launch with debugger support.
- **Tune Path**: Start interactive path tuning.

### Trajectory fine-tuning (Trajectory Tuner)

Use Trajectory Tuner to iteratively adjust trajectory waypoints and actions:

1. Click **Tune Trajectory** above a `@nova.program`.\
   Execution will pause at each execution call.
3. In the **Fine-Tuning** panel, use the following controls:
   - **Forward/Backward**: Hold to step through the trajectory in either direction.
   - **Snap to point**: Toggle on to stop at every action/keyframe; toggle off to glide continuously.
4. Fine-tune points:
   - Use the built-in **Jogging Panel** to move a robot to the desired pose.
   - Save to create a new point, updating the path.
5. Continue stepping and saving until the trajectory is tuned to your needs, then exit the Trajectory Tuner.

## Local development

1. Install the dependencies:

```bash
pnpm install
```

2. Build & package the extension:

```bash
pnpm run package
```

3. Install the extension in VS Code via VSIX. Search for the command `Extensions: Install from VSIX...` and select the created file.

4. Reload the extension with the command `Developer: Reload Window`.

You should now see the extension in the Extensions view & the Wandelbots logo in the activity bar.
