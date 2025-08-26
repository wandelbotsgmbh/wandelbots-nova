# wandelbots-nova

The Wandelbots NOVA extension for VSCode.

## Local Development

1. Install the dependencies:

```bash
pnpm install
```

2. Build & package the extension:

```bash
pnpm run package
```

3. Install the extension in VSCode via VSIX. Search for the command `Extensions: Install from VSIX...` and select the created file.

4. Reload the extension with the command `Developer: Reload Window`.

You should now see the extension in the Extensions view & the Wandelbots logo in the activity bar.
