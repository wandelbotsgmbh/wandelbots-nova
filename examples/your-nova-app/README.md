# Your NOVA app

This template contains a simple python app served by [fastapi](https://github.com/tiangolo/fastapi) provided via NOVAx.
It shows you how to use the [NOVA Python SDK](https://github.com/wandelbotsgmbh/wandelbots-nova) and build a basic app with it.

Robot programs are defined with `@nova.program` (see `app/programs/start_here.py`) and
auto-registered by dropping a module into the `app/programs` directory — the decorator
self-registers on import, so no manual import is needed.

Use the following steps for development:

* make sure you have `uv` installed
    * you can follow these steps https://docs.astral.sh/uv/getting-started/installation/
* ensure proper environment variables are set in `.env`
    * note: you might need to set/update `NOVA_ACCESS_TOKEN` and `NOVA_API`
* use `uv run app` to run the server
    * access the docs on `http://localhost:3000/docs`
* build, push and install the app with `nova app install`

## quick run (no app needed)

To iterate on a program from dev without scaffolding an app, point the `novax` CLI at a
file with `@nova.program` functions — they auto-register:

```bash
uv run novax run app/programs/start_here.py --cell cell
```


## hot reload on a Nova instance (Skaffold)

When you need the program actually registered **on a Nova instance** (not just run
locally), use the Skaffold setup to run the app in the cell and hot-reload it on file
changes — edit a program file → Skaffold syncs it into the running pod → `uvicorn
--reload` restarts → the program re-registers on the instance. No image rebuild per
change.

Requirements:

* `kubectl` access to the instance's cluster (`export KUBECONFIG=/path/to/kubeconfig`)
* [Skaffold](https://skaffold.dev/docs/install/) installed
* logged in to the image registry so Skaffold can push (`docker login wandelbots.azurecr.io`,
  e.g. via `az acr login --name wandelbots`)

The registry Skaffold pushes to is set in `skaffold.env` (`SKAFFOLD_DEFAULT_REPO`,
defaults to the same ACR novaflow uses, which the cluster can already pull from).
Override it there or with `--default-repo` if you use a different registry.

```bash
skaffold dev
```

Adjust the namespace and env in
`k8s/app.yaml` for your cell.

> **Notes**
> * Skaffold deploys an `App` custom resource (`k8s/app.yaml`). The in-cluster
>   app-operator reconciles it into a Deployment/Service/Ingress **and** registers
>   the home-screen tile — the same thing `nova app install` does — so `skaffold dev`
>   alone is enough; no separate install step is needed.
> * Hot reload still works: the `App` sets `DEV_RELOAD=true`, so the container runs
>   `uvicorn --reload`; Skaffold syncs changed program files into the pod and uvicorn
>   restarts, re-registering the programs. The dev image is built with
>   `INSTALL_DEV=true` (adds `watchfiles`); production builds stay lean (`--no-dev`).


## formatting

```bash
uv run ruff format
uv run ruff check --select I --fix
```