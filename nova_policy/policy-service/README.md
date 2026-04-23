# policy_service

NOVA-native policy service app.

## Run locally

```bash
cd nova_policy/policy-service
uv run python -m policy_service
```

Server starts on `http://localhost:3000`.

## API

- `GET /healthz`
- `GET /policies`
- `GET /policies/{policy}`
- `POST /policies/{policy}/start`
- `POST /policies/{policy}/stop`
- `GET /policies/{policy}/runs/{run}`

## Runtime notes

- Policy path must be provided via URL or `request.policy.path`.
- Device is resolved from `request.policy.device`, default `cuda`.
- Run responses expose state telemetry via `metadata`.

## Deploy to GPU cluster (AKS + Flux)

### 1) Prerequisites

```bash
az account set --subscription b734f948-4064-4718-a0ac-84b1ff8d74f5
az aks get-credentials --resource-group developer-portal-gpucluster-dev --name developer-portal-gpucluster-dev --overwrite-existing
az acr login --name wandelbots
brew install fluxcd/tap/flux  # if missing
```

### 2) Build and push image (amd64)

From `nova_policy/policy-service`:

```bash
docker buildx build \
  --platform linux/amd64 \
  -t wandelbots.azurecr.io/ai/nova-policy-service:YYYY-MM-DD-NN \
  --push .
```

> Note: this image is large (PyTorch/CUDA stack), so first push can take a while.

### 3) Update Flux image tag

In `flux-apps` repo:

- file: `apps/nova-policy-service/kustomization.yaml`

```yaml
images:
  - name: wandelbots.azurecr.io/ai/nova-policy-service
    newTag: YYYY-MM-DD-NN
```

Commit and push.

### 4) Reconcile Flux

```bash
flux reconcile source git physical-ai-flux-apps --namespace team-embodied-ai
flux reconcile kustomization apps-nova-policy-service --namespace team-embodied-ai
```

### 5) Verify rollout

```bash
kubectl rollout status deploy/nova-policy-service -n team-embodied-ai --timeout=600s
kubectl get pods -n team-embodied-ai -l app=nova-policy-service
kubectl logs -n team-embodied-ai -l app=nova-policy-service --tail=30
curl -s -o /dev/null -w "%{http_code}\n" https://nova-policy-service.ai.gpucluster-dev.wandelbots.io/healthz
```

Expected healthz response code: `200`.
