# DEV.md — Waypoint Jogging Development Notes

## Updating WBR and Service Manager on the Robot

Kubeconfig: `~/Downloads/physical-ai`

### Find latest image tags from GitLab MRs

```bash
# WBR (MR !2192) — find latest pipeline's deploy:container job
GITLAB_HOST=code.wabo.run glab api "projects/robotics%2Fwbr/merge_requests/2192/pipelines" | python3 -c "import sys,json; data=json.load(sys.stdin); [print(f'{p[\"id\"]} {p[\"status\"]} {p[\"created_at\"][:10]}') for p in data[:5]]"

# Get the image tag from the deploy job trace (use latest pipeline ID)
GITLAB_HOST=code.wabo.run glab api "projects/robotics%2Fwbr/jobs/<JOB_ID>/trace" 2>&1 | grep "naming to wandelbots.azurecr.io"

# Service Manager (MR !2345)
GITLAB_HOST=code.wabo.run glab api "projects/wandelos%2Fservice-manager/merge_requests/2345/pipelines" | python3 -c "import sys,json; data=json.load(sys.stdin); [print(f'{p[\"id\"]} {p[\"status\"]} {p[\"created_at\"][:10]}') for p in data[:5]]"
```

### Deploy to robot

```bash
export KUBECONFIG=~/Downloads/physical-ai

# Update WBR (rae)
kubectl set image deployment/rae -n cell \
  rae-container=wandelbots.azurecr.io/development-nova-services/robotics-rae:<TAG>

# Update Service Manager (api-gateway)
kubectl set image deployment/api-gateway -n cell \
  api-gateway=wandelbots.azurecr.io/development-nova-services/api-gateway:<TAG>

# Watch rollout
kubectl rollout status deployment/rae -n cell --timeout=60s
kubectl rollout status deployment/api-gateway -n cell --timeout=60s
```

### If image pull fails (missing pull secret)

```bash
# Get a fresh ACR token
az acr login --name wandelbots --subscription "developer-portal-shared" --expose-token

# Create the pull secret (use token from above)
kubectl create secret docker-registry pull-secret-wandelbots-azurecr-io \
  --docker-server=wandelbots.azurecr.io \
  --docker-username=00000000-0000-0000-0000-000000000000 \
  --docker-password="<TOKEN>" \
  -n cell
```

### Current versions (as of 2026-05-21)

| Service | Image | Tag |
|---------|-------|-----|
| WBR | `wandelbots.azurecr.io/development-nova-services/robotics-rae` | `3.250.1-feat-waypoint-jogging` |
| Service Manager | `wandelbots.azurecr.io/development-nova-services/api-gateway` | `26.4.0-mr2345` |
