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

### ⚠️ Service-manager operator will reset your images

The cluster runs a `service-manager` operator (`deploy/service-manager` in `wandelbots` namespace)
that reconciles `App` CRs and the Foundation (RAE, api-gateway, etc.) every 10 minutes.
When it runs, it **resets all images back to the stock release** (currently 26.3.0),
which wipes your dev images AND can recreate app resources with stale ConfigMap mounts.

**Keep the operator scaled to 0 while using dev images:**

```bash
export KUBECONFIG=~/Downloads/physical-ai

# Disable the operator (prevents reconciliation)
kubectl scale deploy/service-manager -n wandelbots --replicas=0
```

**When you need `nova app install` to work** (e.g. deploying the GR00T app):

```bash
# 1. Scale operator to 1 temporarily
kubectl scale deploy/service-manager -n wandelbots --replicas=1
kubectl rollout status deploy/service-manager -n wandelbots --timeout=120s

# 2. If operator fails with ImagePullBackOff, restore the pull secret:
#    (copy from 'tools' namespace which has a service principal, not an expiring token)
python3 - <<'PY' >/tmp/pull-secret.json
import subprocess, json
cmd=['kubectl','--kubeconfig','$HOME/Downloads/physical-ai','get','secret',
     'pull-secret-wandelbots-azurecr-io','-n','tools','-o','json']
obj=json.loads(subprocess.check_output(cmd))
for k in ['uid','resourceVersion','creationTimestamp','managedFields','annotations']:
    obj['metadata'].pop(k, None)
obj['metadata']['namespace']='wandelbots'
print(json.dumps(obj))
PY
kubectl apply -f /tmp/pull-secret.json
kubectl delete pod -n wandelbots -l app.kubernetes.io/name=service-manager --force

# 3. Run nova app install (fresh create — delete stale app first if needed)
NOVA_API=http://172.31.11.129 uv run nova app delete gr00t-dual-arm || true
kubectl delete app gr00t-dual-arm -n cell --ignore-not-found
kubectl delete deploy/app-gr00t-dual-arm svc/app-gr00t-dual-arm \
  ingress/app-gr00t-dual-arm secret/app-gr00t-dual-arm-regcred \
  configmap/gr00t-app-code -n cell --ignore-not-found

cd policy/examples/apps/gr00t/gr00t-dual-arm-controller
NOVA_API=http://172.31.11.129 uv run nova app install .

# 4. Wait for app to come up, then immediately scale operator back to 0
kubectl rollout status deploy/app-gr00t-dual-arm -n cell --timeout=120s
kubectl scale deploy/service-manager -n wandelbots --replicas=0

# 5. Restore dev images (operator just reset them to stock)
kubectl set image deploy/rae -n cell \
  rae-container=wandelbots.azurecr.io/development-nova-services/robotics-rae:3.251.1-feat-waypoint-jogging
kubectl set image deploy/api-gateway -n cell \
  cell=wandelbots.azurecr.io/development-nova-services/api-gateway:26.4.0-mr2345
kubectl rollout status deploy/rae deploy/api-gateway -n cell --timeout=120s
```

**Key insight:** The operator is only needed for the initial `App` CR → k8s resources
reconciliation. Once the app deployment exists, the operator can be disabled again.
Your dev images for RAE/api-gateway survive as long as the operator stays at 0.

### Current versions (as of 2026-05-26)

| Service | Image | Tag |
|---------|-------|-----|
| WBR | `wandelbots.azurecr.io/development-nova-services/robotics-rae` | `3.251.1-feat-waypoint-jogging` |
| Service Manager | `wandelbots.azurecr.io/development-nova-services/api-gateway` | `26.4.0-mr2345` |
| API Client | `wandelbots-api-client` | `26.4.0a2345+b88ffc36` |
