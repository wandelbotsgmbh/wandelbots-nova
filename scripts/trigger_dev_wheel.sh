#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Trigger the "Build dev wheel" GitHub Actions workflow for the current branch,
# wait for it to finish, and print the pip install string for the built wheel.
#
# The dev wheel build only makes sense on a feature branch, so this script
# refuses to run on main / master / release branches.
#
# Requirements: GitHub CLI (`gh`) installed and authenticated (`gh auth login`).
# ------------------------------------------------------------------------------

WORKFLOW="nova-build-dev.yaml"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}$*${NC}"; }
warn()  { echo -e "${YELLOW}$*${NC}"; }
error() { echo -e "${RED}$*${NC}" >&2; }

# --- 1) CHECK PREREQUISITES ---------------------------------------------------
if ! command -v gh >/dev/null 2>&1; then
  error "[ERROR] GitHub CLI (gh) is not installed. See https://cli.github.com/"
  exit 1
fi

if ! gh auth status >/dev/null 2>&1; then
  error "[ERROR] GitHub CLI is not authenticated. Run: gh auth login"
  exit 1
fi

# --- 2) DETERMINE & VALIDATE BRANCH -------------------------------------------
BRANCH="$(git rev-parse --abbrev-ref HEAD)"

case "$BRANCH" in
  main|master|release|release/*)
    error "[ERROR] Refusing to build a dev wheel from '$BRANCH'."
    error "        Switch to a feature branch first (git switch -c my-feature)."
    exit 1
    ;;
esac

info "Branch: $BRANCH"

# --- 3) ENSURE THE BRANCH IS PUSHED -------------------------------------------
# The workflow builds the remote ref, so the local commit must exist on origin.
LOCAL_SHA="$(git rev-parse HEAD)"
if ! git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
  error "[ERROR] Branch '$BRANCH' does not exist on origin. Push it first:"
  error "        git push -u origin $BRANCH"
  exit 1
fi

REMOTE_SHA="$(git ls-remote origin "refs/heads/$BRANCH" | cut -f1)"
if [ "$LOCAL_SHA" != "$REMOTE_SHA" ]; then
  warn "[WARN] Local HEAD ($LOCAL_SHA) differs from origin/$BRANCH ($REMOTE_SHA)."
  warn "       The workflow will build the version currently on origin."
fi

# --- 4) TRIGGER THE WORKFLOW --------------------------------------------------
# Record the newest existing run so we can detect the one we are about to start.
PREV_RUN_ID="$(gh run list --workflow "$WORKFLOW" --branch "$BRANCH" \
  --limit 1 --json databaseId -q '.[0].databaseId' 2>/dev/null || echo "")"

info "Triggering '$WORKFLOW' on '$BRANCH'..."
gh workflow run "$WORKFLOW" --ref "$BRANCH"

# --- 5) FIND THE NEW RUN ------------------------------------------------------
info "Waiting for the run to be registered..."
RUN_ID=""
for _ in $(seq 1 30); do
  CANDIDATE="$(gh run list --workflow "$WORKFLOW" --branch "$BRANCH" \
    --event workflow_dispatch --limit 1 \
    --json databaseId -q '.[0].databaseId' 2>/dev/null || echo "")"
  if [ -n "$CANDIDATE" ] && [ "$CANDIDATE" != "$PREV_RUN_ID" ]; then
    RUN_ID="$CANDIDATE"
    break
  fi
  sleep 2
done

if [ -z "$RUN_ID" ]; then
  error "[ERROR] Could not find the newly triggered run. Check: gh run list --workflow $WORKFLOW"
  exit 1
fi

RUN_URL="$(gh run view "$RUN_ID" --json url -q '.url')"
info "Run started: $RUN_URL"

# --- 6) WAIT FOR COMPLETION ---------------------------------------------------
info "Watching run until completion..."
if ! gh run watch "$RUN_ID" --exit-status; then
  error "[ERROR] Workflow run failed. See: $RUN_URL"
  exit 1
fi

# --- 7) SHOW THE INSTALL STRING -----------------------------------------------
BUILT_SHA="$(gh run view "$RUN_ID" --json headSha -q '.headSha')"

SPEC="wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git@${BUILT_SHA}"

echo ""
echo "------------------------------------------------------------"
info "📦  Dev wheel built successfully. Install it with:"
echo ""
echo "  # uv, add to the current project"
echo "    uv add \"wandelbots-nova @ git+https://github.com/wandelbotsgmbh/wandelbots-nova.git\" --rev ${BUILT_SHA}"
echo ""
echo "  # pip"
echo "    pip install \"${SPEC}\""
echo ""
echo "  # pyproject.toml"
echo "    [project]"
echo "    dependencies = ["
echo "        \"${SPEC}\","
echo "    ]"
echo ""
echo "(Pinned to the commit that produced run $RUN_ID.)"
echo "Run details: $RUN_URL"
echo "------------------------------------------------------------"
