#!/usr/bin/env bash
# Run ansible-lint from the ansible/ directory (required for roles_path resolution).
# Usage: from repo root, run: ./scripts/lint-ansible.sh
# Uses the root project's locked ansible-lint (uv run) once uv.lock carries it, else the same pinned version via uvx.
# Fails when ansible-lint processed fewer than MIN_FILES files: a broken file discovery still reports "Passed".
set -eo pipefail
cd "$(dirname "$0")/.."
ANSIBLE_LINT_VERSION="26.3.0"
MIN_FILES=50
if [ -f uv.lock ] && grep -q '^name = "ansible-lint"$' uv.lock; then
    LINT=(uv run ansible-lint .)
else
    LINT=(uvx --from "ansible-lint==${ANSIBLE_LINT_VERSION}" ansible-lint .)
fi
OUT=$(cd ansible && "${LINT[@]}" 2>&1) || { printf '%s\n' "$OUT"; exit 1; }
printf '%s\n' "$OUT"
PROCESSED=$(printf '%s\n' "$OUT" | sed -E 's/\x1b\[[0-9;]*m//g' | sed -nE 's/.* in ([0-9]+) files processed.*/\1/p' | tail -1)
if [ -z "$PROCESSED" ] || [ "$PROCESSED" -lt "$MIN_FILES" ]; then
    echo "ansible-lint processed ${PROCESSED:-no} files (expected >= ${MIN_FILES}): file discovery is broken" >&2
    exit 1
fi
