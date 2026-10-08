#!/usr/bin/env bash
# Run ansible-lint from the ansible/ directory (required for roles_path resolution).
# Usage: from repo root, run: ./scripts/lint-ansible.sh
# Uses the root project's locked ansible-lint (uv run) once uv.lock carries it, else the same pinned version via uvx.
set -e
cd "$(dirname "$0")/.."
ANSIBLE_LINT_VERSION="26.3.0"
if [ -f uv.lock ] && grep -q '^name = "ansible-lint"$' uv.lock; then
    (cd ansible && uv run ansible-lint .)
else
    (cd ansible && uvx --from "ansible-lint==${ANSIBLE_LINT_VERSION}" ansible-lint .)
fi
