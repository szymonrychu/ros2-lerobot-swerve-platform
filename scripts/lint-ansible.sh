#!/usr/bin/env bash
# Run ansible-lint from the ansible/ directory (required for roles_path resolution).
# Usage: from repo root, run: ./scripts/lint-ansible.sh
# Or: poetry run poe lint-ansible
set -e
cd "$(dirname "$0")/.."
(cd ansible && ansible-lint .)
