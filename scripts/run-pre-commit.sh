#!/usr/bin/env bash
# Run pre-commit on the whole repo. Pre-commit only sees files known to git;
# if nothing is tracked yet, we stage everything so hooks actually run.
# Usage: from repo root, run: ./scripts/run-pre-commit.sh
set -e
cd "$(dirname "$0")/.."
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a git repository. Run 'git init' first."
  exit 1
fi
# So that pre-commit run --all-files has files to check (e.g. before first commit)
if [ -z "$(git ls-files)" ]; then
  echo "No tracked files yet; staging all files so pre-commit can run..."
  git add -A
fi
pre-commit run --all-files
