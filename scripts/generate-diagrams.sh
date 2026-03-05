#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIAGRAMS_DIR="$REPO_ROOT/docs/diagrams"
IMAGE="plantuml/plantuml:latest"

if [ ! -d "$DIAGRAMS_DIR" ]; then
  echo "Error: $DIAGRAMS_DIR does not exist" >&2
  exit 1
fi

puml_files=("$DIAGRAMS_DIR"/*.puml)
if [ ${#puml_files[@]} -eq 0 ]; then
  echo "No .puml files found in $DIAGRAMS_DIR" >&2
  exit 1
fi

echo "Generating diagrams from ${#puml_files[@]} .puml files using Docker ($IMAGE)..."

docker run --rm \
  -v "$DIAGRAMS_DIR":/data \
  "$IMAGE" \
  -tpng /data/*.puml

echo "Generated PNGs:"
ls -lh "$DIAGRAMS_DIR"/*.png 2>/dev/null || echo "  (none)"
