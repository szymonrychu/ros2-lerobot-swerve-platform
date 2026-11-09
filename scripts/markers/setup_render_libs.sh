#!/usr/bin/env bash
# Fetch the OSMesa software-GL libraries from Debian bookworm debs into .cache/render-libs (no root, no apt) so
# pyvista/VTK can render offscreen on a headless box. Idempotent: existing debs are not downloaded again.
# Prints the two exports to eval:  eval "$(bash scripts/markers/setup_render_libs.sh | grep '^export')"
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MIRROR="https://deb.debian.org/debian"
INDEX_URL="$MIRROR/dists/bookworm/main/binary-amd64/Packages.xz"
CACHE="$REPO/.cache/render-libs"
DEBS="$CACHE/debs"
PACKAGES=(libgl1 libglvnd0 libglx0 libx11-6 libxcb1 libxau6 libxdmcp6 libosmesa6 libglapi-mesa libllvm15 libz3-4 libxml2 libicu72)
declare -A PINS=(
  [libgl1]="1.6.0-1" [libglvnd0]="1.6.0-1" [libglx0]="1.6.0-1"
  [libosmesa6]="22.3.6-1+deb12u2" [libglapi-mesa]="22.3.6-1+deb12u2" [libllvm15]="1:15.0.6-4+b1"
)

mkdir -p "$DEBS"
INDEX="$CACHE/Packages"
curl -fsSL "$INDEX_URL" | xz -dc > "$INDEX"

mismatch=0
for pkg in "${PACKAGES[@]}"; do
  read -r version filename < <(awk -v p="$pkg" 'BEGIN { RS = ""; FS = "\n" }
    $1 == "Package: " p { v = ""; f = ""
      for (i = 1; i <= NF; i++) { if ($i ~ /^Version: /) v = substr($i, 10); if ($i ~ /^Filename: /) f = substr($i, 11) }
      print v, f; exit }' "$INDEX")
  if [[ -z "${filename:-}" ]]; then
    echo "package $pkg not found in $INDEX_URL" >&2
    mismatch=1
    continue
  fi
  if [[ -n "${PINS[$pkg]:-}" && "$version" != "${PINS[$pkg]}" ]]; then
    echo "version mismatch: $pkg is $version in the index, pinned ${PINS[$pkg]}" >&2
    mismatch=1
    continue
  fi
  deb="$DEBS/$(basename "$filename")"
  if [[ ! -s "$deb" ]]; then
    curl -fsSL -o "$deb" "$MIRROR/$filename"
  fi
  dpkg-deb -x "$deb" "$CACHE"
done
if [[ "$mismatch" -ne 0 ]]; then
  exit 1
fi

echo "export LD_LIBRARY_PATH=$CACHE/usr/lib/x86_64-linux-gnu:\${LD_LIBRARY_PATH:-}"
echo "export VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow"
