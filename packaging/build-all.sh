#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

# Location of the Genesis engine source tree (needed for the genesis-world
# and genesis-world-assets debs). Overridable via env. Defaults to a sibling
# Genesis checkout one directory up from this repo.
: "${GENESIS_SRC:=$(cd "$REPO/../Genesis" 2>/dev/null && pwd || echo "")}"
if [ -z "$GENESIS_SRC" ] || [ ! -f "$GENESIS_SRC/pyproject.toml" ]; then
    echo "ERROR: GENESIS_SRC must point at a Genesis checkout containing pyproject.toml." >&2
    echo "Got: '$GENESIS_SRC'" >&2
    echo "Set the env var or place a Genesis checkout at $REPO/../Genesis." >&2
    exit 1
fi
export GENESIS_SRC
echo "[build-all] GENESIS_SRC = $GENESIS_SRC"

# Version: base from Genesis's pyproject, snapshot from this repo's git HEAD.
BASE_VERSION="$(python3 -c "import re; m=re.search(r'version\s*=\s*[\"\x27]([^\"\x27]+)', open('$GENESIS_SRC/pyproject.toml').read()); print(m.group(1) if m else '0.0.0')")"
cd "$REPO"
GIT_DATE="$(git log -1 --format=%cd --date=format:%Y%m%d 2>/dev/null || date +%Y%m%d)"
GIT_SHA="$(git rev-parse --short=8 HEAD 2>/dev/null || echo 0000000)"
VERSION="${BASE_VERSION}+git${GIT_DATE}.${GIT_SHA}-1"
export VERSION
echo "[build-all] version = $VERSION"

target="${1:-all}"

build_genesis_world() {
    echo "[build-all] === genesis-world ==="
    "$HERE/genesis-world/build.sh"
}
build_assets() {
    echo "[build-all] === genesis-world-assets ==="
    "$HERE/genesis-world-assets/build.sh"
}
build_ros() {
    echo "[build-all] === ros-jazzy-genesis-ros ==="
    "$HERE/ros-jazzy-genesis-ros/build.sh"
}

case "$target" in
    all)                   build_genesis_world; build_assets; build_ros ;;
    genesis-world)         build_genesis_world ;;
    genesis-world-assets)  build_assets ;;
    ros|ros-jazzy-genesis-ros) build_ros ;;
    *) echo "unknown target: $target" >&2; exit 2 ;;
esac

echo "[build-all] done. artifacts in $OUT"
ls -lh "$OUT"
