#!/usr/bin/env bash
# Drive build-all.sh inside a Noble/Jazzy container so the vendored venv
# targets Python 3.12 and bloom has the right rosdep/tooling available —
# regardless of what the build host runs.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMG="genesis-deb-builder:noble"

# Genesis engine source tree. Override with GENESIS_SRC=/path/to/Genesis if
# your checkout lives elsewhere. Default: sibling "Genesis/" directory next
# to this repo.
: "${GENESIS_SRC:=$(cd "$REPO/../Genesis" 2>/dev/null && pwd || echo "")}"
if [ -z "$GENESIS_SRC" ] || [ ! -f "$GENESIS_SRC/pyproject.toml" ]; then
    echo "ERROR: GENESIS_SRC must point at a Genesis checkout." >&2
    echo "Set GENESIS_SRC=/path/to/Genesis or place it at $REPO/../Genesis." >&2
    exit 1
fi

# Rebuild the builder image when it doesn't exist OR when Dockerfile.noble
# has been modified since the image was last built. Prevents the classic
# "I updated the Dockerfile but the cached image is stale" footgun.
_image_mtime="$(docker image inspect --format '{{.Created}}' "$IMG" 2>/dev/null \
    | xargs -I{} date -d {} +%s 2>/dev/null || echo 0)"
_dockerfile_mtime="$(stat -c %Y "$HERE/Dockerfile.noble")"
if [ "$_image_mtime" -lt "$_dockerfile_mtime" ]; then
    echo "[build-in-docker] (re)building image $IMG"
    docker build -t "$IMG" -f "$HERE/Dockerfile.noble" "$HERE"
fi

PIP_CACHE="$HERE/.pip-cache"
mkdir -p "$PIP_CACHE"

# --network host so pip reaches PyPI and rosdep reaches github.
# Mount a persistent pip cache so rebuilds don't re-download ~3 GB of wheels.
docker run --rm \
    --network host \
    -v "$REPO":/work \
    -v "$GENESIS_SRC":/genesis-src:ro \
    -v "$PIP_CACHE":/root/.cache/pip \
    -w /work \
    -e HOME=/root \
    -e GENESIS_SRC=/genesis-src \
    "$IMG" \
    bash -c "git config --global --add safe.directory /work && ./packaging/build-all.sh ${*:-all}"

echo "[build-in-docker] artifacts:"
ls -lh "$REPO/packaging/out/"*.deb
