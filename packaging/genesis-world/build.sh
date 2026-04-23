#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../out"
VERSION="${VERSION:?run via ../build-all.sh}"

cd "$HERE"
# Seed debian/changelog with the snapshot version.
cat > debian/changelog <<EOF
genesis-world ($VERSION) noble; urgency=medium

  * Snapshot build from git HEAD.

 -- Genesis Packaging <packaging@example.com>  $(date -R)
EOF

chmod +x debian/rules debian/postinst debian/prerm wrapper/genesis

# dpkg-buildpackage needs the source tree name to match <pkg>-<upstream>.
# We build in-place with --build=binary to skip the orig.tar dance.
dpkg-buildpackage -us -uc -b -rfakeroot

# Artefacts land one level up from the source dir.
mv ../genesis-world_*.deb  "$OUT/" 2>/dev/null || true
mv ../genesis-world_*.buildinfo "$OUT/" 2>/dev/null || true
mv ../genesis-world_*.changes "$OUT/" 2>/dev/null || true
