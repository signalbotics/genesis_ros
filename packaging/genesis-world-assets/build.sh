#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../out"
VERSION="${VERSION:?run via ../build-all.sh}"

cd "$HERE"
cat > debian/changelog <<EOF
genesis-world-assets ($VERSION) noble; urgency=medium

  * Snapshot of genesis/assets/ from git HEAD.

 -- Genesis Packaging <packaging@example.com>  $(date -R)
EOF

chmod +x debian/rules

dpkg-buildpackage -us -uc -b -rfakeroot

mv ../genesis-world-assets_*.deb "$OUT/" 2>/dev/null || true
mv ../genesis-world-assets_*.buildinfo "$OUT/" 2>/dev/null || true
mv ../genesis-world-assets_*.changes "$OUT/" 2>/dev/null || true
