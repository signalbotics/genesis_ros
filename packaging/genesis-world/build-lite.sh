#!/usr/bin/env bash
# Build the genesis-world-lite deb (no torch / no RL stack).
# Same source tree as build.sh; we swap in debian/control.lite,
# debian/rules.lite, debian/postinst.lite for the duration of the build,
# then restore the originals on exit (even on failure).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../out"
VERSION="${VERSION:?run via ../build-all.sh or set VERSION=...}"

cd "$HERE"

# Stash + swap in lite variants. Trap restores them no matter what.
stash() { mv "debian/$1" "debian/$1.orig"; cp "debian/$1.lite" "debian/$1"; }
restore() {
    for f in control rules postinst; do
        if [ -f "debian/$f.orig" ]; then
            mv -f "debian/$f.orig" "debian/$f"
        fi
    done
}
trap restore EXIT

stash control
stash rules
stash postinst

cat > debian/changelog <<EOF
genesis-world ($VERSION) noble; urgency=medium

  * Snapshot build (lite variant: no torch, no RL stack).

 -- Genesis Packaging <packaging@example.com>  $(date -R)
EOF

chmod +x debian/rules debian/postinst debian/prerm wrapper/genesis

dpkg-buildpackage -us -uc -b -rfakeroot

mv ../genesis-world-lite_*.deb       "$OUT/" 2>/dev/null || true
mv ../genesis-world-lite_*.buildinfo "$OUT/" 2>/dev/null || true
mv ../genesis-world-lite_*.changes   "$OUT/" 2>/dev/null || true
# dpkg-buildpackage also writes a generic genesis-world_*.changes from the
# Source: line — keep it next to the lite artefacts.
mv ../genesis-world_*.changes        "$OUT/" 2>/dev/null || true
