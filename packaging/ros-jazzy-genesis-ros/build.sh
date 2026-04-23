#!/usr/bin/env bash
# Wraps bloom-generate to turn genesis_ros/ into ros-jazzy-genesis-ros_*.deb.
#
# bloom needs the package.xml dependencies to resolve via rosdep. Everything
# in genesis_ros/package.xml is either a standard ROS 2 key or a well-known
# community package (simulation_interfaces, topic_based_ros2_control). The
# non-rosdep runtime dep on genesis-world is injected post-bloom into
# debian/control because rosdep can't know about our in-house package.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../out"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
VERSION="${VERSION:?run via ../build-all.sh}"
ROSDISTRO="${ROSDISTRO:-jazzy}"
UBUNTU_REL="${UBUNTU_REL:-noble}"

if ! command -v bloom-generate >/dev/null; then
    echo "bloom-generate not found. sudo apt install python3-bloom" >&2
    exit 1
fi

# Some buildtool keys (ament_python) aren't in the stock rosdep sources.
# Register a local override pointing at the apt packages shipped with ROS 2.
LOCAL_ROSDEP="/tmp/genesis-rosdep.yaml"
cat > "$LOCAL_ROSDEP" <<EOF
ament_python:
  ubuntu:
    noble: [ros-${ROSDISTRO}-ament-package]
EOF
LOCAL_LIST="/etc/ros/rosdep/sources.list.d/10-genesis-local.list"
if [ -w /etc/ros/rosdep/sources.list.d ] 2>/dev/null || [ "$(id -u)" = "0" ]; then
    mkdir -p /etc/ros/rosdep/sources.list.d
    echo "yaml file://$LOCAL_ROSDEP" > "$LOCAL_LIST"
    rosdep update 2>&1 | tail -1
fi
if [ -z "${ROS_DISTRO:-}" ] && [ ! -d "/opt/ros/$ROSDISTRO" ]; then
    echo "ROS 2 $ROSDISTRO not detected under /opt/ros/$ROSDISTRO" >&2
    exit 1
fi

# Work in a throwaway copy so bloom's edits don't touch the source tree.
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cp -a "$REPO_ROOT/genesis_ros" "$WORK/genesis_ros"
cd "$WORK/genesis_ros"

# catkin_pkg only accepts strict MAJOR.MINOR.PATCH in package.xml <version>.
# Strip the +git... snapshot suffix for package.xml; the full snapshot
# version still ends up in debian/changelog via dch below.
UPSTREAM_VER="${VERSION%+*}"
UPSTREAM_VER="${UPSTREAM_VER%-*}"
python3 - "$UPSTREAM_VER" <<'PY'
import re, sys
p = "package.xml"
v = sys.argv[1]
src = open(p).read()
src = re.sub(r"<version>[^<]+</version>", f"<version>{v}</version>", src, count=1)
# Drop community rosdep keys bloom can't resolve. topic_based_ros2_control
# and simulation_interfaces are optional — consumers apt-install them
# separately when they want those features.
for key in ("simulation_interfaces", "topic_based_ros2_control"):
    src = re.sub(rf"\s*<exec_depend>{key}</exec_depend>\n?", "\n", src)
open(p, "w").write(src)
PY

# bloom-generate prompts on /dev/tty when it can't resolve a rosdep key.
# We run it under `script` to allocate a pty so the prompt actually works,
# and feed "y" on stdin so unresolved optional keys are skipped.
script -qfec "yes y | bloom-generate rosdebian \
    --os-name ubuntu --os-version $UBUNTU_REL --ros-distro $ROSDISTRO" /dev/null || true

[ -f debian/control ] || { echo "bloom did not generate debian/control" >&2; exit 1; }

# Inject Depends on genesis-world (our sibling .deb — rosdep can't see it).
python3 - <<'PY'
import re
p = "debian/control"
src = open(p).read()
src = re.sub(r"^(Depends:[^\n]*)", r"\1, genesis-world", src, count=1, flags=re.M)
open(p, "w").write(src)
PY

# Stamp the changelog with our snapshot version.
DEBFULLNAME="Genesis Packaging" DEBEMAIL="packaging@example.com" \
dch --newversion "$VERSION" --distribution "$UBUNTU_REL" --force-distribution \
    --controlmaint "Snapshot build from git HEAD."

chmod +x debian/rules

dpkg-buildpackage -us -uc -b -rfakeroot

cd ..
mv ros-"$ROSDISTRO"-genesis-ros_*.deb       "$OUT/" 2>/dev/null || true
mv ros-"$ROSDISTRO"-genesis-ros_*.buildinfo "$OUT/" 2>/dev/null || true
mv ros-"$ROSDISTRO"-genesis-ros_*.changes   "$OUT/" 2>/dev/null || true
