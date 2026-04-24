#!/usr/bin/env bash
# Wraps bloom-generate to turn genesis_ros2_control/ (native C++
# hardware_interface plugin) into ros-jazzy-genesis-ros2-control_*.deb.
#
# All runtime deps (hardware_interface, pluginlib, rclcpp,
# rclcpp_lifecycle, controller_manager) are stock ROS 2 keys that
# rosdep resolves cleanly -- no post-bloom fixups needed.
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

if [ -z "${ROS_DISTRO:-}" ] && [ ! -d "/opt/ros/$ROSDISTRO" ]; then
    echo "ROS 2 $ROSDISTRO not detected under /opt/ros/$ROSDISTRO" >&2
    exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cp -a "$REPO_ROOT/genesis_ros2_control" "$WORK/genesis_ros2_control"
cd "$WORK/genesis_ros2_control"

# Pin the package.xml version to the stamped snapshot (strip +git suffix
# for catkin_pkg's strict MAJOR.MINOR.PATCH parser).
UPSTREAM_VER="${VERSION%+*}"
UPSTREAM_VER="${UPSTREAM_VER%-*}"
python3 - "$UPSTREAM_VER" <<'PY'
import re, sys
p = "package.xml"
v = sys.argv[1]
src = open(p).read()
src = re.sub(r"<version>[^<]+</version>", f"<version>{v}</version>", src, count=1)
open(p, "w").write(src)
PY

script -qfec "yes y | bloom-generate rosdebian \
    --os-name ubuntu --os-version $UBUNTU_REL --ros-distro $ROSDISTRO" /dev/null || true

[ -f debian/control ] || { echo "bloom did not generate debian/control" >&2; exit 1; }

DEBFULLNAME="Genesis Packaging" DEBEMAIL="packaging@example.com" \
dch --newversion "$VERSION" --distribution "$UBUNTU_REL" --force-distribution \
    --controlmaint "Snapshot build from git HEAD."

chmod +x debian/rules

dpkg-buildpackage -us -uc -b -rfakeroot

cd ..
mv ros-"$ROSDISTRO"-genesis-ros2-control_*.deb       "$OUT/" 2>/dev/null || true
mv ros-"$ROSDISTRO"-genesis-ros2-control_*.buildinfo "$OUT/" 2>/dev/null || true
mv ros-"$ROSDISTRO"-genesis-ros2-control_*.changes   "$OUT/" 2>/dev/null || true
