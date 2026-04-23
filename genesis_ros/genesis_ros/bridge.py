"""Console entry point: ``ros2 run genesis_ros genesis_bridge``.

Builds a minimal empty scene so the bridge can be smoke-tested in isolation
(before any robots are loaded) and hands control to
:meth:`GenesisRosBridge.spin`. Real launches (Group 9) construct their own
scenes and call into the :mod:`genesis_ros` library directly; this entry
point exists so ``colcon build`` + ``ros2 run`` yields a working ``/clock``
out of the box.
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional, Sequence

from .node import GenesisRosBridge
from .publishers.clock import ClockPublisher


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="genesis_bridge",
        description=(
            "Run a minimal Genesis<->ROS 2 bridge (empty scene, /clock live)."
        ),
    )
    parser.add_argument(
        "--node-name",
        default="genesis_bridge",
        help="Name for the root rclpy node (default: genesis_bridge).",
    )
    parser.add_argument(
        "--env-idx",
        type=int,
        default=0,
        help="Which env to expose when scene.n_envs > 1 (default: 0).",
    )
    parser.add_argument(
        "--no-sim-time",
        action="store_true",
        help="Do not set use_sim_time=True on the bridge node.",
    )
    # ros2 run appends --ros-args ...; tolerate unknowns and let rclpy.init
    # pick them up implicitly.
    args, _ros_extra = parser.parse_known_args(argv)
    return args


def _build_default_scene():
    """Construct a minimal CPU-backed empty scene with a ground plane.

    Kept import-local so simply importing this module does not spin Genesis up
    (important for the test suite).
    """
    import genesis as gs

    try:
        gs.init()
    except Exception:
        # Already initialised -- reuse the global state.
        pass

    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.build()
    return scene


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    scene = _build_default_scene()
    bridge = GenesisRosBridge(
        scene,
        node_name=args.node_name,
        env_idx=args.env_idx,
        use_sim_time=not args.no_sim_time,
    )
    bridge.register_publisher(
        ClockPublisher(bridge.node, scene, bridge.registry, cfg={})
    )
    bridge.spin()
    return 0


if __name__ == "__main__":  # pragma: no cover - console entry point
    raise SystemExit(main())
