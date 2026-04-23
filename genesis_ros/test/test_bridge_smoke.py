"""Smoke test for the Genesis<->ROS 2 bridge main loop.

Runs headless. Skips when either Genesis or ``rclpy`` are not importable so
the test suite can still execute on machines that carry only one of the two
stacks. ROS 2 runtime tests (topic discovery, launch_testing, etc.) live in
Group 10's ``test_integration.py``.
"""
from __future__ import annotations

import pytest


rclpy = pytest.importorskip("rclpy")
gs = pytest.importorskip("genesis")


def _build_scene():
    """Minimal CPU scene, one step ~ 10 ms."""
    try:
        gs.init(backend=gs.cpu)
    except Exception:
        pass
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.build()
    return scene


def test_bridge_runs_ten_iterations():
    """Instantiate the bridge, drive the main loop by hand, verify RTF+shutdown."""
    from genesis_ros.node import GenesisRosBridge

    scene = _build_scene()
    bridge = GenesisRosBridge(scene, node_name="genesis_bridge_smoke")
    try:
        for _ in range(10):
            bridge._run_one_iteration()

        # RTF starts at 1.0 and is updated via an EMA; it must remain finite
        # and strictly positive for the run to be considered healthy.
        assert bridge.rtf > 0.0
        # The main loop never asked for shutdown.
        assert bridge._shutdown is False
    finally:
        bridge.shutdown()
