"""Spawn a URDF-backed entity into a running ``genesis_bridge``.

Mirrors the ergonomics of ``ros_gz_sim spawn``: one-shot launch that
calls ``/genesis/spawn_urdf`` and exits. Example:

.. code-block:: bash

   ros2 launch genesis_ros spawn_entity.launch.py \\
       name:=box urdf_path:=/tmp/box.urdf x:=0.5 y:=0.0 z:=0.2

Arguments
---------
name        Required. Entity name (becomes the ROS namespace).
urdf_path   Absolute path to a URDF/xacro file. Xacro files are expanded
            at launch time via the ``xacro`` CLI.
urdf_xml    Raw URDF XML string (alternative to ``urdf_path``).
x, y, z     Spawn position (default 0 0 0).
qw qx qy qz Spawn orientation as w,x,y,z quaternion (default 1 0 0 0).
fixed       If true, attach the base to the world (default false).
topic       Service name (default ``/genesis/spawn_urdf``).

``genesis_bridge`` has to be already running; the call defers until the
next scene reset boundary when the scene is already built and Genesis
cannot add entities on the fly (see ``services/entity.py``).
"""
import json
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_call(context, *_args, **_kwargs):
    name = LaunchConfiguration("name").perform(context).strip()
    urdf_path = LaunchConfiguration("urdf_path").perform(context).strip()
    urdf_xml = LaunchConfiguration("urdf_xml").perform(context)
    topic = LaunchConfiguration("topic").perform(context).strip() or "/genesis/spawn_urdf"

    try:
        pos = [
            float(LaunchConfiguration("x").perform(context)),
            float(LaunchConfiguration("y").perform(context)),
            float(LaunchConfiguration("z").perform(context)),
        ]
    except ValueError:
        pos = [0.0, 0.0, 0.0]
    try:
        quat = [
            float(LaunchConfiguration("qw").perform(context)),
            float(LaunchConfiguration("qx").perform(context)),
            float(LaunchConfiguration("qy").perform(context)),
            float(LaunchConfiguration("qz").perform(context)),
        ]
    except ValueError:
        quat = [1.0, 0.0, 0.0, 0.0]
    fixed_raw = LaunchConfiguration("fixed").perform(context).lower()
    fixed = fixed_raw in ("1", "true", "yes", "on")

    if not name:
        raise RuntimeError("spawn_entity.launch.py: 'name' is required")
    if not urdf_path and not urdf_xml:
        raise RuntimeError(
            "spawn_entity.launch.py: either 'urdf_path' or 'urdf_xml' is required"
        )

    # Expand xacro inline when asked for one.
    if urdf_path and urdf_path.endswith(".xacro"):
        try:
            urdf_xml = subprocess.check_output(["xacro", urdf_path]).decode("utf-8")
            urdf_path = ""
        except Exception:
            pass
    if urdf_path and not os.path.isabs(urdf_path):
        urdf_path = os.path.abspath(urdf_path)

    payload = {"name": name, "pose": pos, "quat": quat, "fixed": fixed}
    if urdf_path:
        payload["urdf_path"] = urdf_path
    if urdf_xml:
        payload["urdf_xml"] = urdf_xml

    # The service is std_srvs/Trigger by default (the JSON payload is the
    # actual argument vehicle); simulation_interfaces/SpawnEntity ships
    # with its own fields but this launch stays on the JSON channel so it
    # works regardless of which backend genesis_ros detects at runtime.
    # We shuttle the payload through a ROS parameter (spawn_urdf.args) on
    # the genesis_bridge node before calling the service, since
    # std_srvs/Trigger has no request body.
    param_set = ExecuteProcess(
        cmd=[
            "ros2", "param", "set", "/genesis_bridge",
            "spawn_urdf.args", json.dumps(payload),
        ],
        output="screen",
        shell=False,
    )
    service_call = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", topic, "std_srvs/srv/Trigger", "{}",
        ],
        output="screen",
        shell=False,
    )
    return [param_set, service_call]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("name", default_value="", description="Entity name / namespace."),
        DeclareLaunchArgument("urdf_path", default_value="", description="Absolute URDF / xacro path."),
        DeclareLaunchArgument("urdf_xml", default_value="", description="Inline URDF XML string."),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("qw", default_value="1.0"),
        DeclareLaunchArgument("qx", default_value="0.0"),
        DeclareLaunchArgument("qy", default_value="0.0"),
        DeclareLaunchArgument("qz", default_value="0.0"),
        DeclareLaunchArgument("fixed", default_value="false"),
        DeclareLaunchArgument("topic", default_value="/genesis/spawn_urdf"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_call)])
