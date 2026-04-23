"""Tensor to ROS message conversion helpers.

All helpers are pure functions; heavy dependencies such as cv_bridge are
imported lazily inside the functions that need them.
"""
from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


def _as_numpy(x):
    """Best-effort conversion of torch / taichi / jax tensors to numpy."""
    if isinstance(x, np.ndarray):
        return x
    detach = getattr(x, "detach", None)
    if detach is not None:
        try:
            return detach().cpu().numpy()
        except Exception:
            pass
    return np.asarray(x)


def select_env(tensor, env_idx: int):
    """Slice the leading batch dimension when the scene is vectorised."""
    arr = _as_numpy(tensor)
    if arr.ndim == 0:
        return arr
    if arr.shape[0] > env_idx and arr.ndim > 1:
        return arr[env_idx]
    return arr


def ros_time_from_ns(nanoseconds: int):
    """Build a builtin_interfaces.msg.Time from an int ns value."""
    from builtin_interfaces.msg import Time

    nanoseconds = int(nanoseconds)
    msg = Time()
    msg.sec = nanoseconds // 1_000_000_000
    msg.nanosec = nanoseconds % 1_000_000_000
    return msg


def quat_wxyz_to_ros(q):
    """Convert a Genesis (w, x, y, z) quat to geometry_msgs/Quaternion.

    Genesis uses (w, x, y, z) ordering; ROS uses (x, y, z, w).
    """
    from geometry_msgs.msg import Quaternion

    arr = _as_numpy(q).reshape(-1)
    w, x, y, z = float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])
    out = Quaternion()
    out.x = x
    out.y = y
    out.z = z
    out.w = w
    return out


def xyz_quat_to_transform_stamped(
    xyz,
    quat_wxyz,
    *,
    stamp,
    frame_id: str,
    child_frame_id: str,
):
    """Assemble a geometry_msgs/TransformStamped from Genesis pose data."""
    from geometry_msgs.msg import TransformStamped, Vector3

    pos = _as_numpy(xyz).reshape(-1)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation = Vector3(
        x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
    )
    msg.transform.rotation = quat_wxyz_to_ros(quat_wxyz)
    return msg


_IMAGE_ENCODINGS = {
    "rgb8": (3, 1, "uint8"),
    "bgr8": (3, 1, "uint8"),
    "mono8": (1, 1, "uint8"),
    "mono16": (1, 2, "uint16"),
    "32FC1": (1, 4, "float32"),
}


def np_to_image_msg(array, encoding: str, *, stamp=None, frame_id: str = ""):
    """Manually pack a numpy array into sensor_msgs/Image.

    Deliberately avoids cv_bridge so conversions.py has no hard ROS image
    deps; publishers that want cv_bridge can import it themselves.
    """
    from sensor_msgs.msg import Image

    if encoding not in _IMAGE_ENCODINGS:
        raise ValueError(
            "np_to_image_msg: unsupported encoding "
            + repr(encoding)
            + ". Supported: "
            + str(sorted(_IMAGE_ENCODINGS))
        )
    channels, bytes_per_channel, dtype = _IMAGE_ENCODINGS[encoding]
    arr = _as_numpy(array)
    if channels == 1 and arr.ndim == 2:
        arr = arr[..., None]
    if arr.ndim != 3 or arr.shape[-1] != channels:
        raise ValueError(
            "np_to_image_msg: array shape "
            + str(arr.shape)
            + " does not match encoding "
            + repr(encoding)
        )
    if arr.dtype != np.dtype(dtype):
        arr = arr.astype(dtype, copy=False)
    arr = np.ascontiguousarray(arr)

    msg = Image()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1] * channels * bytes_per_channel)
    msg.data = arr.tobytes()
    return msg


def np_to_pointcloud2(
    points,
    *,
    stamp=None,
    frame_id: str = "",
    fields: Sequence[str] = ("x", "y", "z"),
    extra_fields: Iterable = (),
):
    """Pack an (N, 3) cloud (optionally with extra float32 fields) into
    sensor_msgs/PointCloud2.
    """
    from sensor_msgs.msg import PointCloud2, PointField

    xyz = _as_numpy(points).reshape(-1, 3).astype(np.float32, copy=False)
    n = xyz.shape[0]

    field_arrays = [
        (fields[0], xyz[:, 0]),
        (fields[1], xyz[:, 1]),
        (fields[2], xyz[:, 2]),
    ]
    for name, values in extra_fields:
        flat = _as_numpy(values).reshape(-1).astype(np.float32, copy=False)
        if flat.shape[0] != n:
            raise ValueError(
                "np_to_pointcloud2: extra field "
                + repr(name)
                + " has length "
                + str(flat.shape[0])
                + " but xyz has "
                + str(n)
            )
        field_arrays.append((name, flat))

    dtype = np.dtype({
        "names": [name for name, _ in field_arrays],
        "formats": ["<f4"] * len(field_arrays),
        "offsets": [4 * i for i in range(len(field_arrays))],
        "itemsize": 4 * len(field_arrays),
    })
    buf = np.empty(n, dtype=dtype)
    for name, values in field_arrays:
        buf[name] = values

    msg = PointCloud2()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(
            name=name,
            offset=4 * i,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for i, (name, _) in enumerate(field_arrays)
    ]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = dtype.itemsize * n
    msg.data = buf.tobytes()
    msg.is_dense = True
    return msg


__all__ = [
    "select_env",
    "ros_time_from_ns",
    "quat_wxyz_to_ros",
    "xyz_quat_to_transform_stamped",
    "np_to_image_msg",
    "np_to_pointcloud2",
]
