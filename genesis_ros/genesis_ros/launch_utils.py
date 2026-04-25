"""Helpers shared by ``genesis_ros/launch/*.py``.

Every launch file that hands an RViz config path to ``rviz2`` should
funnel it through :func:`resolve_rviz_config` so the user can save
display tweaks. Otherwise RViz tries to write back to
``/opt/ros/jazzy/share/genesis_ros/config/rviz/<name>.rviz`` which is
owned by root.
"""
from __future__ import annotations

import os
import shutil


def _user_config_dir() -> str:
    """``$XDG_CONFIG_HOME/genesis_ros`` (or ``~/.config/genesis_ros``)."""
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config")
    return os.path.join(base, "genesis_ros", "rviz")


def resolve_rviz_config(path: str) -> str:
    """Return a writable copy of ``path``.

    If ``path`` is already user-writable, returns it unchanged. If it
    sits in a read-only system location (typical post-deb-install),
    copies it to ``~/.config/genesis_ros/rviz/<basename>`` on first
    call and returns that path. Subsequent launches reuse the copy,
    which means user edits + RViz's auto-save survive across launches.

    On any I/O error the source path is returned unchanged so the
    launch still runs (RViz will just fail to save quietly).
    """
    if not path:
        return path
    try:
        if os.access(path, os.W_OK):
            return path
        if not os.path.isfile(path):
            return path
        dst_dir = _user_config_dir()
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, os.path.basename(path))
        # Refresh the cache if the share-installed config has been
        # updated more recently than the user-local copy. Without this,
        # the very first launch wins forever and subsequent deb upgrades
        # silently regress (RViz keeps loading the stale local copy).
        if (not os.path.exists(dst)
                or os.path.getmtime(path) > os.path.getmtime(dst)):
            shutil.copy2(path, dst)
        return dst
    except Exception:
        return path


__all__ = ["resolve_rviz_config"]
