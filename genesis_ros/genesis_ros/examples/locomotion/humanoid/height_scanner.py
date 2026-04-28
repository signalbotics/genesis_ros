"""Height-scan sensor — IsaacLab `RayCaster` parity for static terrain.

Builds a fixed grid of XY offsets in the robot's heading-aligned (yaw-only)
frame, transforms them to world coords each step, and looks up terrain
heights via `TerrainImporter.sample_height`. Returns `base_z - hit_z` clipped
to ±1 m, matching the `mdp.height_scan` observation in IsaacLab.

For static terrain this is exact and orders of magnitude faster than
raycasting; we keep the API narrow so a raycast backend could replace it
later if dynamic terrain shows up.
"""

from __future__ import annotations

import torch

import genesis as gs


class HeightScanner:
    def __init__(
        self,
        terrain,
        *,
        size: tuple[float, float] = (1.6, 1.0),
        resolution: float = 0.1,
        z_offset: float = 0.5,
        device: str | None = None,
    ):
        self.terrain = terrain
        self.z_offset = z_offset
        self.device = device or gs.device

        # Pattern grid in robot-yaw frame: shape (N, 2).
        sx, sy = size
        nx = int(round(sx / resolution)) + 1
        ny = int(round(sy / resolution)) + 1
        xs = torch.linspace(-sx / 2, sx / 2, nx, device=self.device)
        ys = torch.linspace(-sy / 2, sy / 2, ny, device=self.device)
        gx, gy = torch.meshgrid(xs, ys, indexing="ij")
        self.pattern = torch.stack([gx.reshape(-1), gy.reshape(-1)], dim=-1)  # [P, 2]
        self.num_rays = self.pattern.shape[0]

    def scan(self, base_pos: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """base_pos: [N,3], yaw: [N]. Returns [N, num_rays] = base_z - hit_z, clipped ±1m."""
        cos_y = torch.cos(yaw).unsqueeze(-1)  # [N,1]
        sin_y = torch.sin(yaw).unsqueeze(-1)
        px = self.pattern[:, 0].unsqueeze(0)  # [1, P]
        py = self.pattern[:, 1].unsqueeze(0)
        # Rotate pattern by yaw, translate to base_xy.
        wx = base_pos[:, 0:1] + px * cos_y - py * sin_y
        wy = base_pos[:, 1:2] + px * sin_y + py * cos_y
        xy = torch.stack([wx, wy], dim=-1)  # [N, P, 2]
        hit_z = self.terrain.sample_height(xy)  # [N, P]
        # IsaacLab returns sensor_z - hit_z - offset, clipped to [-1, 1].
        sensor_z = (base_pos[:, 2] + self.z_offset).unsqueeze(-1)
        return torch.clamp(sensor_z - hit_z - self.z_offset, -1.0, 1.0)
