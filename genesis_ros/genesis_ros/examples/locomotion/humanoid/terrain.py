"""Procedural terrain generator + importer ported from IsaacLab.

Generates a grid of (num_rows x num_cols) sub-terrains as one big
heightfield, then loads it into the Genesis scene via `gs.morphs.Terrain`
(SDF-based collision, no convex decomposition — required for non-convex
geometry like stairs and rough hills).

Per-env spawn origins are stored in `env_origins[N, 3]`. The terrain-level
curriculum mutates per-env (row, col) indices on `update_env_origins(...)`,
mirroring `isaaclab.terrains.TerrainImporter`.

The same heightfield is also kept as a torch tensor on-device so the
height-scan sensor can do O(1) bilinear lookup without raycasting.
"""

from __future__ import annotations

import dataclasses
from typing import Callable

import numpy as np
import torch

import genesis as gs


@dataclasses.dataclass
class SubTerrainCfg:
    """One sub-terrain generator entry."""

    name: str
    proportion: float
    fn: Callable[[np.random.Generator, float, "TerrainGeneratorCfg", int], np.ndarray]
    """Signature: (rng, difficulty in [0,1], parent cfg, level_idx) -> (H, W) heightfield in metres."""


@dataclasses.dataclass
class TerrainGeneratorCfg:
    """Mirrors `isaaclab.terrains.TerrainGeneratorCfg` keys we care about."""

    size: tuple[float, float] = (8.0, 8.0)
    border_width: float = 20.0
    num_rows: int = 10
    num_cols: int = 20
    horizontal_scale: float = 0.1
    vertical_scale: float = 0.005  # not used (we keep heights in metres)
    slope_threshold: float = 0.75
    sub_terrains: list[SubTerrainCfg] = dataclasses.field(default_factory=list)
    seed: int = 0


# ---------------------------------------------------------------------------
# Sub-terrain heightfield generators. Each returns an (H, W) float32 array in
# metres. H = size_y / horizontal_scale, W = size_x / horizontal_scale.
# ---------------------------------------------------------------------------


def _grid_dims(cfg: TerrainGeneratorCfg) -> tuple[int, int]:
    h = int(round(cfg.size[1] / cfg.horizontal_scale))
    w = int(round(cfg.size[0] / cfg.horizontal_scale))
    return h, w


def hf_random_uniform(
    rng: np.random.Generator, difficulty: float, cfg: TerrainGeneratorCfg, level: int
) -> np.ndarray:
    h, w = _grid_dims(cfg)
    noise_min, noise_max = 0.02, 0.10
    amp = noise_min + (noise_max - noise_min) * difficulty
    step = 0.02
    # Discretize to multiples of `step` to match IsaacLab's HfRandomUniform.
    raw = rng.uniform(-amp, amp, size=(h, w))
    raw = np.round(raw / step) * step
    return raw.astype(np.float32)


def hf_pyramid_slope(
    rng: np.random.Generator, difficulty: float, cfg: TerrainGeneratorCfg, level: int, *, inverted: bool = False
) -> np.ndarray:
    """Pyramid of constant slope, with a flat platform on top (or bottom)."""
    h, w = _grid_dims(cfg)
    slope = 0.0 + 0.4 * difficulty
    sx, sy = cfg.size
    cx, cy = sx / 2, sy / 2
    platform_w = 2.0
    xs = np.linspace(0, sx, w, dtype=np.float32)
    ys = np.linspace(0, sy, h, dtype=np.float32)
    X, Y = np.meshgrid(xs, ys)
    dx = np.abs(X - cx)
    dy = np.abs(Y - cy)
    dist = np.maximum(dx, dy)
    plat_extent = platform_w / 2.0
    height = np.maximum(0.0, dist - plat_extent) * slope
    if inverted:
        height = -height
    return height.astype(np.float32)


def hf_inverted_pyramid_slope(rng, difficulty, cfg, level):
    return hf_pyramid_slope(rng, difficulty, cfg, level, inverted=True)


def mesh_pyramid_stairs(
    rng: np.random.Generator, difficulty: float, cfg: TerrainGeneratorCfg, level: int, *, inverted: bool = False
) -> np.ndarray:
    """Concentric square steps. Each ring is one step higher (or lower)."""
    h, w = _grid_dims(cfg)
    step_height = 0.05 + (0.23 - 0.05) * difficulty
    step_width = 0.3
    platform_w = 3.0
    border = 1.0
    sx, sy = cfg.size
    cx, cy = sx / 2, sy / 2
    xs = np.linspace(0, sx, w, dtype=np.float32)
    ys = np.linspace(0, sy, h, dtype=np.float32)
    X, Y = np.meshgrid(xs, ys)
    dist = np.maximum(np.abs(X - cx), np.abs(Y - cy))
    inner = platform_w / 2.0
    outer_limit = min(sx, sy) / 2.0 - border
    n_steps = int(np.floor((outer_limit - inner) / step_width))
    height = np.zeros_like(dist, dtype=np.float32)
    for k in range(1, n_steps + 1):
        r = inner + k * step_width
        height = np.where(dist >= r, k * step_height, height)
    if inverted:
        height = -height
    return height


def mesh_inverted_pyramid_stairs(rng, difficulty, cfg, level):
    return mesh_pyramid_stairs(rng, difficulty, cfg, level, inverted=True)


def mesh_random_grid(
    rng: np.random.Generator, difficulty: float, cfg: TerrainGeneratorCfg, level: int
) -> np.ndarray:
    """Tiles the patch with random-height boxes (constant within each cell)."""
    h, w = _grid_dims(cfg)
    grid_w_m = 0.45
    plat_w = 2.0
    height_min, height_max = 0.05, 0.05 + 0.15 * difficulty
    cells_x = max(1, int(cfg.size[0] / grid_w_m))
    cells_y = max(1, int(cfg.size[1] / grid_w_m))
    cell_h = rng.uniform(height_min, height_max, size=(cells_y, cells_x)).astype(np.float32)
    field = np.zeros((h, w), dtype=np.float32)
    for iy in range(cells_y):
        for ix in range(cells_x):
            y0 = int(iy * h / cells_y)
            y1 = int((iy + 1) * h / cells_y)
            x0 = int(ix * w / cells_x)
            x1 = int((ix + 1) * w / cells_x)
            field[y0:y1, x0:x1] = cell_h[iy, ix]
    # Carve out central platform.
    sx, sy = cfg.size
    xs = np.linspace(0, sx, w, dtype=np.float32)
    ys = np.linspace(0, sy, h, dtype=np.float32)
    X, Y = np.meshgrid(xs, ys)
    cx, cy = sx / 2, sy / 2
    in_platform = (np.abs(X - cx) < plat_w / 2) & (np.abs(Y - cy) < plat_w / 2)
    field = np.where(in_platform, 0.0, field)
    return field


# Default rough-terrain sub-terrains, weights matching IsaacLab `ROUGH_TERRAINS_CFG`.
DEFAULT_ROUGH_SUB_TERRAINS = [
    SubTerrainCfg("pyramid_stairs", 0.2, mesh_pyramid_stairs),
    SubTerrainCfg("pyramid_stairs_inv", 0.2, mesh_inverted_pyramid_stairs),
    SubTerrainCfg("boxes", 0.2, mesh_random_grid),
    SubTerrainCfg("random_rough", 0.2, hf_random_uniform),
    SubTerrainCfg("hf_pyramid_slope", 0.1, hf_pyramid_slope),
    SubTerrainCfg("hf_pyramid_slope_inv", 0.1, hf_inverted_pyramid_slope),
]


def default_rough_cfg() -> TerrainGeneratorCfg:
    return TerrainGeneratorCfg(
        size=(8.0, 8.0),
        border_width=20.0,
        num_rows=10,
        num_cols=20,
        horizontal_scale=0.1,
        vertical_scale=0.005,
        slope_threshold=0.75,
        sub_terrains=list(DEFAULT_ROUGH_SUB_TERRAINS),
        seed=0,
    )


# ---------------------------------------------------------------------------
# TerrainImporter: builds the global mesh, holds env-origin/curriculum state.
# ---------------------------------------------------------------------------


class TerrainImporter:
    """Build the rough-terrain grid mesh, manage per-env origins + curriculum."""

    def __init__(self, scene: gs.Scene, cfg: TerrainGeneratorCfg, num_envs: int, device: str | None = None):
        self.cfg = cfg
        self.num_envs = num_envs
        self.device = device or gs.device

        rng = np.random.default_rng(cfg.seed)

        # 1) Generate every sub-terrain heightfield.
        # Difficulty per row: row 0 = 0.0, last row = 1.0.
        # Each row contains num_cols sub-terrains chosen by `proportion`-weighted indices.
        weights = np.array([s.proportion for s in cfg.sub_terrains], dtype=np.float64)
        weights = weights / weights.sum()
        # Assign one terrain type per (row, col) — same column index = same type so move_up keeps type.
        # IsaacLab assigns column->type as `cumulative_sum_of_weights * num_cols`.
        col_type = np.zeros(cfg.num_cols, dtype=np.int64)
        cumw = 0.0
        idx_so_far = 0
        for ti, w in enumerate(weights):
            target = (cumw + w) * cfg.num_cols
            while idx_so_far < min(cfg.num_cols, int(round(target))):
                col_type[idx_so_far] = ti
                idx_so_far += 1
            cumw += w
        col_type[idx_so_far:] = len(weights) - 1

        h_cells, w_cells = _grid_dims(cfg)
        # Whole-grid heightfield is rows*h_cells x cols*w_cells.
        big_h = cfg.num_rows * h_cells
        big_w = cfg.num_cols * w_cells
        big_field = np.zeros((big_h, big_w), dtype=np.float32)
        sub_origins = np.zeros((cfg.num_rows, cfg.num_cols, 3), dtype=np.float32)

        for r in range(cfg.num_rows):
            difficulty = r / max(1, cfg.num_rows - 1)
            for c in range(cfg.num_cols):
                ti = col_type[c]
                sub_cfg = cfg.sub_terrains[ti]
                field = sub_cfg.fn(rng, difficulty, cfg, r)
                big_field[
                    r * h_cells : (r + 1) * h_cells,
                    c * w_cells : (c + 1) * w_cells,
                ] = field
                # Spawn z = max height in a robot-footprint window around center,
                # not just the single center cell. Otherwise stairs/boxes that are
                # taller than the center cell intersect the feet at reset.
                foot_radius_m = 0.4
                rad_cells = max(1, int(foot_radius_m / cfg.horizontal_scale))
                ci, cj = h_cells // 2, w_cells // 2
                window = field[
                    max(0, ci - rad_cells): ci + rad_cells + 1,
                    max(0, cj - rad_cells): cj + rad_cells + 1,
                ]
                sub_origins[r, c] = (
                    c * cfg.size[0] + cfg.size[0] / 2.0,
                    r * cfg.size[1] + cfg.size[1] / 2.0,
                    float(window.max()),
                )

        self.size = cfg.size
        self.h_cells = h_cells
        self.w_cells = w_cells
        self.big_field_np = big_field
        self.big_origin_xy = (0.0, 0.0)

        # 2) Hand the heightfield to Genesis's Terrain morph, which uses
        # SDF collision (preserves non-convex geometry — required for
        # stairs/hills/slopes). Heights are in metres so vertical_scale=1.
        scene.add_entity(
            gs.morphs.Terrain(
                height_field=big_field,
                horizontal_scale=cfg.horizontal_scale,
                vertical_scale=1.0,
                pos=(self.big_origin_xy[0], self.big_origin_xy[1], 0.0),
                collision=True,
                visualization=True,
            )
        )

        # 3) Rasterized heightfield for height-scan lookup. Stored on device.
        self.height_field = torch.from_numpy(big_field).to(self.device)
        self.hf_origin = torch.tensor(self.big_origin_xy, device=self.device, dtype=torch.float32)
        self.hf_scale = float(cfg.horizontal_scale)

        # 4) Per-env curriculum state.
        self.sub_origins = torch.from_numpy(sub_origins).to(self.device)  # [R, C, 3]
        self.col_type = torch.from_numpy(col_type).to(self.device)
        # Initial level uniform in [0, max_init_level]; default mid-range.
        max_init = max(0, cfg.num_rows // 2)
        self.terrain_levels = torch.randint(
            0, max_init + 1, (num_envs,), device=self.device, dtype=torch.long
        )
        self.terrain_types = torch.randint(0, cfg.num_cols, (num_envs,), device=self.device, dtype=torch.long)
        self.env_origins = self._compute_origins()

    def _compute_origins(self) -> torch.Tensor:
        return self.sub_origins[self.terrain_levels, self.terrain_types]

    def update_env_origins(self, env_ids: torch.Tensor, move_up: torch.Tensor, move_down: torch.Tensor) -> None:
        """Curriculum step. `env_ids` is a LongTensor of indices being reset."""
        self.terrain_levels[env_ids] += move_up.long() - move_down.long()
        # Wrap robots that mastered the hardest row to a random row.
        max_lvl = self.cfg.num_rows - 1
        wrap = self.terrain_levels[env_ids] >= self.cfg.num_rows
        if wrap.any():
            new_lvl = torch.randint(0, max_lvl + 1, (int(wrap.sum().item()),), device=self.device, dtype=torch.long)
            local = self.terrain_levels[env_ids]
            local[wrap] = new_lvl
            self.terrain_levels[env_ids] = local
        self.terrain_levels[env_ids] = torch.clamp(self.terrain_levels[env_ids], 0, max_lvl)
        self.env_origins = self._compute_origins()

    def sample_height(self, xy: torch.Tensor) -> torch.Tensor:
        """Bilinear lookup into the rasterized heightfield. xy: (..., 2). Out: (...,)."""
        u = (xy[..., 0] - self.hf_origin[0]) / self.hf_scale
        v = (xy[..., 1] - self.hf_origin[1]) / self.hf_scale
        H, W = self.height_field.shape
        u0 = torch.clamp(u.floor().long(), 0, W - 1)
        v0 = torch.clamp(v.floor().long(), 0, H - 1)
        u1 = torch.clamp(u0 + 1, 0, W - 1)
        v1 = torch.clamp(v0 + 1, 0, H - 1)
        fu = (u - u0.float()).clamp(0.0, 1.0)
        fv = (v - v0.float()).clamp(0.0, 1.0)
        h00 = self.height_field[v0, u0]
        h10 = self.height_field[v0, u1]
        h01 = self.height_field[v1, u0]
        h11 = self.height_field[v1, u1]
        h0 = h00 * (1 - fu) + h10 * fu
        h1 = h01 * (1 - fu) + h11 * fu
        return h0 * (1 - fv) + h1 * fv
