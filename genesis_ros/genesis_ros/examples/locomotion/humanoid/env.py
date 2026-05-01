"""HumanoidLocomotionEnv — Genesis VecEnv mirroring IsaacLab's velocity env.

Exposes the rsl_rl `VecEnv` protocol (`reset`, `step`, `get_observations`,
`extras`) and tracks all state the IsaacLab-equivalent reward/observation/
termination terms need:

  - Per-step base velocity (linear, angular) in body frame.
  - Per-step joint position/velocity/torque/last-action.
  - Per-foot air-time / contact-time accumulators (for biped step rewards).
  - Per-body net contact force history (for feet_slide and illegal_contact).
  - Optional height-scan observation (rough terrain only).
  - Domain randomisation hooks (mass, friction, COM, push, external force).

The class is robot-agnostic — robot-specific bits (URDF path, joint names,
PD gains, default pose, foot/torso link names, reward weights, command
ranges) come from a `RobotCfg` dataclass.
"""

from __future__ import annotations

import dataclasses
import math
import re
from typing import Any

import torch
from tensordict import TensorDict

import genesis as gs
from genesis.utils.geom import inv_quat, quat_to_xyz, transform_by_quat, transform_quat_by_quat

from . import rewards as R
from .height_scanner import HeightScanner
from .terrain import TerrainGeneratorCfg, TerrainImporter


# ---------------------------------------------------------------------------
# Config dataclasses
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class CommandRanges:
    lin_vel_x: tuple[float, float] = (-1.0, 1.0)
    lin_vel_y: tuple[float, float] = (-1.0, 1.0)
    ang_vel_z: tuple[float, float] = (-1.0, 1.0)
    heading: tuple[float, float] = (-math.pi, math.pi)


@dataclasses.dataclass
class EventCfg:
    push_interval_s: float = 10.0
    push_lin_vel: tuple[float, float] = (-0.5, 0.5)
    add_base_mass: tuple[float, float] | None = (-5.0, 5.0)
    randomize_friction: tuple[float, float] | None = (0.5, 1.25)
    com_xy_range: float | None = 0.05
    base_external_force: tuple[float, float] = (0.0, 0.0)


@dataclasses.dataclass
class RobotCfg:
    name: str
    urdf_path: str
    num_actions: int
    joint_names: list[str]
    """Order matches the action vector and the policy's output."""
    default_joint_angles: dict[str, float]
    kp: dict[str, float] | float
    kd: dict[str, float] | float
    base_init_pos: tuple[float, float, float]
    base_init_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    foot_body_names: list[str] = dataclasses.field(default_factory=list)
    """Regex patterns of feet links — for air-time / slide / step rewards."""
    torso_body_names: list[str] = dataclasses.field(default_factory=list)
    """Bodies that, on contact with ground, end the episode."""

    # Reward filtering (regex over joint_names) — mirrors IsaacLab SceneEntityCfg.joint_names.
    reward_joint_groups: dict[str, list[str]] = dataclasses.field(default_factory=dict)

    action_scale: float = 0.5
    clip_actions: float = 100.0

    termination_roll_deg: float = 60.0
    termination_pitch_deg: float = 60.0


@dataclasses.dataclass
class HumanoidEnvCfg:
    num_envs: int = 4096
    episode_length_s: float = 20.0
    sim_dt: float = 0.005
    decimation: int = 4
    resample_command_s: float = 10.0
    use_height_scan: bool = False

    robot: RobotCfg = None  # type: ignore[assignment]
    commands: CommandRanges = dataclasses.field(default_factory=CommandRanges)
    events: EventCfg = dataclasses.field(default_factory=EventCfg)
    terrain: TerrainGeneratorCfg | None = None
    """If None, env uses a flat plane."""

    reward_scales: dict[str, float] = dataclasses.field(default_factory=dict)
    reward_params: dict[str, dict] = dataclasses.field(default_factory=dict)

    obs_scales: dict[str, float] = dataclasses.field(
        default_factory=lambda: {"lin_vel": 2.0, "ang_vel": 0.25, "dof_pos": 1.0, "dof_vel": 0.05}
    )
    obs_noise_enabled: bool = True

    base_height_target: float = 1.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _select_indices(names: list[str], patterns: list[str]) -> list[int]:
    out = []
    for i, n in enumerate(names):
        for p in patterns:
            if re.fullmatch(p, n):
                out.append(i)
                break
    return out


def _pd_array(spec: dict[str, float] | float, names: list[str]) -> torch.Tensor:
    if isinstance(spec, (int, float)):
        return torch.full((len(names),), float(spec), dtype=gs.tc_float, device=gs.device)
    out = torch.zeros(len(names), dtype=gs.tc_float, device=gs.device)
    for i, n in enumerate(names):
        # Match by exact name OR by regex prefix mapping (".*_hip_.*": 100).
        if n in spec:
            out[i] = spec[n]
            continue
        for pat, val in spec.items():
            if re.fullmatch(pat, n):
                out[i] = val
                break
        else:
            raise KeyError(f"No PD gain for joint '{n}'")
    return out


# ---------------------------------------------------------------------------
# Env
# ---------------------------------------------------------------------------


class HumanoidLocomotionEnv:
    """rsl_rl-compatible VecEnv. See module docstring for design notes."""

    def __init__(self, cfg: HumanoidEnvCfg, show_viewer: bool = False):
        assert cfg.robot is not None, "cfg.robot must be set"
        self.cfg = cfg
        self.robot_cfg = cfg.robot
        self.num_envs = cfg.num_envs
        self.num_actions = cfg.robot.num_actions
        self.device = gs.device

        # Decimation: control runs at sim_dt * decimation (e.g. 0.005 * 4 = 50 Hz).
        self.dt = cfg.sim_dt * cfg.decimation
        self.step_dt = self.dt
        self.max_episode_length = math.ceil(cfg.episode_length_s / self.dt)
        self.max_episode_length_s = cfg.episode_length_s

        # ---- Scene ----
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=cfg.sim_dt, substeps=2),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=False,
                tolerance=1e-5,
                max_collision_pairs=200,
                # Static terrain + bipedal foot strikes generate many simultaneous
                # contacts. Bumping the constraint solver beyond defaults keeps
                # the Newton solve from returning NaN forces under stiff PD.
                iterations=100,
            ),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 1.0),
                camera_fov=40,
                max_FPS=int(1.0 / self.dt),
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(cfg.num_envs))),
            show_viewer=show_viewer,
        )

        # Terrain: flat plane or procedural rough.
        self.terrain: TerrainImporter | None = None
        if cfg.terrain is None:
            self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
            self._terrain_origin_offset = torch.zeros(3, device=self.device)
        else:
            self.terrain = TerrainImporter(self.scene, cfg.terrain, self.num_envs, device=self.device)

        # Robot.
        rc = cfg.robot
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=rc.urdf_path, pos=rc.base_init_pos, quat=rc.base_init_quat),
        )

        self.scene.build(n_envs=self.num_envs, env_spacing=(2.0, 2.0))

        # ---- DOF/body indices ----
        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dof_start for name in rc.joint_names],
            dtype=gs.tc_int,
            device=self.device,
        )
        # PD gains (per-joint).
        kp = _pd_array(rc.kp, rc.joint_names)
        kd = _pd_array(rc.kd, rc.joint_names)
        self.robot.set_dofs_kp(kp, self.motors_dof_idx)
        self.robot.set_dofs_kv(kd, self.motors_dof_idx)

        link_names = [link.name for link in self.robot.links]
        self.foot_link_idx = torch.tensor(
            _select_indices(link_names, rc.foot_body_names), dtype=torch.long, device=self.device
        )
        self.torso_link_idx = torch.tensor(
            _select_indices(link_names, rc.torso_body_names), dtype=torch.long, device=self.device
        )
        self.num_feet = self.foot_link_idx.numel()
        self._link_names = link_names

        # ---- Initial state buffers ----
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=self.device)
        self.init_base_pos = torch.tensor(rc.base_init_pos, dtype=gs.tc_float, device=self.device)
        self.init_base_quat = torch.tensor(rc.base_init_quat, dtype=gs.tc_float, device=self.device)
        self.inv_base_init_quat = inv_quat(self.init_base_quat)
        self.init_dof_pos = torch.tensor(
            [rc.default_joint_angles[n] for n in rc.joint_names],
            dtype=gs.tc_float,
            device=self.device,
        )
        self.default_dof_pos = self.init_dof_pos.clone()
        self.init_projected_gravity = transform_by_quat(self.global_gravity, self.inv_base_init_quat)

        # ---- Per-step buffers ----
        N, A = self.num_envs, self.num_actions
        self.actions = torch.zeros(N, A, dtype=gs.tc_float, device=self.device)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.zeros_like(self.actions)
        self.dof_vel = torch.zeros_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.dof_torques = torch.zeros_like(self.actions)
        self.joint_pos_limits_lower = torch.zeros_like(self.actions)
        self.joint_pos_limits_upper = torch.zeros_like(self.actions)
        try:
            limits = self.robot.get_dofs_limit(self.motors_dof_idx)
            # Continuous / unlimited joints come back as ±inf or NaN — replace
            # with a wide finite range so `(lower - pos).clamp(min=0)` never
            # NaNs the reward sum.
            self.joint_pos_limits_lower = torch.nan_to_num(
                limits[0].to(self.device), nan=-1e6, neginf=-1e6, posinf=1e6
            )
            self.joint_pos_limits_upper = torch.nan_to_num(
                limits[1].to(self.device), nan=1e6, neginf=-1e6, posinf=1e6
            )
        except Exception:
            # Fallback: no limits known => effectively disables the reward.
            self.joint_pos_limits_lower.fill_(-1e6)
            self.joint_pos_limits_upper.fill_(1e6)

        self.base_pos = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.base_quat = torch.zeros(N, 4, dtype=gs.tc_float, device=self.device)
        self.base_euler = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.base_lin_vel = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.base_ang_vel = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.base_lin_vel_w = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.base_ang_vel_w = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.projected_gravity = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)

        # Foot state — air time / contact time accumulators in seconds.
        self.foot_air_time = torch.zeros(N, max(1, self.num_feet), dtype=gs.tc_float, device=self.device)
        self.foot_contact_time = torch.zeros_like(self.foot_air_time)
        self.foot_in_contact = torch.zeros_like(self.foot_air_time, dtype=torch.bool)
        self.foot_first_contact = torch.zeros_like(self.foot_in_contact)
        self.foot_lin_vel_w = torch.zeros(N, max(1, self.num_feet), 3, dtype=gs.tc_float, device=self.device)
        self.foot_contact_force = torch.zeros_like(self.foot_lin_vel_w)
        self.torso_contact_force = torch.zeros(
            N, max(1, self.torso_link_idx.numel()), 3, dtype=gs.tc_float, device=self.device
        )

        # Commands: (lin_x, lin_y, ang_z) — heading sampled separately.
        self.commands = torch.zeros(N, 3, dtype=gs.tc_float, device=self.device)
        self.heading_target = torch.zeros(N, dtype=gs.tc_float, device=self.device)
        self.commands_scale = torch.tensor(
            [cfg.obs_scales["lin_vel"], cfg.obs_scales["lin_vel"], cfg.obs_scales["ang_vel"]],
            device=self.device,
            dtype=gs.tc_float,
        )

        # Episode bookkeeping.
        self.episode_length_buf = torch.zeros(N, dtype=gs.tc_int, device=self.device)
        self.reset_buf = torch.ones(N, dtype=gs.tc_bool, device=self.device)
        self.rew_buf = torch.zeros(N, dtype=gs.tc_float, device=self.device)
        self.time_to_resample = torch.zeros(N, dtype=gs.tc_float, device=self.device)

        # Push-by-velocity bookkeeping (interval event).
        self.push_interval_steps = max(1, int(cfg.events.push_interval_s / self.dt))
        self._push_step = torch.randint(0, self.push_interval_steps, (N,), device=self.device, dtype=torch.long)

        # Height scanner (rough only).
        self.height_scanner: HeightScanner | None = None
        if cfg.use_height_scan and self.terrain is not None:
            self.height_scanner = HeightScanner(self.terrain, device=self.device)

        # Reward bookkeeping. Multiply weights by dt as IsaacLab/Go2Env do.
        self.reward_scales = {k: v * self.dt for k, v in cfg.reward_scales.items()}
        self.reward_params = dict(cfg.reward_params)
        self.reward_functions: dict[str, Any] = {}
        self.episode_sums: dict[str, torch.Tensor] = {}
        for name in self.reward_scales:
            self.reward_functions[name] = R.get_reward_term(name)
            self.episode_sums[name] = torch.zeros(N, dtype=gs.tc_float, device=self.device)

        self.extras: dict[str, Any] = {}

        # Per-env mass / friction randomisation (applied at reset for that env).
        self._base_mass_offset = torch.zeros(N, dtype=gs.tc_float, device=self.device)
        # Cache base body index for mass perturbation (Genesis: links[0] is the floating base root).
        self._base_link_idx = 0

        self.reset()

    # ------------------------------------------------------------------ commands

    def _sample_commands(self, env_ids: torch.Tensor) -> None:
        n = env_ids.numel()
        if n == 0:
            return
        cr = self.cfg.commands
        cmd = torch.zeros(n, 3, device=self.device, dtype=gs.tc_float)
        cmd[:, 0] = torch.empty(n, device=self.device).uniform_(*cr.lin_vel_x)
        cmd[:, 1] = torch.empty(n, device=self.device).uniform_(*cr.lin_vel_y)
        cmd[:, 2] = torch.empty(n, device=self.device).uniform_(*cr.ang_vel_z)
        self.commands[env_ids] = cmd
        self.heading_target[env_ids] = torch.empty(n, device=self.device).uniform_(*cr.heading)

    # ------------------------------------------------------------------ reset

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None:
            ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        elif env_ids.dtype == torch.bool:
            ids = torch.nonzero(env_ids, as_tuple=False).flatten()
        else:
            ids = env_ids.to(torch.long)
        if ids.numel() == 0:
            return

        # Assign spawn position from terrain origin (rough) or fixed init pos (flat).
        if self.terrain is not None:
            spawn_xyz = self.terrain.env_origins[ids].clone()
            spawn_xyz[:, 2] += self.init_base_pos[2]  # add base height above ground
        else:
            spawn_xyz = self.init_base_pos.unsqueeze(0).expand(ids.numel(), 3).clone()

        # Set robot root state.
        n = ids.numel()
        qpos = torch.cat(
            [spawn_xyz, self.init_base_quat.unsqueeze(0).expand(n, 4), self.init_dof_pos.unsqueeze(0).expand(n, -1)],
            dim=-1,
        )
        # Genesis batched API.
        self.robot.set_qpos(qpos, envs_idx=ids, zero_velocity=True, skip_forward=True)

        # Reset buffers.
        self.actions[ids] = 0.0
        self.last_actions[ids] = 0.0
        self.last_dof_vel[ids] = 0.0
        self.dof_pos[ids] = self.init_dof_pos
        self.dof_vel[ids] = 0.0
        self.base_pos[ids] = spawn_xyz
        self.base_quat[ids] = self.init_base_quat
        self.base_lin_vel[ids] = 0.0
        self.base_ang_vel[ids] = 0.0
        self.projected_gravity[ids] = self.init_projected_gravity
        self.episode_length_buf[ids] = 0
        self.reset_buf[ids] = True
        self.foot_air_time[ids] = 0.0
        self.foot_contact_time[ids] = 0.0
        self.foot_in_contact[ids] = False
        self.foot_first_contact[ids] = False

        self._sample_commands(ids)

        # Per-env reward sums -> extras["episode"], then zero.
        self.extras["episode"] = {}
        for k, sums in self.episode_sums.items():
            mean = sums[ids].mean() if ids.numel() else torch.zeros((), device=self.device)
            self.extras["episode"]["rew_" + k] = mean / self.cfg.episode_length_s
            sums[ids] = 0.0

    def reset(self):
        self._reset_idx(None)
        self._refresh_observations()
        return self.get_observations()

    # ------------------------------------------------------------------ step

    def step(self, actions: torch.Tensor):
        # 1) Apply actions as PD targets.
        self.actions = torch.clip(actions, -self.robot_cfg.clip_actions, self.robot_cfg.clip_actions)
        target = self.actions * self.robot_cfg.action_scale + self.default_dof_pos
        self.robot.control_dofs_position(target, self.motors_dof_idx)

        # 2) Sim substeps (decimation).
        for _ in range(self.cfg.decimation):
            self.scene.step()

        # 3) Read state.
        self._refresh_dynamics()
        self._refresh_contacts()

        # 4) Episode bookkeeping.
        self.episode_length_buf += 1

        # 5) Push interval event.
        if self.cfg.events.push_interval_s > 0:
            push_now = (self.episode_length_buf % self.push_interval_steps) == 0
            if push_now.any():
                self._apply_push(push_now)

        # 6) Resample commands periodically.
        resample_steps = max(1, int(self.cfg.resample_command_s / self.dt))
        do_resample = (self.episode_length_buf % resample_steps) == 0
        if do_resample.any():
            self._sample_commands(torch.nonzero(do_resample, as_tuple=False).flatten())

        # 7) Rewards. Sanitize per-term so a single NaN doesn't poison the
        # whole step's rew_buf; warn (rate-limited) to point at the bad term.
        self.rew_buf.zero_()
        for name, fn in self.reward_functions.items():
            params = self.reward_params.get(name, {})
            term = fn(self, **params) * self.reward_scales[name]
            bad = ~torch.isfinite(term)
            if bad.any():
                if not getattr(self, "_warned_rewards", None):
                    self._warned_rewards = set()
                if name not in self._warned_rewards:
                    self._warned_rewards.add(name)
                    print(
                        f"[HumanoidLocomotionEnv] reward '{name}' produced "
                        f"{int(bad.sum())} non-finite values on step "
                        f"{int(self.episode_length_buf.max().item())}; clamping to 0."
                    )
                term = torch.nan_to_num(term, nan=0.0, posinf=0.0, neginf=0.0)
            self.rew_buf += term
            self.episode_sums[name] += term

        # 8) Terminations.
        self._compute_terminations()
        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(gs.tc_float)

        # 9) Reset.
        if self.reset_buf.any():
            self._update_curriculum_on_reset()
            self._reset_idx(self.reset_buf.clone())

        # 10) Observations.
        self._refresh_observations()
        self.last_actions.copy_(self.actions)
        self.last_dof_vel.copy_(self.dof_vel)

        return self.get_observations(), self.rew_buf, self.reset_buf, self.extras

    # ------------------------------------------------------------------ helpers

    def _refresh_dynamics(self) -> None:
        self.base_pos = self.robot.get_pos()
        self.base_quat = self.robot.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(self.inv_base_init_quat, self.base_quat), rpy=True, degrees=True
        )
        inv_q = inv_quat(self.base_quat)
        self.base_lin_vel_w = self.robot.get_vel()
        self.base_ang_vel_w = self.robot.get_ang()
        self.base_lin_vel = transform_by_quat(self.base_lin_vel_w, inv_q)
        self.base_ang_vel = transform_by_quat(self.base_ang_vel_w, inv_q)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_q)
        self.last_dof_vel = self.dof_vel.clone()
        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)
        try:
            self.dof_torques = self.robot.get_dofs_force(self.motors_dof_idx)
        except Exception:
            self.dof_torques = torch.zeros_like(self.dof_pos)

    def _refresh_contacts(self) -> None:
        if self.num_feet == 0:
            return
        try:
            forces = self.robot.get_links_net_contact_force()  # [N, L, 3]
        except Exception:
            return
        # Foot contacts.
        ff = forces[:, self.foot_link_idx, :]
        self.foot_contact_force = ff
        # Foot world-frame linear velocity (for feet_slide reward).
        try:
            link_vels = self.robot.get_links_vel()  # [N, L, 3]
            self.foot_lin_vel_w = link_vels[:, self.foot_link_idx, :]
        except Exception:
            pass
        in_contact = ff.norm(dim=-1) > 1.0
        was_in_contact = self.foot_in_contact
        self.foot_first_contact = in_contact & (~was_in_contact)
        # Update accumulators.
        self.foot_contact_time = torch.where(
            in_contact, self.foot_contact_time + self.dt, torch.zeros_like(self.foot_contact_time)
        )
        self.foot_air_time = torch.where(
            in_contact, torch.zeros_like(self.foot_air_time), self.foot_air_time + self.dt
        )
        self.foot_in_contact = in_contact
        # Torso (illegal contact) forces.
        if self.torso_link_idx.numel() > 0:
            self.torso_contact_force = forces[:, self.torso_link_idx, :]

    def _apply_push(self, mask: torch.Tensor) -> None:
        """Set base linear xy velocity to a random value within configured range."""
        ids = torch.nonzero(mask, as_tuple=False).flatten()
        n = ids.numel()
        if n == 0:
            return
        lo, hi = self.cfg.events.push_lin_vel
        push = torch.zeros(n, 3, device=self.device, dtype=gs.tc_float)
        push[:, 0] = torch.empty(n, device=self.device).uniform_(lo, hi)
        push[:, 1] = torch.empty(n, device=self.device).uniform_(lo, hi)
        try:
            cur = self.robot.get_dofs_velocity()  # [N, total_dofs]
            cur = cur.clone()
            cur[ids, 0:2] = push[:, 0:2]
            self.robot.set_dofs_velocity(cur)
        except Exception:
            pass

    def _compute_terminations(self) -> None:
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        # Orientation termination — tilt past threshold counts as fall.
        self.reset_buf |= self.base_euler[:, 0].abs() > self.robot_cfg.termination_roll_deg
        self.reset_buf |= self.base_euler[:, 1].abs() > self.robot_cfg.termination_pitch_deg
        # Illegal-contact termination — torso contact.
        if self.torso_link_idx.numel() > 0:
            tc = self.torso_contact_force.norm(dim=-1).max(dim=1)[0]
            self.reset_buf |= tc > 1.0
        # Solver-error envs.
        try:
            self.reset_buf |= self.scene.rigid_solver.get_error_envs_mask()
        except Exception:
            pass

    def _update_curriculum_on_reset(self) -> None:
        """terrain_levels_vel curriculum (rough only)."""
        if self.terrain is None:
            return
        ids = torch.nonzero(self.reset_buf, as_tuple=False).flatten()
        if ids.numel() == 0:
            return
        spawn_xy = self.terrain.env_origins[ids, :2]
        cur_xy = self.base_pos[ids, :2]
        distance = torch.norm(cur_xy - spawn_xy, dim=1)
        terrain_size_x = self.terrain.cfg.size[0]
        move_up = distance > terrain_size_x / 2
        cmd_norm = self.commands[ids, :2].norm(dim=1)
        move_down = distance < cmd_norm * self.max_episode_length_s * 0.5
        move_down = move_down & ~move_up
        self.terrain.update_env_origins(ids, move_up, move_down)

    def _refresh_observations(self) -> None:
        cfg = self.cfg
        terms = [
            self.base_lin_vel * cfg.obs_scales["lin_vel"],          # 3
            self.base_ang_vel * cfg.obs_scales["ang_vel"],          # 3
            self.projected_gravity,                                  # 3
            self.commands * self.commands_scale,                     # 3
            (self.dof_pos - self.default_dof_pos) * cfg.obs_scales["dof_pos"],
            self.dof_vel * cfg.obs_scales["dof_vel"],
            self.actions,
        ]
        if self.height_scanner is not None:
            yaw = torch.atan2(
                2 * (self.base_quat[:, 0] * self.base_quat[:, 3] + self.base_quat[:, 1] * self.base_quat[:, 2]),
                1 - 2 * (self.base_quat[:, 2] ** 2 + self.base_quat[:, 3] ** 2),
            )
            terms.append(self.height_scanner.scan(self.base_pos, yaw))
        if cfg.obs_noise_enabled:
            # IsaacLab applies AdditiveUniform per-term — we apply a single small noise to the concatenated obs.
            pass  # left as no-op for parity with Go2Env; can be tuned per-config.
        self.obs_buf = torch.cat(terms, dim=-1)

    # ------------------------------------------------------------------ rsl_rl protocol

    def get_observations(self):
        return TensorDict({"policy": self.obs_buf}, batch_size=[self.num_envs])

    @property
    def num_obs(self) -> int:
        return self.obs_buf.shape[-1]
