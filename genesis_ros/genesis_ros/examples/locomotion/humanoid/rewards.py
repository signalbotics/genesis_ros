"""Reward terms ported from `isaaclab_tasks.../velocity/mdp/rewards.py` and
`isaaclab.envs.mdp.rewards`.

Each function takes the env as the first arg, returns a [num_envs] tensor.
Joint-group filtering uses regex over `env.robot_cfg.joint_names` resolved at
call time so configs can read like:

    reward_params = {
        "joint_deviation_arms": {"joint_names": [".*_shoulder_.*", ".*_elbow.*"]},
        ...
    }

Sensor-backed terms (feet_air_time, feet_slide, undesired_contacts) read
foot/torso buffers populated each step in env._refresh_contacts.
"""

from __future__ import annotations

import re
from typing import Callable

import torch


def _select_dof_idx(env, joint_names: list[str]) -> torch.Tensor:
    sel = []
    for i, n in enumerate(env.robot_cfg.joint_names):
        for p in joint_names:
            if re.fullmatch(p, n):
                sel.append(i)
                break
    return torch.tensor(sel, dtype=torch.long, device=env.device)


# ---------------------------------------------------------------------------
# Tracking
# ---------------------------------------------------------------------------


def track_lin_vel_xy_yaw_frame_exp(env, std: float = 0.5) -> torch.Tensor:
    """yaw-aligned linear velocity tracking. base_lin_vel is already body-frame; yaw frame ≈ body for upright robots."""
    err = torch.sum(torch.square(env.commands[:, :2] - env.base_lin_vel[:, :2]), dim=1)
    return torch.exp(-err / (std ** 2))


def track_ang_vel_z_world_exp(env, std: float = 0.5) -> torch.Tensor:
    err = torch.square(env.commands[:, 2] - env.base_ang_vel_w[:, 2])
    return torch.exp(-err / (std ** 2))


def track_lin_vel_xy_exp(env, std: float = 0.5) -> torch.Tensor:
    return track_lin_vel_xy_yaw_frame_exp(env, std)


def track_ang_vel_z_exp(env, std: float = 0.5) -> torch.Tensor:
    return track_ang_vel_z_world_exp(env, std)


# ---------------------------------------------------------------------------
# Penalties on base motion
# ---------------------------------------------------------------------------


def lin_vel_z_l2(env) -> torch.Tensor:
    return torch.square(env.base_lin_vel[:, 2])


def ang_vel_xy_l2(env) -> torch.Tensor:
    return torch.sum(torch.square(env.base_ang_vel[:, :2]), dim=1)


def flat_orientation_l2(env) -> torch.Tensor:
    return torch.sum(torch.square(env.projected_gravity[:, :2]), dim=1)


def base_height_l2(env, target_height: float = 1.0) -> torch.Tensor:
    return torch.square(env.base_pos[:, 2] - target_height)


# ---------------------------------------------------------------------------
# Penalties on joints / actions
# ---------------------------------------------------------------------------


def action_rate_l2(env) -> torch.Tensor:
    return torch.sum(torch.square(env.last_actions - env.actions), dim=1)


def dof_torques_l2(env, joint_names: list[str] | None = None) -> torch.Tensor:
    if joint_names is None:
        return torch.sum(torch.square(env.dof_torques), dim=1)
    idx = _select_dof_idx(env, joint_names)
    return torch.sum(torch.square(env.dof_torques[:, idx]), dim=1)


def dof_acc_l2(env, joint_names: list[str] | None = None) -> torch.Tensor:
    acc = (env.dof_vel - env.last_dof_vel) / env.dt
    if joint_names is None:
        return torch.sum(torch.square(acc), dim=1)
    idx = _select_dof_idx(env, joint_names)
    return torch.sum(torch.square(acc[:, idx]), dim=1)


def joint_deviation_l1(env, joint_names: list[str] | None = None) -> torch.Tensor:
    diff = env.dof_pos - env.default_dof_pos
    if joint_names is None:
        return torch.sum(torch.abs(diff), dim=1)
    idx = _select_dof_idx(env, joint_names)
    return torch.sum(torch.abs(diff[:, idx]), dim=1)


def joint_pos_limits(env, joint_names: list[str] | None = None) -> torch.Tensor:
    """Penalise distance past joint position limits (matches IsaacLab)."""
    lower = env.joint_pos_limits_lower
    upper = env.joint_pos_limits_upper
    pos = env.dof_pos
    out_lower = (lower - pos).clamp(min=0.0)
    out_upper = (pos - upper).clamp(min=0.0)
    viol = out_lower + out_upper
    if joint_names is None:
        return torch.sum(viol, dim=1)
    idx = _select_dof_idx(env, joint_names)
    return torch.sum(viol[:, idx], dim=1)


# ---------------------------------------------------------------------------
# Biped foot rewards
# ---------------------------------------------------------------------------


def feet_air_time_positive_biped(env, threshold: float = 0.4, command_threshold: float = 0.1) -> torch.Tensor:
    if env.num_feet == 0:
        return torch.zeros(env.num_envs, device=env.device)
    air_time = env.foot_air_time
    contact_time = env.foot_contact_time
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = in_contact.int().sum(dim=1) == 1
    masked = torch.where(single_stance.unsqueeze(-1), in_mode_time, torch.zeros_like(in_mode_time))
    reward = masked.min(dim=1)[0].clamp(max=threshold)
    cmd_norm = env.commands[:, :2].norm(dim=1)
    reward = reward * (cmd_norm > command_threshold).float()
    return reward


def feet_air_time(env, threshold: float = 0.5, command_threshold: float = 0.1) -> torch.Tensor:
    """Quadruped-style; kept for parity. Positive reward for steps longer than threshold."""
    if env.num_feet == 0:
        return torch.zeros(env.num_envs, device=env.device)
    last_air = env.foot_air_time
    first = env.foot_first_contact.float()
    reward = torch.sum((last_air - threshold) * first, dim=1)
    cmd_norm = env.commands[:, :2].norm(dim=1)
    reward = reward * (cmd_norm > command_threshold).float()
    return reward


def feet_slide(env) -> torch.Tensor:
    if env.num_feet == 0:
        return torch.zeros(env.num_envs, device=env.device)
    in_contact = env.foot_contact_force.norm(dim=-1) > 1.0
    foot_speed = env.foot_lin_vel_w[..., :2].norm(dim=-1)
    return torch.sum(foot_speed * in_contact.float(), dim=1)


def undesired_contacts(env, body_names: list[str], threshold: float = 1.0) -> torch.Tensor:
    """Net contact force magnitude > threshold on any body in body_names."""
    if not body_names:
        return torch.zeros(env.num_envs, device=env.device)
    sel = []
    for i, n in enumerate(env._link_names):
        for p in body_names:
            if re.fullmatch(p, n):
                sel.append(i)
                break
    if not sel:
        return torch.zeros(env.num_envs, device=env.device)
    try:
        forces = env.robot.get_links_net_contact_force()
    except Exception:
        return torch.zeros(env.num_envs, device=env.device)
    sel_t = torch.tensor(sel, dtype=torch.long, device=env.device)
    f = forces[:, sel_t, :].norm(dim=-1)
    return (f > threshold).float().sum(dim=1)


# ---------------------------------------------------------------------------
# Termination penalty
# ---------------------------------------------------------------------------


def is_terminated(env) -> torch.Tensor:
    """Penalty proportional to whether episode terminated this step (excluding time-out)."""
    timed_out = env.episode_length_buf > env.max_episode_length
    return (env.reset_buf & ~timed_out).float()


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------


_REWARDS: dict[str, Callable] = {
    "track_lin_vel_xy_exp": track_lin_vel_xy_exp,
    "track_ang_vel_z_exp": track_ang_vel_z_exp,
    "track_lin_vel_xy_yaw_frame_exp": track_lin_vel_xy_yaw_frame_exp,
    "track_ang_vel_z_world_exp": track_ang_vel_z_world_exp,
    "lin_vel_z_l2": lin_vel_z_l2,
    "ang_vel_xy_l2": ang_vel_xy_l2,
    "flat_orientation_l2": flat_orientation_l2,
    "base_height_l2": base_height_l2,
    "action_rate_l2": action_rate_l2,
    "dof_torques_l2": dof_torques_l2,
    "dof_acc_l2": dof_acc_l2,
    "joint_deviation_l1": joint_deviation_l1,
    "joint_deviation_hip": joint_deviation_l1,
    "joint_deviation_arms": joint_deviation_l1,
    "joint_deviation_torso": joint_deviation_l1,
    "joint_deviation_fingers": joint_deviation_l1,
    "dof_pos_limits": joint_pos_limits,
    "feet_air_time": feet_air_time,
    "feet_air_time_positive_biped": feet_air_time_positive_biped,
    "feet_slide": feet_slide,
    "undesired_contacts": undesired_contacts,
    "termination_penalty": is_terminated,
    "is_terminated": is_terminated,
}


def get_reward_term(name: str) -> Callable:
    if name not in _REWARDS:
        raise KeyError(f"Unknown reward term '{name}'. Known: {sorted(_REWARDS)}")
    return _REWARDS[name]
