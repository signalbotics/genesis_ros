"""Train a Go2 backflip policy.

Design: mirrors the canonical Genesis-backflip recipe
(github.com/ziyanx02/Genesis-backflip) -- a known-working set of
reward shapings + hyperparameters that converges in ~1000 PPO
iterations with 4-10k parallel envs.

Key design points:
  * Time-varying reference quaternion. The keystone reward
    `_reward_orientation_control` traces a desired body orientation
    that smoothly rotates 0 -> 2pi around the y-axis between t=0.5s
    and t=1.0s. Penalising deviation from this trajectory gives a
    dense, well-shaped gradient that points exactly at "rotate
    backward at the right time". This succeeds where pure
    phase-windowed task rewards stall: the policy can never
    accidentally discover the takeoff motion from random
    exploration, so it needs an explicit signal of "what direction
    to spin".
  * Signed pitch ang-vel reward, not |ang_vel|. Sign-agnostic lets
    the policy farm reward by rolling sideways; signed forces it
    to commit to backward pitch.
  * Liftoff (lin_vel_z) and rotation (ang_vel_y) rewards are tightly
    windowed in absolute time (0.5-0.75s and 0.5-1.0s respectively),
    not phase-fractional.
  * No pitch accumulator. Reference tracking happens at every
    instant; integrating body-frame ang-vel to a world rotation is
    fragile under large pitch.
  * Shorter rollouts (24 steps vs full episode). PPO benefits from
    more frequent updates; advantages still capture the backflip
    chain because gamma=0.99 and reward density is high.
  * init_at_random_ep_len=True. Phase is part of the observation
    (sin/cos of episode_length), so the policy can condition on it.
    Random init gives diverse value-function samples from the start.
"""
from __future__ import annotations

import argparse
import math
import os
import pickle
import shutil
from importlib import metadata

import torch
from tensordict import TensorDict

try:
    if int(metadata.version("rsl-rl-lib").split(".")[0]) < 5:
        raise ImportError
except (metadata.PackageNotFoundError, ImportError) as e:
    raise ImportError("Please install 'rsl-rl-lib>=5.0.0'.") from e
from rsl_rl.runners import OnPolicyRunner

import genesis as gs
from genesis.utils.geom import (
    inv_quat,
    transform_by_quat,
    transform_quat_by_quat,
)

from .go2_backflip import BackflipEnv, get_cfgs as _get_eval_cfgs
from .go2_env import Go2Env


_TWO_PI = 2.0 * math.pi


def _quat_pitch(angle: torch.Tensor) -> torch.Tensor:
    """Build (w, x, y, z) quaternion for a rotation `angle` around +y axis.

    angle: (N,) tensor of radians. returns (N, 4).
    """
    half = angle * 0.5
    w = torch.cos(half)
    y = torch.sin(half)
    zeros = torch.zeros_like(w)
    return torch.stack([w, zeros, y, zeros], dim=-1)


class BackflipTrainEnv(BackflipEnv):
    """Backflip env with reference-quaternion-tracking reward (Ziyan Zhou recipe)."""

    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg,
                 show_viewer=False, num_flips: int = 1):
        self._num_flips = int(num_flips)
        # Pre-build the desired-pitch axis on device (used in orientation_control).
        super().__init__(num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=show_viewer)
        self._global_gravity_b = self.global_gravity.unsqueeze(0).expand(self.num_envs, -1).contiguous()

    # --- rsl_rl 5.x compatibility shim --------------------------------------

    def get_observations(self):
        super().get_observations()  # populates self.obs_buf (60-dim phase obs)
        return TensorDict({"policy": self.obs_buf}, batch_size=[self.num_envs])

    def step(self, actions):
        return Go2Env.step(self, actions)

    def reset(self):
        return Go2Env.reset(self)

    # --- helpers ------------------------------------------------------------

    def _t(self) -> torch.Tensor:
        """Current absolute time in seconds (per env)."""
        return self.episode_length_buf.float() * self.dt

    # --- reward functions ---------------------------------------------------
    # All reward functions return per-env tensors; reward_scales (multiplied
    # by dt in Go2Env.__init__) determine sign/magnitude.

    def _reward_ang_vel_y(self):
        """Signed pitch angular velocity during takeoff window.

        Backflip = negative pitch in this coord (-y rotation), so we
        reward `-base_ang_vel[:, 1]`. Saturated at 7.2 rad/s so a
        single huge spike doesn't dominate.
        """
        t = self._t()
        in_window = ((t > 0.5) & (t < 1.0)).float()
        ang_vel = (-self.base_ang_vel[:, 1]).clamp(min=-7.2, max=7.2)
        return ang_vel * in_window

    def _reward_ang_vel_z(self):
        """Penalise yaw drift (no spinning in z)."""
        return torch.abs(self.base_ang_vel[:, 2])

    def _reward_lin_vel_z(self):
        """Reward upward velocity during liftoff window."""
        t = self._t()
        in_window = ((t > 0.5) & (t < 0.75)).float()
        # World-frame z velocity; clamped so it doesn't reward absurd numbers.
        lin_vel = self.robot.get_vel()[:, 2].clamp(max=3.0)
        return lin_vel * in_window

    def _reward_orientation_control(self):
        """KEYSTONE: penalise deviation from a time-varying reference orientation.

        The reference rotates 0 -> 2*pi*num_flips around the y-axis
        smoothly between t=0.5s and t=1.0s. Outside that window the
        reference is upright (matches base_init_quat), so the policy
        is penalised for tilting before takeoff or after landing.

        Implementation: compute the projected_gravity that *should*
        be observed if the body were following the reference, then
        L2-distance against the actual projected_gravity. This is
        smoother than a quaternion-difference loss.
        """
        t = self._t()
        phase = (t - 0.5).clamp(min=0.0, max=0.5)
        angle = 4.0 * phase * math.pi * self._num_flips  # 0 .. 2*pi*num_flips
        quat_pitch = _quat_pitch(angle)  # (N, 4)

        # desired_base_quat = quat_pitch * base_init_quat
        base_init = self.init_base_quat.unsqueeze(0).expand_as(quat_pitch).contiguous()
        desired_base_quat = transform_quat_by_quat(quat_pitch, base_init)

        inv_desired = inv_quat(desired_base_quat)
        desired_proj_grav = transform_by_quat(self._global_gravity_b, inv_desired)

        return torch.sum(torch.square(self.projected_gravity - desired_proj_grav), dim=1)

    def _reward_height_control(self):
        """Penalise base-height deviation from 0.3m, but only OUTSIDE the
        airborne window (t < 0.4s or t > 1.4s). During flight the body
        is in free-fall so penalising height would fight liftoff."""
        t = self._t()
        outside_air = ((t < 0.4) | (t > 1.4)).float()
        return torch.square(self.base_pos[:, 2] - 0.30) * outside_air

    def _reward_gravity_y(self):
        """Penalise lateral tilt (no roll). projected_gravity[:, 1] is
        zero when the body's y-axis is horizontal."""
        return torch.square(self.projected_gravity[:, 1])

    def _reward_actions_symmetry(self):
        """Force left/right leg actions to mirror.

        Joint order (from go2_backflip.py): FR_hip,thigh,calf, FL_hip,thigh,calf,
        RR_hip,thigh,calf, RL_hip,thigh,calf.
        - hips mirror with opposite sign (a[0]+a[3], a[6]+a[9])
        - thigh/calf mirror with same sign (a[1:3]==a[4:6], a[7:9]==a[10:12])
        """
        a = self.actions
        diff = torch.square(a[:, 0] + a[:, 3])
        diff = diff + torch.sum(torch.square(a[:, 1:3] - a[:, 4:6]), dim=1)
        diff = diff + torch.square(a[:, 6] + a[:, 9])
        diff = diff + torch.sum(torch.square(a[:, 7:9] - a[:, 10:12]), dim=1)
        return diff

    def _reward_action_rate(self):
        return torch.sum(torch.square(self.actions - self.last_actions), dim=1)


def get_cfgs(num_flips: int = 1):
    env_cfg, obs_cfg, _, command_cfg = _get_eval_cfgs()
    # Episode is 2s for a single flip; double has 0.5s extra airtime.
    env_cfg["episode_length_s"] = 2.0 if num_flips == 1 else 2.5
    # No early termination: the body must invert.
    env_cfg["termination_if_pitch_greater_than"] = 1e6
    env_cfg["termination_if_roll_greater_than"] = 1e6
    obs_cfg["num_obs"] = 60

    reward_cfg = {
        "tracking_sigma": 0.25,
        "base_height_target": 0.30,
        "reward_scales": {
            # All values per the canonical Genesis-backflip recipe.
            "ang_vel_y":           5.0,
            "ang_vel_z":          -1.0,
            "lin_vel_z":          20.0,
            "orientation_control": -1.0,
            "height_control":    -10.0,
            "gravity_y":         -10.0,
            "actions_symmetry":   -0.1,
            "action_rate":       -0.001,
        },
    }
    return env_cfg, obs_cfg, reward_cfg, command_cfg


def get_train_cfg(exp_name: str):
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 1.0e-3,  # was 5e-4; reference uses 1e-3
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": [512, 256, 128],
            "activation": "elu",
            "distribution_cfg": {
                "class_name": "GaussianDistribution",
                "init_std": 1.0,
                "std_type": "scalar",
            },
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": [512, 256, 128],
            "activation": "elu",
        },
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        # Short rollouts (~0.5s of sim time) -> more frequent PPO
        # updates. Reference uses 24; gamma=0.99 still captures the
        # full takeoff -> landing credit chain across rollout boundaries.
        "num_steps_per_env": 24,
        "save_interval": 100,
        "run_name": exp_name,
        "logger": "tensorboard",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="single",
                        choices=["single", "double"])
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    num_flips = 1 if args.exp_name == "single" else 2
    log_dir = f"logs/{args.exp_name}"

    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs(num_flips=num_flips)
    train_cfg = get_train_cfg(args.exp_name)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)
    with open(f"{log_dir}/cfgs.pkl", "wb") as f:
        pickle.dump([env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg], f)

    gs.init(backend=gs.gpu, precision="32", seed=args.seed, performance_mode=True)

    env = BackflipTrainEnv(
        num_envs=args.num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        num_flips=num_flips,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    # init_at_random_ep_len=True: phase is encoded in the observation
    # (sin/cos of episode_length_buf), so the policy conditions on it
    # explicitly. Random ep-len init gives the value function diverse
    # phase samples from iteration 0 instead of N envs all at t=0.
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
