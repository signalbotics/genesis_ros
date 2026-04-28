"""G1 policy eval — load a checkpoint and run with the viewer.

Usage:
    g1_eval -e flat
    g1_eval -e rough --checkpoint 500
    g1_eval --task Isaac-Velocity-Rough-G1-v0 --load_run g1-rough-v2
"""

from __future__ import annotations

import argparse
import os
import pickle

import torch
from rsl_rl.runners import OnPolicyRunner

import genesis as gs

from ..humanoid.cli import add_eval_args, find_checkpoint, resolve_env
from ..humanoid.env import HumanoidLocomotionEnv
from ..humanoid.runner import get_train_cfg
from . import env_cfg as _g1_cfg


def main() -> None:
    parser = argparse.ArgumentParser()
    add_eval_args(parser)
    args = parser.parse_args()

    variant = resolve_env(args.env)
    cfg_fn = _g1_cfg.g1_flat_cfg if variant == "flat" else _g1_cfg.g1_rough_cfg
    env_cfg = cfg_fn(num_envs=args.num_envs)
    exp_name = args.exp_name or f"g1-{variant}"
    log_dir = f"logs/{args.load_run or exp_name}"

    # Reuse the exact train_cfg the checkpoint was trained with, so MLP shapes
    # match. Fall back to per-variant defaults only if cfg.pkl is missing.
    cfg_pkl = os.path.join(log_dir, "cfg.pkl")
    if os.path.isfile(cfg_pkl):
        with open(cfg_pkl, "rb") as f:
            saved = pickle.load(f)
        train_cfg = saved["train_cfg"]
    else:
        hidden = [256, 128, 128] if variant == "flat" else [512, 256, 128]
        train_cfg = get_train_cfg(exp_name, hidden_dims=hidden)

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=args.seed, performance_mode=True)
    show_viewer = not args.headless
    env = HumanoidLocomotionEnv(env_cfg, show_viewer=show_viewer)
    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)

    ckpt_path = find_checkpoint(log_dir, args.checkpoint)
    print(f"[g1_eval] Loading {ckpt_path}")
    runner.load(ckpt_path)
    policy = runner.get_inference_policy(device=gs.device)

    obs = env.get_observations()
    with torch.no_grad():
        while True:
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)


if __name__ == "__main__":
    main()
