"""G1 PPO training entry point.

Usage:
    g1_train -e flat
    g1_train -e rough --max_iterations 3000
    g1_train --task Isaac-Velocity-Rough-G1-v0
    g1_train -e rough --resume                  # continue from latest ckpt
    g1_train -e rough --resume --checkpoint 500 # continue from model_500.pt
    g1_train -e rough --load_run g1-rough-v2 --resume
"""

from __future__ import annotations

import argparse
import os
import pickle
import shutil

from rsl_rl.runners import OnPolicyRunner

import genesis as gs

from ..humanoid.cli import add_train_args, find_checkpoint, resolve_env
from ..humanoid.env import HumanoidLocomotionEnv
from ..humanoid.runner import get_train_cfg
from . import env_cfg as _g1_cfg


def main() -> None:
    parser = argparse.ArgumentParser()
    add_train_args(parser)
    args = parser.parse_args()

    variant = resolve_env(args.env)
    cfg_fn = _g1_cfg.g1_flat_cfg if variant == "flat" else _g1_cfg.g1_rough_cfg
    env_cfg = cfg_fn(num_envs=args.num_envs)

    exp_name = args.exp_name or f"g1-{variant}"
    max_iter = args.max_iterations or (1500 if variant == "flat" else 3000)
    hidden = [256, 128, 128] if variant == "flat" else [512, 256, 128]
    train_cfg = get_train_cfg(exp_name, hidden_dims=hidden, max_iterations=max_iter)
    train_cfg["logger"] = args.logger

    # --checkpoint implies --resume.
    resume = args.resume or (args.checkpoint is not None)
    log_dir = f"logs/{args.load_run or exp_name}"

    if resume:
        ckpt_path = find_checkpoint(log_dir, args.checkpoint)
    else:
        if os.path.exists(log_dir):
            shutil.rmtree(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        ckpt_path = None
        with open(f"{log_dir}/cfg.pkl", "wb") as f:
            pickle.dump({"env_cfg": env_cfg, "train_cfg": train_cfg, "variant": variant}, f)

    gs.init(backend=gs.gpu, precision="32", logging_level="warning", seed=args.seed, performance_mode=True)

    show_viewer = args.show_viewer and not args.headless
    env = HumanoidLocomotionEnv(env_cfg, show_viewer=show_viewer)
    runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
    if ckpt_path:
        print(f"[g1_train] Resuming from {ckpt_path}")
        runner.load(ckpt_path)
    runner.learn(num_learning_iterations=max_iter, init_at_random_ep_len=not resume)


if __name__ == "__main__":
    main()
