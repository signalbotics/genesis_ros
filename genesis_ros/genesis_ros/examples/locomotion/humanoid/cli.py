"""Shared CLI helpers for `<robot>_{train,eval}` entry points.

Mirrors IsaacLab/rsl_rl flag conventions (`--resume`, `--load_run`,
`--checkpoint`, `--headless`, `--logger`, `--seed`, `-B`/`--num_envs`,
`-n`/`--exp_name`, and `-e`/`--env`/`--task`) so users with IsaacLab muscle
memory can swap binaries without re-learning the CLI.
"""

from __future__ import annotations

import argparse
import os
import re


def resolve_env(value: str) -> str:
    """Accept 'flat'/'rough' or IsaacLab-style 'Isaac-Velocity-Flat-G1-v0'."""
    v = value.lower()
    if "flat" in v:
        return "flat"
    if "rough" in v:
        return "rough"
    raise SystemExit(
        f"Unrecognized env/task '{value}'. "
        f"Use 'flat', 'rough', or an IsaacLab-style 'Isaac-Velocity-{{Flat,Rough}}-<robot>-v0'."
    )


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-e", "--env", "--task", dest="env", type=str, default="flat",
                        help="Env to use: flat | rough (alias: --task).")
    parser.add_argument("-n", "--exp_name", type=str, default=None,
                        help="Run name. Default: '<robot>-<env>'.")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--headless", action="store_true",
                        help="No viewer (default).")
    parser.add_argument("--show_viewer", action="store_true",
                        help="Open the GUI viewer (overrides --headless).")


def add_train_args(parser: argparse.ArgumentParser) -> None:
    add_common_args(parser)
    parser.add_argument("--max_iterations", type=int, default=None)
    parser.add_argument("--logger", choices=["tensorboard", "wandb", "neptune"],
                        default="tensorboard")
    # Resume / checkpoint loading.
    parser.add_argument("--resume", action="store_true",
                        help="Continue training from a checkpoint instead of clearing the log dir.")
    parser.add_argument("--load_run", type=str, default=None,
                        help="Run dir to resume from (default: same as --exp_name).")
    parser.add_argument("--checkpoint", "--ckpt", dest="checkpoint", type=str, default=None,
                        help="Checkpoint to load: path, integer iter (e.g. 500), or 'latest'. "
                             "Implies --resume.")


def add_eval_args(parser: argparse.ArgumentParser) -> None:
    add_common_args(parser)
    parser.set_defaults(num_envs=1)
    parser.add_argument("--load_run", type=str, default=None)
    parser.add_argument("--checkpoint", "--ckpt", dest="checkpoint", type=str, default="latest",
                        help="Checkpoint to load: path, integer iter, or 'latest' (default).")


def find_checkpoint(log_dir: str, spec: str | None) -> str:
    """Resolve `spec` to a checkpoint path inside `log_dir`.

    Accepts:
      - None or "latest" -> latest model_*.pt
      - integer string ("500") -> model_500.pt
      - absolute/relative path to a .pt file
    """
    if spec and os.path.isfile(spec):
        return spec
    if not os.path.isdir(log_dir):
        raise FileNotFoundError(f"Log dir does not exist: {log_dir}")
    if spec and re.fullmatch(r"\d+", spec):
        path = os.path.join(log_dir, f"model_{spec}.pt")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Checkpoint not found: {path}")
        return path
    # latest
    ckpts = sorted(
        (f for f in os.listdir(log_dir) if f.startswith("model_") and f.endswith(".pt")),
        key=lambda f: int(re.search(r"model_(\d+)\.pt", f).group(1)),  # type: ignore[union-attr]
    )
    if not ckpts:
        raise FileNotFoundError(f"No model_*.pt checkpoints in {log_dir}")
    return os.path.join(log_dir, ckpts[-1])
