"""rsl_rl OnPolicyRunner glue — same defaults as the existing go2_train.py."""

from __future__ import annotations

from importlib import metadata


def get_train_cfg(exp_name: str, *, hidden_dims: list[int] | None = None, max_iterations: int = 3000) -> dict:
    try:
        if int(metadata.version("rsl-rl-lib").split(".")[0]) < 5:
            raise ImportError
    except (metadata.PackageNotFoundError, ImportError) as e:
        raise ImportError("Please install 'rsl-rl-lib>=5.0.0'.") from e

    hidden = hidden_dims or [512, 256, 128]
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 1.0e-3,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": hidden,
            "activation": "elu",
            "distribution_cfg": {
                "class_name": "GaussianDistribution",
                "init_std": 1.0,
                "std_type": "scalar",
            },
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": hidden,
            "activation": "elu",
        },
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "num_steps_per_env": 24,
        "save_interval": 100,
        "run_name": exp_name,
        "logger": "tensorboard",
        "max_iterations": max_iterations,
    }
