import argparse

import torch
from .go2_env import Go2Env

import genesis as gs


def get_cfgs():
    env_cfg = {
        "num_actions": 12,
        # joint/link names
        "default_joint_angles": {  # [rad]
            "FL_hip_joint": 0.0,
            "FR_hip_joint": 0.0,
            "RL_hip_joint": 0.0,
            "RR_hip_joint": 0.0,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
        },
        "joint_names": [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ],
        # PD
        "kp": 70.0,
        "kd": 3.0,
        # termination
        "termination_if_roll_greater_than": 1000,  # degree
        "termination_if_pitch_greater_than": 1000,
        # base pose
        "base_init_pos": [0.0, 0.0, 0.35],
        "base_init_quat": [0.0, 0.0, 0.0, 1.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.5,
        "simulate_action_latency": True,
        "clip_actions": 100.0,
    }
    obs_cfg = {
        "num_obs": 60,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }
    reward_cfg = {
        "reward_scales": {},
    }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0, 0],
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [0, 0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


class BackflipEnv(Go2Env):
    def get_observations(self):
        phase = torch.pi * self.episode_length_buf[:, None] / self.max_episode_length
        self.obs_buf = torch.cat(
            [
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 12
                self.dof_vel * self.obs_scales["dof_vel"],  # 12
                self.actions,  # 12
                self.last_actions,  # 12
                torch.sin(phase),
                torch.cos(phase),
                torch.sin(phase / 2),
                torch.cos(phase / 2),
                torch.sin(phase / 4),
                torch.cos(phase / 4),
            ],
            axis=-1,
        )

        return self.obs_buf

    def step(self, actions):
        super().step(actions)
        self.get_observations()
        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="single")
    parser.add_argument(
        "--ckpt",
        type=int,
        default=None,
        help=(
            "If set, load the rsl_rl checkpoint logs/<exp_name>/model_<ckpt>.pt "
            "(produced by go2_backflip_train). Default: load the upstream "
            "TorchScript file ./backflip/<exp_name>.pt."
        ),
    )
    parser.add_argument("--cpu", action="store_true",
                        help="Force CPU backend (default: GPU).")
    args = parser.parse_args()

    gs.init(backend=gs.cpu if args.cpu else gs.gpu)

    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()

    if args.exp_name == "single":
        env_cfg["episode_length_s"] = 2
    elif args.exp_name == "double":
        env_cfg["episode_length_s"] = 3
    else:
        raise RuntimeError

    if args.ckpt is not None:
        # Training horizon for the trainer matches eval; loosen termination
        # so the loaded policy isn't truncated mid-flip during playback.
        env_cfg["termination_if_pitch_greater_than"] = 1e6
        env_cfg["termination_if_roll_greater_than"] = 1e6

    env = BackflipEnv(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )

    if args.ckpt is None:
        # Upstream-style: TorchScript flat file produced by their separate
        # research repo and downloaded to ./backflip/<exp_name>.pt.
        policy = torch.jit.load(f"./backflip/{args.exp_name}.pt")
        policy.to(device=gs.device)
        obs, _ = env.reset()
        with torch.no_grad():
            while True:
                actions = policy(obs)
                obs, rews, dones, infos = env.step(actions)
    else:
        # Trainer-style: load via rsl_rl OnPolicyRunner (same as go2_eval).
        import os
        import pickle

        from rsl_rl.runners import OnPolicyRunner

        # Wrap step/reset/get_observations so the runner sees a TensorDict
        # like the trainer expects. Mirrors BackflipTrainEnv's shim.
        from tensordict import TensorDict

        from .go2_env import Go2Env

        _orig_get_obs = env.get_observations
        _num_envs = env.num_envs

        def _get_obs_dict():
            _orig_get_obs()  # populates env.obs_buf
            return TensorDict({"policy": env.obs_buf}, batch_size=[_num_envs])

        env.get_observations = _get_obs_dict
        env.step = lambda actions, _e=env: Go2Env.step(_e, actions)
        env.reset = lambda _e=env: Go2Env.reset(_e)

        log_dir = f"logs/{args.exp_name}"
        with open(os.path.join(log_dir, "cfgs.pkl"), "rb") as fh:
            _, _, _, _, train_cfg = pickle.load(fh)
        runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
        runner.load(os.path.join(log_dir, f"model_{args.ckpt}.pt"))
        policy = runner.get_inference_policy(device=gs.device)

        obs_dict = env.reset()
        with torch.no_grad():
            while True:
                actions = policy(obs_dict)
                obs_dict, rews, dones, infos = env.step(actions)


if __name__ == "__main__":
    main()

"""
# evaluation
python examples/locomotion/go2_backflip.py -e single
python examples/locomotion/go2_backflip.py -e double
"""
