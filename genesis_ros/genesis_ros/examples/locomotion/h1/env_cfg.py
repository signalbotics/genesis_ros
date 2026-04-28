"""H1 humanoid env configs — flat and rough variants.

Mirrors `isaaclab_tasks/.../velocity/config/h1/{flat,rough}_env_cfg.py`.
"""

from __future__ import annotations

from ..humanoid.env import CommandRanges, EventCfg, HumanoidEnvCfg, RobotCfg
from ..humanoid.terrain import default_rough_cfg


# 19-DOF H1.
H1_JOINT_ORDER = [
    "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_joint",
    "torso_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",
]

H1_DEFAULT_JOINT_ANGLES = {n: 0.0 for n in H1_JOINT_ORDER}
H1_DEFAULT_JOINT_ANGLES.update({
    "left_hip_pitch_joint": -0.4,
    "right_hip_pitch_joint": -0.4,
    "left_knee_joint": 0.8,
    "right_knee_joint": 0.8,
    "left_ankle_joint": -0.4,
    "right_ankle_joint": -0.4,
    "left_shoulder_roll_joint": 0.16,
    "right_shoulder_roll_joint": -0.16,
    "left_elbow_joint": 0.87,
    "right_elbow_joint": 0.87,
})


def make_h1_robot_cfg() -> RobotCfg:
    return RobotCfg(
        name="h1",
        urdf_path="urdf/h1/h1.urdf",
        num_actions=len(H1_JOINT_ORDER),
        joint_names=H1_JOINT_ORDER,
        default_joint_angles=H1_DEFAULT_JOINT_ANGLES,
        # PD gains tuned for Genesis's solver — see notes in g1/env_cfg.py.
        kp={
            ".*_hip_.*": 50.0,
            ".*_knee_joint": 60.0,
            ".*_ankle_joint": 20.0,
            "torso_joint": 60.0,
            ".*_shoulder_.*": 20.0,
            ".*_elbow_joint": 20.0,
        },
        kd={
            ".*_hip_.*": 1.5,
            ".*_knee_joint": 1.5,
            ".*_ankle_joint": 0.5,
            "torso_joint": 1.5,
            ".*_shoulder_.*": 0.5,
            ".*_elbow_joint": 0.5,
        },
        base_init_pos=(0.0, 0.0, 1.10),
        foot_body_names=[".*_ankle_link"],
        torso_body_names=[".*torso_link"],
        action_scale=0.25,
        clip_actions=100.0,
        termination_roll_deg=60.0,
        termination_pitch_deg=60.0,
    )


def _h1_rough_rewards() -> tuple[dict, dict]:
    scales = {
        "termination_penalty": -200.0,
        "track_lin_vel_xy_yaw_frame_exp": 1.0,
        "track_ang_vel_z_world_exp": 1.0,
        "feet_air_time_positive_biped": 0.25,
        "feet_slide": -0.25,
        "dof_pos_limits": -1.0,
        "joint_deviation_hip": -0.2,
        "joint_deviation_arms": -0.2,
        "joint_deviation_torso": -0.1,
        "flat_orientation_l2": -1.0,
        "action_rate_l2": -0.005,
        "dof_acc_l2": -1.25e-7,
        "dof_torques_l2": 0.0,
    }
    params = {
        "track_lin_vel_xy_yaw_frame_exp": {"std": 0.5},
        "track_ang_vel_z_world_exp": {"std": 0.5},
        "feet_air_time_positive_biped": {"threshold": 0.4},
        "dof_pos_limits": {"joint_names": [".*_ankle_joint"]},
        "joint_deviation_hip": {"joint_names": [".*_hip_yaw_joint", ".*_hip_roll_joint"]},
        "joint_deviation_arms": {"joint_names": [".*_shoulder_.*", ".*_elbow_joint"]},
        "joint_deviation_torso": {"joint_names": ["torso_joint"]},
    }
    return scales, params


def _h1_flat_rewards() -> tuple[dict, dict]:
    scales, params = _h1_rough_rewards()
    # No flat-specific overrides in H1FlatEnvCfg apart from terrain/heightscan toggles
    # (those are handled at HumanoidEnvCfg level).
    return scales, params


def h1_flat_cfg(num_envs: int = 4096) -> HumanoidEnvCfg:
    scales, params = _h1_flat_rewards()
    return HumanoidEnvCfg(
        num_envs=num_envs,
        episode_length_s=20.0,
        sim_dt=0.005,
        decimation=4,
        resample_command_s=10.0,
        use_height_scan=False,
        robot=make_h1_robot_cfg(),
        commands=CommandRanges(lin_vel_x=(0.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-1.0, 1.0)),
        events=EventCfg(push_interval_s=10.0, push_lin_vel=(-0.5, 0.5), add_base_mass=None, com_xy_range=None),
        terrain=None,
        reward_scales=scales,
        reward_params=params,
        base_height_target=1.05,
    )


def h1_rough_cfg(num_envs: int = 4096) -> HumanoidEnvCfg:
    scales, params = _h1_rough_rewards()
    return HumanoidEnvCfg(
        num_envs=num_envs,
        episode_length_s=20.0,
        sim_dt=0.005,
        decimation=4,
        resample_command_s=10.0,
        use_height_scan=True,
        robot=make_h1_robot_cfg(),
        commands=CommandRanges(lin_vel_x=(0.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-1.0, 1.0)),
        events=EventCfg(push_interval_s=10.0, push_lin_vel=(-0.5, 0.5), add_base_mass=None, com_xy_range=None),
        terrain=default_rough_cfg(),
        reward_scales=scales,
        reward_params=params,
        base_height_target=1.05,
    )
