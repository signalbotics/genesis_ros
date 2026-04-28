"""G1 humanoid env configs — flat and rough variants.

Reward weights, command ranges, joint groups copied from
`isaaclab_tasks/.../velocity/config/g1/{flat,rough}_env_cfg.py` so policies
trained here are directly comparable.
"""

from __future__ import annotations

from ..humanoid.env import CommandRanges, EventCfg, HumanoidEnvCfg, RobotCfg
from ..humanoid.terrain import default_rough_cfg


# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------

# 29-DOF G1 — actuated joints in URDF order.
G1_JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# Default (nominal stance) joint angles — mostly zeros with knee bend & elbow bend.
G1_DEFAULT_JOINT_ANGLES = {n: 0.0 for n in G1_JOINT_ORDER}
G1_DEFAULT_JOINT_ANGLES.update({
    "left_hip_pitch_joint": -0.1,
    "right_hip_pitch_joint": -0.1,
    "left_knee_joint": 0.3,
    "right_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2,
    "right_ankle_pitch_joint": -0.2,
    "left_shoulder_pitch_joint": 0.2,
    "right_shoulder_pitch_joint": 0.2,
    "left_elbow_joint": 0.3,
    "right_elbow_joint": 0.3,
})


def make_g1_robot_cfg() -> RobotCfg:
    return RobotCfg(
        name="g1",
        urdf_path="urdf/g1/g1.urdf",
        num_actions=len(G1_JOINT_ORDER),
        joint_names=G1_JOINT_ORDER,
        default_joint_angles=G1_DEFAULT_JOINT_ANGLES,
        # PD gains tuned for Genesis's solver — IsaacLab values (kp ~100-200)
        # assume PhysX implicit-PD and diverge here; ~3x lower is stable.
        kp={
            ".*_hip_.*": 40.0,
            ".*_knee_joint": 60.0,
            ".*_ankle_.*": 20.0,
            "waist_.*": 60.0,
            ".*_shoulder_.*": 20.0,
            ".*_elbow_.*": 20.0,
            ".*_wrist_.*": 10.0,
        },
        kd={
            ".*_hip_.*": 1.0,
            ".*_knee_joint": 1.5,
            ".*_ankle_.*": 0.5,
            "waist_.*": 1.5,
            ".*_shoulder_.*": 0.5,
            ".*_elbow_.*": 0.5,
            ".*_wrist_.*": 0.3,
        },
        base_init_pos=(0.0, 0.0, 0.85),
        foot_body_names=[".*_ankle_roll_link"],
        torso_body_names=["torso_link"],
        action_scale=0.25,
        clip_actions=100.0,
        termination_roll_deg=60.0,
        termination_pitch_deg=60.0,
    )


# ---------------------------------------------------------------------------
# Reward set (matches G1RoughEnvCfg / G1FlatEnvCfg)
# ---------------------------------------------------------------------------


def _g1_rough_rewards() -> tuple[dict, dict]:
    scales = {
        "termination_penalty": -200.0,
        "track_lin_vel_xy_yaw_frame_exp": 1.0,
        "track_ang_vel_z_world_exp": 2.0,
        "feet_air_time_positive_biped": 0.25,
        "feet_slide": -0.1,
        "dof_pos_limits": -1.0,
        "joint_deviation_hip": -0.1,
        "joint_deviation_arms": -0.1,
        "joint_deviation_torso": -0.1,
        "lin_vel_z_l2": 0.0,
        "flat_orientation_l2": -1.0,
        "action_rate_l2": -0.005,
        "dof_acc_l2": -1.25e-7,
        "dof_torques_l2": -1.5e-7,
    }
    params = {
        "track_lin_vel_xy_yaw_frame_exp": {"std": 0.5},
        "track_ang_vel_z_world_exp": {"std": 0.5},
        "feet_air_time_positive_biped": {"threshold": 0.4},
        "dof_pos_limits": {"joint_names": [".*_ankle_pitch_joint", ".*_ankle_roll_joint"]},
        "joint_deviation_hip": {"joint_names": [".*_hip_yaw_joint", ".*_hip_roll_joint"]},
        "joint_deviation_arms": {
            "joint_names": [
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint",
            ]
        },
        "joint_deviation_torso": {"joint_names": ["waist_.*_joint"]},
        "dof_acc_l2": {"joint_names": [".*_hip_.*", ".*_knee_joint"]},
        "dof_torques_l2": {"joint_names": [".*_hip_.*", ".*_knee_joint", ".*_ankle_.*"]},
    }
    return scales, params


def _g1_flat_rewards() -> tuple[dict, dict]:
    scales, params = _g1_rough_rewards()
    # Flat-only overrides from G1FlatEnvCfg.
    scales["track_ang_vel_z_world_exp"] = 1.0
    scales["lin_vel_z_l2"] = -0.2
    scales["action_rate_l2"] = -0.005
    scales["dof_acc_l2"] = -1.0e-7
    scales["feet_air_time_positive_biped"] = 0.75
    scales["dof_torques_l2"] = -2.0e-6
    params["feet_air_time_positive_biped"] = {"threshold": 0.4}
    params["dof_torques_l2"] = {"joint_names": [".*_hip_.*", ".*_knee_joint"]}
    return scales, params


# ---------------------------------------------------------------------------
# Top-level cfgs
# ---------------------------------------------------------------------------


def g1_flat_cfg(num_envs: int = 4096) -> HumanoidEnvCfg:
    scales, params = _g1_flat_rewards()
    return HumanoidEnvCfg(
        num_envs=num_envs,
        episode_length_s=20.0,
        sim_dt=0.005,
        decimation=4,
        resample_command_s=10.0,
        use_height_scan=False,
        robot=make_g1_robot_cfg(),
        commands=CommandRanges(lin_vel_x=(0.0, 1.0), lin_vel_y=(-0.5, 0.5), ang_vel_z=(-1.0, 1.0)),
        events=EventCfg(push_interval_s=10.0, push_lin_vel=(-0.5, 0.5), add_base_mass=None, com_xy_range=None),
        terrain=None,
        reward_scales=scales,
        reward_params=params,
        base_height_target=0.78,
    )


def g1_rough_cfg(num_envs: int = 4096) -> HumanoidEnvCfg:
    scales, params = _g1_rough_rewards()
    return HumanoidEnvCfg(
        num_envs=num_envs,
        episode_length_s=20.0,
        sim_dt=0.005,
        decimation=4,
        resample_command_s=10.0,
        use_height_scan=True,
        robot=make_g1_robot_cfg(),
        commands=CommandRanges(lin_vel_x=(0.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-1.0, 1.0)),
        events=EventCfg(push_interval_s=10.0, push_lin_vel=(-0.5, 0.5), add_base_mass=None, com_xy_range=None),
        terrain=default_rough_cfg(),
        reward_scales=scales,
        reward_params=params,
        base_height_target=0.78,
    )
