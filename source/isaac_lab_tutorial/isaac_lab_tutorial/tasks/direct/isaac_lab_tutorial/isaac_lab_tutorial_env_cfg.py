# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaac_lab_tutorial.robots.jetbot import JETBOT_CONFIG

from isaaclab.assets  import ArticulationCfg
from isaaclab.envs    import DirectRLEnvCfg
from isaaclab.scene   import InteractiveSceneCfg
from isaaclab.sim     import SimulationCfg
from isaaclab.utils   import configclass

@configclass
class IsaacLabTutorialEnvCfg(DirectRLEnvCfg):
    # ===== Environment parameters =====
    decimation = 10           # Render frame skipping rate (render every 2 simulation steps)
    episode_length_s = 2     # Length of each episode in seconds

    # ===== Action and observation space sizes =====
    action_space = 2       # Dimension of the action space (e.g., 2 control inputs)
    observation_space = 3  # Dimension of the observation space 
    state_space = 0                     # Dimension of the state space, here set to 0

    # ===== Simulation settings =====
    sim: SimulationCfg = SimulationCfg(dt=1/120, render_interval=decimation)
    # ===== Robot configuration =====
    robot_cfg: ArticulationCfg = JETBOT_CONFIG.replace(prim_path="/World/envs/env_.*/Robot")
    # ===== Scene (environment) settings =====
    scene: InteractiveSceneCfg=InteractiveSceneCfg(num_envs=2, env_spacing=4.0, replicate_physics=True)

    # ===== Names of the degrees of freedom (joints) controlled =====
    dof_names = ["left_wheel_joint", "right_wheel_joint"]   # Names of robot's wheel joints

##### BELOW PART BELONGS BEFORE TUTORIAL MODIFICATIONS ######
# from isaaclab_assets.robots.cartpole import CARTPOLE_CFG

# from isaaclab.assets import ArticulationCfg
# from isaaclab.envs import DirectRLEnvCfg
# from isaaclab.scene import InteractiveSceneCfg
# from isaaclab.sim import SimulationCfg
# from isaaclab.utils import configclass


# @configclass
# class IsaacLabTutorialEnvCfg(DirectRLEnvCfg):
#     # env
#     decimation = 2
#     episode_length_s = 5.0
#     # - spaces definition
#     action_space = 1
#     observation_space = 4
#     state_space = 0

#     # simulation
#     sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

#     # robot(s)
#     robot_cfg: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="/World/envs/env_.*/Robot")

#     # scene
#     scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

#     # custom parameters/scales
#     # - controllable joint
#     cart_dof_name = "slider_to_cart"
#     pole_dof_name = "cart_to_pole"
#     # - action scale
#     action_scale = 100.0  # [N]
#     # - reward scales
#     rew_scale_alive = 1.0
#     rew_scale_terminated = -2.0
#     rew_scale_pole_pos = -1.0
#     rew_scale_cart_vel = -0.01
#     rew_scale_pole_vel = -0.005
#     # - reset states/conditions
#     initial_pole_angle_range = [-0.25, 0.25]  # pole angle sample range on reset [rad]
#     max_cart_pos = 3.0  # reset if cart exceeds this position [m]