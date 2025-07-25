# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import torch
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform

from .isaac_lab_tutorial_env_cfg import IsaacLabTutorialEnvCfg

from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaaclab.utils.math as math_utils

def define_markers() -> VisualizationMarkers:
    """Define markers with various different shapes."""
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/myMarkers",
        markers={
                "forward": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0)),
                ),
                "command": sim_utils.UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
                    scale=(0.25, 0.25, 0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
        },
    )
    return VisualizationMarkers(cfg=marker_cfg)

class IsaacLabTutorialEnv(DirectRLEnv):
    cfg: IsaacLabTutorialEnvCfg

    def __init__(self, cfg: IsaacLabTutorialEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.dof_idx, _ = self.robot.find_joints(self.cfg.dof_names)

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot_cfg)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground",cfg=GroundPlaneCfg())
        #clone and replicate (exact copies of the original source environment)
        self.scene.clone_environments(copy_from_source=False)
        # add articulation to scene
        self.scene.articulations["robot"] = self.robot
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)


        self.visualization_markers = define_markers()

        # setting aside useful variables for later
        self.up_dir = torch.tensor([0.0, 0.0, 1.0]).cuda()
        self.yaws = torch.zeros((self.cfg.scene.num_envs, 1)).cuda()
        self.commands = torch.randn((self.cfg.scene.num_envs, 3)).cuda()
        self.commands[:,-1] = 0.0
        self.commands = self.commands/torch.linalg.norm(self.commands, dim=1, keepdim=True)

        # offsets to account for atan range and keep things on [-pi, pi]
        ratio = self.commands[:,1]/(self.commands[:,0]+1E-8)
        gzero = torch.where(self.commands > 0, True, False)
        lzero = torch.where(self.commands < 0, True, False)
        plus = lzero[:,0]*gzero[:,1]
        minus = lzero[:,0]*lzero[:,1]
        offsets = torch.pi*plus - torch.pi*minus
        self.yaws = torch.atan(ratio).reshape(-1,1) + offsets.reshape(-1,1)

        self.marker_locations = torch.zeros((self.cfg.scene.num_envs, 3)).cuda()
        self.marker_offset = torch.zeros((self.cfg.scene.num_envs, 3)).cuda()
        self.marker_offset[:,-1] = 0.5
        self.forward_marker_orientations = torch.zeros((self.cfg.scene.num_envs, 4)).cuda()
        self.command_marker_orientations = torch.zeros((self.cfg.scene.num_envs, 4)).cuda()

    def _visualize_markers(self):
        self.marker_locations = self.robot.data.root_pos_w
        self.forward_marker_orientations = self.robot.data.root_quat_w
        self.command_marker_orientations = math_utils.quat_from_angle_axis(self.yaws, self.up_dir).squeeze()

        loc = self.marker_locations + self.marker_offset
        loc = torch.vstack((loc, loc))
        rots = torch.vstack((self.forward_marker_orientations, self.command_marker_orientations))

        all_envs = torch.arange(self.cfg.scene.num_envs)
        indices = torch.hstack((torch.zeros_like(all_envs), torch.ones_like(all_envs)))

        self.visualization_markers.visualize(loc, rots, marker_indices=indices)

    def _pre_physics_step(self, actions: torch.Tensor)->None:
        #Bu fonksiyon, her simülasyon adımında RL ajanından gelen action tensorünü alır
        #Gelen actions, self.actions değişkenine kopyalanarak saklanır.
        self.actions = actions.clone()
        self._visualize_markers()

    def _apply_action(self)->None:
        self.robot.set_joint_effort_target(self.actions, joint_ids=self.dof_idx)

    def _get_observations(self)->dict:
        self.velocity = self.robot.data.root_com_vel_w
        self.forwards = math_utils.quat_apply(self.robot.data.root_link_quat_w, self.robot.data.FORWARD_VEC_B)
        
        dot = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
        cross = torch.cross(self.forwards, self.commands, dim=-1)[:,-1].reshape(-1,1)
        forward_speed = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
        # Amac yukardaki 3 bilgiyi birlestirip gözlem vektörü oluşturmak 
        obs = torch.hstack((dot, cross, forward_speed))

        observations={"policy": obs}
        return observations

    def _get_rewards(self) -> torch.Tensor:
        forward_reward = self.robot.data.root_com_lin_vel_b[:,0].reshape(-1,1)
        alignment_reward = torch.sum(self.forwards * self.commands, dim=-1, keepdim=True)
        total_reward = forward_reward*torch.exp(alignment_reward) 

        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        return False, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        # pick new commands for reset envs
        self.commands[env_ids] = torch.randn((len(env_ids), 3)).cuda()
        self.commands[env_ids,-1] = 0.0
        self.commands[env_ids] = self.commands[env_ids]/torch.linalg.norm(self.commands[env_ids], dim=1, keepdim=True)

        # recalculate the orientations for the command markers with the new commands
        ratio = self.commands[env_ids][:,1]/(self.commands[env_ids][:,0]+1E-8)
        gzero = torch.where(self.commands[env_ids] > 0, True, False)
        lzero = torch.where(self.commands[env_ids]< 0, True, False)
        plus = lzero[:,0]*gzero[:,1]
        minus = lzero[:,0]*lzero[:,1]
        offsets = torch.pi*plus - torch.pi*minus
        self.yaws[env_ids] = torch.atan(ratio).reshape(-1,1) + offsets.reshape(-1,1)

        # set the root state for the reset envs
        default_root_state = self.robot.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.robot.write_root_state_to_sim(default_root_state, env_ids)
        self._visualize_markers()

    ##### BELOW PART BELONGS BEFORE TUTORIAL MODIFICATIONS (cardpole exemple)######
    # def __init__(self, cfg: IsaacLabTutorialEnvCfg, render_mode: str | None = None, **kwargs):
    #     super().__init__(cfg, render_mode, **kwargs)

    #     self._cart_dof_idx, _ = self.robot.find_joints(self.cfg.cart_dof_name)
    #     self._pole_dof_idx, _ = self.robot.find_joints(self.cfg.pole_dof_name)

    #     self.joint_pos = self.robot.data.joint_pos
    #     self.joint_vel = self.robot.data.joint_vel

    # def _setup_scene(self):
    #     self.robot = Articulation(self.cfg.robot_cfg)
    #     # add ground plane
    #     spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    #     # clone and replicate
    #     self.scene.clone_environments(copy_from_source=False)
    #     # we need to explicitly filter collisions for CPU simulation
    #     if self.device == "cpu":
    #         self.scene.filter_collisions(global_prim_paths=[])
    #     # add articulation to scene
    #     self.scene.articulations["robot"] = self.robot
    #     # add lights
    #     light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    #     light_cfg.func("/World/Light", light_cfg)

    # def _pre_physics_step(self, actions: torch.Tensor) -> None:
    #     self.actions = actions.clone()

    # def _apply_action(self) -> None:
    #     self.robot.set_joint_effort_target(self.actions * self.cfg.action_scale, joint_ids=self._cart_dof_idx)

    # def _get_observations(self) -> dict:
    #     obs = torch.cat(
    #         (
    #             self.joint_pos[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
    #             self.joint_vel[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
    #             self.joint_pos[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
    #             self.joint_vel[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
    #         ),
    #         dim=-1,
    #     )
    #     observations = {"policy": obs}
    #     return observations

    # def _get_rewards(self) -> torch.Tensor:
    #     total_reward = compute_rewards(
    #         self.cfg.rew_scale_alive,
    #         self.cfg.rew_scale_terminated,
    #         self.cfg.rew_scale_pole_pos,
    #         self.cfg.rew_scale_cart_vel,
    #         self.cfg.rew_scale_pole_vel,
    #         self.joint_pos[:, self._pole_dof_idx[0]],
    #         self.joint_vel[:, self._pole_dof_idx[0]],
    #         self.joint_pos[:, self._cart_dof_idx[0]],
    #         self.joint_vel[:, self._cart_dof_idx[0]],
    #         self.reset_terminated,
    #     )
    #     return total_reward

    # def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    #     self.joint_pos = self.robot.data.joint_pos
    #     self.joint_vel = self.robot.data.joint_vel

    #     time_out = self.episode_length_buf >= self.max_episode_length - 1
    #     out_of_bounds = torch.any(torch.abs(self.joint_pos[:, self._cart_dof_idx]) > self.cfg.max_cart_pos, dim=1)
    #     out_of_bounds = out_of_bounds | torch.any(torch.abs(self.joint_pos[:, self._pole_dof_idx]) > math.pi / 2, dim=1)
    #     return out_of_bounds, time_out

    # def _reset_idx(self, env_ids: Sequence[int] | None):
    #     if env_ids is None:
    #         env_ids = self.robot._ALL_INDICES
    #     super()._reset_idx(env_ids)

    #     joint_pos = self.robot.data.default_joint_pos[env_ids]
    #     joint_pos[:, self._pole_dof_idx] += sample_uniform(
    #         self.cfg.initial_pole_angle_range[0] * math.pi,
    #         self.cfg.initial_pole_angle_range[1] * math.pi,
    #         joint_pos[:, self._pole_dof_idx].shape,
    #         joint_pos.device,
    #     )
    #     joint_vel = self.robot.data.default_joint_vel[env_ids]

    #     default_root_state = self.robot.data.default_root_state[env_ids]
    #     default_root_state[:, :3] += self.scene.env_origins[env_ids]

    #     self.joint_pos[env_ids] = joint_pos
    #     self.joint_vel[env_ids] = joint_vel

    #     self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
    #     self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
    #     self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)


# @torch.jit.script
# def compute_rewards(
#     rew_scale_alive: float,
#     rew_scale_terminated: float,
#     rew_scale_pole_pos: float,
#     rew_scale_cart_vel: float,
#     rew_scale_pole_vel: float,
#     pole_pos: torch.Tensor,
#     pole_vel: torch.Tensor,
#     cart_pos: torch.Tensor,
#     cart_vel: torch.Tensor,
#     reset_terminated: torch.Tensor,
# ):
#     rew_alive = rew_scale_alive * (1.0 - reset_terminated.float())
#     rew_termination = rew_scale_terminated * reset_terminated.float()
#     rew_pole_pos = rew_scale_pole_pos * torch.sum(torch.square(pole_pos).unsqueeze(dim=1), dim=-1)
#     rew_cart_vel = rew_scale_cart_vel * torch.sum(torch.abs(cart_vel).unsqueeze(dim=1), dim=-1)
#     rew_pole_vel = rew_scale_pole_vel * torch.sum(torch.abs(pole_vel).unsqueeze(dim=1), dim=-1)
#     total_reward = rew_alive + rew_termination + rew_pole_pos + rew_cart_vel + rew_pole_vel
#     return total_reward