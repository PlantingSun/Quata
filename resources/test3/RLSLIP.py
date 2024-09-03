from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot

# def get_euler_xyz(q):
#     qx, qy, qz, qw = 0, 1, 2, 3
#     # roll (x-axis rotation)
#     sinr_cosp = 2.0 * (q[:, qw] * q[:, qx] + q[:, qy] * q[:, qz])
#     cosr_cosp = q[:, qw] * q[:, qw] - q[:, qx] * \
#         q[:, qx] - q[:, qy] * q[:, qy] + q[:, qz] * q[:, qz]
#     roll = torch.atan2(sinr_cosp, cosr_cosp)

#     # pitch (y-axis rotation)
#     sinp = 2.0 * (q[:, qw] * q[:, qy] - q[:, qz] * q[:, qx])
#     pitch = torch.where(torch.abs(sinp) >= 1, copysign(
#         np.pi / 2.0, sinp), torch.asin(sinp))

#     # yaw (z-axis rotation)
#     siny_cosp = 2.0 * (q[:, qw] * q[:, qz] + q[:, qx] * q[:, qy])
#     cosy_cosp = q[:, qw] * q[:, qw] + q[:, qx] * \
#         q[:, qx] - q[:, qy] * q[:, qy] - q[:, qz] * q[:, qz]
#     yaw = torch.atan2(siny_cosp, cosy_cosp)

#     return roll % (2*np.pi), pitch % (2*np.pi), yaw % (2*np.pi)

class RLSLIP(LeggedRobot):
    # def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
    #     super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
    #     # print(self.gravity_vec)
    #     print(self.projected_gravity)

    def compute_observations(self):
        qx = self.base_quat[:,0:1]
        qy = self.base_quat[:,1:2]
        qz = self.base_quat[:,2:3]
        qw = self.base_quat[:,3:4]
        roll = torch.atan2(2.0 * (qw * qx + qy * qz),
                           1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = torch.asin(2.0 * (qw * qy - qz * qx))

        self.obs_buf = torch.cat((
                                    self.base_lin_vel,
                                    self.base_ang_vel,
                                    roll, pitch,
                                    # self.projected_gravity,
                                    self.commands[:, :3],
                                    (self.dof_pos - self.default_dof_pos),
                                    # self.dof_vel * self.obs_scales.dof_vel,
                                    # self.actions
                                    ),dim=-1)
        if self.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec

    def _get_noise_scale_vec(self, cfg):
        noise_vec = torch.zeros_like(self.obs_buf[0])
        self.add_noise = self.cfg.noise.add_noise
        noise_scales = self.cfg.noise.noise_scales
        noise_level = self.cfg.noise.noise_level
        noise_vec[:3] = noise_scales.lin_vel * noise_level
        noise_vec[3:6] = noise_scales.ang_vel * noise_level
        noise_vec[6:8] = 0.
        noise_vec[8:11] = 0. # commands
        noise_vec[11:14] = noise_scales.dof_pos * noise_level

        # noise_vec[:3] = noise_scales.lin_vel * noise_level * self.obs_scales.lin_vel
        # noise_vec[3:6] = noise_scales.ang_vel * noise_level
        # noise_vec[6:9] = noise_scales.gravity * noise_level
        # noise_vec[9:12] = 0. # commands
        # noise_vec[12:15] = noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos
        # noise_vec[15:18] = noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel
        # noise_vec[18:21] = 0. # previous actions
        return noise_vec