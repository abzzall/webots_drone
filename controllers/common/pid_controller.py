# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  MIT Licence
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: pid_controller.py

A simple PID controller for the Crazyflie
ported from pid_controller.c in the c-based controller of the Crazyflie
in Webots
"""

# import numpy as np
import torch
from grid import device

class pid_velocity_fixed_height_controller():
    def __init__(self):
        self.past_v_error = torch.zeros(2,   dtype=torch.float64, device=device)
        self.past_alt_error  =torch.tensor(0,    dtype=torch.float64, device=device)
        self.past_pitch_error  =torch.tensor(0,    dtype=torch.float64, device=device)
        self.past_roll_error =torch.tensor(0,    dtype=torch.float64, device=device)
        self.altitude_integrator =torch.tensor(0,    dtype=torch.float64, device=device)
        self.last_time  =torch.tensor(0,    dtype=torch.float64, device=device)

    def pid(self, dt, desired_v, desired_yaw_rate, desired_altitude, actual_roll, actual_pitch, actual_yaw_rate,
            actual_altitude, actual_v):
        # Velocity PID control (converted from Crazyflie c code)
        gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                 "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}

        # Velocity PID control
        v_error = desired_v - actual_v
        v_deriv = (v_error - self.past_v_error) / dt
        desired_pitch = gains["kp_vel_xy"] * torch.clip(v_error[0], -1, 1) + gains["kd_vel_xy"] * v_deriv[0]
        desired_roll = -gains["kp_vel_xy"] * torch.clip(v_error[1], -1, 1) - gains["kd_vel_xy"] * v_deriv[1]
        self.past_v_error = v_error

        # Altitude PID control
        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
            gains["ki_z"] * torch.clip(self.altitude_integrator, -2, 2) + 48
        self.past_alt_error = alt_error

        # Attitude PID control
        pitch_error = desired_pitch - actual_pitch
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - actual_roll
        roll_deriv = (roll_error - self.past_roll_error) / dt
        yaw_rate_error = desired_yaw_rate - actual_yaw_rate
        roll_command = gains["kp_att_rp"] * torch.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        pitch_command = -gains["kp_att_rp"] * torch.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        yaw_command = gains["kp_att_y"] * torch.clip(yaw_rate_error, -1, 1)
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error

        # Motor mixing
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        if(m1>600 or m1<0):
            print(f'wrong m1: m1={m1}')
        if (m2>600 or m2<0):
            print(f'wrong m2: m2={m2}')
        if (m3>600 or m3<0):
            print(f'wrong m3: m3={m3}')
        if (m4>600 or m4<0):
            print(f'wrong m4: m4={m4}')

        # Limit the motor command
        m1 = torch.clip(m1, 0, 600)
        m2 = torch.clip(m2, 0, 600)
        m3 = torch.clip(m3, 0, 600)
        m4 = torch.clip(m4, 0, 600)

        return torch.tensor([m1, m2, m3, m4],    dtype=torch.float64, device=device)
