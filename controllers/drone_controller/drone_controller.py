# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2023 Bitcraze


"""
file: crazyflie_py_wallfollowing.py

Controls the crazyflie and implements a wall following method in webots in Python

Author:   Kimberly McGuire (Bitcraze AB)
"""
import numpy
import numpy as np
import sys
# Add the libraries directory to the Python path
sys.path.append('../..')
from controller import Robot
from controller import Keyboard

from controllers.common.grid import *
from controllers.common.pid_controller import pid_velocity_fixed_height_controller
import json
FLYING_ATTITUDE = 1
V_MAX=1
dV=0.1

def angle_between_vectors(v1, v2):
    if np.all(v1 == 0) or np.all(v2 == 0):
        return np.arccos(0)
    """Calculate the signed angle between two 2D vectors using NumPy."""
    dot = np.dot(v1, v2)  # Dot product
    det = np.linalg.det([v1, v2])  # Determinant (cross product in 2D)
    angle = np.arctan2(det, dot)  # Signed angle in radians
    return angle



def get_target_vector(direction:str, x, y):
    nx, ny = find_grid_coordinates(x, y)
    border_x1, border_y1, border_x2, border_y2=grid_borders(nx, ny)
    if direction == 'left_bottom':
        target_x=border_x1
        target_y=border_y1
    elif direction == 'bottom':
        target_x=(border_x1+border_x2)/2
        target_y=border_y1
    elif direction == 'right_bottom':
        target_x=border_x2
        target_y=border_y1
    elif direction == 'right':
        target_x=border_x2
        target_y=(border_x1 + border_y2)/2
    elif direction == 'right_top':
        target_x=border_x2
        target_y=border_y2
    elif direction == 'top':
        target_x=(border_x1+border_x2)/2
        target_y=border_y2
    elif direction == 'left_top':
        target_x=border_x1
        target_y=border_y2
    elif direction == 'left':
        target_x=border_x1
        target_y=(border_y1+border_y2)/2
    return np.array( [target_x-x, target_y-y])

def vector_length(x, y):
    return np.sqrt(x**2 + y**2)

def Fa(directions: dict, x, y,  v_x, v_y):
    S_x, S_y = 0, 0

    for direction, priority in directions.items():
        target_vector = get_target_vector(direction, x, y)
        target_vector_length = vector_length(target_vector[0], target_vector[1])
        target_angle = angle_between_vectors([v_x, v_y], target_vector)
        koef=priority+0.0001*np.cos(target_angle)+0.0001*target_vector_length/MAX_TARGET_DISTANCE
        S_x+=target_vector[0]*koef/target_vector_length
        S_y+=target_vector[1]*koef/target_vector_length

    return np.array([S_x, S_y])




def V(Fa, v_x, v_y, dV_max=dV):
    Fa_len=vector_length(Fa[0], Fa[1])
    dv=Fa*dV_max/Fa_len
    new_v_x=dv[0]+v_x
    new_v_y=dv[1]+v_y
    magnitude = np.sqrt(new_v_x ** 2 + new_v_y ** 2)
    if magnitude > V_MAX:
        scale = V_MAX / magnitude
        new_v_x *= scale
        new_v_y *= scale
    return np.array([new_v_x, new_v_y])



if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    print('drone controller started')
    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)
    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)
    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)
    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    name=robot.getName()
    id=int(name[6:])
    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize variables

    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE

    #
    #
    # print("\n")
    #
    # print("====== Controls =======\n\n")
    #
    # print(" The Crazyflie can be controlled from your keyboard!\n")
    # print(" All controllable movement is in body coordinates\n")
    # print("- Use the up, back, right and left button to move in the horizontal plane\n")
    # print("- Use Q and E to rotate around yaw\n ")
    # print("- Use W and S to go up and down\n ")
    # print("- Press A to start autonomous mode\n")
    # print("- Press D to disable autonomous mode\n")
    MESSAGE_ID = -1
    # Main loop:
    pause = True
    v=[0, 0]


    desired_v=0
    updated_time=robot.getTime()
    yaw_desired = 0
    desired_turn=0
    move_started = False
    while robot.step(timestep) != -1:




        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()

            first_time = False

        # Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        altitude = gps.getValues()[2]

        # Get body fixed velocities
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw
        if((v_x==0 and v_y==0)or (v_x_global==0 and v_y_global==0)):
            print(f'{robot.getTime() }: {desired_v}, {updated_time}, {yaw_desired}, {desired_turn}')
        # Initialize values
        desired_state = [0, 0, 0, 0]
        # forward_desired = 0
        # sideways_desired = 0

        height_diff_desired = 0

        message_received = False

        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            receiver.nextPacket()  # Move to the next message

            try:
                # Parse JSON message
                message_data = json.loads(message)
                # Process received data
                message_id = message_data["MESSAGE_ID"]
                if (message_id > MESSAGE_ID):

                    command=message_data["COMMAND"]
                    # print(f"Command received{command}")
                    if (command == 'pause'):
                        pause=True
                    elif (command == 'up'):
                        height_diff_desired+=float(message_data["CONTENT"])
                    elif (command == 'go'):

                        message_received=True
                        new_info=message_data["CONTENT"][str(id)]
                        # print(f'start moving: {new_info}')

            except json.JSONDecodeError:
                print("Error decoding JSON message")


        if message_received:
            print(f'actual yaw_rate: {yaw_rate}, estimated yaw_rate: {desired_turn}')


            fa=Fa(directions=new_info, v_x=v_x_global, v_y=v_y_global, x=x_global, y=y_global)
            if not pause:
                algorithmic_dt = robot.getTime() - updated_time
                desired_turn=angle_between_vectors([v_x_global, v_y_global], fa)*dt/algorithmic_dt

            else:
                pause=False
                algorithmic_dt=1
                desired_turn=np.atan2(fa)
                desired_v=0.1
        elif pause:
            desired_v=0
            yaw_desired=0

            # desired_turn= numpy.clip( desired_turn*dt, 0, numpy.pi*dt/2)
            # height_desired=altitude
            print(f'new yaw_desired: {desired_turn}')

            print(f'dt: {dt}, algorithmic dt: {algorithmic_dt}')
            updated_time=robot.getTime()


            print(f'desired yaw_desired: {yaw_desired}')


        height_desired += height_diff_desired * dt

        camera_data = camera.getImage()

        # get range in meters
        range_front_value = range_front.getValue() / 1000
        range_right_value = range_right.getValue() / 1000
        range_left_value = range_left.getValue() / 1000

        # Choose a wall following direction
        # if you choose direction left, use the right range value
        # if you choose direction right, use the left range value
        range_side_value = range_right_value



        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, desired_v, 0,
                                        desired_turn, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)
        # print(f'motor_power: {motor_power}')
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
        # print(f'GPS position: {[x_global, y_global]}')

#encrypting_key === private
# signature

#decrypting_key  === public
#decrypt(signature, public) = original