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
import sys


# Add the libraries directory to the Python path
sys.path.append('../..')
import torch
from controller import Robot
from controller import Keyboard

from controllers.common.grid import *
from controllers.common.pid_controller import pid_velocity_fixed_height_controller
import json
# from controllers.common.db_handler import DatabaseHandler
# from controllers.common.async_db_handler import DroneDBLogger
# Initialize Database (Session starts in constructor)
# init_pool()
FLYING_ATTITUDE = torch.tensor( 1,  dtype=torch.float64, device=device)

ALTITUDE_THRESHOLD = torch.tensor(0.15, dtype=torch.float64, device=device)  # 10 cm threshold
ANGULAR_THRESHOLD = torch.deg2rad(torch.tensor(20.0, dtype=torch.float64, device=device))
V_MAX=torch.tensor( 1,  dtype=torch.float64, device=device)
dV=torch.tensor(0.1,  dtype=torch.float64, device=device)


def angle_between_vectors(v1: torch.Tensor, v2: torch.Tensor):
    # Check if either vector is a zero vector (using torch.equal for scalar comparison)
    if torch.all(v1 == 0) or torch.all(v2 == 0):
        return torch.arccos(torch.tensor(0.0))  # Return 0 angle for zero vectors

    # Calculate the dot product
    dot = torch.dot(v1, v2)

    # Calculate the determinant (cross product in 2D)
    det = v1[0] * v2[1] - v1[1] * v2[0]  # Cross product in 2D

    # Calculate the signed angle in radians using arctan2
    angle = torch.atan2(det, dot)

    return angle


def get_target_vector(direction:str, pos, gridBorders):

    left_x, middle_x, right_x, bottom_y, middle_y, top_y=gridBorders
    if direction == 'left_bottom':
        return torch.tensor([left_x, bottom_y], dtype=torch.float64, device=device)-pos
    elif direction == 'bottom':
        return torch.tensor([middle_x, bottom_y], dtype=torch.float64, device=device)-pos
    elif direction == 'right_bottom':
        return torch.tensor([right_x, bottom_y], dtype=torch.float64, device=device)-pos
    elif direction == 'right':
        return torch.tensor([right_x, middle_y], dtype=torch.float64, device=device)-pos
    elif direction == 'right_top':
        return torch.tensor([right_x, top_y], dtype=torch.float64, device=device)-pos
    elif direction == 'top':
        return torch.tensor([middle_x, top_y], dtype=torch.float64, device=device)-pos
    elif direction == 'left_top':
        return torch.tensor([left_x, top_y], dtype=torch.float64, device=device)-pos
    elif direction == 'left':
        return torch.tensor([left_x, middle_y], dtype=torch.float64, device=device)-pos

def vector_length(vector):
    return torch.norm(vector)

def Fa(directions: dict, pos,  v_old, gridBorders):
    result = torch.zeros(2, dtype=torch.float64, device=device)

    for direction, priority in directions.items():
        target_vector = get_target_vector(direction, pos, gridBorders)
        target_vector_length = vector_length(target_vector)
        target_angle = angle_between_vectors(v_old, target_vector)
        koef=priority+0.01*torch.cos(target_angle)+0.01*target_vector_length/MAX_TARGET_DISTANCE
        result+=target_vector*koef/target_vector_length
    return result

def calc_dv(Fa, dV_max=dV):
    return Fa*dV_max/vector_length(Fa)

def V(v_old, dv, alpha=0.5):
    new_v=dv+v_old
    new_v=alpha*v_old+(1-alpha)*new_v
    magnitude = torch.norm(new_v)
    if magnitude > V_MAX:
        scale = V_MAX / magnitude
        new_v *= scale

    return new_v


def globalSpeedToLocal(v_global, yaw):
    cos_yaw = torch.cos(yaw)
    sin_yaw = torch.sin(yaw)
    v_x = v_global[0] * cos_yaw + v_global[1] * sin_yaw
    v_y = - v_global[0] * sin_yaw + v_global[1] * cos_yaw
    return torch.tensor([v_x, v_y], dtype=torch.float64, device=device)

if __name__ == '__main__':
    t=0
    robot = Robot()
    # Retrieve the current episode ID
    timestep = int(robot.getBasicTimeStep())
    print(f'{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: drone controller started')
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
    # camera = robot.getDevice("camera")
    # camera.enable(timestep)
    # range_front = robot.getDevice("range_front")
    # range_front.enable(timestep)
    # range_left = robot.getDevice("range_left")
    # range_left.enable(timestep)
    # range_back = robot.getDevice("range_back")
    # range_back.enable(timestep)
    # range_right = robot.getDevice("range_right")
    # range_right.enable(timestep)
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    name=robot.getName()
    id=int(name[6:])
    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize variables

    past_pos_global = torch.zeros(2, dtype=torch.float64, device=device)

    past_time = torch.tensor( 0, dtype=torch.float64, device=device)
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


    # db_logger=DroneDBLogger(id)
    while robot.step(timestep) != -1:
        # db_logger.next_step()



        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_pos_global = torch.tensor( gps.getValues()[:2], dtype=torch.float64, device=device)
            past_time = robot.getTime()
            first_time = False
            ideal_yaw = torch.tensor( imu.getRollPitchYaw()[2], dtype=torch.float64, device=device)

        # Get sensor data
        roll = torch.tensor( imu.getRollPitchYaw()[0], dtype=torch.float64, device=device)
        pitch = torch.tensor( imu.getRollPitchYaw()[1], dtype=torch.float64, device=device)
        yaw = torch.tensor( imu.getRollPitchYaw()[2], dtype=torch.float64, device=device)
        yaw_rate = torch.tensor(gyro.getValues()[2], dtype=torch.float64, device=device)
        pos_global = torch.tensor(gps.getValues()[:2], dtype=torch.float64, device=device)
        v_global = (pos_global - past_pos_global)/dt

        altitude = torch.tensor(gps.getValues()[2], dtype=torch.float64, device=device)
        # Get body fixed velocities
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
        v_current = globalSpeedToLocal(v_global, yaw)

        # Initialize values
        desired_state = torch.zeros(4, dtype=torch.float64, device=device)
        # forward_desired = 0
        # sideways_desired = 0
        yaw_desired = ideal_yaw - yaw
        height_diff_desired = torch.tensor(0, dtype=torch.float64, device=device)

        message_received = False
        altitude_error = torch.abs(altitude - height_desired)
        pitch_error = torch.abs(pitch)
        roll_error = torch.abs(roll)
        if altitude_error <= ALTITUDE_THRESHOLD and pitch_error <= ANGULAR_THRESHOLD and roll_error <= ANGULAR_THRESHOLD:

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
                        # db_logger.insert_msg_received(message_id, message)
                            # print(f'start moving: {new_info}')

                except json.JSONDecodeError:
                    print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}:Error decoding JSON message")


            if message_received:
                if pause:
                    pause=False
                    # print('Starting movement')
                    #start going with
                    v=torch.tensor([1, 0] , dtype=torch.float64, device=device)
                    calculated_v=torch.tensor([0, 0] , dtype=torch.float64, device=device) #first is wrong (to avoid NullPointer)

            elif pause:
                v=torch.tensor([0,0], dtype=torch.float64, device=device)
            elif not pause:
                # print(
                #     f'Estimated velocity: {calculated_v},{yaw_desired}  real velocity: {[v_current, yaw]}, global velocity: [{v_global}]')
                # print(f'dSpeed: {v - v_current},[{yaw_desired}], {calculated_v - v_global})]')
                row, column = find_grid_coordinates(pos_global)
                gridBorders = grid_borders(row, column)
                fa = Fa(directions=new_info, v_old=v_global, pos=pos_global, gridBorders=gridBorders)

                dv = calc_dv(fa)

                calculated_v = V(v_old=v_global, dv=dv)

                v = globalSpeedToLocal(calculated_v, yaw)
                # height_desired=altitude
                # print(f'new velocity: {v}')
                # db_logger.insert_d_velocity(dv_x=dv[0], dv_y=dv[1])
                # db_logger.insert_fa(fa_x=fa[0], fa_y=fa[1])
                # db_logger.insert_velocity(velocity_x=calculated_v[0], velocity_y=calculated_v[1])
        else:
            if pause:

                v=torch.tensor([0,0], dtype=torch.float64, device=device)
            v=v_current*0.2+v*0.8
            print(f'{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: dumping: {id}: altitude error: {altitude_error}, pitch error: {pitch_error}, roll error: {roll_error}, timestep: {timestep}')

        height_desired += height_diff_desired * dt

        # camera_data = camera.getImage()

        # get range in meters
        # range_front_value = range_front.getValue() / 1000
        # range_right_value = range_right.getValue() / 1000
        # range_left_value = range_left.getValue() / 1000

        # Choose a wall following direction
        # if you choose direction left, use the right range value
        # if you choose direction right, use the left range value

        # db_logger.insert_drone_observed_state( roll=roll, pitch=pitch, yaw=yaw, yaw_rate=yaw_rate,
        #                             x_global=pos_global[0], y_global=pos_global[1], v_x_global=v_global[0], v_y_global=v_global[1],
        #                             altitude=altitude, v_x=v[0], v_y=v[1], dt=dt)
        #
        # db_logger.insert_drone_desired_state( desired_vx=v[0], desired_vy=v[1],
        #                            desired_yaw_rate=yaw_desired, desired_altitude=height_desired)
        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, v,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_current)
        # motor1_speed = Column(Float, nullable=False)
        # motor2_speed = Column(Float, nullable=False)
        # motor3_speed = Column(Float, nullable=False)
        # motor4_speed = Column(Float, nullable=False)
        # db_logger.insert_drone_motor_power(motor_power)
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_pos_global = pos_global


        t=t+1

        # print(f'GPS position: {[x_global, y_global]}')
    # db_logger.close()
#encrypting_key === private
# signature

#decrypting_key  === public
#decrypt(signature, public) = original