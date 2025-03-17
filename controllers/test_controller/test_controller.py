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
import numpy as np

from controller import Robot
from controller import Keyboard

from controllers.common.grid import *
from controllers.common.pid_controller import pid_velocity_fixed_height_controller
import json
from controllers.common.db_handler import DatabaseHandler

# Initialize Database (Session starts in constructor)
db = DatabaseHandler()
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



def get_target_vector(direction:str, x, y, gridBorders):

    left_x, middle_x, right_x, bottom_y, middle_y, top_y=gridBorders
    if direction == 'left_bottom':
        target_x=left_x
        target_y=bottom_y
    elif direction == 'bottom':
        target_x=middle_x
        target_y=bottom_y
    elif direction == 'right_bottom':
        target_x=right_x
        target_y=bottom_y
    elif direction == 'right':
        target_x=right_x
        target_y=middle_y
    elif direction == 'right_top':
        target_x=right_x
        target_y=top_y
    elif direction == 'top':
        target_x=middle_x
        target_y=top_y
    elif direction == 'left_top':
        target_x=left_x
        target_y=top_y
    elif direction == 'left':
        target_x=left_x
        target_y=middle_y
    return np.array( [target_x-x, target_y-y])

def vector_length(x, y):
    return np.sqrt(x**2 + y**2)

def Fa(directions: dict, x, y,  v_x, v_y, gridBorders):
    S_x, S_y = 0, 0

    for direction, priority in directions.items():
        target_vector = get_target_vector(direction, x, y, gridBorders)
        target_vector_length = vector_length(target_vector[0], target_vector[1])
        target_angle = angle_between_vectors([v_x, v_y], target_vector)
        koef=priority+0.1*np.cos(target_angle)+0.1*target_vector_length/MAX_TARGET_DISTANCE
        S_x+=target_vector[0]*koef/target_vector_length
        S_y+=target_vector[1]*koef/target_vector_length

    return np.array([S_x, S_y])

def calc_dv(Fa, dV_max=dV):
    return Fa*dV_max/vector_length(Fa[0], Fa[1])

def V(v_x, v_y, dv):

    new_v_x=dv[0]+v_x
    new_v_y=dv[1]+v_y
    magnitude = np.sqrt(new_v_x ** 2 + new_v_y ** 2)
    if magnitude > V_MAX:
        scale = V_MAX / magnitude
        new_v_x *= scale
        new_v_y *= scale
    return np.array([new_v_x, new_v_y])


def globalSpeedToLocal(v_x_global, v_y_global, yaw):
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
    v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw
    return np.array([v_x, v_y])

if __name__ == '__main__':
    t=0
    robot = Robot()
    # Retrieve the current episode ID
    db.current_episode_id = db.get_latest_episode_id()
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



    while robot.step(timestep) != -1:




        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False
            ideal_yaw = imu.getRollPitchYaw()[2]

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

        # Initialize values
        desired_state = [0, 0, 0, 0]
        # forward_desired = 0
        # sideways_desired = 0
        yaw_desired = ideal_yaw - yaw
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
            if pause:
                pause=False
                print('Starting movement')
                #start going with
                v=[1, 0]
                calculated_v=[0, 0]#first is wrong (to avoid NullPointer)
            else:
                print(f'Estimated velocity: {calculated_v},{yaw_desired}  real velocity: {[v_x, v_y, yaw]}, global velocity: [{v_x_global}, {v_y_global}]')
                print(f'dSpeed: [{v[0]-v_x}, {v[1]-v_y}],[{yaw_desired}], [{calculated_v[0]-v_x_global}, {calculated_v[1]-v_y_global}])]')
                row, column = find_grid_coordinates(x_global, y_global)
                gridBorders=grid_borders(row, column)
                fa=Fa(directions=new_info, v_x=v_x_global, v_y=v_y_global, x=x_global, y=y_global, gridBorders=gridBorders)

                dv=calc_dv(fa)

                calculated_v=V(v_x= v_x_global,v_y= v_y_global, dv=dv)

                v=globalSpeedToLocal(calculated_v[0], calculated_v[1], yaw)
                # height_desired=altitude
                print(f'new velocity: {v}')
        elif pause:
            v=[0,0]


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
        db.log_drone_observed_state(timestep=t, drone_id=id, roll=roll, pitch=pitch, yaw=yaw, yaw_rate=yaw_rate,
                                    x_global=x_global, y_global=y_global, v_x_global=v_x_global, v_y_global=v_y_global,
                                    altitude=altitude, v_x=v_x, v_y=v_y, dt=dt)

        db.log_drone_desired_state(timestep=t, drone_id=id, desired_vx=v[0], desired_vy=v[1],
                                   desired_yaw_rate=yaw_desired, desired_altitude=height_desired)
        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, v[0], v[1],
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)
        # motor1_speed = Column(Float, nullable=False)
        # motor2_speed = Column(Float, nullable=False)
        # motor3_speed = Column(Float, nullable=False)
        # motor4_speed = Column(Float, nullable=False)
        db.log_drone_motor_power(timestep=t, drone_id=id,
                                 motor1_speed=motor_power[0], motor2_speed=motor_power[1],
                                 motor3_speed=motor_power[2], motor4_speed=motor_power[3],)
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global

        t=t+1
        # print(f'GPS position: {[x_global, y_global]}')
db.close()
#encrypting_key === private
# signature

#decrypting_key  === public
#decrypt(signature, public) = original