# -*- coding: utf-8 -*-
"""
Created on Wed Dec  7 17:24:50 2022

@author: emzhang
"""

"""
University of Michigan
ENG100-400
"""
import random
import sys, platform
from pathlib import Path
if platform.system() == 'Darwin':
    airsim_install = '$HOME/AirSim'
else:
    airsim_install = 'C:\\AirSim'
sys.path.append(str(Path(airsim_install) / 'PythonClient'))
sys.path.append(str(Path(airsim_install) / 'PythonClient' / 'multirotor'))

############### import a few useful libraries ###########

import setup_path
import time
import numpy as np
import math
import matplotlib.pyplot as plt

############### establish the link to AirSim ###########

import airsim              # import AirSim API
import E100_functions      # import drone simulator library

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

alpha_alt = 1
alpha_lidar = 1.0

target_alt = 10
K_P = 2.4
K_I = 0.2
K_D = 1.5
alt_integration_term = 0
alt_flag = 1
throttle = 0.5  # initialize Throttle
##############################################
target_front_dist = 15
pitch_rate = 0
pitch_K_P = 2
pitch_K_I = 0.55
pitch_K_D = 1.5
pitch_integration_term = 0
pitch_flag = 1
desired_pitch = 0
##############################################
target_right_dist = 5
roll_K_P = 2
roll_K_I = 0.55
roll_K_D = 1.5
roll_integration_term = 0
roll_flag = 1
desired_roll = 0 
desired_yaw = 0

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
alt_log = []

while True:
    #choose a random wind speed between -7 and 7
    x_wind = random.randint(-7, 7)   
    y_wind = random.randint(-7, 7) 
    z_wind = random.randint(-7, 7) 
   
    
    
    now = time.time()
    if now - start >5:#hover for 5 seconds
        E100_functions.set_wind(client, x_wind, y_wind, z_wind) #set a wind with random speeds in x y and z direction
    if now - start > 42: #set target altitude to 0 after 42
        target_alt = 0
    if now - start > 45:#end script after 45 seconds
        break
    
    
    
    #control signal 
    E100_functions.set_quadcopter(client,desired_roll,-1*desired_pitch,desired_yaw,throttle)
    altitude = E100_functions.get_altitude(client) # read quadcopter's altitude 
    
    x,y = E100_functions.get_XY(client)
    #left right is x
    #forward back is y
    x_pos.append(x)
    y_pos.append(y)
    alt_log.append(altitude)
    plt.plot(x_pos)
   
    #################### get sensor readings #####################
    
               
   
    
    
   
    
    
    
    
    
    if alt_flag == 1:
        error_old = target_alt-altitude
        alt_flag = 0
    else:
        error_old = error    
    error= target_alt-altitude
    alt_integration_term += error*dt
    alt_differential_term = (error - error_old)/dt  
    throttle = K_P*error + K_I*alt_integration_term + K_D*alt_differential_term
    if throttle < 0:
        throttle = 0
    elif throttle > 1:
        throttle = 1       
    thorttle = throttle/2 + 0.55/2
    
    ##############################################################
    #front dist hold part(PID controller+activation function)####
    ##############################################################     
    if pitch_flag == 1:
        x_fill,y_initial = E100_functions.get_XY(client)
        pitch_error_old = y_initial-y
        pitch_flag = 0
    else:
        pitch_error_old = pitch_error    
    pitch_error= y_initial-y
    pitch_integration_term += pitch_error*dt
    pitch_differential_term = (pitch_error - pitch_error_old)/dt
    desired_pitch = (pitch_K_P*pitch_error + pitch_K_I*pitch_integration_term + pitch_K_D*pitch_differential_term)
    desired_pitch = math.degrees(0.15*np.tanh(desired_pitch))  # use tanh function to limit the maximum and minimum of the pitch value
    
    ##############################################################
    #center dist hold part(PID controller+activation function)####
    ##############################################################    
    if roll_flag == 1:
        x_initial,y_fill = E100_functions.get_XY(client)
        roll_error_old = x_initial-x
        roll_flag = 0
    else:
        roll_error_old = roll_error    
    roll_error=  x_initial-x
    roll_integration_term += roll_error*dt
    roll_differential_term = (roll_error - roll_error_old)/dt    
    desired_roll = roll_K_P*roll_error + roll_K_I*roll_integration_term + roll_K_D*roll_differential_term
    desired_roll = math.degrees(0.2*np.tanh(0.5*desired_roll))
    desired_yaw = 0
    
    

   


#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)






















