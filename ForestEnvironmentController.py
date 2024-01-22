# -*- coding: utf-8 -*-
"""
Created on Tue Nov 29 12:31:58 2022

@author: emzhang
"""

"""
University of Michigan
ENG100-400
"""

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

# Modify the following PID parameters to observe the altitude control performance
K_P = 2.4
K_I = 0.2
K_D = 1.5
#############

target_alt = 8
integration_term = 0
alt_flag = 1
throttle = 0.5  # initialize Throttle
alt_log = []
wind_flag = 1
step = 0
speed = 8
start = time.time()
while True:
    now = time.time()
    if now-start>150 :
        break
           
#break
   
    #################### get sensor readings #####################      
    altitude = E100_functions.get_altitude(client) # read quadcopter's altitude 
    
    alt_log.append(altitude)  # record the altitude reading
    plt.plot(alt_log)  # plot the altitude versus time
    
    ##############################################################
    #Altitude hold (PID controller)##########################
    ##############################################################
    if alt_flag == 1:  # When the program runs the first time, the error is still undefined.
        error_old = target_alt-altitude
        alt_flag = 0
    else:
        error_old = error    # Move e(t) to e(t-dt) 
    error= target_alt-altitude  # Determine the current error e(t)
    integration_term += error*dt  # This is a shorthand for integration_term = integration_term + error*dt
    differential_term = (error - error_old)/dt    
    if K_P*error + K_I*integration_term + K_D*differential_term <0:
        throttle = 0  # Make sure we don't send a negative throttle signal to the drone.
    else:
        throttle = K_P*error + K_I*integration_term + K_D*differential_term
    if throttle >= 1:
        throttle = 1
    throttle = throttle/2 + 0.55/2
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings
    
    ####### change the engine throttle ###################################
    if (step == 0): #go forward
        E100_functions.set_quadcopter(client,0,-0.9*speed,0,throttle)
        if((front<20) and (front>0)):
            step = 1 
    
    if (step == 1): #go slow down going forward
        E100_functions.set_quadcopter(client,0,4.7*speed,0,throttle)
        if ((E100_functions.get_linear_velocity(client)[1]<0.3*speed/2) and (right>left)):
            step = 2 #go right
        elif ((E100_functions.get_linear_velocity(client)[1]<0.3*speed/2) and (left>right)):
            step = 20 #go left
        elif ((E100_functions.get_linear_velocity(client)[1]<0.3*speed/2) and (left==right)):
            step = 2 #go right
            
            
            
    if (step == 2): #go right
        E100_functions.set_quadcopter(client,1.5*speed,0,0,throttle)
        if ((right<20) or (front>25)) or( math.isnan(front)==1):#if space in front is open or too close to the right, go forward
            step = 3
    if (step == 20): #go left
        E100_functions.set_quadcopter(client,-1.5*speed,0,0,throttle)
        if ((left<20) or (front>25)) or( math.isnan(front)==1): #if space in front is open or too close to the left, go forward
            step = 30
            
    if (step == 3): #slow down going right
        E100_functions.set_quadcopter(client,-4*speed,0,0,throttle)
        if (E100_functions.get_linear_velocity(client)[0]<0.6*speed/2):
            step = 0 #go forward
            
    if (step == 30): #slow down going left
        E100_functions.set_quadcopter(client,4*speed,0,0,throttle)
        if (E100_functions.get_linear_velocity(client)[0]>-0.6*speed/2):
            step = 0 #go forward
        
      
        
    
    
    if (step == 10):
        print('Done in')
        print((time.time()-start))
        print('seconds')
        break
    
#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)

