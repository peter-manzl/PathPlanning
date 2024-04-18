# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 13:05:56 2024

@author: Peter Manzl
original code from Daniels Kraus: 
    https://github.com/DanielsKraus/SLAM-python/

"""

# 03 #Robot moving and sensiong

import numpy as np
import matplotlib.pyplot as plt
# import random
# import helper function
from helpers import display_world

from robot_class import robot

#%% Create the world

import numpy as np
from helpers import make_data, get_poses_landmarks, print_all
plt.close('all')
# your implementation of slam should work with the following inputs
# feel free to change these input values and see how it responds!

# world parameters
num_landmarks      = 5        # number of landmarks
N                  = 20       # time steps
world_size         = 100.0    # size of world (square)

# robot parameters
measurement_range  = 50.0     # range at which we can sense landmarks
motion_noise       = 2.0      # noise in robot motion
measurement_noise  = 2.0      # noise in the measurements
distance           = 20.0     # distance by which robot (intends to) move each iteratation 


# make_data instantiates a robot, AND generates random landmarks for a given world size and number of landmarks
data = make_data(N, num_landmarks, world_size, measurement_range, motion_noise, measurement_noise, distance)

#%% 
# print out some stats about the data
time_step = 0

print('Example measurements: \n', data[time_step][0])
print('\n')
print('Example motion: \n', data[time_step][1])

#%% 
def initialize_constraints(N, num_landmarks, world_size):
    ''' This function takes in a number of time steps N, number of landmarks, and a world_size,
        and returns initialized constraint matrices, omega and xi.'''
    
    ## Recommended: Define and store the size (rows/cols) of the constraint matrix in a variable
    size = (N + num_landmarks)*2
    rows = size
    cols = size

    ## TODO: Define the constraint matrix, Omega, with two initial "strength" values
    ## for the initial x, y location of our robot
    omega = np.zeros((rows,cols))
    
    strength = 1
    
    mid = world_size / 2
    initial_x = mid
    initial_y = mid
    
    omega[0,0] = strength
    omega[1,1] = strength
    ## TODO: Define the constraint *vector*, xi
    ## you can assume that the robot starts out in the middle of the world with 100% confidence
    xi = np.zeros((rows, 1))
    xi[0] = initial_x
    xi[1] = initial_y
    return omega, xi


#
#%% 
# import data viz resources
import matplotlib.pyplot as plt
from pandas import DataFrame
import seaborn as sns

# define a small N and world_size (small for ease of visualization)
N_test = 5
num_landmarks_test = 2
small_world = 10

# initialize the constraints
initial_omega, initial_xi = initialize_constraints(N_test, num_landmarks_test, small_world)
# define figure size
plt.rcParams["figure.figsize"] = (10,7)

# display omega
sns.heatmap(DataFrame(initial_omega), cmap='Blues', annot=True, linewidths=.5)

# define  figure size
plt.rcParams["figure.figsize"] = (1,7)

# display xi
sns.heatmap(DataFrame(initial_xi), cmap='Oranges', annot=True, linewidths=.5)

#%% 
## TODO: Complete the code to implement SLAM

## slam takes in 6 arguments and returns mu, 
## mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    ## TODO: Use your initilization to create constraint matrices, omega and xi
    omega, xi = initialize_constraints(N, num_landmarks, world_size)
    
    ## TODO: Iterate through each time step in the data
    ## get all the motion and measurement data as you iterate
    for i in range(len(data)):
        measurements = data[i][0]
        motion = data[i][1]
        dx = motion[0]
    
        ## TODO: update the constraint matrix/vector to account for all *measurements*
        ## this should be a series of additions that take into account the measurement noise
        nweight = 1 / measurement_noise
        for m in measurements:
            
            landmark = m[0]
            x = m[1]
            y=  m[2]
            
            omega[2*i, 2*i] += nweight
            omega[2*i, 2*N + 2*landmark] += -nweight
            omega[2*N + 2*landmark, 2*i] += -nweight
            omega[2*N + 2*landmark, 2*N + 2*landmark] += nweight 

            xi[2*i, 0] += -x *nweight
            xi[2*N + 2*landmark, 0] += x *nweight
            
            omega[2*i + 1, 2*i + 1] += nweight
            omega[2*i + 1, 2*N + 2*landmark + 1] += -nweight
            omega[2*N + 2*landmark + 1, 2*i + 1] += -nweight
            omega[2*N + 2*landmark + 1, 2*N + 2*landmark + 1] += nweight
            xi[2*i + 1, 0] += -y *nweight
            xi[2*N + 2*landmark + 1, 0] += y *nweight  
            
     ## TODO: update the constraint matrix/vector to account for all *motion* and motion noise       
        dx = motion[0]             
        dy = motion[1]
        mweight = 1 / motion_noise
    
        omega[2*i, 2*i] +=  mweight
        omega[2*i, 2*i + 2] += - mweight
        omega[2*i + 2, 2*i] += - mweight
        omega[2*i + 2, 2*i + 2] += mweight
        
        xi[2*i, 0] += -dx  * mweight             
        xi[2*i + 2, 0] += dx * mweight   
        
  
        omega[2*i + 1, 2*i + 1] += mweight
        omega[2*i + 1, 2*i + 3] += - mweight
        omega[2*i + 3, 2*i + 1] += - mweight
        omega[2*i + 3, 2*i + 3] += mweight 
        
        xi[2*i + 1, 0] += -dy * mweight
        xi[2*i + 3, 0] += dy * mweight
        
    ## TODO: After iterating through all the data
    ## Compute the best estimate of poses and landmark positions
    ## using the formula, omega_inverse * Xi
    mu = np.linalg.inv(np.matrix(omega)) * xi
    return mu # return `mu`


        
# call your implementation of slam, passing in the necessary parameters
mu = slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise)

# print out the resulting landmarks and poses
if(mu is not None):
    # get the lists of poses and landmarks
    # and print them out
    poses, landmarks = get_poses_landmarks(mu, N, num_landmarks)
    print_all(poses, landmarks)
    


# Display the final world!

# define figure size
plt.rcParams["figure.figsize"] = (20,20)
plt.figure()
# check if poses has been created
if 'poses' in locals():
    # print out the last pose
    print('Last pose: ', poses[-1])
    # display the last position of the robot *and* the landmark positions
    display_world(int(world_size), poses[-1], landmarks)
    