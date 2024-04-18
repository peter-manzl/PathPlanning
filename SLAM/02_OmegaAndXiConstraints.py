# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 13:05:56 2024

@author: Peter Manzl
original code from Daniels Kraus: 
    https://github.com/DanielsKraus/SLAM-python/

"""

# 02 # 

import numpy as np
import matplotlib.pyplot as plt
# import random
# import helper function
from helpers import display_world

from robot_class import robot

#%% 
# matrix: all robot poses (xi); landmarks: Li; 
# define omega and xi as in the example
omega = np.array([[1,0,0],
                  [-1,1,0],
                  [0,-1,1]])

xi = np.array([[-3],
               [5],
               [3]])

# calculate the inverse of omega
omega_inv = np.linalg.inv(np.matrix(omega))

# calculate the solution, mu
mu = omega_inv*xi

# print out the values of mu (x0, x1, x2)
print(mu)
