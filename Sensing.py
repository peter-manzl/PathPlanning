"""

autor:  Peter Manzl
date:   23.04.2024

description:    
        Based on the provided environment for path planning, here sensing of the 
        environment is implemented. 
"""

import numpy as np

import sys
sys.path.append('Sampling_based_Planning')

from rrt_2D import env

class Robot():
    def __init__(self, x0, y0, sensorLidar = {'dphi': 5, 'range': 10}, env = None): 
         self.x0 = x0
         self.y0 = y0
         
         # time series for plotting path
         self.x = [x0] 
         self.y =  [y0]
         self.sensorLidar = sensorLidar
         
         if env is None: 
             print('Warning: robot initialized without environment!')
         else: 
            self.env = env
            
    def checkRectangles(self, phi, length): 
        for x0, y0, dx, dy in self.env.obs_boundary + self.env.obs_rectangle:
            # works equal to rectangles! 
            print(x0, y0, dx, dy)
            pass
            
        return 
    
    def checkCircles(self, phi, length): 
        return
            # obs_circle
 
            
        
    def sense(self): 
         for phi in np.linspace(self.sensorLidar['dphi'], 360, int(360/self.sensorLidar['dphi'])):
            # print('phi: ', phi)
            # check if one of the following obstacles is seen by the sensor
            # self.checkBoundary(phi, self.sensorLidar['range'])
            self.checkRectangles(phi, self.sensorLidar['range'])
            self.checkCircles(phi, self.sensorLidar['range'])
            
    
        
            

myEnv = env.Env(envType = 1)
myRobot = Robot(2, 2.5, env=myEnv)

myEnv.plot(myRobot)
myRobot.sense()

