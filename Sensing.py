"""

autor:  Peter Manzl
date:   23.04.2024

description:    
        Based on the provided environment for path planning, here sensing of the 
        environment is implemented. 
"""

import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('Sampling_based_Planning')
from rrt_2D import env

def line_intersection(p1,p2, p3, p4):
    # Unpack points
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    # Calculate differences
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3

    # Calculate the determinant of the coefficient matrix
    det = dx1 * dy2 - dy1 * dx2
    if det == 0:
        return None  # Lines are parallel or collinear

    # Calculate the determinant for t and u
    det_t = (x3 - x1) * dy2 + (y1 - y3) * dx2
    det_u = (x3 - x1) * dy1 + (y1 - y3) * dx1

    # Calculate the parameters t and u
    t = det_t / det
    u = det_u / det
    
    # Check if the intersection point is on both line segments
    if 0 <= t <= 1 and 0 <= u <= 1: 
        # xC = [x1 + t*dx1, y1 + t*dy1]
        # print(u)
        return (x1 + t * dx1, y1 + t * dy1)


    else: 
        return None
    
    
class Robot():
    def __init__(self, x0, y0, sensorLidar = {'dphi': 10, 'range': 10}, env = None): 
         self.x0 = x0
         self.y0 = y0
         
         # time series for plotting path
         self.x = [x0] 
         self.y =  [y0]
         self.sensorLidar = sensorLidar
         self.alphaPlotLidar = 0.2
         if env is None: 
             print('Warning: robot initialized without environment!')
         else: 
            self.env = env
            
    def checkRectangles(self, phi, length, verbose=False): 
        pRobot = [self.x[-1], self.y[-1]]
        pMaxSensing = [self.x[-1] + (length * np.cos(phi)), self.y[-1] + length*np.sin(phi)]
        # point1 = pRobot
        # point2 = pMaxSensing
        
        
        distCurrent = np.inf
        xCurrent = None
        for x0, y0, dx, dy in self.env.obs_boundary + self.env.obs_rectangle:
            for point1, point2 in [([x0, y0], [x0+dx, y0]), 
                                    ([x0+dx, y0],  [x0+dx, y0+dy]),
                                    ([x0, y0], [x0, y0+dy]),
                                    ([x0, y0+dy], [x0+dx, y0+dy])]: 
                
                xC = line_intersection(pRobot, pMaxSensing, point1, point2)
                if not(xC is None): 
                    myDistance = np.linalg.norm(pRobot - np.array(xC))
                    if myDistance < distCurrent: 
                        distCurrent = myDistance
                        xCurrent = xC
        for x0, y0, r in self.env.obs_circle: 
            pass
        
        if verbose: 
            if not(xCurrent is None): 
                print('dist: ', distCurrent)
                plt.plot(xCurrent[0], xCurrent[1], 'bo')
                plt.plot([pRobot[0], xCurrent[0]], [pRobot[1], xCurrent[1]], 'r--', alpha = self.alphaPlotLidar)
            else: 
                plt.plot([pRobot[0], pMaxSensing[0]], [pRobot[1], pMaxSensing[1]], 'r--', alpha =  self.alphaPlotLidar)
            pass
            
        return 

        
    def sense(self, verbose=False): 
         for phi in np.pi/ 180 * np.linspace(self.sensorLidar['dphi'], 360, int(360/self.sensorLidar['dphi'])):
            # print('phi: ', phi)
            # check if one of the following obstacles is seen by the sensor
            # self.checkBoundary(phi, self.sensorLidar['range'])
            self.checkRectangles(phi, self.sensorLidar['range'], verbose=verbose)
            self.checkCircles(phi, self.sensorLidar['range'])
            
    
        
            

myEnv = env.Env(envType = 1)
myRobot = Robot(2, 2.5, env=myEnv)

myEnv.plot(myRobot)
myRobot.sense(True)
sys.exit()

#%% 

x0 = 5
y0 = 5
dy = 2
dx = 20
# plt.plot([x0, x0 + dx], [y0, y0], label='1')
# plt.plot([x0+dx, x0 + dx],  [y0, y0+dy], label='2')
# plt.plot([x0, x0], [y0, y0+dy], label='3')
# plt.plot([x0, x0+dx], [y0+dy, y0+dy], label='4')
plt.legend()



    

x0_, y0_, dx_, dy_ = 10,2,5,5
point1 = (x0, y0)
point2 = (x0+dx, y0+dy)
point3 = (x0_, y0_)
point4 = (x0_+dx_, y0+y0_)
xC, u = line_intersection(point1, point2, point3, point4)
plt.plot([point1[0], point2[0]], [point1[1], point2[1]])
plt.plot([point3[0], point4[0]], [point3[1], point4[1]], '--')
plt.plot(xC[0], xC[1], 'x')

# np.array(point3) - xC

#%% 
point1 = (1, 1)
point2 = (4, 4)
point3 = (1, 8)
point4 = (2, 0)
intersection_point = line_intersection(point1, point2, point3, point4)
if intersection_point:
    plt.plot(intersection_point[0], intersection_point[1], 'x')
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]])
    plt.plot([point3[0], point4[0]], [point3[1], point4[1]], '--')
    