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
    
def circle_line_intersection_point(point1, point2, xCircle, yCircle, r, flagDebug=False):
    if flagDebug: 
        plt.plot([point1[0],point2[0]], [point1[1],point2[1]], 'x-')
        
        pass
    
    if point2[0] - point1[0] < 1e-10: 
        x = point2[0]
        y_sq = r**2 - (x - xCircle)**2
    else: 
        k = (point2[1] - point1[1]) / (point2[0] - point1[0])    
        b = point1[1] - k * point1[0]
    
        # Coefficients in the quadratic equation Ax^2 + Bx + C = 0
        A = 1 + k**2
        B = 2 * (k * b - k * yCircle - xCircle)
        C = xCircle**2 + b**2 - 2 * b * yCircle + yCircle**2 - r**2
        
    # Calculate the discriminant
    discriminant = B**2 - 4 * A * C
    
    if discriminant < 0:
        return None
    else:
        # Calculate x coordinates of the intersection points
        sqrt_discriminant = np.sqrt(discriminant)
        x1 = (-B + sqrt_discriminant) / (2 * A)
        x2 = (-B - sqrt_discriminant) / (2 * A)
        
        # Corresponding y coordinates
        y1 = k * x1 + b
        y2 = k * x2 + b
        # print(discriminant, A, B)
        if discriminant == 0:
            return [x1, y1]
        else:
            r1 = [x1 - point1[0],  y1 - point1[1]]
            r2 = [x2 - point1[0],  y2 - point1[1]]
            rPoints = [point2[0] - point1[0], point2[1] - point1[1]]
            # phi = np.arctan2(point2[1] - point1[1],point2[0] - point1[0])
            
            e = np.array([point2[0] - point1[0], point2[1] - point1[1]])
            e = e / np.linalg.norm(e)     # unity vector
            
            
            dir1 = (r1 / e)[0]
            dir2 = (r2 / e)[0]
            
            # check if intersection is not between but outside of points
            if np.linalg.norm(r1) > np.linalg.norm(rPoints): 
                dir1 = -1 
            if np.linalg.norm(r2) > np.linalg.norm(rPoints): 
                dir2 = -1  
                
            if dir1 > 0 and dir2 < 0:
                return [x1,y1]
            elif dir1 > 0 and dir2 > 0: 
                # both in direction of 2:
                if dir1 > dir2: # further away!
                    return [x2, y2]
                else:
                    return [x1,y1]
            elif dir1 < 0 and dir2 > 0: 
                return [x2,y2]
            else: 
                return None
                
    
class Robot():
    def __init__(self, x0, y0, sensorLidar = {'dphi': 10, 'range': 100}, env = None): 
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
            
    def checkSensing(self, phi, length, verbose=False): 
        pRobot = [self.x[-1], self.y[-1]]
        pMaxSensing = [self.x[-1] + (length * np.cos(phi)), self.y[-1] + length*np.sin(phi)]
        # point1 = pRobot
        # point2 = pMaxSensing
        
        
        distCurrent = np.inf
        xCurrent = None
        if phi == np.pi * 3/2: 
            flagDebug = True
        else: 
            flagDebug = False
            
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
            xC = circle_line_intersection_point(pRobot, pMaxSensing, x0, y0, r, flagDebug)
            
            if not(xC is None): 
                myDistance = np.linalg.norm(pRobot - np.array(xC))
                if myDistance < distCurrent: 
                    distCurrent = myDistance
                    xCurrent = xC
                    
        if verbose: 
            if not(xCurrent is None): 
                print('phi: ', round(phi*180/np.pi, 2), 'dist: ', distCurrent)
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
            self.checkSensing(phi, self.sensorLidar['range'], verbose=verbose)
            # self.checkCircles(phi, self.sensorLidar['range'])
            
    
        
            

myEnv = env.Env(envType = 1)
myRobot = Robot(10, 22.5, env=myEnv)

myEnv.plot(myRobot)
myRobot.sense(True)
sys.exit()

#%% 
# import math

def find_circle_line_intersection(h, k, r, m, b):
    # Coefficients in the quadratic equation Ax^2 + Bx + C = 0
    A = 1 + m**2
    B = 2 * (m * b - m * k - h)
    C = h**2 + b**2 - 2 * b * k + k**2 - r**2
    print(h,k,r,m,b)
    # Calculate the discriminant
    discriminant = B**2 - 4 * A * C
    
    if discriminant < 0:
        return None
    else:
        # Calculate x coordinates of the intersection points
        sqrt_discriminant = np.sqrt(discriminant)
        x1 = (-B + sqrt_discriminant) / (2 * A)
        x2 = (-B - sqrt_discriminant) / (2 * A)
        
        # Corresponding y coordinates
        y1 = m * x1 + b
        y2 = m * x2 + b
        
        if discriminant == 0:
            return [[x1, y1]]
        else:
            return [[x1, y1], [x2, y2]]
        



# Example usage
# h, k, r = 0, 0, 5  # Circle center at (0,0) and radius 5
# m, b = 1, 0       # Line y = xl0 = 10
# xC = find_circle_line_intersection(h, k, r, m, b)
phi = np.linspace(0, np.pi*2, 1001)
np.random.seed(42)
for i in range(10): 
    plt.figure()
    
    r = 5* (np.random.random()-0.5) * 2
    p1 = (np.random.random(2)-0.5) * 10 # [-2, -10]  
    p2 = (np.random.random(2)-0.5) * 10 #[10, 10]
    xCircleM, yCircleM = 0,0
    xC = find_circle_line_intersection_point(p1, p2, xCircleM, yCircleM, r)

    # plt.plot([h, h+l0*m], [k, k+l0*m])
    # plt.plot([h, h-l0*m], [k, k-l0*m])
    
    xCircle = r* np.cos(phi) + xCircleM
    yCircle = r* np.sin(phi) + yCircleM
    plt.plot(xCircle, yCircle, '-')
    plt.axis('equal')
    # for data in xC: 
        # plt.plot(data[0], data[1], 'x')
    if not(xC is None): 
        # for data in xC: 
        plt.plot(xC[0], xC[1], 'o')
        plt.title('contact at: {}, {}'.format(round(xC[0], 3), round(xC[1], 3)))
    else: 
        plt.title('no contact')
    plt.plot([p1[0],p2[0]], [p1[1],p2[1]], 'x-')
    plt.text(p1[0]+0.1, p1[1], 'p1')
    plt.grid()
    # print("sC == xC2: ", xC == xC2)
    
    