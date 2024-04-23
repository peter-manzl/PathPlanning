"""
Environment for rrt_2D
@author: huiming zhou
edited by: Peter Manzl
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Env:
    def __init__(self, envType = 1):
        print(envType)
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        if envType == 0: 
            self.obs_circle = [[1,28,0]]    
            self.obs_rectangle = [[45,1,0,0]]

        if envType == 1: 
            self.obs_circle = self.obs_circle()
            self.obs_rectangle = self.obs_rectangle()
        
        if envType == 2: 
            self.obs_circle = [[1,28,0]]    
            self.obs_rectangle = []
            for i, gap in zip([10,20,30,40], [25, 10, 25, 5]): 
                
                self.obs_rectangle += [[i,1,1,gap-2]]
                self.obs_rectangle += [[i,gap+2,1,self.y_range[1] - gap-2]]
            
    def plot(self, robot = None): 
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_boundary:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )
        if not(self.obs_rectangle is None): 
            for (ox, oy, w, h) in self.obs_rectangle:
                ax.add_patch(
                    patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
        if not(self.obs_circle is None): 
            for (ox, oy, r) in self.obs_circle:
                ax.add_patch(
                    patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
        if not(robot is None): 
            plt.plot(robot.x[-1], robot.y[-1], "bs", linewidth=3)
        # plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)

        # plt.title('')
        plt.axis("equal")
            
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            # x0, y0, dx, dy
            [0, 0, 1, 30],
            [0, 30,50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            # x0, y0, r
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir
    
if __name__ == '__main__': 
    env0 = Env(0)
    env1 = Env(1)
    env2 = Env(2)
    
    for env in [env0, env1, env2]: 
        env.plot()
    
