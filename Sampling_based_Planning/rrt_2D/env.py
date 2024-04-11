"""
Environment for rrt_2D
@author: huiming zhou
"""


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
                pass
            
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
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
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir
