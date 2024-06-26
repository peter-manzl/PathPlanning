"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self, envType=0):
        self.envType = envType
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        
        self.obs = self.obs_map()
        

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        
        x = self.x_range
        y = self.y_range
        obs = set()
        # add obstacles point-wise at (x,y)
        # this is the border of the area: 
        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))
        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))
            
        if self.envType == 0: 
            # empty environment without additional obstacles
            pass
                
            
        if self.envType == 1: 
            for i in range(10, 21):
                obs.add((i, 15))
            for i in range(15):
                obs.add((20, i))
    
            for i in range(15, 30):
                obs.add((30, i))
            for i in range(16):
                obs.add((40, i))
                
        if self.envType == 2: 
            for j, gap in zip([10, 20, 30, 40], [25,10,27,4]):
                for i in range(1, gap-1): 
                    obs.add((j, i))
                for i in range(gap, 30): 
                    obs.add((j, i))
            
        return obs
