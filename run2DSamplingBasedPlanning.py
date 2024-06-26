"""

autor:  Peter Manzl
date:   11.04.2024

description:    
        The functionality of the repository is forked from 
        github.com/zhm-real/PathPlanning
        and updated with few new functionalities, e.g. the possibility to 
        change the environment by setting environmenttypes and calling the 
        algorithms main functions with starting and end point. 
"""

import numpy as np
import sys
import matplotlib.pyplot as plt

sys.path.append('Search_based_Planning')
from Search_2D import Anytime_D_star, ARAstar, Astar, Best_First, bfs, Bidirectional_a_star
from Search_2D import dfs, Dijkstra, D_star, D_star_Lite, LPAstar, LRTAstar, RTAAStar

if __name__ == "__main__": 
    envType = 2
    s_start = (5,5)
    s_end = (45, 25)
    
    algorithm_list = [                      # Anytime_D_star, 
                      # ARAstar, 
                      Astar, 
                      # Best_First, 
                      # bfs, 
                      # Bidirectional_a_star, 
                      # dfs, 
                      # Dijkstra, 
                      # D_star, 
                      # D_star_Lite ,
                      # LPAstar, 
                      # LRTAstar,
                      # RTAAStar
                      ]

    # create new windows for all algorithms
    for lib in algorithm_list: 
        plt.figure()
        print('running: ', lib.__name__)
        lib.main(envType, s_start, s_end)
        
