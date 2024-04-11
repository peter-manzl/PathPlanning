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

sys.path.append('Sampling_based_Planning')
from rrt_2D import batch_informed_trees, dubins_rrt_star, dynamic_rrt, extended_rrt
from rrt_2D import fast_marching_trees, informed_rrt_star
from rrt_2D import rrt, rrt_connect, rrt_star, rrt_star_smart

if __name__ == "__main__": 
    envType = 0
    s_start = (5,5)
    s_end = (45, 25)
    
    algorithm_list = [batch_informed_trees, dubins_rrt_star, dynamic_rrt, extended_rrt , 
                      fast_marching_trees, informed_rrt_star, 
                      rrt, rrt_connect, rrt_star, rrt_star_smart]

    
    # sys.exit()
    # create new windows for all algorithms
    for lib in algorithm_list: 
        plt.figure()
        print('running: ', lib.__name__)
        lib.main(envType, s_start, s_end)

