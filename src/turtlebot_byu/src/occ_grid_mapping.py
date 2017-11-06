#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
import tf.transformations as tft
import stat_filter as st
import sensor_models as sm
from pdb import set_trace as brake
from copy import copy
import numpy as np

def to_coords(occ_grid,i):
    x = (i%occ_grid.info.width + 0.5)*occ_grid.info.resolution + occ_grid.info.origin.position.x
    y = (i/occ_grid.info.width + 0.5)*occ_grid.info.resolution + occ_grid.info.origin.position.y
    return [x, y]

def occupancy_grid_mapping(occ_grid, x, z, theta_k, true_pos, true_neg, rot, trans):
    
    R_bot_world = tft.euler_matrix(rot[0],rot[1],rot[2])
    R_bot_world[0:3,3] = [trans[0],trans[1],trans[2]]
    for i in range(len(occ_grid.data)):
        mx = to_coords(occ_grid,i)
        temp = copy(mx)
        temp.append(0)
        temp.append(1)
        p_trans = np.dot(R_bot_world.tolist(),temp)
        p_x = p_trans[0]
        p_y = p_trans[1]

        if (np.linalg.norm(np.array(mx)-np.array([p_x,p_y])) < 10) and (p_x > 0) and (p_y < p_x and p_y > -p_x):
            l_last = st.log_odds(occ_grid.data[i]/100.0)
            l_next = l_last + sm.inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg)
            occ_grid.data[i] = st.log_odds_to_prob(l_next)*100.0


    return occ_grid


