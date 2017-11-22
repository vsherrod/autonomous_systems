#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
import tf.transformations as tft
import stat_filter as st
import sensor_models as sm
from pdb import set_trace as brake
from copy import copy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

def initialize_occ_grid(frame_id = "odom", res = 1.0, width = 100, height = 100, origin = Pose(Point(0.0,0.0,0.0),Quaternion(0.0,0.0,0.0,1.0))):
    occ_grid = OG()
    occ_grid.header.frame_id = frame_id
    occ_grid.info.resolution = res
    occ_grid.info.width = width
    occ_grid.info.height = height
    occ_grid.info.origin = origin
    occ_grid.data = occ_grid.info.width*occ_grid.info.height*[50]

    return occ_grid

def to_coords(occ_grid,i):
    x = (i%occ_grid.info.width + 0.5)*occ_grid.info.resolution + occ_grid.info.origin.position.x
    y = (i/occ_grid.info.width + 0.5)*occ_grid.info.resolution + occ_grid.info.origin.position.y
    return [x, y]

def occupancy_grid_mapping(occ_grid, x, z, theta_k, true_pos, true_neg, alpha = 1.0, beta = 5.0*np.pi/180.0, z_max = 150):
    # rotation of bot frame relative to the world frame.
    R_bot_world = np.array([[np.cos(x[2]),-np.sin(x[2])],[np.sin(x[2]),np.cos(x[2])]])
    # change it to the the world frame relative to the bot frame and add translation
    R_T = R_bot_world.T
    d = np.array([[x[0]],[x[1]]])
    T_world_bot = np.append(np.append(R_T,np.dot(R_T,d), axis=1),np.array([[0,0,1]]), axis=0)
    
    for i in range(len(occ_grid.data)):
        mx = to_coords(occ_grid,i)
        temp = copy(mx)
        temp.append(1)
        p_trans = np.dot(T_world_bot.tolist(),temp)
        p_x = p_trans[0]
        p_y = p_trans[1]

        if (np.linalg.norm(np.array(mx)-np.array([p_x,p_y])) < 10) and (p_x > 0) and (p_y < p_x and p_y > -p_x):
            l_last = st.log_odds(occ_grid.data[i]/100.0)
            l_next = l_last + sm.inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg, alpha, beta, z_max)
            occ_grid.data[i] = st.log_odds_to_prob(l_next)*100.0


    return occ_grid

def likelihood_field_range_finder_model(zt, xt, th_k, z_max, m):
    q = 1
    # xk_sens, yk_sens, and zk_sens are the offsets of the laser range finders from the base_link frame
    xk_sens = -0.102
    yk_sens = 0.00
    zk_sens = 0.272
    for k in xrange(len(zt)):
        if zt[k] < z_max: 
            x_zk =  xt[0] + xk_sens*np.cos(x[2])-yk_sens*np.sin(x[2])+zk_sens*np.cos(x[2]+th_k[k])
            x_zk =  xt[0] + yk_sens*np.cos(x[2])-xk_sens*np.sin(x[2])+zk_sens*np.cos(x[2]+th_k[k])
            
        # dist = 'this is the min distance function that vallan is coding up'

        q = q*(z_hit*stat_filter.prob(dist, sigma_hit)+z_rand/z_max)

    return q
