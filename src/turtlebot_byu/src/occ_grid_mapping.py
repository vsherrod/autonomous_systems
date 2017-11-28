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
import math

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

def to_index(occ_grid,x,y):
    delta_x = round((x - occ_grid.info.origin.position.x)/occ_grid.info.resolution -0.5)#*occ_grid.info.resolution
    delta_y = round((y - occ_grid.info.origin.position.y)/occ_grid.info.resolution -0.5)#*occ_grid.info.resolution
    i = int(occ_grid.info.width*delta_y + delta_x)
    return i

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

def is_occupied(occ_grid,idx):
    if idx > len(occ_grid.data)-1 or idx < 0:
        return False
    else:
        return occ_grid.data[idx] > 50

def find_dist_to_nearest_neighbor(occ_grid, x_start, y_start):
    nearest_neigbor = find_nearest_neighbor(occ_grid, x_start, y_start)

    dist = math.sqrt((y_start - nearest_neigbor[1])**2 + (x_start - nearest_neigbor[0])**2)

    return dist


def find_nearest_neighbor(occ_grid, x_start, y_start):
    #this finds the nearest neigbor by checking in rings around the starting cell.  It returns the first one it finds in a ring that is occupied, so it is not guaranteed to give the shortest distance by the 2-norm, but rather by the infinity norm
    start_idx = to_index(occ_grid, x_start, y_start)

    if is_occupied(occ_grid,start_idx):
        return to_coords(occ_grid, start_idx)

    search_dist = 1
    search_max = max([occ_grid.info.width, occ_grid.info.height])

    while search_dist < search_max:
        cur_idx = start_idx - search_dist
        if is_occupied(occ_grid, cur_idx):
            return to_coords(occ_grid, cur_idx)

        cur_idx = start_idx + search_dist
        if is_occupied(occ_grid, cur_idx):
            return to_coords(occ_grid, cur_idx)

        for i in range(2*search_dist-2):
            rows_shifted = (-1**i)*(i/2+1)
            #maybe should keep track of which one is the min distance from here
            cur_idx = start_idx+rows_shifted*occ_grid.info.width - search_dist
            if is_occupied(occ_grid, cur_idx):
                return to_coords(occ_grid, cur_idx)

            cur_idx = start_idx+rows_shifted*occ_grid.info.width + search_dist
            if is_occupied(occ_grid, cur_idx):
                return to_coords(occ_grid, cur_idx)


        for i in range(-search_dist, search_dist+1):
            cur_idx = start_idx+search_dist*occ_grid.info.width + i
            if is_occupied(occ_grid, cur_idx):
                return to_coords(occ_grid, cur_idx)

            cur_idx = start_idx-search_dist*occ_grid.info.width + i
            if is_occupied(occ_grid, cur_idx):
                return to_coords(occ_grid, cur_idx)

        search_dist += 1


if __name__ == '__main__':
    #occupancy grid parameters
    res = 0.1
    width = 100
    height = 100
    origin_x = -width/2.0*res
    origin_y = -height/2.0*res
    occ_grid = initialize_occ_grid("world", res, width, height,Pose(Point(origin_x,origin_y,0.0),Quaternion(0.0,0.0,0.0,1.0)))

    idx = 5
    occ_grid.data[idx] = 100
    occ_grid.data[7] = 100




    nearest_neigbor = find_nearest_neighbor(occ_grid, -4.0, -4.7)
    print "Nearest: ", nearest_neigbor
    print "Actual: ", to_coords(occ_grid, idx)
    print "Dist: ", find_dist_to_nearest_neighbor(occ_grid, -4.0, -4.7)




