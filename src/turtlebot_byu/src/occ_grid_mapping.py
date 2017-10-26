#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
import stat_filter as st
import sensor_models as sm

def to_coords(occ_grid,i):
    x = i%occ_grid.info.width + 0.5*occ_grid.info.resolution
    y = i/occ_grid.info.width + 0.5*occ_grid.info.resolution

    return [x, y]

def occupancy_grid_mapping(occ_grid, x, z, theta_k, true_pos, true_neg):

    for i in range(len(occ_grid.data)):
        mx = to_coords(occ_grid,i)
        l_last = st.log_odds(occ_grid.data[i]/100.0)
        l_next = l_last + sm.inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg)
        occ_grid.data[i] = st.log_odds_to_prob(l_next)*100.0


    return occ_grid


