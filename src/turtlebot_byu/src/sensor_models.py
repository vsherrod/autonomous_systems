#!/usr/bin/env python
import math
import numpy as np
import stat_filter as st
import dynamics 
import pdb

def inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg, alpha = 1.0, beta = 5.0*np.pi/180.0, z_max = 150):

    r = math.sqrt((mx[0] - x[0])*(mx[0] - x[0]) + (mx[1] - x[1])*(mx[1] - x[1]))
    phi = dynamics.wrap_angle(np.arctan2(mx[1]-x[1],mx[0]-x[0]) - x[2])

    k_val = []
    for i in range(len(z)):
        k_val.append(math.fabs(phi-theta_k[i]))
        
    min_k_val = min(k_val)
        
    k = k_val.index(min_k_val)

    if np.isnan(z[k][0]):
        z[k][0] = z_max + 1.0


    if r > min(z_max, z[k][0] + alpha/2.0) or min_k_val > beta/2.0:
        return st.log_odds(0.5)

    if z[k][0] < z_max and math.fabs(r - z[k][0]) < alpha/2.0:
        return st.log_odds(true_pos)

    if r <= z[k][0]:
        return st.log_odds(true_neg)



