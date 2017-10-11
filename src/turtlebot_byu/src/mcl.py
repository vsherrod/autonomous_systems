#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter
import pdb

def mcl_turtlebot(chi, v_c, w_c, ranges, bearings, landmark_pts, dt, alpha, sigma_r, sigma_phi):
    num_points = len(chi)
    num_states = len(chi[0])
    chi_bar = np.zeros((num_points,num_states,1))
    weights = []
    for i in range(0, num_points):
        v_c_cov = alpha[0]*v_c*v_c + alpha[1]*w_c*w_c
        w_c_cov = alpha[2]*v_c*v_c + alpha[3]*w_c*w_c
        x_next = dynamics.propogate_next_state(chi[i], v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        for j in range(0,num_states):
            chi_bar[i][j] = x_next[j]

        weight = 1
        for j in range(0, len(landmark_pts)):
            f = [ranges[j], bearings[j]]
            weight = weight * dynamics.landmark_measurement_model(f, chi_bar[i], landmark_pts[j], sigma_r, sigma_phi)

        weights.append(weight)

    weights = weights/np.sum(weights)

    chi = stat_filter.low_variance_sampler(chi_bar, weights)

    return chi

