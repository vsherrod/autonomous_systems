#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter

def mcl_turtlebot(chi, v_c, w_c, ranges, bearings, landmark_pts, dt, alpha, sigma_r, sigma_phi):
    num_points = len(chi)
    chi_bar = []
    weights = []
    for i in range(0, num_points):
        v_c_cov = alpha[0]*v_c*v_c + alpha[1]*w_c*w_c
        w_c_cov = alpha[2]*v_c*v_c + alpha[3]*w_c*w_c
        cow = dynamics.propogate_next_state(chi[i], v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        chi_bar.append(cow)

        print ("Monkey chi_bar: ", chi_bar)
        print ("Cow: ", cow)
        weight = 1
        for j in range(0, len(landmark_pts)):
            f = [ranges[j], bearings[j]]
            weight = weight * dynamics.landmark_measurement_model(f, chi_bar[i], landmark_pts[j], sigma_r, sigma_phi)

        weights.append(weight)

    for i in range(0, num_points):
        chi = stat_filter.low_variance_sampler(chi_bar, weights)

    return chi

