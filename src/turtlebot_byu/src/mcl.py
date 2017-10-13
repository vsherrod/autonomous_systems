#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter
import pdb

def mcl_turtlebot(chi, v_c, w_c, ranges, bearings, landmark_pts, dt, alpha, sigma_r, sigma_phi, sigma):
    num_points = len(chi)
    num_states = len(chi[0])
    chi_bar = np.zeros((num_points,num_states,1))
    weights = []
    v_c_cov = alpha[0]*v_c*v_c + alpha[1]*w_c*w_c
    w_c_cov = alpha[2]*v_c*v_c + alpha[3]*w_c*w_c

    for i in range(0, num_points):
        x_next = dynamics.propogate_next_state(chi[i], v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        for j in range(0,num_states):
            chi_bar[i][j] = x_next[j]

        weight = 1
        for j in range(0, len(landmark_pts)):
            f = [ranges[j], bearings[j]]
            weight = weight * dynamics.landmark_measurement_model(f, chi_bar[i], landmark_pts[j], sigma_r, sigma_phi)

        weights.append(weight)

    # pdb.set_trace()
    print "Max before sum: ", np.amax(weights)
    weights = weights/np.sum(weights)

    x_particles = np.zeros(len(chi))
    y_particles = np.zeros(len(chi))
    theta_particles = np.zeros(len(chi))

    for j in range(0, len(chi)):
        x_particles[j] = chi_bar[j][0]
        y_particles[j] = chi_bar[j][1]
        theta_particles[j] = chi_bar[j][2]

    sigma[0][0] = np.std(x_particles)**2
    sigma[1][1] = np.std(y_particles)**2
    sigma[2][2] = np.std(theta_particles)**2

    mu_next = np.zeros((3,1))
    mu_next[0] = np.mean(x_particles)
    mu_next[1] = np.mean(y_particles)
    mu_next[2] = np.mean(theta_particles)

    chi = stat_filter.low_variance_sampler(chi_bar, weights)

    unique = len(np.unique(chi, axis=0))

    if float(unique)/num_points < 0.1:
        #add random noise so that it gets more unique points back.  There are definitely smarter ways to do this
        chi = chi + np.random.randn(num_points,num_states,1)*.001

    print "unique: ", unique

    # print "chi: ", chi

    return chi, sigma, mu_next

