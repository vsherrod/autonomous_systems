#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter
import landmark
import copy
from pdb import set_trace as pause
# import occupancy_grid_mapping as ogc
import occ_grid_mapping as ogc
import fast_slam_particle

def occupancy_grid_fast_slam(particles, u, z, thk, motion_noise, dt, true_pos, true_neg, alpha, beta, z_max, first_time):

    v_c = u[0]
    w_c = u[1]
    v_c_cov = motion_noise[0]*v_c*v_c + motion_noise[1]*w_c*w_c
    w_c_cov = motion_noise[2]*v_c*v_c + motion_noise[3]*w_c*w_c

    for i in range(len(particles)):
         #sample the motion model
        x = np.array([[particles[i].x], [particles[i].y], [particles[i].theta]])
        x_next = dynamics.propogate_next_state(x, v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        particles[i].x = copy.deepcopy(x_next[0][0])
        particles[i].y = copy.deepcopy(x_next[1][0])
        particles[i].theta = copy.deepcopy(dynamics.wrap_angle(x_next[2][0]))

        #calculate weights from measurement model
        if first_time:
            particles[i].weight = 1.0/len(particles)
        else:
            particles[i].weight = ogc.likelihood_field_range_finder_model(z, x_next, thk, z_max, particles[i].occ_grid)

        #update the map for the particle
        X = [particles[i].x, particles[i].y, particles[i].theta]
        particles[i].occ_grid = ogc.occupancy_grid_mapping(particles[i].occ_grid, X, z, thk, true_pos, true_neg, alpha, beta, z_max)


    particles = fast_slam_particle.normalize_weights(particles)
    new_particles = stat_filter.low_variance_sampler(particles)

    return new_particles

def calc_H(landmark, particle):
    delta = np.array([[landmark.x],[landmark.y]]) - np.array([[particle.x],[particle.y]])
    q = np.dot(delta.T,delta)[0][0]
    H = np.array([[math.sqrt(q)*delta[0][0], math.sqrt(q)*delta[1][0]],
                        [-delta[1][0], delta[0][0]]])
    H = 1/q*H

    return H, delta, q

def calc_z_hat_and_H(landmark, particle):

    H, delta, q = calc_H(landmark, particle)

    z_hat = np.array([[math.sqrt(q)],[np.arctan2(delta[1],delta[0]) - particle.theta]])


    return z_hat, H

def fast_slam_known_correspondence_turtlebot(v_c, w_c, ranges, bearings, correspondences, particles, dt, Q, alpha):

    #calculated a fit over experiments with values
    exponent = -0.0556*len(ranges) + 8.5556

    scale = 10**exponent

    counter = 0

    for i in range(len(particles)):
        v_c_cov = alpha[0]*v_c*v_c + alpha[1]*w_c*w_c
        w_c_cov = alpha[2]*v_c*v_c + alpha[3]*w_c*w_c
        x = [[particles[i].x], [particles[i].y], [particles[i].theta]]
        # print "x before: ", particles[i].x
        # print "y before: ", particles[i].y
        # print "theta before: ", particles[i].theta
        x_next = dynamics.propogate_next_state(x, v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        particles[i].x = copy.deepcopy(x_next[0][0])
        particles[i].y = copy.deepcopy(x_next[1][0])
        particles[i].theta = copy.deepcopy(dynamics.wrap_angle(x_next[2][0]))

        # print i
        # print "x_p0: ", particles[0].x
        # print "y_p0: ", particles[0].y
        # print "theta_p0: ", particles[0].theta
        # print "x: ", particles[i].x
        # print "y: ", particles[i].y
        # print "theta: ", particles[i].theta

        # pdb.set_trace()
        particles[i].weight = 1.0

        for k in range(len(ranges)):

            j = correspondences[k]

            if particles[i].seen.count(j) < 1:
                particles[i].seen.append(j)

                # print "particle len: ", len(particles)
                # print "range len: ", len(ranges)
                # counter += 1
                # print counter
                x = particles[i].x + ranges[k]*np.cos(bearings[k] + particles[i].theta)
                y = particles[i].y + ranges[k]*np.sin(bearings[k] + particles[i].theta)

                ldm = landmark.Landmark(j,x,y)
                particles[i].landmarks.append(ldm)

                H = calc_H(ldm, particles[i])[0]
                H_inv = np.linalg.inv(H)
                ldm.sigma = np.dot(np.dot(H_inv,Q),H_inv.T)
                # print "sigma: ", ldm.sigma
                # print "H: ", H
                ldm.weight = scale*1.0/len(particles)
                particles[i].weight *= ldm.weight
                # particles[i].weight = 1.0/len(particles)

            else:
                idx = particles[i].seen.index(j)

                ldm = particles[i].landmarks[idx]

                z_hat, H = calc_z_hat_and_H(ldm , particles[i])

                S = np.dot(np.dot(H,ldm.sigma),H.T) + Q
                K = np.dot(np.dot(ldm.sigma, H.T), np.linalg.inv(S))

                z = np.array([[ranges[k]], [bearings[k]]])

                res = np.array(z-z_hat)

                res[1][0] = dynamics.wrap_angle(res[1][0])

                mu = np.array([[ldm.x],[ldm.y]]) + np.dot(K, res)

                ldm.x = mu[0][0]
                ldm.y = mu[1][0]

                ldm.sigma = np.dot(np.eye(2)-np.dot(K,H),ldm.sigma)

                particles[i].landmarks[idx] = ldm

                # particles[i].weight = np.linalg.det(2*np.pi*S)*np.exp(-.5*np.dot(np.dot(res.T,np.linalg.inv(S)),res))[0][0]
                ldm.weight = scale*(np.linalg.det(2*np.pi*S)*np.exp(-.5*np.dot(np.dot(res.T,np.linalg.inv(S)),res))[0][0])
                particles[i].weight *= ldm.weight

                # print "Weights: ", particles[i].weight, " For: ", i

        print "Weights: ", particles[i].weight, " For: ", i

    particles = fast_slam_particle.normalize_weights(particles)
    new_particles = stat_filter.low_variance_sampler(particles)

    return new_particles






