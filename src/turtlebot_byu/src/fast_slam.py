#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter
import landmark
import copy
import pdb

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

def normalize_weights(particles):
    weight_sum = 0.0
    for i in range(len(particles)):
        weight_sum += particles[i].weight

    for i in range(len(particles)):
        particles[i].weight = particles[i].weight/weight_sum


    return particles




def fast_slam_known_correspondence_turtlebot(v_c, w_c, ranges, bearing, correspondence, particles, dt, Q, alpha):

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

        print i
        print "x_p0: ", particles[0].x
        print "y_p0: ", particles[0].y
        print "theta_p0: ", particles[0].theta
        print "x: ", particles[i].x
        print "y: ", particles[i].y
        print "theta: ", particles[i].theta

        # pdb.set_trace()

        j = correspondence

        if particles[i].seen.count(j) < 1:
            particles[i].seen.append(j)

            x = particles[i].x + ranges*np.cos(bearing + particles[i].theta)
            y = particles[i].y + ranges*np.sin(bearing + particles[i].theta)

            ldm = landmark.Landmark(j,x,y)
            particles[i].landmarks.append(ldm)

            H = calc_H(ldm, particles[i])[0]
            H_inv = np.linalg.inv(H)
            ldm.sigma = np.dot(np.dot(H_inv,Q),H_inv.T)
            particles[i].weight = 1.0/len(particles)

        else:
            idx = particles[i].seen.index(j)

            ldm = particles[i].landmarks[idx]

            z_hat, H = calc_z_hat_and_H(ldm , particles[i])

            Q = np.dot(np.dot(H,ldm.sigma),H.T) + Q
            K = np.dot(np.dot(ldm.sigma, H.T), np.linalg.inv(Q))

            z = np.array([[ranges], [bearing]])

            res = np.array(z-z_hat)

            res[1][0] = dynamics.wrap_angle(res[1][0])

            mu = np.array([[ldm.x],[ldm.y]]) + np.dot(K, res)

            ldm.x = mu[0][0]
            ldm.y = mu[1][0]

            ldm.sigma = np.dot(np.eye(2)-np.dot(K,H),ldm.sigma)

            particles[i].landmarks[idx] = ldm


            particles[i].weight = np.linalg.det(2*np.pi*Q)*np.exp(-.5*np.dot(np.dot(res.T,np.linalg.inv(Q)),res))[0][0]

    particles = normalize_weights(particles)
    new_particles = stat_filter.low_variance_sampler(particles)

    return new_particles






