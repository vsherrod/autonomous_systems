#!/usr/bin/env python
import math
import numpy as np
import dynamics
import stat_filter
import landmark

def calc_H(landmark, particle):
        delta = np.array([[landmark.x],[landmark.y]]) - np.array([[particle.x],[particle.y]])
        q = np.dot(delta.T,delta)[0][0]
        H = [[math.sqrt(q)*delta[0][0], math.sqrt(q)*delta[1][0]],
                          [-delta[1][0], delta[0][0]]]
        H = 1/q*H

        return H




def fast_slam_known_correspondence_turtlebot(v_c, w_c, ranges, bearing, correspondence, particles, dt, Q, alpha):

    for i in range(len(particles)):
        v_c_cov = alpha[0]*v_c*v_c + alpha[1]*w_c*w_c
        w_c_cov = alpha[2]*v_c*v_c + alpha[3]*w_c*w_c
        x = [particles[i].x, particles[i].y, particles[i].theta]
        x_next = dynamics.propogate_next_state(x, v_c + stat_filter.noise(v_c_cov), w_c + stat_filter.noise(w_c_cov), dt)
        particles[i].x = x_next[0]
        particles[i].y = x_next[1]
        particles[i].theta = x_next[2]

        j = correspondence

        if particles[i].seen.count(j) < 1:
            particles[i].seen.append(j)

            x = particles[i].x + ranges*np.cos(bearing + particles[i].theta)
            y = particles[i].y + ranges*np.sin(bearing + particles[i].theta)

            ldm = landmark.Landmark(j,x,y)
            particles[i].landmarks.append(ldm)

            H = calc_H(ldm, particles[i])
            H_inv = numpy.linalg.inv(H)
            ldm.sigma = np.dot(np.dot(H_inv,Q),H_inv.T)
            particles[i].weight = .5






