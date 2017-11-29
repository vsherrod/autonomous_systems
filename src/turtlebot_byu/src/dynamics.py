#!/usr/bin/env python
import math
import numpy as np
import stat_filter
from pdb import set_trace as pause

def propogate_next_state(x, v, w, dt):
    # pause()
    delta = np.zeros((3,1))
    if np.abs(w)<1e-6:
        delta[0]=v*np.cos(x[2])*dt
        delta[1]=v*np.sin(x[2])*dt
        delta[2]=0.0
    else:
        delta[0]=-v/w*np.sin(x[2]) + v/w*np.sin(x[2] + w*dt)
        delta[1]=v/w*np.cos(x[2]) - v/w*np.cos(x[2] + w*dt)
        delta[2]=w*dt

    return x + delta

def norm(x):
    return math.sqrt(x[0]*x[0] + x[1]*x[1])

def landmark_measurement_model(f, x, landmark_pt, sigma_r, sigma_phi):
    r = norm([landmark_pt[0]-x[0],landmark_pt[1]-x[1]])
    phi = np.arctan2(landmark_pt[1]-x[1],landmark_pt[0]-x[0]) - x[2]
    q = stat_filter.prob(f[0] - r, sigma_r)*stat_filter.prob(f[1] - phi, sigma_phi)
    return q

def wrap_angle(angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle <= -np.pi:
        angle += 2*np.pi

    return angle

