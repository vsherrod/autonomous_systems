#!/usr/bin/env python
import math
import numpy as np

def propogate_next_state(x, v, w, dt):
    delta = np.zeros((3,1))
    delta[0]=-v/w*np.sin(x[2]) + v/w*np.sin(x[2] + w*dt)
    delta[1]=v/w*np.cos(x[2]) - v/w*np.cos(x[2] + w*dt)
    delta[2]=w*dt

    return x + delta

def norm(x):
    return math.sqrt(x[0]*x[0] + x[1]*x[1])
