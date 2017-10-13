#!/usr/bin/env python
import math
import numpy as np

def noise(covariance):
    return math.sqrt(covariance)*np.random.randn()

def prob(a, variance):
    return 1.0/math.sqrt(2.0*np.pi*variance)*np.exp(-0.5*a*a/variance)

def low_variance_sampler(chi, weights):
    chi_bar = []
    M = len(chi)
    r = np.random.rand()*(1.0/M)
    c = weights[0]
    i = 0
    print "max: ", np.amax(weights)

    for m in range(0, M):
        U = r + (m)*1.0/M
        while U > c:
            i = i + 1
            c = c + weights[i]
        chi_bar.append(chi[i])
    return chi_bar

def uniform_rand(lb,ub):
    return np.random.rand()*(ub - lb) + lb

def generate_particles_uniform_dist(lb, ub, num_particles):
    particles = np.zeros((num_particles,len(lb),1))

    for i in range(0,num_particles):
        for j in range(0, len(lb)):
            particles[i][j] = uniform_rand(lb[j],ub[j])

    return particles


