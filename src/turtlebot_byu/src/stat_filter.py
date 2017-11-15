#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import copy

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
            # print i
            # print "m: ", m
            # print "U: ", U
            # print "length weights: ", len(weights)
            # print "length chi: ", len(chi)
            i = i + 1
            c = c + weights[i]
        chi_bar.append(chi[i])
        # print "length: ", len(chi_bar)

    return chi_bar

def low_variance_sampler(particles):
    new_particles = []
    M = len(particles)
    r = np.random.rand()*(1.0/M)
    c = particles[0].weight
    i = 0

    for m in range(0, M):
        U = r + (m)*1.0/M
        while U > c:
            # print i
            # print "m: ", m
            # print "U: ", U
            # print "length weights: ", len(weights)
            # print "length chi: ", len(chi)
            i = i + 1
            c = c + particles[i].weight
        print "copy ", i
        new_particles.append(copy.deepcopy(particles[i]))
        # print "length: ", len(chi_bar)

    return new_particles

def uniform_rand(lb,ub):
    return np.random.rand()*(ub - lb) + lb

def generate_particles_uniform_dist(lb, ub, num_particles):
    particles = np.zeros((num_particles,len(lb),1))

    for i in range(0,num_particles):
        for j in range(0, len(lb)):
            particles[i][j] = uniform_rand(lb[j],ub[j])

    return particles

def log_odds(prob):

    return np.log(prob/(1-prob))

def log_odds_to_prob(l_odds):

    return 1 - 1.0/(1+np.exp(l_odds))

def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip


