#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import fast_slam
import dynamics
import stat_filter
import fast_slam_particle
import pdb

# landmark_pts = [[6,4],[-7,8],[6,-4]]
landmark_pts = np.random.uniform(-10,10,(10,2))
# landmark_pts = [[-7,4],[-7,0],[-7,-4]]
# landmark_pts = [[6,4]]

def plot_point(x,y,color='b'):
    plt.scatter(x,y,plt.rcParams['lines.markersize'] ** 2,color,'x')

def plot_landmarks():
    for i in range(0,len(landmark_pts)):
        plot_point(landmark_pts[i][0],landmark_pts[i][1])

def plot_estimated_landmarks(particle):
    for i in range(len(particle.landmarks)):
        plot_point(particle.landmarks[i].x,particle.landmarks[i].y,'r')
        stat_filter.plot_cov_ellipse(particle.landmarks[i].sigma,[particle.landmarks[i].x,particle.landmarks[i].y],2,None, facecolor='none')


def simulate_sensor_data(x, sigma_r, sigma_phi, fov=360.0*np.pi/180.0):
    ranges = []
    bearings = []
    correspondence = []
    for i in range(0,len(landmark_pts)):
        bearing_map = np.arctan2(landmark_pts[i][1]-x[1][0],landmark_pts[i][0]-x[0][0])
        bearing = bearing_map-x[2][0] + stat_filter.noise(sigma_phi*sigma_phi)
        if bearing <= fov and bearing >= -fov:
            bearings.append(bearing)
            ranges.append(dynamics.norm([landmark_pts[i][0]-x[0],landmark_pts[i][1]-x[1]]) + stat_filter.noise(sigma_r*sigma_r))
            correspondence.append(i)

    return ranges, bearings, correspondence

def plot_bearing_measurements(x_last, ranges, bearings):
    for j in range(0,len(bearings)):
        x_land = ranges[j]*np.cos(bearings[j]+x_last[2]) + x_last[0]
        y_land = ranges[j]*np.sin(bearings[j]+x_last[2]) + x_last[1]
        plt.plot([x_last[0], x_land], [x_last[1], y_land], 'b-')

def plot_iteration(x_plot, y_plot, x_last, x_plot_est, y_plot_est, ranges, bearings, particle):
    plt.clf()
    plt.axis([-10, 10, -10, 10])

    plot_landmarks()
    plot_estimated_landmarks(particle)
    plt.scatter(x_plot, y_plot)
    plt.scatter(x_plot_est, y_plot_est, plt.rcParams['lines.markersize'] ** 2,'k','x')

    x_head = x_last[0] + r*np.cos(x_last[2])
    y_head = x_last[1] + r*np.sin(x_last[2])

    plt.plot([x_last[0], x_head], [x_last[1], y_head], 'k-')

    plot_bearing_measurements(x_last, ranges, bearings)

    plt.pause(0.05)

def plot_data(ax, x, y, labels, title, xlabel, ylabel):
    for i in range(0, len(x)):
        ax.plot(x[i], y[i], label = labels[i])
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()

def generate_particles(x0, num_particles):
    particles = []
    for i in range(num_particles):
        particle = fast_slam_particle.FastSlamParticleFeatures(x0[0][0], x0[1][0], x0[2][0])
        particles.append(particle)

    return particles

def find_highest_weighted_particle(particles):
    highest_weight = 0.0
    for i in range(len(particles)):
        if particles[i].weight > highest_weight:
            idx = i
            highest_weight = particles[i].weight

    return particles[idx]

if __name__=='__main__':

    x0 = np.array([[-5.0],[-3.0],[np.pi/2.0]])
    num_particles = 100

    r = 1

    dt = 0.1
    t_final = 20.0
    n_intervals = t_final/dt + 1
    t = np.linspace(0.0, 20.0, n_intervals)

    v_c = 1 + 0.5*np.cos(2.0*np.pi*0.2*t)
    w_c = -0.2 + 2.0*np.cos(2.0*np.pi*0.6*t)

    alpha1 = 0.1
    alpha4 = 0.1
    alpha2 = 0.01
    alpha3 = 0.01

    alpha = [alpha1, alpha2, alpha3, alpha4]

    sigma_r = 0.1
    sigma_phi = 0.05
    sigma_s = 0.0

    Q = np.array([[sigma_r*sigma_r , 0.0],
                  [0.0, sigma_phi*sigma_phi]])

    v = []
    w = []

    sigma = np.zeros((3,3))

    sigma_x = [2.0]
    sigma_y = [2.0]
    sigma_theta = [2.0]

    for i in range(0,len(v_c)):
        v.append(v_c[i] + stat_filter.noise(alpha1*v_c[i]*v_c[i] + alpha2*w_c[i]*w_c[i]))
        w.append(w_c[i] + stat_filter.noise(alpha3*v_c[i]*v_c[i] + alpha4*w_c[i]*w_c[i]))

    x = []
    mu = []

    x.append(x0)
    mu.append(x0)

    x_plot = []
    y_plot = []
    theta_plot = []

    x_plot_est = []
    y_plot_est = []
    theta_plot_est = []

    particles = generate_particles(x0, num_particles)

    x_plot.append(x[0][0])
    y_plot.append(x[0][1])
    theta_plot.append(x[0][2])

    x_plot_est.append(x0[0][0])
    y_plot_est.append(x0[1][0])
    theta_plot_est.append(x0[2][0])

    x_err = []
    y_err = []
    theta_err = []

    x_err.append(0.0)
    y_err.append(0.0)
    theta_err.append(0.0)

    t_plot = [[],[],[]]

    plt.ion()

    for i in range(0, len(v_c)-1):
        x_next = dynamics.propogate_next_state(x[i], v[i], w[i], dt)
        x.append(x_next)
        x_plot.append(x[i+1][0])
        y_plot.append(x[i+1][1])
        theta_plot.append(x[i+1][2])

        ranges, bearings, correspondence = simulate_sensor_data(x_next, sigma_r, sigma_phi)

        ldm_idx = i%len(ranges)

        particles = fast_slam.fast_slam_known_correspondence_turtlebot(v_c[i], w_c[i], ranges[ldm_idx], bearings[ldm_idx], correspondence[ldm_idx], particles, dt, Q, alpha)
        # landmark_idx = i%len(landmark_pts)
        # mu_next, sigma, K_out =  ukf.ukf_turtlebot(mu[i], sigma, v_c[i], w_c[i], ranges[landmark_idx], bearings[landmark_idx], landmark_pts[landmark_idx], dt, Q, alpha)
        particle = find_highest_weighted_particle(particles)

        x_ext_next = np.array([[particle.x],[particle.y],[particle.theta]])
        mu.append(x_ext_next)

        # K_plot[landmark_idx].append(K_out[0][0])

        # t_plot[landmark_idx].append(t[i])
        # K_plot[1][i] = K_out[1][0]
        # K_plot[2][i] = K_out[2][0]

        # sigma_x.append(2*math.sqrt(sigma[0][0]))
        # sigma_y.append(2*math.sqrt(sigma[1][1]))
        # sigma_theta.append(math.sqrt(2*sigma[2][2]))

        x_plot_est.append(mu[i+1][0][0])
        y_plot_est.append(mu[i+1][1][0])
        theta_plot_est.append(mu[i+1][2][0])

        x_err.append(mu[i+1][0][0] - x[i+1][0])
        y_err.append(mu[i+1][1][0] - x[i+1][1])
        theta_err.append(mu[i+1][2][0] - x[i+1][2])

        plot_iteration(x_plot, y_plot,mu[i+1], x_plot_est, y_plot_est, ranges, bearings, particle)

        # pdb.set_trace()



    fig2 = plt.figure()
    ax1 = fig2.add_subplot(211)
    plot_data(ax1, [t, t], [x_plot, x_plot_est], ['Truth', 'Estimate'], "X Position vs Time", "Time (s)", "X Position (m)")

    ax2 = fig2.add_subplot(212)
    plot_data(ax2, [t, t, t], [x_err, sigma_x, -1*np.array(sigma_x)], ['Estimate Error in X', '95% Certainty', '95% Certainty'], "X Position Error vs Time", "Time (s)", "Error (m)")

    fig3 = plt.figure()
    ax3 = fig3.add_subplot(211)
    plot_data(ax3, [t, t], [y_plot, y_plot_est], ['Truth', 'Estimate'], "Y Position vs Time", "Time (s)", "Y Position (m)")

    ax4 = fig3.add_subplot(212)
    plot_data(ax4, [t, t, t], [y_err, sigma_y, -1*np.array(sigma_y)], ['Estimate Error in Y', '95% Certainty', '95% Certainty'], "Y Position Error vs Time", "Time (s)", "Error (m)")

    fig4 = plt.figure()
    ax5 = fig4.add_subplot(211)
    plot_data(ax5, [t, t], [theta_plot, theta_plot_est], ['Truth', 'Estimate'], "Heading vs Time", "Time (s)", "Heading (rad)")

    ax6 = fig4.add_subplot(212)
    plot_data(ax6, [t, t, t], [theta_err, sigma_theta, -1*np.array(sigma_theta)], ['Estimate Error in Heading', '95% Certainty', '95% Certainty'], "Heading Error vs Time", "Time (s)", "Error (rad)")

    fig5 = plt.figure()
    plt.imshow(sigma)
    plt.show()


    raw_input("Press enter....")



