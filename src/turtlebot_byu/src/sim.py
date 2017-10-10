#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
import pdb
import ekf
import ukf
import dynamics

# landmark_pts = [[6,4],[-7,8],[6,-4]]
# landmark_pts = [[-7,4],[-7,0],[-7,-4]]
landmark_pts = [[6,4]]

def noise(covariance):
    return math.sqrt(covariance)*np.random.randn()

def plot_point(x,y):
    plt.scatter(x,y,plt.rcParams['lines.markersize'] ** 2,'b','x')

def plot_landmarks():
    for i in range(0,len(landmark_pts)):
        plot_point(landmark_pts[i][0],landmark_pts[i][1])

def simulate_sensor_data(x, sigma_r, sigma_phi):
    ranges = []
    bearings = []
    for i in range(0,len(landmark_pts)):
        ranges.append(dynamics.norm([landmark_pts[i][0]-x[0],landmark_pts[i][1]-x[1]]) + noise(sigma_r*sigma_r))
        bearing_map = np.arctan2(landmark_pts[i][1]-x[1],landmark_pts[i][0]-x[0])
        bearings.append(bearing_map-x[2] + noise(sigma_phi*sigma_phi))

    return ranges, bearings

def plot_bearing_measurements(x_last, ranges, bearings):
    for j in range(0,len(landmark_pts)):
        x_land = ranges[j]*np.cos(bearings[j]+x_last[2]) + x_last[0]
        y_land = ranges[j]*np.sin(bearings[j]+x_last[2]) + x_last[1]
        plt.plot([x_last[0], x_land], [x_last[1], y_land], 'b-')

def plot_iteration(x_plot, y_plot, x_last, x_plot_est, y_plot_est, ranges, bearings):
    plt.clf()
    plt.axis([-10, 10, -10, 10])

    plot_landmarks()
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


if __name__=='__main__':

    x0 = np.array([[-5.0],[-3.0],[np.pi/2.0]])

    dt = 0.1
    t_final = 20.0
    n_intervals = t_final/dt + 1
    t = np.linspace(0.0, 20.0, n_intervals)

    r = 1

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

    sigma = np.eye(3)

    sigma_x = [2.0]
    sigma_y = [2.0]
    sigma_theta = [2.0]

    for i in range(0,len(v_c)):
        v.append(v_c[i] + noise(alpha1*v_c[i]*v_c[i] + alpha2*w_c[i]*w_c[i]))
        w.append(w_c[i] + noise(alpha3*v_c[i]*v_c[i] + alpha4*w_c[i]*w_c[i]))

    x = []
    mu = []

    x_plot = []
    y_plot = []
    theta_plot = []

    x_plot_est = []
    y_plot_est = []
    theta_plot_est = []

    x.append(x0)
    mu.append(x0)

    x_plot.append(x[0][0])
    y_plot.append(x[0][1])
    theta_plot.append(x[0][2])

    x_plot_est.append(mu[0][0])
    y_plot_est.append(mu[0][1])
    theta_plot_est.append(mu[0][2])

    x_err = []
    y_err = []
    theta_err = []

    x_err.append(mu[0][0]-x[0][0])
    y_err.append(mu[0][1]-x[0][1])
    theta_err.append(mu[0][2]-x[0][2])

    K_plot = [[],[],[]]
    t_plot = [[],[],[]]
    print t_plot

    plt.ion()

    for i in range(0, len(v_c)-1):
        x_next = dynamics.propogate_next_state(x[i], v[i], w[i], dt)
        x.append(x_next)
        x_plot.append(x[i+1][0])
        y_plot.append(x[i+1][1])
        theta_plot.append(x[i+1][2])

        ranges, bearings = simulate_sensor_data(x_next, sigma_r, sigma_phi)

        # mu_next, sigma, K_out =  ekf.ekf_turtlebot(mu[i], sigma, v_c[i], w_c[i], ranges, bearings, landmark_pts, dt, Q, alpha)
        landmark_idx = i%len(landmark_pts)
        mu_next, sigma, K_out =  ukf.ukf_turtlebot(mu[i], sigma, v_c[i], w_c[i], ranges[landmark_idx], bearings[landmark_idx], landmark_pts[landmark_idx], dt, Q, alpha)

        mu.append(mu_next)

        K_plot[landmark_idx].append(K_out[0][0])

        t_plot[landmark_idx].append(t[i])
        # K_plot[1][i] = K_out[1][0]
        # K_plot[2][i] = K_out[2][0]

        sigma_x.append(2*math.sqrt(sigma[0][0]))
        sigma_y.append(2*math.sqrt(sigma[1][1]))
        sigma_theta.append(math.sqrt(2*sigma[2][2]))

        x_plot_est.append(mu[i+1][0][0])
        y_plot_est.append(mu[i+1][1][0])
        theta_plot_est.append(mu[i+1][2][0])

        x_err.append(mu[i+1][0][0] - x[i+1][0])
        y_err.append(mu[i+1][1][0] - x[i+1][1])
        theta_err.append(mu[i+1][2][0] - x[i+1][2])

        plot_iteration(x_plot, y_plot, x[i+1], x_plot_est, y_plot_est, ranges, bearings)


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
    ax7 = fig5.add_subplot(311)
    plot_data(ax7, [t_plot[0]], [K_plot[0]], ['Norm of Kalman Gains Landmark 1'], "Norm of Landmark 1 Kalman Gains vs Time", "Time (s)", "Gain")

    if len(landmark_pts)>1:
        ax8 = fig5.add_subplot(312)
        plot_data(ax8, [t_plot[1]], [K_plot[1]], ['Norm of Kalman Gains Landmark 2'], "Norm of Landmark 2 Kalman Gains vs Time", "Time (s)", "Gain")

    if len(landmark_pts)>2:
        ax9 = fig5.add_subplot(313)
        plot_data(ax9, [t_plot[2]], [K_plot[2]], ['Norm of Kalman Gains Landmark 3'], "Norm of Landmark 3 Kalman Gains vs Time", "Time (s)", "Gain")

    raw_input("Press enter....")



