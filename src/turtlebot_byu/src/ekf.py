#!/usr/bin/env python
import math
import numpy as np
import sys
import dynamics

def ekf_turtlebot(mu, sigma, v_c, w_c, ranges, bearings, landmark_pts, dt, Q, alpha):

        theta = mu[2][0]

        G = np.array([[1.0, 0.0, -v_c/w_c*np.cos(theta) + v_c/w_c*np.cos(theta + w_c*dt)],
                     [0.0, 1.0, -v_c/w_c*np.sin(theta) + v_c/w_c*np.sin(theta + w_c*dt)],
                     [0.0, 0.0, 1.0]])

        V = np.array([[(-np.sin(theta) + np.sin(theta + w_c*dt))/w_c, (v_c*(np.sin(theta) - np.sin(theta + w_c*dt)))/(w_c*w_c) + (v_c*np.cos(theta + w_c*dt)*dt)/w_c],
                     [(np.cos(theta) - np.cos(theta + w_c*dt))/w_c, (-v_c*(np.cos(theta) - np.cos(theta + w_c*dt)))/(w_c*w_c) + (v_c*np.sin(theta + w_c*dt)*dt)/w_c],
                     [0.0, dt]])

        M = np.array([[alpha[0]*v_c*v_c + alpha[1]*w_c*w_c, 0.0],
                     [0.0, alpha[2]*v_c*v_c + alpha[3]*w_c*w_c]])

        mu = dynamics.propogate_next_state(mu, v_c, w_c, dt)

        sigma_bar = np.dot(np.dot(G,sigma),np.transpose(G)) + np.dot(np.dot(V, M),np.transpose(V))

        K_plot = np.full((len(landmark_pts), 1),0.0)


        for j in range(0, len(landmark_pts)):
            q = dynamics.norm([landmark_pts[j][0] - mu[0], landmark_pts[j][1] - mu[1]]) #this is not the same q as in the book (it is sqrt(q))

            z_hat = np.array([[q],[np.arctan2(landmark_pts[j][1]-mu[1],landmark_pts[j][0]-mu[0]) - mu[2]]])

            H = np.array([[-(landmark_pts[j][0] - mu[0][0])/q, -(landmark_pts[j][1] - mu[1][0])/q, 0.0],
                          [(landmark_pts[j][1] - mu[1][0])/(q*q), -(landmark_pts[j][0] - mu[0][0])/(q*q), -1.0]])

            S = np.dot(np.dot(H, sigma_bar),np.transpose(H)) + Q

            K = np.dot(np.dot(sigma_bar,np.transpose(H)),np.linalg.inv(S))

            z = np.array([[ranges[j]], [bearings[j]]])

            mu = mu + np.dot(K,(z - z_hat))

            sigma_bar = np.dot((np.eye(3) - np.dot(K,H)), sigma_bar)

            K_plot[j][0] = np.linalg.norm(K)

        sigma = sigma_bar

        return mu, sigma, K_plot
