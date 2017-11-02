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


def ekf_slam_turtlebot(mu, sigma, v_c, w_c, ranges, bearings, correspondence, dt, Q, alpha):

        num_states = len(mu)

        theta = mu[2][0]

        F = np.zeros((3,num_states))

        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1

        mu[0:3] = dynamics.propogate_next_state(mu[0:3], v_c, w_c, dt)

        G = np.array([[0.0, 0.0, -v_c/w_c*np.cos(theta) + v_c/w_c*np.cos(theta + w_c*dt)],
                     [0.0, 0.0, -v_c/w_c*np.sin(theta) + v_c/w_c*np.sin(theta + w_c*dt)],
                     [0.0, 0.0, 0.0]])

        G = np.eye(num_states) + np.dot(F.T,np.dot(G,F))

        V = np.array([[(-np.sin(theta) + np.sin(theta + w_c*dt))/w_c, (v_c*(np.sin(theta) - np.sin(theta + w_c*dt)))/(w_c*w_c) + (v_c*np.cos(theta + w_c*dt)*dt)/w_c],
                     [(np.cos(theta) - np.cos(theta + w_c*dt))/w_c, (-v_c*(np.cos(theta) - np.cos(theta + w_c*dt)))/(w_c*w_c) + (v_c*np.sin(theta + w_c*dt)*dt)/w_c],
                     [0.0, dt]])

        M = np.array([[alpha[0]*v_c*v_c + alpha[1]*w_c*w_c, 0.0],
                     [0.0, alpha[2]*v_c*v_c + alpha[3]*w_c*w_c]])

        sigma_bar = np.dot(np.dot(G,sigma),G.T) + np.dot(F.T,np.dot(np.dot(np.dot(V, M),V.T),F))

        # K_plot = np.full((len(landmark_pts), 1),0.0)


        for i in range(0, len(ranges)):

            j = correspondence[i]

            if ekf_slam_turtlebot.seen.count(j) < 1:

                ekf_slam_turtlebot.seen.append(j)

                mu_j = mu[0:2] + np.array([ranges[i]*np.cos(bearings[i] + mu[2][0]),
                                           ranges[i]*np.sin(bearings[i] + mu[2][0])])

                mu = np.append(mu,mu_j,axis=0)

                sigma_bar = np.append(sigma_bar,np.zeros((2,len(sigma_bar))),axis=0)
                sigma_bar = np.append(sigma_bar,np.zeros((len(sigma_bar),2)),axis=1)
                sigma_bar[len(sigma_bar)-1][len(sigma_bar)-1] = 10
                sigma_bar[len(sigma_bar)-2][len(sigma_bar)-2] = 10

            idx = ekf_slam_turtlebot.seen.index(j)

            delta = mu[3+2*idx:3+2*(idx+1)] - mu[0:2]

            q = np.dot(delta.T,delta)[0][0]

            z_hat = np.array([[math.sqrt(q)],[np.arctan2(delta[1],delta[0]) - mu[2][0]]])

            Fx_j = np.zeros((6,len(mu)))

            Fx_j[0][0] = 1
            Fx_j[1][1] = 1
            Fx_j[2][2] = 1
            Fx_j[3][3+2*idx] = 1
            Fx_j[4][4+2*idx] = 1

            H = [[-1.0*math.sqrt(q)*delta[0][0], -1.0*math.sqrt(q)*delta[1][0], 0.0, math.sqrt(q)*delta[0][0], math.sqrt(q)*delta[1][0], 0.0],
                          [delta[1][0], -delta[0][0], -q, -delta[1][0], delta[0][0],0]]

            H = 1/q*np.dot(H,Fx_j)

            S = np.dot(np.dot(H, sigma_bar),H.T) + Q

            K = np.dot(np.dot(sigma_bar,H.T),np.linalg.inv(S))

            z = np.array([[ranges[i]], [bearings[i]]])

            z_int = z - z_hat

            z_int[1] = dynamics.wrap_angle(z_int[1])

            mu = mu + np.dot(K,(z_int))

            sigma_bar = np.dot((np.eye(len(mu)) - np.dot(K,H)), sigma_bar)

            # K_plot[j][0] = np.linalg.norm(K)

        sigma = sigma_bar

        return mu, sigma #, K_plot

ekf_slam_turtlebot.seen=[]
