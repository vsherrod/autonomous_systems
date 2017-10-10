#!/usr/bin/env python
import math
import numpy as np
import dynamics
import pdb

def fill_sigma_aug(sigma_aug, sigma, M, Q):

    start_idx = len(sigma)

    for i in range(0,start_idx):
        sigma_aug[i][i] = sigma[i][i]


    for i in range(0,len(M)):
        sigma_aug[i+start_idx][i+start_idx] = M[i][i]

    start_idx = start_idx + len(M)

    for i in range(0,len(Q)):
        sigma_aug[i+start_idx][i+start_idx] = Q[i][i]

    return sigma_aug

def generate_sigma_points_and_weights(mu_aug, sigma_aug):

    n = len(mu_aug)

    sigma_points = np.zeros((n, (2*n+1)))
    wm = np.zeros((2*n+1))
    wc = np.zeros((2*n+1))

    sigma_points[:,[0]] = mu_aug

    alpha = 0.1

    kappa = 10.0

    beta = 2.0

    Lambda = alpha*alpha*(n + kappa) - n

    gamma = math.sqrt(n + Lambda)

    wm[0] = Lambda/(n + Lambda)
    wc[0] = wm[0] + (1 - alpha*alpha + beta)

    mega_matrix = gamma*np.transpose(np.linalg.cholesky(sigma_aug))

    for i in range(1,2*n+1):
        if i <= n:
            sigma_points[:,[i]] = mu_aug + mega_matrix[:,[i-1]]
        else:
            sigma_points[:,[i]] = mu_aug - mega_matrix[:,[i-1-n]]

        wm[i] = 1/(2.0*(n + Lambda))
        wc[i] = wm[i]

    return sigma_points, wm, wc


def ukf_turtlebot(mu, sigma, v_c, w_c, range_ldm, bearing_ldm, landmark_pts, dt, Q, alpha):

        M = np.array([[alpha[0]*v_c*v_c + alpha[1]*w_c*w_c, 0.0],
                     [0.0, alpha[2]*v_c*v_c + alpha[3]*w_c*w_c]])

        mu_aug = np.zeros((7,1))
        sigma_aug = np.zeros((7,7))

        for i in range(0,len(mu)):
            mu_aug[i][0] = mu[i][0]

        sigma_aug = fill_sigma_aug(sigma_aug, sigma, M, Q)

        sigma_points, wm, wc = generate_sigma_points_and_weights(mu_aug, sigma_aug)

        n = len(mu_aug)

        x_sigma_points = np.zeros((len(mu), (2*n+1)))
        Z_bar = np.zeros((len(Q),(2*n+1)))

        #Pass sigma points through motion model and predict observations
        for i in range(0,len(sigma_points[0])):
            x_sigma_points[:,[i]] = dynamics.propogate_next_state(sigma_points[0:3,[i]], v_c+sigma_points[3][i], w_c+sigma_points[4][i], dt)
            q = dynamics.norm([landmark_pts[0] - x_sigma_points[0][i], landmark_pts[1] - x_sigma_points[1][i]]) #this is not the same q as in the book (it is sqrt(q))
            Z_bar[:,[i]] = np.array([[q],[np.arctan2(landmark_pts[1]-x_sigma_points[1][i],landmark_pts[0]-x_sigma_points[0][i]) - x_sigma_points[2][i]]]) + sigma_points[5:7,[i]]

        #compute Gaussian statistics
        mu_bar = np.zeros((len(mu), 1))
        sigma_bar = np.zeros((len(mu), len(mu)))
        z_hat = np.zeros((len(Q),1))
        S = np.zeros((len(Q),len(Q)))
        sigma_xy = np.zeros((len(mu),len(Q)))

        for i in range(0,2*n+1):
            mu_bar = mu_bar + wm[i]*x_sigma_points[:,[i]]
        for i in range(0,2*n+1):
            sigma_bar = sigma_bar + wc[i]*np.dot((x_sigma_points[:,[i]] - mu_bar),np.transpose((x_sigma_points[:,[i]] - mu_bar)))
        for i in range(0,2*n+1):
            z_hat = z_hat + wm[i]*Z_bar[:,[i]]
        for i in range(0,2*n+1):
            S = S + wc[i]*np.dot((Z_bar[:,[i]] - z_hat),np.transpose((Z_bar[:,[i]] - z_hat)))
            sigma_xy = sigma_xy + wc[i]*np.dot((x_sigma_points[:,[i]] - mu_bar),np.transpose((Z_bar[:,[i]] - z_hat)))

        #update mean and covariance
        K_plot = np.zeros((1, 1))
        K = np.dot(sigma_xy, np.linalg.inv(S))

        z = np.array([[range_ldm], [bearing_ldm]])

        mu = mu_bar + np.dot(K, (z - z_hat))

        sigma = sigma_bar - np.dot(np.dot(K,S),np.transpose(K))

        K_plot[0][0] = np.linalg.norm(K)

        return mu, sigma, K_plot
