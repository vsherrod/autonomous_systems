#!/usr/bin/env python
import math
import numpy as np
import stat_filter as st
import dynamics
import pdb

def potato(phi, theta_k):
    k_val = np.absolute(phi-theta_k)

    # k_val = []
    # for i in range(len(z)):
    #     k_val.append(math.fabs(phi-theta_k[i]))

    k = np.argmin(k_val)

    return k

def cow(phi, theta_k):
    delta_ang = theta_k[1] - theta_k[0]

    idx = int((phi - theta_k[0])/delta_ang)

    # if np.absolute(phi - theta_k[idx]) < np.absolute(phi - theta_k[idx+1]):
    #     k = idx
    # else:
    #     k = idx+1

    return idx

def part_1(z, z_max):
    if np.isnan(z):
        z = z_max + 1.0

    return z

def inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg, alpha = 1.0, beta = 5.0*np.pi/180.0, z_max = 150):

    r = math.sqrt((mx[0] - x[0])*(mx[0] - x[0]) + (mx[1] - x[1])*(mx[1] - x[1]))
    phi = dynamics.wrap_angle(np.arctan2(mx[1]-x[1],mx[0]-x[0]) - x[2])

    # k_val = np.absolute(phi-theta_k)

    # k_val = []
    # for i in range(len(z)):
    #     k_val.append(math.fabs(phi-theta_k[i]))

    # k = np.argmin(k_val)

    k = potato(phi, theta_k)
    # k = cow(phi, theta_k)

    #maybe shouldn't be doing this here.  Could be preprocessed
    # if np.isnan(z[k][0]):
    #     z[k][0] = z_max + 1.0

    z[k][0] = part_1(z[k][0], z_max)


    if r > min(z_max, z[k][0] + alpha/2.0) or theta_k[k] > beta/2.0:
        print "1st"
        return st.log_odds(0.5)

    if z[k][0] < z_max and math.fabs(r - z[k][0]) < alpha/2.0:
        print "2nd"
        return st.log_odds(true_pos)

    if r <= z[k][0]:
        print "3rd"
        return st.log_odds(true_neg)


if __name__ == '__main__':
    #timing test
    z = np.array([[1.31072283],
           [1.32035029],
           [1.33296597],
           [1.34925175],
           [1.36942065],
           [1.3936621],
           [1.42212439],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan],
           [np.nan]])

    theta_k = np.array([[-0.52156788],
       [-0.47246721],
       [-0.42336655],
       [-0.37426588],
       [-0.32516521],
       [-0.27606455],
       [-0.22696388],
       [-0.17786322],
       [-0.12876255],
       [-0.07966188],
       [-0.03056122],
       [ 0.01853945],
       [ 0.06764012],
       [ 0.11674078],
       [ 0.16584145],
       [ 0.21494212],
       [ 0.26404278],
       [ 0.31314345],
       [ 0.36224412],
       [ 0.41134478],
       [ 0.46044545],
       [ 0.50954612],
       [ 0.52427632]])

    x = [0.001050991113748398, 0.012164368674377481, -0.03563179461331168]

    mx = [3.3499999999999996, -3.55]

    true_pos = 0.7
    true_neg = 0.4
    alpha = 0.2
    z_max = 10.0
    beta = 0.04908738521234052

    for i in range(60000):
        result = inverse_range_sensor_model(mx, x, z, theta_k, true_pos, true_neg, alpha, beta, z_max)

    print result


    # time
    # for i in range(0,60000):

