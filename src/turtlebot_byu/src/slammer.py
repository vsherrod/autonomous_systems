#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
import fast_slam
import fast_slam_particle
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import tf.transformations as tft
from std_msgs.msg import String
import laser_scan_subscriber
import twist_subscriber
import numpy as np
import occ_grid_mapping as ogc
import os
import scipy.io as sio
import tf2_ros
import tf


def occ_grid_fast_slam(laser_sub, laser_res, velocity_sub):

    #initial position
    x0 = np.array([[0.0],[0.0],[0.0]])

    #true pos and true neg values
    true_pos = 0.7
    true_neg = 0.4

    #sensor model parameters
    alpha = 0.2
    beta = 60.0/(640.0/laser_res)*np.pi/180.0
    z_max = 10

    #occupancy grid parameters
    res = 0.1
    width = 100
    height = 100
    origin_x = -width/2.0*res
    origin_y = -height/2.0*res

    #motion model noise parameters
    alpha1 = 0.1
    alpha4 = 0.1
    alpha2 = 0.01
    alpha3 = 0.01

    alpha_vec = [alpha1, alpha2, alpha3, alpha4]

    #number of particles
    num_particles = 10


    #initialize ros node and publisher
    pub = rospy.Publisher('map', OG, queue_size=1)

    rate = rospy.Rate(20) #hz

    #initialize the occ_grid msg
    occ_grid = ogc.initialize_occ_grid("world", res, width, height,Pose(Point(origin_x,origin_y,0.0),Quaternion(0.0,0.0,0.0,1.0)))

    #generate particles
    particles = fast_slam_particle.generate_particles(x0, num_particles, occ_grid)

    #initialize the tf listener
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        # pass in the fraction of laser data you want to use
        timestamp, z, thk = laser_sub.getData()

        # grab the linear and angular velocity command data
        linear, angular = velocity_sub.getData()

        u = [linear.x, angular.z]

        fast_slam.occupancy_grid_fast_slam(particles, u, z, thk, alpha_vec, dt, true_pos, true_neg, rot2, trans2, alpha, beta, z_max):

        # if timestamp!=None:

        #     #tf listener to get transform from odom to baselink at timestamp
        #     listener.waitForTransform('world', 'base_link_truth', timestamp, rospy.Duration(0.05))
        #     (trans, rot) = listener.lookupTransform('world', 'base_link_truth', timestamp)

        #     # convert quaternion to euler angles
        #     rot = tft.euler_from_quaternion(rot)

        #     # get rotation about the z-axis
        #     theta = rot[2]

        #     # assign the states to the correct variable
        #     X = trans[:2]
        #     X.append(theta)

        #     occ_grid.header.stamp = timestamp

        #     trans = [trans[0],trans[1],trans[2]]

        #     (trans2, rot2) = listener.lookupTransform('base_link_truth', 'world', timestamp)

        #     rot2 = tft.euler_from_quaternion(rot2)

        #     # pass in rot so that the points can be expressed in the bot frame
        #     occ_grid = ogc.occupancy_grid_mapping(occ_grid, X, z, thk, true_pos, true_neg, rot2, trans2, alpha, beta, z_max)

        #     pub.publish(occ_grid)

        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('slammer', anonymous=True)
    laser_res = 40
    laser_sub = laser_scan_subscriber.LaserScanSubscriber("/scan",laser_res)
    velocity_sub = twist_subscriber.TwistSubscriber("/mobile_base/commands/velocity")


    try:
        occ_grid_fast_slam(laser_sub, laser_res, velocity_sub)

    except rospy.ROSInterruptException:
        pass
